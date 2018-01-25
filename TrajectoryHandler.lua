local torch = require 'torch'
local ros = require 'ros'
require 'TrajectorySampler'
local meca500 = require 'meca500_env'


local GOAL_CONVERGENCE_POSITION_THRESHOLD = 0.005   -- in rad
local GOAL_CONVERGENCE_VELOCITY_THRESHOLD = 0.01    -- in rad/s
local MAX_CONVERGENCE_CYCLES = 100
local LOOK_AHEAD_SECONDS = 0.5
local MAX_VELOCITY_RAD = meca500.MAX_VELOCITY_RAD
local MIN_VOLOCITY_RAD = meca500.MIN_VOLOCITY_RAD


local TrajectoryHandlerStatus = {
  --ProtocolError = -3,
  --ConnectionLost = -2,
  Canceled = -1,
  Fresh = 0,
  Streaming = 1,
  Flushing = 2,
  Completed = 1000,
}
meca500.TrajectoryHandlerStatus = TrajectoryHandlerStatus


local TrajectoryHandler = torch.class('TrajectoryHandler')


function TrajectoryHandler:__init(traj, controlStream, realtimeState, dt, logger)
  self.traj = traj
  self.controlStream = controlStream
  self.realtimeState = realtimeState
  self.dt = dt
  self.logger = logger or ur5.DEFAULT_LOGGER
  self.noResponse = 0
  self.status = TrajectoryHandlerStatus.Fresh
  self.sampler = TrajectorySampler(traj, dt)
  self.noResponse = 0
  self.convergenceCycle = 0
  self.startTime = ros.Time.now()
  self.flush = true
  self.waitCovergence = true
  self.singleWaypoint = traj.singleWaypoint or (traj.time:size(1) == 1)
end
  

function TrajectoryHandler:cancel()
  if self.status > 0 then
    self.controlStream:clearMotion()    -- send stop message to robot
    self.status = TrajectoryHandlerStatus.Canceled
  end
end


local function reachedGoal(self)
  local q_goal = self.sampler:getGoalPosition()
  local q_actual = self.realtimeState.q_actual
  local qd_actual = self.realtimeState.qd_actual

  local goal_distance = torch.norm(q_goal - q_actual)

  self.logger.debug('Convergence cycle %d: |qd_actual|: %f; goal_distance (joints): %f;', self.convergenceCycle, qd_actual:norm(), goal_distance)

  self.convergenceCycle = self.convergenceCycle + 1
  if self.convergenceCycle >= MAX_CONVERGENCE_CYCLES then
    error(string.format('Did not reach goal after %d convergence cycles.', MAX_CONVERGENCE_CYCLES))
  end

  return qd_actual:norm() < GOAL_CONVERGENCE_VELOCITY_THRESHOLD and goal_distance < GOAL_CONVERGENCE_POSITION_THRESHOLD
end


function TrajectoryHandler:update()
  if self.status < 0 or self.status == TrajectoryHandlerStatus.Completed then
    return false
  end

  if self.sampler:atEnd() and self.flush == false then
    self.status = TrajectoryHandlerStatus.Completed
    return false  -- all data sent, nothing to do
  end

  local now = ros.Time.now()
  local elapsed = now - self.startTime

  if not self.sampler:atEnd() then
    self.status = TrajectoryHandlerStatus.Streaming
    -- compute time of maximum lookahead trajectory point to send to robot
    local queueEndTime = (elapsed + LOOK_AHEAD_SECONDS):toSec()
    local q_last = self.sampler:evaluateAt(self.sampler:getCurrentTime())
    local pos,vel = self.sampler:generatePointsUntil(queueEndTime)
    local q_actual = self.realtimeState.q_actual
    local velocity = MAX_VELOCITY_RAD   -- scalar value used for all joints (2x for j4-j6)

    -- send points to robot
    for i,q_desired in ipairs(pos) do

      if self.singleWaypoint then
        local delta = q_desired - q_actual
        -- compute the minimum time when target will be reached (assume max cruising speed)
        local minTime = torch.cdiv(delta, meca500.JOINT_VELOCITY_LIMITS[{{},2}]):abs():max()
        if delta:norm() > 1e-5 and minTime > 2 * self.dt then
          -- truncate goal
          q_desired = q_actual + delta * (2 * self.dt / minTime)
        end
        velocity = MAX_VELOCITY_RAD   -- use maximum velocity for single waypoints
      else
        -- estimate required time to reach next setpoint
        local delta = q_desired - q_last
        local qd_desired = torch.abs(vel[i])        -- work with absolute velocity
        local qd_safe = qd_desired:clone()
        qd_safe[qd_desired:lt(1e-3)] = 1e-3  -- establish a lower bound to prevent inf/NaN
        local minTime = torch.cdiv(delta, qd_safe):abs():max()
        if minTime > self.dt then
          -- computed waypoint velocity value is too low
          qd_safe:mul(minTime/self.dt):clamp(MIN_VOLOCITY_RAD, MAX_VELOCITY_RAD)
        end
        velocity = qd_safe:max()
      end

      self.controlStream:setJointVel(velocity)
      self.controlStream:moveJoints(q_desired)
      q_last = q_desired
    end

  else

    -- snap to goal
    local q_goal = self.sampler:getGoalPosition()
    self.controlStream:setJointVel(MAX_VELOCITY_RAD)
    self.controlStream:moveJoints(q_goal)

    if elapsed:toSec() >= self.sampler:getEndTime() and (reachedGoal(self) or not self.waitCovergence) then      -- wait for convergence
      self.status = TrajectoryHandlerStatus.Completed
      return false
    else
      self.status = TrajectoryHandlerStatus.Flushing
    end
  end

end
