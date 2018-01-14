local torch = require 'torch'
local ros = require 'ros'
require 'TrajectorySampler'
local meca500 = require 'meca500_env'


local GOAL_CONVERGENCE_POSITION_THRESHOLD = 0.00051   -- in rad
local GOAL_CONVERGENCE_VELOCITY_THRESHOLD = 0.001    -- in rad/s
local MAX_CONVERGENCE_CYCLES = 50
local LOOK_AHEAD_SECONDS = 0.5


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

  local goal_distance = torch.norm(q_goal - q_actual)

  self.logger.info('Convergence cycle %d: |qd_actual|: %f; goal_distance (joints): %f;', self.convergenceCycle, qd_actual:norm(), goal_distance)

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

  if not self.samper:atEnd() then
    self.status = TrajectoryHandlerStatus.Streaming
    -- compute time of maximum lookahead trajectory point to send to robot
    local queueEndTime = ((now + LOOK_AHEAD_SECONDS) - self.startTime):toSec()
    local pos,vel = self.sampler:generatePointsUntil(queueEndTime)
    -- send points to robot

    for i,v in ipairs(pos) do
      print('would send', v)
      --self:controlStream:moveJoints(v)
    end

  else
    if (reachedGoal(self) or not self.waitCovergence) then      -- when buffer is empty we are done
      self.status = TrajectoryHandlerStatus.Completed
      return false
    else
      self.status = TrajectoryHandlerStatus.Flushing
    end
  end

end
