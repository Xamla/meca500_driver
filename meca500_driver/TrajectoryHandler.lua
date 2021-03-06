--[[
This file is part of the driver for Meca 500 robots.
Copyright (C) 2018 Xamla and/or its affiliates

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
]]


local torch = require 'torch'
local ros = require 'ros'
require 'TrajectorySampler'
local meca500 = require 'meca500_env'


local MAX_NO_RESPONSE = 100   -- number of cycles without avail count message from driver before entering ConnectionLost state
local STOP_CYCLE_COUNT = 3  -- number of cycles with v < threshold before robot is considered stopped
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
  Cancelling = 3,     -- stop request has been sent to robot, waiting for zero velocity
  Completed = 1000,
}
meca500.TrajectoryHandlerStatus = TrajectoryHandlerStatus


local TrajectoryHandler = torch.class('TrajectoryHandler')


function TrajectoryHandler:__init(traj, controlStream, realtimeState, dt,
  maxConvergenceCycles, goalPositionThreshold, goalVelocityThreshold, logger)

  assert(maxConvergenceCycles > 0, "Argument 'maxConvergenceCycles' must be greater than zero.")
  assert(goalPositionThreshold > 0, "Argument 'goalPositionThreshold' must be greater than zero.")
  assert(goalVelocityThreshold >= 0, "Argument 'goalVelocityThreshold' must be greater than or equal zero.")
  self.lastJointStateStamp = realtimeState.q_actual_time
  self.traj = traj
  self.controlStream = controlStream
  self.realtimeState = realtimeState
  self.dt = dt
  self.maxConvergenceCycles = maxConvergenceCycles
  self.goalPositionThreshold = goalPositionThreshold
  self.goalVelocityThreshold = goalVelocityThreshold
  self.logger = logger or ur5.DEFAULT_LOGGER
  self.status = TrajectoryHandlerStatus.Fresh
  self.sampler = TrajectorySampler(traj, dt)
  self.convergenceCycle = 0
  self.noMotionCycle = 0
  self.startTime = ros.Time.now()
  self.flush = true
  self.waitCovergence = true
  self.singleWaypoint = traj.singleWaypoint or (traj.time:size(1) == 1)
end


function TrajectoryHandler:cancel()
  if self.status > 0 and self.status ~= TrajectoryHandlerStatus.Cancelling then
    self.controlStream:clearMotion()    -- send stop message to robot
    self.status = TrajectoryHandlerStatus.Cancelling
  end
end


local function isRobotStopped(self)
  return self.noMotionCycle > STOP_CYCLE_COUNT
end


local function checkRobotStopped(self)
  local qd_actual = self.realtimeState.qd_actual
  if qd_actual:norm() < self.goalVelocityThreshold then
    self.noMotionCycle = self.noMotionCycle + 1
  else
    self.noMotionCycle = 0
  end
  return isRobotStopped(self)
end


local function reachedGoal(self)
  local q_goal = self.sampler:getGoalPosition()
  local q_actual = self.realtimeState.q_actual
  local qd_actual = self.realtimeState.qd_actual

  local goal_distance = torch.norm(q_goal - q_actual)

  self.logger.debug('Convergence cycle %d: |qd_actual|: %f; goal_distance (joints): %f;', self.convergenceCycle, qd_actual:norm(), goal_distance)

  self.convergenceCycle = self.convergenceCycle + 1
  if self.convergenceCycle >= self.maxConvergenceCycles then
    error(string.format('[TrajectoryHandler] Did not reach goal after %d convergence cycles. Goal distance: %f; |qd_actual|: %f;', self.maxConvergenceCycles, goal_distance, qd_actual:norm()))
  end

  return isRobotStopped(self) and goal_distance < self.goalPositionThreshold
end


local function clipping(self, q_desired)
  local q_actual = self.realtimeState.q_actual
  local delta = q_desired - q_actual
  -- compute the minimum time when target will be reached (assume max cruising speed)
  local age = (ros.Time.now() - self.realtimeState.q_actual_time):toSec()
  local minTime = math.max(torch.cdiv(delta, meca500.JOINT_VELOCITY_LIMITS[{{},2}]):abs():max() - age, 0)
  if delta:norm() > 1e-5 and minTime > 2 * self.dt then
    -- truncate goal
    self.logger.debug('truncate goal: %f, joint state age %f', 2 * self.dt / minTime, age)
    q_desired = q_actual + delta * 2 * self.dt / minTime
  end
  return q_desired
end


function TrajectoryHandler:update()
  if self.status < 0 or self.status == TrajectoryHandlerStatus.Completed then
    return false
  end

  if self.sampler:atEnd() and self.flush == false and self.status ~= TrajectoryHandlerStatus.Cancelling then
    self.status = TrajectoryHandlerStatus.Completed
    return false  -- all data sent, nothing to do
  end

  local now = ros.Time.now()
  local elapsed = now - self.startTime

  if self.lastJointStateStamp >= self.realtimeState.q_actual_time then
    return
  end
  self.lastJointStateStamp = self.realtimeState.q_actual_time

  if self.status == TrajectoryHandlerStatus.Cancelling then
    checkRobotStopped(self)

    if checkRobotStopped(self) then    -- wait for robot to stop
      self.status = TrajectoryHandlerStatus.Canceled
      return false
    else

      -- check limited number of cycles and generate error when robot does not hold
      self.convergenceCycle = self.convergenceCycle + 1
      if self.convergenceCycle >= self.maxConvergenceCycles then
        error(string.format('[TrajectoryHandler] Error: Robot did not stop after %d convergence cycles. Goal distance: %f; |qd_actual|: %f;', self.maxConvergenceCycles, goal_distance, qd_actual:norm()))
      end

    end

  elseif not self.sampler:atEnd() then
    self.status = TrajectoryHandlerStatus.Streaming
    if self.singleWaypoint then

      local q_desired = self.sampler:evaluateAt(elapsed:toSec())
      self.sampler.t = elapsed:toSec()

      q_desired = clipping(self, q_desired)

      local velocity = MAX_VELOCITY_RAD   -- use maximum velocity for single waypoints
      self.controlStream:setJointVel(velocity)
      self.controlStream:moveJoints(q_desired)

    else

      local q_last = self.sampler:evaluateAt(self.sampler:getCurrentTime())
      -- compute time of maximum lookahead trajectory point to send to robot
      local queueEndTime = (elapsed + LOOK_AHEAD_SECONDS):toSec()
      local pos,vel = self.sampler:generatePointsUntil(queueEndTime)

      -- send points to robot
      for i,q_desired in ipairs(pos) do

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
        local velocity = qd_safe:max()
        self.controlStream:setJointVel(velocity)
        self.controlStream:moveJoints(q_desired)
        q_last = q_desired
      end

    end

  else

    checkRobotStopped(self)

    -- snap to goal
    local q_goal = self.sampler:getGoalPosition()

    if self.singleWaypoint then
      q_goal = clipping(self, q_goal)
    end

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
