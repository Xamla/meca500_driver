#!/usr/bin/env th

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
require 'Meca500Driver'
require 'ros.actionlib.ActionServer'
require 'ros.actionlib.SimpleActionServer'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local actionlib = ros.actionlib
local meca500 = require 'meca500_env'

local nh                        -- ros node handle
local jointStatePublisher       -- joint state publisher
local robotStatusPublisher
local gripperStatusPublisher
local lastJointStateStamp
local lastRobotStatusStamp
local lastGripperStatusStamp
local jointMsg                  -- joint state message
local robotStatusMsg
local gripperStatusMsg
local jointNames = {}
local followTrajectoryServer    -- action server
local gripperServer             -- action server
local trajectoryQueue = {}      -- list of pending trajectories
local driver
local jointCommandSub            -- open loop position based trajectory controller listener
local currentPosTraj             -- active trajectory of pos traj controller
local activateService
local deactivateService
local homingService
local resetErrorService


-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
  SUCCESSFUL = 0,
  INVALID_GOAL = -1,
  INVALID_JOINTS = -2,
  OLD_HEADER_TIMESTAMP = -3,
  PATH_TOLERANCE_VIOLATED = -4,
  GOAL_TOLERANCE_VIOLATED = -5
}


local function createJointNames(prefix)
  prefix = prefix or ''
  local jointNames = {}
  for i,n in ipairs(meca500.JOINT_NAMES) do
    table.insert(jointNames, prefix .. n)
  end
  return jointNames
end


local function publishRobotStatus(driver)

  local state = driver:getRealtimeState()
  if state.q_actual_time ~= nil and (lastJointStateStamp == nil or lastJointStateStamp < state.q_actual_time) then
    lastJointStateStamp = state.q_actual_time
    jointMsg.header.stamp = state.q_actual_time
    jointMsg.position = state.q_actual
    jointMsg.velocity = state.qd_actual
    jointStatePublisher:publish(jointMsg)
  end

  if state.robotStatusTime ~= nil and (lastRobotStatusStamp == nil or lastRobotStatusStamp < state.robotStatusTime) then
    lastRobotStatusStamp = state.robotStatusTime
    local robotStatus = state.robotStatus
    robotStatusMsg.activated = robotStatus.activated
    robotStatusMsg.homing_performed = robotStatus.homingPerformed
    robotStatusMsg.joint_feed = robotStatus.jointFeed
    robotStatusMsg.error = robotStatus.error
    robotStatusMsg.pose_feed = robotStatus.poseFeed
    robotStatusMsg.eob = robotStatus.eob
    robotStatusMsg.eom = robotStatus.eom
    robotStatusPublisher:publish(robotStatusMsg)
  end

  if state.gripperStatusTime ~= nil and (lastGripperStatusStamp == nil or lastGripperStatusStamp < state.gripperStatusTime) then
    lastGripperStatusStamp = state.gripperStatusTime
    local gripperStatus = state.gripperStatus
    gripperStatusMsg.gripper_enabled = gripperStatus.gripperEnabled
    gripperStatusMsg.homing_done = gripperStatus.homingDone
    gripperStatusMsg.holding_part = gripperStatus.holdingPart
    gripperStatusMsg.limit_reached = gripperStatus.limitReached
    gripperStatusMsg.general_error = gripperStatus.generalError
    gripperStatusMsg.overload = gripperStatus.overload
    gripperStatusPublisher:publish(gripperStatusMsg)
  end

end


local function findIndex(t, condition)
  for i,v in ipairs(t) do
    if condition(v, i) then
      return i
    end
  end
  return -1
end


local function indexOf(t, v)
  for i=1,#t do
    if t[i] == v then
      return i
    end
  end
  return -1
end


local function copyMapped(dst, src, map)
  for k,v in pairs(map) do
    dst[k] = src[v]
  end
end


local function decodeJointTrajectoryMsg(trajectory)
  -- print('trajectory_msg')
  -- print(trajectory)

  -- get joint names and create mapping
  local jointMapping = {}
  for i,name in ipairs(trajectory.joint_names) do
    local j = findIndex(jointNames, function(x) return x == name end)
    if j > 0 then
      jointMapping[j] = i
    end
  end

  local pointCount = #trajectory.points
  local time = torch.zeros(pointCount)       -- convert trajectory to internal tensor format
  local pos = torch.zeros(pointCount, 6)
  local vel = torch.zeros(pointCount, 6)
  local acc = torch.zeros(pointCount, 6)
  local hasVelocity = true
  local hasAcceleration = true

  for i=1,pointCount do
    local pt = trajectory.points[i]
    time[i] = pt.time_from_start:toSec()
    copyMapped(pos[i], pt.positions, jointMapping)

    if pt.velocities ~= nil and pt.velocities:nElement() > 0 then
      copyMapped(vel[i], pt.velocities, jointMapping)
    else
      hasVelocity = false
    end

    if pt.accelerations ~= nil and pt.accelerations:nElement() > 0 then
      copyMapped(acc[i], pt.accelerations, jointMapping)
    else
      hasAcceleration = false
    end
  end

  if not hasAcceleration then
    acc = nil
  end

  --[[
  print('current joint configuration (q_actual):')
  print(driver:getRealtimeState().q_actual)

  print('time:')
  print(time)

  print('pos:')
  print(pos)

  print('vel:')
  print(vel)
  ]]

  return time, pos, vel, acc
end


local function isTerminalGoalStatus(status)
  return status == GoalStatus.PREEMPTED or
    status == GoalStatus.SUCCEEDED or
    status == GoalStatus.ABORTED or
    status == GoalStatus.REJECTED or
    status == GoalStatus.RECALLED or
    status == GoalStatus.LOST
end


-- Called when new follow trajectory action goal is received.
-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionGoal.html
local function FollowJointTrajectory_Goal(goalHandle)
  local g = goalHandle:getGoal()

  -- decode trajectory
  local time, pos, vel, acc = decodeJointTrajectoryMsg(g.goal.trajectory)

  ros.INFO('FollowJointTrajectory_Goal: Trajectory with %d points received.', time:nElement())

  local traj = {
    time = time, pos = pos, vel = vel, acc = acc,
    goalHandle = goalHandle, goal = g,
    accept = function(self)
      local status = goalHandle:getGoalStatus().status
      if status == GoalStatus.PENDING then
        goalHandle:setAccepted('Starting trajectory execution')
        return true
      else
        ros.WARN('Status of queued trajectory is not pending but %d.', status)
        return false
      end
    end,
    proceed = function(self)
      local status = goalHandle:getGoalStatus().status
      if status == GoalStatus.ACTIVE then
        return true
      else
        ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', status)
        return false
      end
    end,
    -- cancel signals the begin of a cancel/stop operation (to enter the preempting action state)
    cancel = function(self)
      local status = goalHandle:getGoalStatus().status
      if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        goalHandle:setCancelRequested()
      end
    end,
    -- abort signals end of trajectory processing
    abort = function(self, msg)
      local status = goalHandle:getGoalStatus().status
      if status == GoalStatus.ACTIVE or status == GoalStatus.PREEMPTING then
        goalHandle:setAborted(nil, msg or 'Error')
      elseif status == GoalStatus.PENDING or status == GoalStatus.RECALLING then
        goalHandle:setCanceled(nil, msg or 'Error')
      elseif not isTerminalGoalStatus(status) then
        ros.ERROR('[abort trajectory] Unexpected goal status: %d', status)
      end
    end,
    completed = function(self)
      local r = goalHandle:createResult()
      r.error_code = TrajectoryResultStatus.SUCCESSFUL
      goalHandle:setSucceeded(r, 'Completed')
    end
  }

  if traj.pos:nElement() == 0 then    -- empty trajectory
    local r = goalHandle:createResult()
    r.error_code = TrajectoryResultStatus.SUCCESSFUL
    goalHandle:setSucceeded(r, 'Completed (nothing to do)')
    ros.WARN('Received empty FollowJointTrajectory request (goal: %s).', goalHandle:getGoalID().id)
  else
    local ok, reason = driver:validateTrajectory(traj)
    if ok then
      driver:doTrajectoryAsync(traj)    -- queue for processing
      ros.INFO('Trajectory queued for execution (goal: %s).', goalHandle:getGoalID().id)
    else
      -- trajectory is not valid, immediately abort it
      ros.WARN('Aborting trajectory processing: ' .. reason)
      local r = goalHandle:createResult()
      r.error_code = TrajectoryResultStatus.INVALID_GOAL
      goalHandle:setRejected(r, 'Validation of trajectory failed')
    end
  end
end


local function FollowJointTrajectory_Cancel(goalHandle)
  ros.INFO('FollowJointTrajectory_Cancel')

  if driver.currentTrajectory ~= nil and driver.currentTrajectory.traj.goalHandle == goalHandle then
    driver:cancelCurrentTrajectory()
  else
    -- check if trajectory is in trajectoryQueue
    local i = findIndex(driver.trajectoryQueue, function(x) return x.goalHandle == goalHandle end)
    if i > 0 then
      -- entry found, simply remove from queue
      table.remove(driver.trajectoryQueue, i)
    else
      ros.WARN('Trajectory to cancel with goal handle \'%s\' not found.', goalHandle:getGoalID().id)
    end
    goalHandle:setCanceled(nil, 'Canceled')
  end
end


local function Gripper_Goal(actionServer)
  local g = actionServer:acceptNewGoal()
  print('Gripper_Goal')
  print(g)

  local r = actionServer:createResult()
  print(r)
  actionServer:setSucceeded(r, 'done')
end


local function Gripper_Preempt(actionServer)
  actionServer:setPreempted(nil, 'blub')
end


local function handleJointCommandMessage(msg, header, subscriber)
  -- msg is trajectory_msgs/JointTrajectory, see http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html

  -- decode trajectory
  local time, pos, vel, acc = decodeJointTrajectoryMsg(msg)

  if time:nElement() == 0 then    -- empty trajectory
    ros.WARN('Received empty JointTrajectory request.')
    return
  end

  ros.DEBUG('handleJointCommandMessage: Trajectory with %d points received.', time:nElement())

  local traj = {
    maxBuffering = 2,
    flush = true,
    waitConvergence = true,
    time = time, pos = pos, vel = vel, acc = acc,
    abort = function(self, msg)
      currentPosTraj = nil
    end,
    completed = function(self)
      currentPosTraj = nil
    end
  }

  local ok, reason = driver:validateTrajectory(traj)
  if not ok then
    -- trajectory is not valid, ignore request
    ros.WARN('Aborting trajectory processing: ' .. reason)
    return
  end

  if currentPosTraj ~= nil then

    if driver.currentTrajectory ~= nil and driver.currentTrajectory.traj == currentPosTraj then
      currentPosTraj = driver:blendTrajectory(traj)
      ros.DEBUG('Blending existing trajectory to new trajectory (posTrajController).')
    else
      local i = indexOf(driver.trajectoryQueue, currentPosTraj)
      if i > 0 then
        currentPosTraj = traj
        driver.trajectoryQueue[i] = traj    -- replace exsting pending trajectory
      else
        ros.WARN('currentPosTraj not found in driver queue.')
        currentPosTraj = traj
        driver:doTrajectoryAsync(traj)      -- queue for processing
      end
    end

  else

    if #driver.trajectoryQueue == 0 then
      currentPosTraj = traj
      driver:doTrajectoryAsync(traj)    -- queue for processing
      ros.DEBUG("Trajectory queued for execution (posTrajController).")
    else
      ros.WARN('Driver busy. Rejecting posTrajController request.')
    end

  end

end


local function handleActivateRobot(request, response, header)
  local ok = driver:activateRobot()
  response.success = ok ~= nil
  response.message = ''
  return true
end


local function handleDeactivateRobot(request, response, header)
  local ok = driver:deactivateRobot()
  response.success = ok ~= nil
  response.message = ''
  return true
end


local function handleHoming(request, response, header)
  local ok = driver:home()
  response.success = ok ~= nil
  response.message = ''
  return true
end


local function handleResetError(request, response, header)
  local ok = driver:resetError()
  response.success = ok ~= nil
  response.message = ''
  return true
end


local function printSplash()
  print([[
   _  __                __
  | |/ /___ _____ ___  / /___ _   "The smaller the better"
  |   / __ `/ __ `__ \/ / __ `/
 /   / /_/ / / / / / / / /_/ /
/_/|_\__,_/_/ /_/ /_/_/\__,_/

Xamla Meca500 ROS driver v1.0
]])
end


local function main()
  printSplash()

  local rosArgs = {}

  -- filter special ROS command line args
  for k,v in pairs(arg) do
    if v:sub(1,2) == '__' then
      table.insert(rosArgs, v)
      arg[k] = nil
    end
  end

  -- parse command line arguments
  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text('Xamla ROS driver for the Mecademic Meca500 robot arm')
  cmd:text()
  cmd:option('-hostname',              '192.168.0.100',   'hostname of robot to connect to')
  cmd:option('-control-port',                    10000,   'control port')
  cmd:option('-path-tolerance',           math.pi / 10,   'max set point distance to current joint configuration')
  cmd:option('-joint-name-prefix',                  '',   'name prefix of published joints')
  cmd:option('-auto-activation',                  true,   'flag: Activate robot if it is turned off')
  cmd:option('-auto-homing',                      true,   'flag: Perform homing automatically')
  cmd:option('-auto-reset-error',                false,   'flag: Try to reset error during init')
  cmd:option('-auto-parking',                    false,   'flag: Move robot to home position on shutdown')
  cmd:option('-max-convergence-cycles',            250,   'max number of cycles to wait for goal convergence')
  cmd:option('-goal-position-threshold',         0.005,   'goal convergence position threshold (in rad)')
  cmd:option('-goal-velocity-threshold',          0.01,   'goal convergence velocity threshold (in rad/s)')
  local opt = cmd:parse(arg or {})

  local logger = {
    debug = ros.DEBUG,
    info = ros.INFO,
    warn = ros.WARN,
    error = ros.ERROR
  }

  -- ros initialization
  local ros_init_options = 0
  ros.init('meca500_driver', ros_init_options, rosArgs)
  nh = ros.NodeHandle('~')

  jointCommandSub = nh:subscribe('joint_command', 'trajectory_msgs/JointTrajectory', 1)
  jointCommandSub:registerCallback(handleJointCommandMessage)

  -- create driver object
  local driverConfiguration = {
    hostname                = opt['hostname'],
    controlPort             = opt['control-port'],
    pathTolerance           = opt['path-tolerance'],
    maxSinglePointTrajectoryDistance = opt['max-single-point-trajectory-distance'],
    jointNamePrefix         = opt['joint-name-prefix'],
    autoActivation          = opt['auto-activation'],
    autoHoming              = opt['auto-homing'],
    autoResetError          = opt['auto-reset-error'],
    autoParking             = opt['auto-parking'],
    maxConvergenceCycles    = opt['max-convergence-cycles'],
    goalPositionThreshold   = opt['goal-position-threshold'],
    goalVelocityThreshold   = opt['goal-velocity-threshold']
  }

  local overrideInputArguments = function (key, value, ok)
    if ok == true  then
      driverConfiguration[key] = value
    end
  end

  overrideInputArguments('hostname', nh:getParamString('hostname'))
  overrideInputArguments('controlPort', nh:getParamInt('control_port'))
  overrideInputArguments('pathTolerance', nh:getParamDouble('path_tolerance'))
  overrideInputArguments('maxSinglePointTrajectoryDistance', nh:getParamDouble('max_single_point_trajectory_distance'))
  overrideInputArguments('jointNamePrefix', nh:getParamString('joint_name_prefix'))
  overrideInputArguments('autoActivation', nh:getParamBool('auto_activation'))
  overrideInputArguments('autoHoming', nh:getParamBool('auto_homing'))
  overrideInputArguments('autoResetError', nh:getParamBool('auto_reset_error'))
  overrideInputArguments('autoParking', nh:getParamBool('auto_parking'))
  overrideInputArguments('maxConvergenceCycles', nh:getParamInt('max_convergence_cycles'))
  overrideInputArguments('goalPositionThreshold', nh:getParamDouble('goal_position_threshold'))
  overrideInputArguments('goalVelocityThreshold', nh:getParamDouble('goal_velocity_threshold'))

  -- print effective options
  print('Effective driver configuration:')
  print(driverConfiguration)

  -- create joint state publisher
  jointNames = createJointNames(driverConfiguration.jointNamePrefix)
  jointStatePublisher = nh:advertise('/joint_states', 'sensor_msgs/JointState', 1)
  jointMsg = jointStatePublisher:createMessage()
  jointMsg.name = jointNames

  robotStatusPublisher = nh:advertise('robot_status', 'meca500_msgs/RobotStatus', 1)
  robotStatusMsg = robotStatusPublisher:createMessage()

  gripperStatusPublisher = nh:advertise('gripper_status', 'meca500_msgs/GripperStatus', 1)
  gripperStatusMsg = gripperStatusPublisher:createMessage()

  -- heartbeat
  local heartbeat = nil
  --local heartbeat = xamal_sysmon.Heartbeat()
  --heartbeat:start(nh, 5)

  driver = Meca500Driver(driverConfiguration, logger, heartbeat)
  driver:addSyncCallback(publishRobotStatus)

  -- set up follow trajectory action server
  ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)
  followTrajectoryServer = actionlib.ActionServer(nh, 'follow_joint_trajectory', 'control_msgs/FollowJointTrajectory')
  followTrajectoryServer:registerGoalCallback(FollowJointTrajectory_Goal)
  followTrajectoryServer:registerCancelCallback(FollowJointTrajectory_Cancel)
  followTrajectoryServer:start()

  activateService = nh:advertiseService('activate_robot', ros.SrvSpec('std_srvs/Trigger'), handleActivateRobot)
  deactivateService = nh:advertiseService('deactivate_robot', ros.SrvSpec('std_srvs/Trigger'), handleDeactivateRobot)
  homingService = nh:advertiseService('homing', ros.SrvSpec('std_srvs/Trigger'), handleHoming)
  resetErrorService = nh:advertiseService('reset_error', ros.SrvSpec('std_srvs/Trigger'), handleResetError)

  gripperServer = actionlib.SimpleActionServer(nh, 'gripper_command', 'control_msgs/GripperCommand')
  gripperServer:registerGoalCallback(Gripper_Goal)
  gripperServer:registerPreemptCallback(Gripper_Preempt)
  gripperServer:start()

  -- main driver loop
  local rate = ros.Rate(200)
  while ros.ok() do
    driver:spin()
    if heartbeat ~= nil then
      heartbeat:publish()
    end
    ros.spinOnce()
    collectgarbage()
    rate:sleep()
  end

  -- tear down components
  gripperServer:shutdown()
  activateService:shutdown()
  deactivateService:shutdown()
  homingService:shutdown()
  resetErrorService:shutdown()
  jointCommandSub:shutdown()
  jointStatePublisher:shutdown()
  robotStatusPublisher:shutdown()
  gripperStatusPublisher:shutdown()
  followTrajectoryServer:shutdown()
  driver:shutdown()
  nh:shutdown()
  ros.shutdown()
  print('Shutdown complete.')
end


main()
