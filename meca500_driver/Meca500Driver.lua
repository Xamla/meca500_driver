local torch = require 'torch'
local sys = require 'sys'
require 'ControlStream'
require 'RealtimeState'
require 'TrajectoryHandler'
local meca500 = require 'meca500_env'
local ResponseCode = meca500.ResponseCode


local ControlStreamState = meca500.ControlStreamState
local TrajectoryHandlerStatus = meca500.TrajectoryHandlerStatus


-- constants
local MAX_SYNC_READ_TRIES = 250
local DEFAULT_MAX_SINGLE_POINT_TRAJECTORY_DISTANCE = 0.5      -- max allowed distance of single point trajectory target relative to current joint pos
local DEFAULT_HOSTNAME = '192.168.0.100'
local DEFAULT_CONTROL_PORT = 10000
local DEFAULT_MAX_CONVERGENCE_CYCLES = 150
local JOINT_NAMES = meca500.JOINT_NAMES
local JOINT_POSITION_LIMITS = meca500.JOINT_POSITION_LIMITS
local JOINT_VELOCITY_LIMITS = meca500.JOINT_VELOCITY_LIMITS
local DEFAULT_GOAL_CONVERGENCE_POSITION_THRESHOLD = 0.005 -- in rad
local DEFAULT_GOAL_CONVERGENCE_VELOCITY_THRESHOLD = 0.01 -- in rad/s


local Meca500Driver = torch.class('Meca500Driver')


function Meca500Driver:__init(cfg, logger, heartbeat)
  self.logger = logger or meca500.DEFAULT_LOGGER
  self.heartbeat = heartbeat

  -- apply configuration
  self.hostname = cfg.hostname or DEFAULT_HOSTNAME
  self.controlPort = cfg.controlPort or DEFAULT_CONTROL_PORT
  self.pathTolerance = cfg.pathTolerance or DEFAULT_PATH_TOLERANCE
  self.maxSinglePointTrajectoryDistance = cfg.maxSinglePointTrajectoryDistance or DEFAULT_MAX_SINGLE_POINT_TRAJECTORY_DISTANCE
  self.jointNamePrefix = cfg.jointNamePrefix
  self.autoActivation = cfg.autoActivation
  self.autoHoming = cfg.autoHoming
  self.autoResetError = cfg.autoResetError
  self.autoParking = cfg.autoParking
  self.maxConvergenceCycles = cfg.maxConvergenceCycles or DEFAULT_MAX_CONVERGENCE_CYCLES
  self.goalPositionThreshold = cfg.goalPositionThreshold or DEFAULT_GOAL_CONVERGENCE_POSITION_THRESHOLD
  self.goalVelocityThreshold = cfg.goalVelocityThreshold or DEFAULT_GOAL_CONVERGENCE_VELOCITY_THRESHOLD

  self.activateSent = false
  self.homeSent = false
  self.resetErrorSent = false

  self.realtimeState = RealtimeState()
  self.controlStream = ControlStream(self.realtimeState, self.logger)
  self.syncCallbacks = {}
  self.trajectoryQueue = {}      -- list of pending trajectories
  self.servoTime = 1/66
end


function Meca500Driver:validateTrajectory(traj)
  local time, pos, vel = traj.time, traj.pos, traj.vel

  if pos == nil or time == nil or pos:nElement() == 0 or time:nElement() == 0 then
    return false, 'Trajectory has no waypoints.'
  end

  if pos:size(1) ~= time:size(1) then
    return false, "Row counts of 'time' and 'pos' fields do not match."
  end

  -- verify that time is strictly increasing
  local last_time = -math.huge
  for i=1,time:size(1) do
    if time[i] < last_time then
      return false, 'time_from_start of trajectory is not strictly increasing.'
    end
    last_time = time[i]

    -- check joints
    for j=1,6 do
      if pos[{i,j}] < JOINT_POSITION_LIMITS[{j,1}] or pos[{i,j}] > JOINT_POSITION_LIMITS[{j,2}] then
        return false, string.format("Waypoint %d: Position %f of joint '%s' lies outside control limits.", i, pos[{i,j}], JOINT_NAMES[j])
      end
      if vel[{i,j}] < JOINT_VELOCITY_LIMITS[{j,1}] or vel[{i,j}] > JOINT_VELOCITY_LIMITS[{j,2}] then
        return false, string.format("Waypoint %d: Velocity %f of joint '%s' lies outside control limits.", i, vel[{i,j}], JOINT_NAMES[j])
      end
    end
  end

  return true, 'ok'
end


function Meca500Driver:blendTrajectory(traj)
  assert(traj and traj.time:nElement() > 0)

  if self.currentTrajectory == nil then
    self:doTrajectoryAsync(traj)
    return traj
  else

    local traj_
    if traj.time:size(1) == 1 and traj.time[1] <= self.servoTime then
      traj_ = traj    -- special case for single point trajectory with 0 time_from_start (no blending)
    else
      -- get position of last command sent to robot
      local sampler = self.currentTrajectory.handler.sampler
      local q,qd = sampler:evaluateAt(sampler:getCurrentTime())

      local time = torch.Tensor(traj.time:size(1)+1)
      local pos = torch.Tensor(traj.pos:size(1)+1, 6)
      local vel = torch.Tensor(traj.vel:size(1)+1, 6)

      time[1] = 0
      time[{{2,time:size(1)}}] = traj.time + self.servoTime

      pos[1] = q
      pos[{{2,pos:size(1)},{}}] = traj.pos
      vel[1] = qd
      vel[{{2,vel:size(1)},{}}] = traj.vel

      -- create new blended trajectory (do not modify caller's trajectory)
      traj_  = {
        time = time,
        pos = pos,
        vel = vel,
        abort = traj.abort,
        completed = traj.completed,
        flush = traj.flush,
        waitConvergence = traj.waitConvergence,
        maxBuffering = traj.maxBuffering,
        singleWaypoint = (traj.time:size(1) == 1)   -- mark this trajectory as blended single point
      }
    end

    local handler_ = self:createTrajectoryHandler(traj_, traj_.flush, traj_.waitConvergence, traj_.maxBuffering)

    self.currentTrajectory = {
      traj = traj_,
      handler = handler_
    }

    return traj_
  end
end


function Meca500Driver:activateRobot()
  return self.controlStream:activateRobot()
end


function Meca500Driver:deactivateRobot()
  return self.controlStream:deactivateRobot()
end


function Meca500Driver:home()
  return self.controlStream:home()
end


function Meca500Driver:resetError()
  return self.controlStream:resetError()
end


function Meca500Driver:doTrajectoryAsync(traj)
  table.insert(self.trajectoryQueue, traj)
end


function Meca500Driver:getRealtimeState()
  return self.realtimeState
end


function Meca500Driver:addSyncCallback(fn)
  table.insert(self.syncCallbacks, fn)
end


function Meca500Driver:removeSyncCallback(fn)
  for i,x in ipairs(self.syncCallbacks) do
    if x == fn then
      table.remove(self.syncCallbacks, i)
      return
    end
  end
end


function Meca500Driver:sync()

  -- request status once per sync
  if self.controlStream:getState() == ControlStreamState.Ready then
    self.controlStream:requestStatus()
  end

  for i=1,MAX_SYNC_READ_TRIES do

    if self.controlStream:getState() ~= ControlStreamState.Ready then
      return false, '[Meca500Driver] Control stream not ready.'
    end

    if self.controlStream:read() then

      -- execute sync callbacks (e.g. to publish joint states)
      for i,fn in ipairs(self.syncCallbacks) do
        fn(self)
      end

      return true
    end

  end

  return false, '[Meca500Driver] Sync timeout.'
end


function Meca500Driver:createTrajectoryHandler(traj, flush, waitCovergence)
  -- TODO: check use of flush & waitConvergence

  --[[if flush == nil then
    flush = true
  end
  if waitCovergence == nil then
    waitCovergence = true
  end]]
  return TrajectoryHandler(
    traj,
    self.controlStream,
    self.realtimeState,
    self.servoTime,
    self.maxConvergenceCycles,
    self.goalPositionThreshold,
    self.goalVelocityThreshold,
    self.logger
  )
end


function Meca500Driver:cancelCurrentTrajectory()
  if self.currentTrajectory == nil then
    return
  end

  self.logger.info('[Meca500Driver] Cancelling trajectory execution.')
  local traj = self.currentTrajectory.traj
  local handler = self.currentTrajectory.handler

  local robotReady = self.realtimeState:isRobotReady()
  if not robotReady then
    -- if robot is not ready we abort the trajectory immediately
    if traj.abort ~= nil then
      traj:abort()        -- abort callback
    end
    self.currentTrajectory = nil
  else
    -- otherwise we try to bring the robot gracefully to a standstill
    handler:cancel()
    if traj.cancel ~= nil then
      traj:cancel()        -- cancel callback (e.g. enter canel requested state)
    end
  end
end


local function dispatchTrajectory(self)
  local robotReady = self.realtimeState:isRobotReady()

  if not robotReady then
    if #self.trajectoryQueue > 0 then
      self.logger.warn('[Meca500Driver] Aborting %d queued trajectories since robot is not ready.', #self.trajectoryQueue)
      -- Cancel all queued trajectories due to robot error.
      -- Otherwise possibly an unexpected execution of old trajectories would start when robot becomes ready again.
      -- Execution of current trajectory will be stopped anyways when robot is not ready (see end of this function)...
      for i,traj in ipairs(self.trajectoryQueue) do
        if traj.abort ~= nil then
          traj:abort('Robot not ready.')        -- abort callback
        end
      end
      self.trajectoryQueue = {}   -- clear
    end
  end

  if self.currentTrajectory == nil then
    if robotReady and #self.trajectoryQueue > 0 then    -- check if new trajectory is available
      while #self.trajectoryQueue > 0 do
        local traj = table.remove(self.trajectoryQueue, 1)
        if traj.accept == nil or traj:accept() then   -- call optional accept callback
          local flush, waitCovergence = true, true
          local maxBuffering = self.ringSize

          if traj.flush ~= nil then
            flush = traj.flush
          end

          if traj.waitCovergence ~= nil then
            waitCovergence = traj.waitCovergence
          end

          if traj.maxBuffering ~= nil then
            maxBuffering = math.max(1, traj.maxBuffering)
          end

          if traj.time:nElement() == 1 then
            local time = torch.Tensor(2)
            time[1] = 0
            time[2] = traj.time[1]

            if not self.realtimeState:isValid() then
              self.logger.error('Dropping single point trajectory execution request since realtime state is not valid.')
              return
            end

            local pos = torch.zeros(2,6)
            pos[1] = self.realtimeState.q_actual
            pos[2] = traj.pos[1]

            local targetDistance = torch.norm(pos[1] - pos[2])
            if targetDistance > self.maxSinglePointTrajectoryDistance then
              self.logger.error(
                'Single point trajectory target lies too far away from current joint configuration (distance: %f; maxSinglePointTrajectoryDistance: %f).',
                targetDistance,
                self.maxSinglePointTrajectoryDistance
              )
              return
            end

            local vel = torch.zeros(2,6)
            -- vel[1] = self.realtimeState.qd_actual

            traj.time = time
            traj.pos = pos
            traj.vel = vel
            traj.acc = nil
          end

          self.currentTrajectory = {
            traj = traj,
            handler = self:createTrajectoryHandler(traj, flush, waitCovergence, maxBuffering)
          }
          break
        end
      end

    elseif #self.trajectoryQueue > 0 then
      self.logger.info(
        '[Meca500Driver] Waiting for robot to become ready. (Robot mode: %s; Safety mode: %s)'
      )
    end
  end

  -- ensure first points are send to robot immediately after accepting trajectory execution
  if self.currentTrajectory ~= nil then     -- if we have an exsting trajectory

    local traj = self.currentTrajectory.traj
    local handler = self.currentTrajectory.handler

    -- check if trajectory execution is still desired or if we are cancelling (waiting for robot stop)
    if robotReady and (handler.status == TrajectoryHandlerStatus.Cancelling or traj.proceed == nil or traj:proceed()) then

      -- execute main update call
      local ok, err = pcall(function() handler:update() end)
      if not ok then
        self.logger.warn('Exception during handler update: %s', err)
      end

      if not ok or handler.status < 0 then    -- error
        if traj.abort ~= nil then
          traj:abort()        -- abort callback
        end
        self.currentTrajectory = nil
      elseif handler.status == TrajectoryHandlerStatus.Completed then
        if traj.completed ~= nil then
          traj:completed()    -- completed callback
        end
        self.currentTrajectory = nil
      end

    else
      -- robot not ready or proceed callback returned false
      self:cancelCurrentTrajectory()
    end
  end

end


local function driverCore(self)
  local ok, err = self:sync()
  if not ok then
    error(err)
  end

  if self.controlStream:getState() == ControlStreamState.Ready then
    if self.realtimeState.robotStatus.error then
      -- robot is in error state
      if self.autoResetError and not self.resetErrorSent then
        self.logger.warn('Resetting error...')
        self.controlStream:resetError()
        self.resetErrorSent = true
      end
    else
      -- robot is not in error state
      self.resetErrorSent = false
      if not self.realtimeState.robotStatus.activated then
        if self.autoActivation and not self.activateSent then
          self.logger.info('Activating robot...')
          self.controlStream:activateRobot()
          self.activateSent = true
        end
      elseif not self.realtimeState.robotStatus.homingPerformed then
        if self.autoHoming and not self.homeSent then
          self.logger.info('Homing...')
          self.controlStream:home()
          self.homeSent = true
        end
      elseif not self.realtimeState.robotStatus.jointFeed then
        self.logger.info('Activating joint feed...')
        self.controlStream:activateJointsFeed()
      end
    end
  end

  dispatchTrajectory(self)
end


function Meca500Driver:spin()
  if self.heartbeat ~= nil then
    if self.realtimeState:isRobotReady() then
      self.heartbeat:updateStatus(self.heartbeat.GO, "")
    else
      self.heartbeat:updateStatus(self.heartbeat.INTERNAL_ERROR, "Robot is not ready.")
    end
  end

  if self.controlStream:getState() == ControlStreamState.Disconnected then
    self.activateSent = false
    self.homeSent = false
    self.logger.info('ControlStream not connected, trying to connect...')
    if self.controlStream:connect(self.hostname, self.controlPort) then
      self.logger.info('Control stream connection established.')
    end
  end

  local ok, err = true, nil
  if self.controlStream:getState() == ControlStreamState.Ready then

    -- sync
    ok, err = pcall(function() driverCore(self) end)
    if not ok then
      self.logger.error("[Meca500Driver] Spin error: " .. err)
    end

  elseif self.controlStream:getState() == ControlStreamState.Handshake then
    self.controlStream:read()
  end

  if not ok or self.controlStream:getState() == ControlStreamState.Error then

    -- abort current trajectory
    if self.currentTrajectory ~= nil then
      local traj = self.currentTrajectory.traj
      if traj.abort ~= nil then
        traj:abort()
      end
      self.currentTrajectory = nil
    end

    -- realtime stream is in error state, recreate stream object

    self.logger.warn('Closing robot communication socket and reconnecting. (RT state: %d)', self.controlStream:getState())
    self:shutdown()

    self.realtimeState:invalidate()
    self.controlStream = ControlStream(self.realtimeState, self.logger)
  end

end


function Meca500Driver:shutdown()
  self.logger.info('Shutting down driver...')

  if self.controlStream:getState() == ControlStreamState.Ready then
    self.logger.info('deactivateJointsFeed')
    self.controlStream:deactivateJointsFeed()
    sys.sleep(0.1)
    self.controlStream:read()

    self.logger.info('clearMotion')
    self.controlStream:clearMotion()
    sys.sleep(0.2)
    self.controlStream:read()

    if self.autoParking then
      self.logger.info('Moving robot to park position...')
      self.controlStream:setEob(1)
      self.controlStream:setJointVel(meca500.MAX_VELOCITY_RAD * 0.5)
      self.controlStream:moveJoints(torch.zeros(6))
      self.controlStream:waitForEob(8.0)
    end

    self.logger.info('clearMotion')
    self.controlStream:clearMotion()
    sys.sleep(0.2)
    self.controlStream:read()

    self.logger.info('deactivateRobot')
    self.controlStream:deactivateRobot()
    sys.sleep(0.5)
    self.controlStream:read()
  end

  self.logger.info('Closing control stream.')
  self.controlStream:close()
end
