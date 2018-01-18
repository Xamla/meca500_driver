local socket = require 'socket'
local sys = require 'sys'
local meca500 = require 'meca500_env'
local ResponseCode = meca500.ResponseCode

local ControlStream = torch.class('ControlStream')


local DEFAULT_HOSTNAME = '192.168.0.100'
local DEFAULT_CONTROL_PORT = 10000
local READ_TIMEOUT = 0.005
local RECEIVE_BUFFER_SIZE = 512
local MAX_VELOCITY_DEG = meca500.MAX_VELOCITY_DEG
local MIN_VOLOCITY_DEG = meca500.MIN_VOLOCITY_DEG
local JOINT_POSITION_LIMITS_MIN = meca500.JOINT_POSITION_LIMITS[{{},1}]:clone()
local JOINT_POSITION_LIMITS_MAX = meca500.JOINT_POSITION_LIMITS[{{},2}]:clone()


local ControlStreamState = {
  Disconnected = 1,
  Handshake = 2,
  Ready = 3,
  Error = 4
}
meca500.ControlStreamState = ControlStreamState


local function resetStash(self, remaining)
  self.block = nil
  self.offset = 0
  self.remaining = remaining
end


function ControlStream:__init(realtimeState, logger)
  self.realtimeState = realtimeState
  self.logger = logger or meca500.DEFAULT_LOGGER
  self.client = socket.tcp()
  self.client:setoption('tcp-nodelay', true)
  self.state = ControlStreamState.Disconnected
  resetStash(self, '')
end


function ControlStream:connect(hostname, port)
  local ok, err = self.client:connect(hostname or DEFAULT_HOSTNAME, port or DEFAULT_CONTROL_PORT)
  if not ok then
    self.logger.error('Connecting failed, error: ' .. err)
    sys.sleep(0.1)
    return false
  end
  self.client:settimeout(READ_TIMEOUT, 't')
  self.state = ControlStreamState.Handshake
  resetStash(self, '')
  return true
end


function ControlStream:getState()
  return self.state
end


function ControlStream:getSocket()
  return self.client
end


function ControlStream:close(abortive)
  if self.client ~= nil then
    if self.state == ControlStreamState.Ready then
      self.client:shutdown('send')
      if abortive then
        self.client:setoption('linger', { on = true, timeout = 0 })
      end
    end
    self.client:close()
    self.client = nil
  end
end


local function nextResponseString(self)
  while self.block ~= nil and self.offset <= #self.block do
    local end_pos = self.block:find('\0', self.offset)
    if end_pos == nil then
      resetStash(self, self.remaining .. self.block:sub(self.offset))
      return nil
    end

    local msg = self.block:sub(self.offset, end_pos - 1)
    self.offset = end_pos + 1
    if self.remaining ~= nil and #self.remaining > 0 then
      msg = self.remaining .. msg
      self.remaining = ''
    end
    return msg
    
  end
  return nil
end


local function parseResponse(self, s)
  -- response format: [4-digit code][message string OR return values.]
  local code, msg = s:match('^%[(%d%d%d%d)%]%[(.*)%]$')
  code = tonumber(code)
  if code == nil then
    self.logger.error('[ControlStream] Invalid response received: ' .. s)
    self.state = ControlStreamState.Error
  end
  return code, msg
end


local function readResponse(self)
  local s = nextResponseString(self)
  if s ~= nil then
    return parseResponse(self, s)
  end

  local r, e, p = self.client:receive(RECEIVE_BUFFER_SIZE)
  if e == 'timeout' then
    r = p
  end
  if not r then
    self.logger.error('[ControlStream] Receive failure: ' .. e)
    self.state = ControlStreamState.Error
    return nil
  end

  if #r > 0 then
    self.block = r
    self.offset = 1
    local s = nextResponseString(self)
    if s ~= nil then
      return parseResponse(self, s)
    end
  end

  return nil
end


function ControlStream:read()
  if self.state == ControlStreamState.Handshake then
    local code, msg = readResponse(self)
    if code ~= nil then
      if code == ResponseCode.Connected then  -- inital connect message of meca
        self.logger.info('Handshake from robot received.')
        self.realtimeState:parseResponse(code, msg)
        self:requestStatus()
      elseif code == ResponseCode.RobotStatus then -- robot status
        self.realtimeState:parseResponse(code, msg)
        self.state = ControlStreamState.Ready
        self.logger.info('Initial robot status received.')
        return true
      else
        self.state = ControlStreamState.Error
      end
    end
    return false
  end

  while true do
    local code, msg = readResponse(self)
    if code == nil then
      break
    end



    -- print(code, msg)
    self.realtimeState:parseResponse(code, msg)
  end

  return true
end


function ControlStream:send(msg)
  return self.client:send(msg)
end


function ControlStream:requestStatus()
  local req = 'GetStatusRobot\0'
  if self.realtimeState.status.activated then
    req = req .. 'GetGripperStatus\0'
  end
  return self:send(req)
end


function ControlStream:activateRobot()
  return self:send('ActivateRobot\0')
end


function ControlStream:deactivateRobot()
  return self:send('DeactivateRobot\0')
end

function ControlStream:resetError()
  return self:send('ResetError\0')
end


function ControlStream:activateJointsFeed()
  return self:send('ActivateJointsFeed\0')
end


function ControlStream:deactivateJointsFeed()
  return self:send('DeactivateJointsFeed\0')
end


function ControlStream:clearMotion()
  return self:send('ClearMotion\0')
end


function ControlStream:home()

  local handler = {
    parseResponse = function (code, msg)
      if code == 2002 then
        --[2002][Homing done.]
        --[2003][Homing already done.]
      end
    end
  }

  return self:send('Home\0')
end


function ControlStream:moveJoints(q)
  assert(
    q ~= nil and q:nElement() == 6 and q:size(1) == 6,
    'Invalid argument: Joint setpoint is nil or in invalid shape (6D tensor expected).'
  )
  local cmd = string.format("MoveJoints(%f,%f,%f,%f,%f,%f)\0",
    math.deg(q[1]), math.deg(q[2]), 
    math.deg(q[3]), math.deg(q[4]), 
    math.deg(q[5]), math.deg(q[6])
  )
  assert(
    q:ge(JOINT_POSITION_LIMITS_MIN):all() and q:le(JOINT_POSITION_LIMITS_MAX):all(),
    'Joint setpoint lies outside joint position limits: ' .. cmd
  )
  return self:send(cmd)
end


function ControlStream:setJointVel(v)
  assert(v ~= nil and type(v) == 'number', 'Invalid argument: Velocity value is nil or no number.')
  assert(meca500.isFiniteNumber(v), 'Velocity is not a finite numbers.')
  v = math.max(math.min(math.deg(v or 0.8), MAX_VELOCITY_DEG), MIN_VOLOCITY_DEG)
  local cmd = string.format('SetJointVel(%f)\0', v)
  return self:send(cmd)
end


function ControlStream:gripper(e)
  if type(e) == 'boolean' then
    e = e and 1 or 0
  end
  local cmd = string.format('Gripper(%d)\0', e)
  return self:send(cmd)
end


function ControlStream:setGripperVel(v)
  local cmd = string.format('SetGripperVel(%f)\0', v)
  return self:send(cmd)
end


function ControlStream:setGripperForce(f)
  local cmd = string.format('SetGripperForce(%f)\0', f)
  return self:send(cmd)
end
