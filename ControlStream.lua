local socket = require 'socket'
local meca500 = require 'meca500_env'

local ControlStream = torch.class('ControlStream')

local DEFAULT_HOSTNAME = '192.168.0.100'
local DEFAULT_CONTROL_PORT = 10000
local READ_TIMEOUT = 0.01
local RECEIVE_BUFFER_SIZE = 512


local ControlStreamState = {
  Disconnected = 1,
  Connected = 2,
  Error = 3
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
    return false
  end
  self.client:settimeout(READ_TIMEOUT, 't')
  self.state = ControlStreamState.Connected
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
    if self.state == ControlStreamState.Connected then
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
  if code == nil then
    self.logger.error('[ControlStream] Invalid response received: ' .. s)
    self.state = ControlStreamState.Error
  end
  return code, msg
end


function ControlStream:read()
  if self.state ~= ControlStreamState.Connected then
    return nil
  end

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


function ControlStream:send(msg)
  return self.client:send(msg)
end
