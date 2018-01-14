local torch = require 'torch'
local ros = require 'ros'
local meca500 = require 'meca500_env'


local RealtimeState = torch.class('RealtimeState')


local function parseCommaSeparatedNumbers(s)
  local x = string.split(s, ',')
  for k,v in pairs(x) do
    x[k] = tonumber(v)
  end
  return x
end


local function parseMecaVersion(self, code, message)
  local version_tag = message:match('^Connected to Meca500 (.*)$')
  assert(version_tag ~= nil, 'Meca Version tag invalid')
  self.versionTag = version_tag
end


local function parseRobotStatus(self, code, message)
  local status = parseCommaSeparatedNumbers(message)
  if #status < 7 then
    error('Unexpected robot status response received')
  end

  self.robotStatus = torch.IntTensor(status)

  local new_status = {
    activated = (status[1] == 1),
    homingPerformed = (status[2] == 1),
    jointFeed = (status[3] == 1),
    error = (status[4] == 1),
    poseFeed = (status[5] == 1),
    eob = (status[6] == 1),
    eom = (status[7] == 1)
  }
  self.status = new_status

end


local function parseGripperStatus(self, code, message)
  local status = parseCommaSeparatedNumbers(message)
  if #status < 6 then
    error('Unexpected gripper status response received')
  end
  self.gripperStatus = torch.IntTensor(status)
end


local function parseJointAngles(self, code, message)
  local now = ros.Time.now()
  local q = parseCommaSeparatedNumbers(message)
  if #q < 6 then
    error('Invalid joint angles received')
  end
  
  local q_last = self.q_actual
  self.q_actual = torch.DoubleTensor(q)
  if self.q_actual_time ~= nil then
    local dt = (now - self.q_actual_time):toSec()
    if dt > 0.001 then
      self.qd_actual = (self.q_actual - q_last) / dt
      --print('qd', self.qd_actual)
      self.valid = true
    end
  else
    self.qd_actual:zero()
  end
  self.q_actual_time = now
end


function RealtimeState:__init()
  self.robotStatus = torch.IntTensor(7):zero()            -- numeric robot status vector
  self.gripperStatus = torch.IntTensor(6):zero()          -- numeric gripper status vector
  self.q_actual = torch.DoubleTensor(6):zero()            -- Actual joint positions (in rad)
  self.qd_actual = torch.DoubleTensor(6):zero()

  -- decoded fields of robot status vector
  self.status = {
    activated = false,            -- motor operation state 
    homingPerformed = false,      -- homing state (`false` for homing not performed, `true` for homing performed)
    jointFeed = false,            -- jointFeed status (`false` for joint feed disabled, `true` for joint feed enabled)
    error = false,                -- error status (`false` for robot not in error mode, `true` for robot in error mode)
    poseFeed = false,             -- pose feed status (`false` for pose feed disabled, `true` for pose feed enabled)
    eob = false,                  -- end of block status (`false` for end of block disabled, `true` for end of block enabled)
    eom = false                   -- ond of movement status (`false` for end of movement disabled, `true` for end of movement
  }
  self:invalidate()

  self.handlerTable = {}
  self.handlerTable[3000] = parseMecaVersion
  self.handlerTable[2007] = parseRobotStatus
  self.handlerTable[2079] = parseGripperStatus
  self.handlerTable[2026] = parseJointAngles
  self.handlerTable[3007] = parseJointAngles
end


function RealtimeState:isValid()
  return self.valid
end
  
  
function RealtimeState:isRobotReady()
  local status = self.status
  return self.valid and status.activated and status.jointFeed and not status.error
end


function RealtimeState:invalidate()
  self.valid = false
end


function RealtimeState:parseResponse(code, message)
  local handler = self.handlerTable[code]
  if handler ~= nil then
    handler(self, code, message)
  end
end
