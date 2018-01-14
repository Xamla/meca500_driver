local torch = require 'torch'
local meca500 = require 'meca500_env'


local RealtimeState = torch.class('RealtimeState')


function RealtimeState:__init()
  self.q_actual = torch.DoubleTensor(6)                   -- Actual joint positions
  self.robotStatus = torch.IntTensor(7)                   -- numeric robot status vector

  -- decoded fields of robot status vector
  self.motorActive = false          -- motor operation state 
  self.homingPerformed = false      -- homing state (`false` for homing not performed, `true` for homing performed)
  self.jointFeed = false            -- jointFeed status (`false` for joint feed disabled, `true` for joint feed enabled)
  self.error = false                -- error status (`false` for robot not in error mode, `true` for robot in error mode)
  self.poseFeed = false             -- pose feed status (`false` for pose feed disabled, `true` for pose feed enabled)
  self.eob = false                  -- end of block status (`false` for end of block disabled, `true` for end of block enabled)
  self.eom = false                  -- ond of movement status (`false` for end of movement disabled, `true` for end of movement
  self:invalidate()
end


function RealtimeState:isValid()
  return self.valid
end
  
  
function RealtimeState:isRobotReady()
  return self.valid and self.motorActive and self.jointFeed and not self.error
end


function RealtimeState:invalidate()
  self.valid = false
end

