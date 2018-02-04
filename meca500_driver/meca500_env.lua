local torch = require 'torch'
local meca500 = {}

meca500.ResponseCode = {
  -- errors
  CommandBufferFull = 1000,   -- [1000][Command buffer is full, please try again.
  InvalidCommand = 10001,     -- [1001][Empty command or command unrecognized. - Command: "..."]
  SyntaxError = 10002,        -- [1002][Syntax error, symbol missing. - Command: "..."]
  InvalidArgument = 10003,    -- [1003][Argument error. - Command: "..."]
  MecaNonActive = 1005,       -- [1005][Meca500 non active. - Command: "..."]
  HomingNotDone = 1006,       -- [1006][Homing was not done. - Command: "..."]
  JointOverLimit = 1007,      -- [1007][Joint over limit. - Command: "..."]
  VelOverLimit = 1008,        -- [1008][Velocity over limit. - Command: "..."]
  AccOverLimit = 1009,        -- [1009][Acceleration over limit. - Command: "..."]
  RobotInError = 1011,        -- [1011][The robot is in error. - Command: "..."]
  Singularity = 1012,         -- [1012][Singularity detected. - Command: "..."]
  ActivationFailed = 1013,    -- [1013][Activation failed. - Command: "..."]
  HomingFailed = 1014,        -- [1014][Homing failed. - Command: "..."]
  MasteringFailed = 1015,     -- [1015][Mastering failed. - Command: "..."]
  PosOutOfReach = 1016,       -- [1016][Position out of reach. - Command: "..."]
  CommFailure = 1017,         -- [1017][Communication failed. - Command: "..."]
  MissingNull = 1018,         -- [1018][’\0’ missing. - Command: "..."]
  RobotNotLeveled = 1019,     -- [1019][The robot isn’t leveled. - Command: "..."]
  BrakesNotReleased = 1020,   -- [1020][Brakes cannot be released. - Command: "..."]
  DeactivationFailed = 1021,  -- [1021][Deactivation failed. - Command: "..."]
  WasNotSaving = 1022,        -- [1022][Robot was not saving the program. Command: "..."]
  IgnoringCommand = 1023,     -- [1023][Ignoring command for offline mode. Command: "..."]
  MasteringNeeded = 1024,     -- [1024][Mastering needed. - Command: "..."]
  ErrorResetFailed = 1025,    -- [1025][Impossible to reset the error. Please, power-cycle the robot. - Command: "..."]
  DeactivationNeeded = 1026,  -- [1026][Deactivation needed to execute the command. - Command: "..."]
  SimToggleError = 1027,      -- [1027][Simulation mode can only be enabled/disabled while the robot is deactivated.]
  NetworkEror = 1028,         -- [1028][Network error.]
  ProgramFull = 1029,         -- [1029][Offline program full.]

  -- command responses
  HomingDone = 2002,          -- [2002][Homing done.]
  HomingAlreadyDone = 2003,   -- [2003][Homing already done.]
  ErrorReset = 2005,          -- [2005][The error was reset.]
  NoErrorToReset = 2006,      -- [2006][There was no error to reset.]
  RobotStatus = 2007,         -- [2007][a, h, j, e, p, eob, eom]
  Wrf = 2013,                 -- [2013][x, y, z, α, β, γ]
  Trf = 2014,                 -- [2014][x, y, z, α, β, γ]
  Pose = 2027,                -- [2027][x, y, z, α, β, γ]
  AutoConfOn = 2034,          -- [2034][The robot configuration will be automatically chosen.]
  AutoConfOff = 2035,         -- [2035][The robot configuration won’t be automatically chosen.]
  CartVelModified = 2020,     -- [2020][TCP velocity modified.]
  JointValues = 2026,         -- [2026][j1, j2, j3, j4, j5, j6 ]
  RobotConfSet = 2028,        -- [2028][The desired robot configuration is set to c 1 /c 3 /c 5 .]
  JointVelModified = 2022,    -- [2022][Joint velocities modified.]
  CorneringEnabled = 2032,    -- [2032][The cornering is enabled.]
  CorneringDisabled = 2033,   -- [2033][The cornering is disabled.]
  MotionPaused = 2042,        -- [2042][Motion paused.]
  MotionResumed = 2043,       -- [2043][Motion resumed.]
  SimEnabled = 2045,          -- [2045][The simulation mode is enabled.]
  SimDisabled = 2046,         -- [2046][The simulation mode is disabled.]
  EobEnabled = 2054,          -- [2054][End of block is enabled.]
  EobDisabled = 2055,         -- [2055][End of block is disabled.]
  WrfRef = 2067,              -- [2067][MoveLinDelta reference is WRF.]
  TrfRef = 2068,              -- [2068][MoveLinDelta reference is TRF.]
  GripperVelModified = 2078,  -- [2078][Gripper velocity modified.]
  GripperStatus = 2079,       -- [2079][g, h, p, l, e, o]

  -- status messages
  Connected = 3000,           -- [3000][Connected to Meca500 x_x_x.x.x.]
  AlreadyInUse = 3001,        -- [3001][Another user is already connected, closing connection.]
  TooManyArguments = 3003,    -- [3003][Command has reach the maximum length, please try again.]
  EndOfMovement = 3004,       -- [3004][End of movement.]
  MotionError = 3005,         -- [3005][Error of motion.]
  JointFeed = 3007,           -- [3007][j1, j2, j3, j4, j5, j6 ]
  InitFailed = 3009,          -- [3009][Robot init failed due to internal error. Restart the robot or contact Mecademic.]
  PoseFeed = 3010,            -- [3010][x, y, z, α, β, γ]
  CartJogSingularity = 3011,  -- [3011][Cartesian jogging stopped because of a singularity.]
  EndOfBlock = 3012,          -- [3012][End of block.]
  EndOfOfflineProgram = 3013, -- [3013][End of offline program.]
  CartJogWorkspace = 3019,    -- [3019][Cartesian jogging will be stopped because of workspace limits.]
}


local function deg2radLimits(joint_limits)
  local rad_limits = {}
  for i,x in ipairs(joint_limits) do
    local lo, hi = unpack(x)
    rad_limits[#rad_limits + 1] = { math.rad(lo), math.rad(hi) }
  end
  return rad_limits
end


meca500.JOINT_NAMES = {
  "joint1",
  "joint2",
  "joint3",
  "joint4",
  "joint5",
  "joint6"
}


meca500.JOINT_POSITION_LIMITS = torch.DoubleTensor(
  deg2radLimits({
    { -175, 175 },      -- joint1 (min, max)
    { -70, 90 },        -- joint2 (min, max)
    { -135, 70 },       -- joint3 (min, max)
    { -170, 170 },      -- joint4 (min, max)
    { -115, 115 },      -- joint5 (min, max)
    { -175, 175 }       -- joint6 (min, max)
  })
)


meca500.JOINT_VELOCITY_LIMITS = torch.DoubleTensor(
    deg2radLimits({
      { -135, 135 },      -- joint1 (min, max)
      { -135, 135 },      -- joint2 (min, max)
      { -135, 135 },      -- joint3 (min, max)
      { -270, 270 },      -- joint4 (min, max)
      { -270, 270 },      -- joint5 (min, max)
      { -270, 270 }       -- joint6 (min, max)
  })
)


meca500.MIN_VOLOCITY_DEG = 1
meca500.MIN_VOLOCITY_RAD = math.rad(meca500.MIN_VOLOCITY_DEG)


meca500.MAX_VELOCITY_DEG = 135
meca500.MAX_VELOCITY_RAD = math.rad(meca500.MAX_VELOCITY_DEG)



local function isFiniteTensor(x)
  -- check for NaN, +/-inf elements
  return x:eq(x):all() and not x:eq(1/0):any() and not x:eq(-1/0):any()
end


local function isFiniteNumber(x)
  -- check for NaN, +/-inf values
  return x == x and x ~= 1/0 and x ~= -1/0
end


local function isFinite(x)
  if torch.isTensor(x) then
    return isFiniteTensor(x)
  elseif type(x) == 'number' then
    return isFiniteNumber(x)
  end
  return false
end


meca500.isFiniteTensor = isFiniteTensor
meca500.isFiniteNumber = isFiniteNumber
meca500.isFinite = isFinite


meca500.DEFAULT_LOGGER = {
  debug = function(...)
    print('[DEBUG] ' .. string.format(...))
  end,
  info = function(...)
    print('[INFO] ' .. string.format(...))
  end,
  warn = function(...)
    print('[WARNING] ' .. string.format(...))
  end,
  error = function(...)
    print('[ERROR] ' .. string.format(...))
  end
}


return meca500
