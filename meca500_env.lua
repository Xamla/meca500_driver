local meca500 = {}


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


meca500.JOINT_POSITION_LIMITS = deg2radLimits({
    { -175, 175 },      -- joint1 (min, max)
    { -70, 90 },        -- joint2 (min, max)
    { -135, 70 },       -- joint3 (min, max)
    { -170, 170 },      -- joint4 (min, max)
    { -115, 115 },      -- joint5 (min, max)
    { -175, 175 }       -- joint6 (min, max)
})


meca500.JOINT_VELOCITY_LIMITS = deg2radLimits({
    { -135, 135 },      -- joint1 (min, max)
    { -135, 135 },      -- joint2 (min, max)
    { -135, 135 },      -- joint3 (min, max)
    { -270, 270 },      -- joint4 (min, max)
    { -270, 270 },      -- joint5 (min, max)
    { -270, 270 }       -- joint6 (min, max)
})


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