bool gripper_enabled  # i.e., present, `false` for disabled, `true` for enabled
bool homing_done # `false` for homing not performed, `true` for homing performed
bool holding_part # `false` if the gripper does not hold a part, `true` otherwise
bool limit_reached # `false` if the fingers are not fully open or closed, `true` otherwise
bool general_error # `false` for absence of error, `true` for presence of error
bool overload # `false` if there is no over load, `true` if the gripper is in force over load