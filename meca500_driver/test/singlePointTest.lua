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


local ros = require 'ros'

local JOINT_NAMES = {
  "joint1",
  "joint2",
  "joint3",
  "joint4",
  "joint5",
  "joint6"
}


local jointCommandPublisher
local pointSpec = ros.get_msgspec('trajectory_msgs/JointTrajectoryPoint')


local function publishGoal(q)
  -- see http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html
  local msg = jointCommandPublisher:createMessage()
  msg.joint_names = JOINT_NAMES

  local pt = ros.Message(pointSpec)
  pt.positions = q
  pt.velocities = torch.zeros(q:size())
  pt.time_from_start = ros.Duration(0.2)

  msg.points[#msg.points + 1] = pt
  print(msg)

  jointCommandPublisher:publish(msg)
end


local function main()
  ros.init('single_point_test')

  local nh = ros.NodeHandle()

  jointCommandPublisher = nh:advertise('/meca500_driver/joint_command', 'trajectory_msgs/JointTrajectory', 1)
  jointCommandPublisher:waitForSubscriber()

  local q = torch.zeros(6)

  local delay = ros.Duration(0.25)
  while ros.ok() do
    local now = ros.Time.now()
    q[1] = (math.random()-0.5) * 2 * 0.2
    q[2] = math.random() * 0.2
    q[2] = math.random() * 0.2
    q[5] = math.random() * 0.2
    q[6] = math.random() * 0.2
    publishGoal(q)
    ros.spinOnce()
    delay:sleep()
  end

  ros.shutdown()
end

main()