# Robotino 2 node

ROS2 node for Festo Robotino 2.

## Implementation

This package doesn't use OpenRobotinoAPI because it is only for x86 application in Windows.

Robotino communication protocol is defined with little "reverse engineering".
Communication between OpenRobotnioAPI and Robotino was analyzed using Wireshark.
Structure of TCP payload of input and output packages was defined with matching TCP payload content and public header of OpenRobotinoAPI. Searched information is in SensorState.h and SetState.h (`/include/rec/iocontrol/remotestate`) in `toQDSAProtocol` and `fromQDSAProtocol` methods.

TCP payload of start package (it begins communication with Robotino 2) was hardcoded in following bytes: `0x02, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00`.

Simple TCP client was implemented using Boost library.

## Node description

Node parameters:

- `ip` &ndash; IP-address of Robotino2. Default: 172.26.1.0
- `port` &ndash; port of Robotino2. Default: 80
- `sample_period` &ndash; sample period of node process (ms). Default: 20 ms

Subscribed topics:

- `/robotino2/cmd_vel` &ndash; set robot speeds
- `/robotino2/cmd_mot_vel` &ndash; set robot motor velocities

Published topics:

- `/robotino2/bumper` &ndash; bumper state (false/true)
- `/robotino2/motor_state` &ndash; state of motors: name, position, velocity, current
- `/robotino2/joint_state` &ndash; state of joints (wheels): name, position and velocity
- `/robotino2/ir` &ndash; state of distance sensors: type, field of view, minimum and maximum ranges, current range

Services:

- `/robotino2/reset_pos` &ndash; reset motor positions

## Dependencies

- ROS2 Foxy or later
- Boost v1.71.0 or later
- [Robotino interfaces](https://github.com/BrOleg5/robotino-ros2-interfaces)
