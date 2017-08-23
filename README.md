# RobControl #
---
A simple C library to control industrial robots.

![RobotControl V1.2](/robot.png)

## Main features ##
- Kinematic models: 3ax Cartesian, 4ax SCARA, 4ax Delta, 4ax Palletizer, 6ax articulated robot
- Manual jogging of axes in joint, base and tool coordinate system
- Automatic run of NC programs and streamed block lists with path-blending
- PTP and interpolated movements (lines, circles, cubic splines)
- Edge rounding between interpolated movements
- Time-optimal trajectory generator
- Compensations for tools and frames
- Conveyor tracking
- External real-time path corrections
- M-functions for PLC synchronization
- Control of user-defined kinematics
- Cartesian and angular feedrate definition
- Configuration of workspace forbidden areas
- Single-step mode
- Execution of nested subroutines
- Tangential axis