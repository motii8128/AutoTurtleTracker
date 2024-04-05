# Auto Turtle Tracker

```mermaid
flowchart LR
    JoyTwistRust --> /target/cmd_vel
    /target/cmd_vel --> TurtleSim
    TurtleSim --> /turtle1/pose
    /turtle1/pose --> TurtleOdometer
    haya_imu_ros2 --> /imu
    /imu --> IMULocalizer
    IMULocalizer --> /odom/real
    TurtleOdometer --> /odom/target
    /odom/real --> VelocityController
    /odom/target --> VelocityController
    VelocityController --> /real/cmd_vel
    /real/cmd_vel --> ZikoichiController
    ZikoichiController --> serial
```
