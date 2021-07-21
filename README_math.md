# Thruster allocation in Ardusub custom firmware
In the [custom firmware](https://github.com/EEEManchester/ArduPilot_MALLARD/blob/733f57fa1fcc381113ecd4b01095a1f895e5a536/libraries/AP_Motors/AP_Motors6DOF.cpp#L131), the thruster allocation is defined in the matrix, which are shown as follow. 

## JOYSTICK_PWM_CONTROL mode
This is the matrix for JOYSTICK_PWM_CONTROL mode. It is used to test whether the custom firmware has been flashed into the Pixhwak 4 successfully. Each movment of command from joystick can make the corressponding individual thruster spin. For instance, when you put input in stick vertical, motor 1 will move. When you put input in stick horizonal, only motor 2 wil spin. When you push the button controlling the yaw (the button depends on what you defined), onle motor 3 will spin. And when you push the button which control the vehicle move up/sown, only motor 4 will spin.

**NOTE:**  In general, motor 1 means the motor connected to channel 1 on Pixhawk.

| Motor | Roll Factor | Pitch Factor | Yaw Factor | Throttle Factor | Forward Factor | Lateral Factor | Testing Order |
| ----- | ------ | ----- | ----- | ----- | ----- | ----- | -----|
AP_MOTORS_MOT_1|0|0|0|0|1.0f|0|1|  
AP_MOTORS_MOT_2|0|0|0|0|0|1.0f|2|  
AP_MOTORS_MOT_3|0|0|1.0f|0|0|0|3|
AP_MOTORS_MOT_4|0|0|0|1.0f|0|0|4|


## SUB_FRAME_CUSTOM mode
This is the matrix for SUB_FRAME_CUSTOM mode. It has been modified for MallARD (new version [Figure]) thruster allocation. When you put input in vertical of analog stick (move along x-axis), motor 1, motor 2 will spin in clockwise and motor 3 and 4 will spin in anti-clockwise that produce a force (F_u) to push the vehicle move towards positive X. Vice versa.   
   
![MallARD thruster allocation](https://user-images.githubusercontent.com/77399327/126422035-619c7d1b-188c-498d-b6d6-6fc6c49fff33.png)

| Motor | Roll Factor | Pitch Factor | Yaw Factor | Throttle Factor | Forward Factor | Lateral Factor | Testing Order |
| ----- | ------ | ----- | ----- | ----- | ----- | ----- | -----|
AP_MOTORS_MOT_1|0|0|-1.0f|0|1.0f|1.0f|1|  
AP_MOTORS_MOT_2|0|0|1.0f|0|1.0f|-1.0f|2|  
AP_MOTORS_MOT_3|0|0|1.0f|0|-1.0f|1.0f|3|
AP_MOTORS_MOT_4|0|0|-1.0f|0|-1.0f|-1.0f|4|


## Refenence
1. [ArduPilot_MALLARD](https://github.com/EEEManchester/ArduPilot_MALLARD/blob/733f57fa1fcc381113ecd4b01095a1f895e5a536/libraries/AP_Motors/AP_Motors6DOF.cpp)

2. [Model Identification of a Small Omnidirectional Aquatic Surface
Vehicle: a Practical Implementation](https://ieeexplore.ieee.org/document/9341142)