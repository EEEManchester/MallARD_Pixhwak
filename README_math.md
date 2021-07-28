# Thruster allocation in Ardusub custom firmware
In the [custom firmware](https://github.com/EEEManchester/ArduPilot_MALLARD/blob/733f57fa1fcc381113ecd4b01095a1f895e5a536/libraries/AP_Motors/AP_Motors6DOF.cpp#L131), the thruster allocation of two modes are defined in the matrix, which are shown as follow. 

## JOYSTICK_PWM_CONTROL mode
This mode is used to test whether the custom firmware has been successfully compiled and flashed on the Pixhawk.

Each movement of command from joystick can make the corresponding individual thruster spin. For instance, when you put input in stick vertical, motor 1 will move. When you put input in stick horizontal, only motor 2 wil spin. When you push the button controlling the yaw (the button depends on what you defined), only motor 3 will spin. And when you push the button which control the vehicle move up/sown, only motor 4 will spin.

**NOTE:**  In general, motor_1 means the motor connected to channel 1 on Pixhawk.

τ = [X, Y, Z, R]<sup>T</sup> is the input control signal from joystick. X means the  input in stick(left) vertical. Y means input in stick(left) horizontal. Z means input in stick(right) vertical. R means input in stick(right) horizontal.  
τ<sub>m</sub> = [M<sub>1</sub> M<sub>2</sub> M<sub>3</sub> M<sub>4</sub>]<sup>T</sup> is the movement of each motor.  
T is thruster allocation matrix, which can be written as:    
![Screenshot from 2021-07-27 16-50-04](https://user-images.githubusercontent.com/77399327/127185861-0f9bf090-6554-4931-ad20-b21db70b1a3a.png)  
T<sup>+</sup> is the inverse of T and T = T<sup>+</sup>. So the relationship between control demand and individual actuator demand is given by:   
τ<sub>m</sub> = T * τ, which is shown as:

![Screenshot from 2021-07-27 16-55-22](https://user-images.githubusercontent.com/77399327/127186898-4a65fea5-94b4-40c2-a222-45e37b8c3e55.png)

Write the T<sup>+</sup> in the factor table in firmware:

| Motor | Roll Factor | Pitch Factor | Yaw Factor | Throttle Factor | Forward Factor | Lateral Factor | Testing Order |
| ----- | ------ | ----- | ----- | ----- | ----- | ----- | -----|
AP_MOTORS_MOT_1|0|0|0|0|1.0f|0|1|  
AP_MOTORS_MOT_2|0|0|0|0|0|1.0f|2|  
AP_MOTORS_MOT_3|0|0|1.0f|0|0|0|3|
AP_MOTORS_MOT_4|0|0|0|1.0f|0|0|4|


## SUB_FRAME_CUSTOM mode
The thruster allocation is used to control MallARD_003. 
   

τ = [U, V, R]<sup>T</sup> is force/moment vector.  
τ<sub>u</sub> = [F<sub>1</sub> F<sub>2</sub> F<sub>3</sub> F<sub>4</sub>]<suP>T</sup> is the vector of force generated from the thrusters.  
T is thruster allocation matrix.   
![Screenshot from 2021-07-27 16-57-56](https://user-images.githubusercontent.com/77399327/127187122-f90e8c6e-7295-4446-9f60-0f3701d2bd6d.png)


τ = T * τ<sub>u</sub>
This can be written as:
![Screenshot from 2021-07-27 17-03-20](https://user-images.githubusercontent.com/77399327/127188027-a81f8ac1-6a06-4d81-971c-f96803b8a649.png)

Rearrange the equation:
τ<sub>u</sub> = T<sup>+</sup> τ   
T<sup>+</sup> is the is the Moore-Penrose inverse of T, which can be written as:

![Screenshot from 2021-07-28 00-51-47](https://user-images.githubusercontent.com/77399327/127250859-3069b373-718a-41f3-89cd-444a42bb7c3b.png)

For MallARD_003, ly₁=ly₂=ly₃=ly₄=0.15m, lx₁=lx₂=lx₃=lx₄=0.205m, θ₁=3/4π, θ₂=5/4π, θ₃=1/4π, θ₄=7/4π  

Write the T<sup>+</sup> in the factor table in firmware:


| Motor | Roll Factor | Pitch Factor | Yaw Factor | Throttle Factor | Forward Factor | Lateral Factor | Testing Order |
| ----- | ------ | ----- | ----- | ----- | ----- | ----- | -----|
AP_MOTORS_MOT_1|0|0| 0.99f|0| 0.35f|-0.35f|1|  
AP_MOTORS_MOT_2|0|0|-0.99f|0| 0.35f| 0.35f|2|  
AP_MOTORS_MOT_3|0|0|-0.99f|0|-0.35f|-0.35f|3|
AP_MOTORS_MOT_4|0|0| 0.99f|0|-0.35f| 0.35f|4|
## Refenence
1. [ArduPilot_MALLARD](https://github.com/EEEManchester/ArduPilot_MALLARD/blob/733f57fa1fcc381113ecd4b01095a1f895e5a536/libraries/AP_Motors/AP_Motors6DOF.cpp)

2. [Model Identification of a Small Omnidirectional Aquatic Surface
Vehicle: a Practical Implementation](https://ieeexplore.ieee.org/document/9341142)  

