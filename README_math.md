# Thruster allocation in Ardusub custom firmware
In the [custom firmware](https://github.com/EEEManchester/ArduPilot_MALLARD/blob/733f57fa1fcc381113ecd4b01095a1f895e5a536/libraries/AP_Motors/AP_Motors6DOF.cpp#L131), the thruster allocation of two modes are defined in the matrix, which are shown as follow. 

## JOYSTICK_PWM_CONTROL mode
This mode is used to test whether the custom firmware has been successfully compiled and flashed on the Pixhawk.

Each movement of command from joystick can make the corresponding individual thruster spin. For instance, when you put input in stick vertical, motor 1 will move. When you put input in stick horizontal, only motor 2 wil spin. When you push the button controlling the yaw (the button depends on what you defined), only motor 3 will spin. And when you push the button which control the vehicle move up/sown, only motor 4 will spin.

**NOTE:**  In general, motor_1 means the motor connected to channel 1 on Pixhawk.

τ = [X, Y, Z, R]<sup>T</sup> is the input control signal from joystick. X means the  input in stick(left) vertical. Y means input in stick(left) horizontal. Z means input in stick(right) vertical. R means input in stick(right) horizontal.  
τ<sub>m</sub> = [M<sub>1</sub> M<sub>2</sub> M<sub>3</sub> M<sub>4</sub>]<sup>T</sup> is the movement of each motor.  
T is thruster allocation matrix, which can be written as:
  $$
T = \begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
$$
T<sup>+</sup> is the inverse of T and T = T<sup>+</sup>. So the relationship between control demand and individual actuator demand is given by:   
τ<sub>m</sub> = T * τ, which is shown as:

$$
\begin{bmatrix}
M₁\\M₂\\M₃\\M₄
\end{bmatrix}
 = 
\begin{bmatrix}
1&0&0&0\\
0&1&0&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}

\begin{bmatrix}
X\\Y\\Z\\R
\end{bmatrix}
$$

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
$$
T = \begin{bmatrix}
cosθ₁&cosθ₂&cosθ₃&cosθ₄\\
sinθ₁&sinθ₂&sinθ₃&sinθ₄\\
ly₁cosθ₁ - lx₁sinθ₁&-ly₂cosθ₂ - lx₂sinθ₂&ly₃cosθ₃ + lx₃sinθ₃&-ly₄cosθ₄ + lx₄sinθ₄\\
\end{bmatrix}
$$

τ = T * τ<sub>u</sub>
This can be written as:
$$
\begin{bmatrix}
U\\V\\R
\end{bmatrix} = 

\begin{bmatrix}
cosθ₁&cosθ₂&cosθ₃&cosθ₄\\
sinθ₁&sinθ₂&sinθ₃&sinθ₄\\
ly₁cosθ₁ - lx₁sinθ₁&-ly₂cosθ₂ - lx₂sinθ₂&ly₃cosθ₃ + lx₃sinθ₃&-ly₄cosθ₄ + lx₄sinθ₄\\
\end{bmatrix}

\begin{bmatrix}
F₁\\F₂\\F₃\\F₄
\end{bmatrix}
$$

Rearrange the equation:
τ<sub>u</sub> = T<sup>+</sup> τ   
T<sup>+</sup> is the is the Moore-Penrose inverse of T, which can be written as:

$$
\begin{bmatrix}
F₁\\F₂\\F₃\\F₄
\end{bmatrix}
 = 
\begin{bmatrix}
0.242&-0.466&-1.494\\
-0.289&-0.289&0.863\\
0.418&0.438&0.863\\
-0.466&0.241&-1.493
\end{bmatrix}

\begin{bmatrix}
U\\V\\R
\end{bmatrix}
$$

For MallARD_003, ly₁=ly₂=ly₃=ly₄=0.15m, lx₁=lx₂=lx₃=lx₄=0.205m, $θ₁=\frac{3}{4}π$, $θ₂=\frac{5}{4}π$, $θ₃=\frac{1}{4}π$, $θ₄=\frac{7}{4}π$  




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
  
