# Line Following Robot :blue_car: :checkered_flag:
This is an arduino sketch for a [line following robot](https://www.electronicshub.org/arduino-line-follower-robot/) based on infrared LEDs and an H-bridge. 

## How it works
The code uses a PID controller to control the wheels of the vehicle so it steers into a dark line. To improve the likelihood of the vehicle sliding off the line, the code decreases the speed of the robot when it enters a sharp turn. 

The speed of the motors is controlled via [pulse frequency modulation](https://en.wikipedia.org/wiki/Pulse-frequency_modulation). This allows the motors to be moved under the influence of cogging torque and stiction as the pulses act like a jackhammer. The end result is motion similar to a stepper motor as the motor moves in discrete jumps of variable length. This greatly improves low speed control of the motors at the cost of jerkyness. At higher throttle settings, the motors switch over to [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) to smoothen the motion.

## Schematic 
The code requires a circuit like the one shown in this schematic:  

<p align="center">
<img src="/main/images/diagram/schematic.png" width="80%"></img>
</p>

## Example
Below are images of the robot this code was written for:  

<p align="center">
<img src="/images/example/bottom_view_res.jpg" width="28%"></img>
<img src="/images/example/diag_view_res.jpg" width="28%"></img>
<img src="/images/example/top_view_res.jpg" width="28%"></img>  
</p>

Here's a [video](https://www.youtube.com/watch?v=NBQjQLE4u1M) of the same robot in operation: 
