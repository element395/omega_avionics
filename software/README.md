# omega_avionics
Delta Space Systems - 2020
Software written by Cole Purtzer

Thrust Vector Control Software thats open-sourced!

Tuning Guide - README
 *  To tune the servo offsets for your TVC Mount go to line 52. 
    To tune just raise the values until the servo switches directions then home in on the perfect value.
 *  To change the ratio between the Servo and the TVC mount go to line 64. 
    Use a seperate sketch and raise the degree the servo moves until it hits the end of the TVC Mount. 
    Then divide the servo degree by 5.
 *  For PID Value Changes go to line 112 - 114.
 *  To change what I/O pins the servos are connected to go to line 139 - 140.  



System State: 
 * 0 = Go/No Go before launch.
 * 1 = PID Controlled Ascent.
 * 2 = MECO - 2 seconds before parachute deployment.
 * 3 = In-flight abort.

 
 This software is used to actively stabilize a model rocket and not to guide it.
 
 In the software you can choose between the BMI088 and MPU6050 based on if you are using MK5 or MK4.
 
*Please do not publish these files without permission :)
