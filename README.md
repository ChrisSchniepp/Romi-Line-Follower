# Romi-Line-Follower
Full Documentation on creating a line follower using a Romi Robot
See Landing page for more information on Python code documentation: https://ChrisSchniepp.github.io/Romi-Line-Follower/html/index.html

## Intro
This project was made for ME 405 at Cal Poly. the project uses a Romi Robot, made by Polulu, to maneuver over a set course. We use a modified Nucleo board as our microcontroller which runs code in micropython. This course includes dashed lines, sharp turns, and an obstacle. The Romi Line Follower outlined below is designed for line courses with black lines on white paper.

## Control Structure:
The main control structure used to make out Romi Line Follower was a cascading controller, where longitudinal velocity and yaw rate are used to set motor speeds. These motor speeds then have their own control loops. Each control loop was done with PID controllers. Gains ranged from 0 to 3. The image below describes the structure well. 
![image](https://github.com/user-attachments/assets/3e9f5e1a-7e69-452e-843c-5f5a68c3afdf)

However, to guide the Romi a line sensor is needed. Data from our line sensor was used to set the yaw rate, and longitudinal velocity was set manually. Our Line sensor, which can be found below in the Sensors section, had 8 sensors that we used to locate where the Romi was above the line. These values were used to find a centroid, where the middle of the line was 3.5. To get this value, we first normalized the values from 0 to 1 and then dictated what normalized value indicated line and no line. To do this we set a threshold, which was tuned based on our line sensors readings. This was found to be .9. Our threshold function then returns an array of 0s and 1s indicating where the line is. These points were then used to get our centroid. We put the line sensor in a PID control loop, whose output was the yaw setpoint.  

## Code:

Our code consists of two main loops, outer loop and inner loop. Outer loop deal with the line, yaw, and longitudal controllers while inner loop corrects the motor speeds. These functions also called upon many helper functions that can be seen in the main.py. The inner loop also has its own miniature state machines for specific movement sequences. These specific sequences were navigating around a box and moving from finish back to start. Finite state machines for inner and outer loop functions can be found below.

Task Diagram
![image](https://github.com/user-attachments/assets/fbef46a5-7817-400f-a241-952cbdea11e3)

Inner Loop

![image](https://github.com/user-attachments/assets/81995771-aa29-4fe3-b032-877bd7be59ee)

Outer Loop

![image](https://github.com/user-attachments/assets/022cfe8e-22f1-47b4-9f6c-7e86920f31c5)

Obstacle Manuever:
![image](https://github.com/user-attachments/assets/11107e4e-e9bb-4eb9-be4e-8a219b160080)




## Video Results
https://youtu.be/_j6A3AOVdVI?si=VVIaFSpuR5w5_0qR

## Sensors and Parts Bought outside of Romi Kit

Romi Robot:
https://www.pololu.com/product/4022

IMU:
https://www.adafruit.com/product/2472

Line Sensor:
https://www.pololu.com/product/3672

Bump Sensors:
Left and right sensors were bought although for our purpose's only one side was needed
https://www.pololu.com/product/3673

### Authors 
Chris Schniepp and 
Miles Mikkelsen
