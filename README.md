# EmbeddedRobotCar
A class final project for Embedded Programming Special Topics Software Engineering tech elective at FGCU, taught by Professor Paul Allen. The task is to write the code to control the following basic 2-wheel drive Arduino robot with an ultrasonic sensor to traverse a basic track with 2 90-degree turns.
<!--
Here is our robot, including the modifications we made to it:
![Robot](picture url)-->

## The Course
Below is an image of what the Rover robots’ course may look like.  The walls are approx. 28” inches apart, and 6” inches high.  Each segment is 14” inches long, so the course can be adjusted to have different lengths, i.e. longer straightaways with a short turn left then right.

![Rover Course](https://github.com/jordankooyman/EmbeddedRobotCar/blob/main/Pictures/Course.png)

The robot will start inside the first segment facing into the course, so there will be walls on either side of the robot when it first starts moving.  The finish will be after the robot exits the course, either at the far end, or the start end if it gets turned around.

## Development Methodology
We decided to use a finite state machine approach to developing the code for this project, as was recommended to us in class. We poll our Ultrasonic sensors each loop through our main code block, then go to one of the state functions to do an action and potentially change states.

Since this project is so heavily dependent on hardware as well as software, mental logic and debugging only works so well.
Therefore, we tested our robot many times on a demo course we built using a corner of a room and a table on its side.
This allowed us to rapidly test and prototype ideas and code, and spend less time trying to think and understand exactly what our code will actually be doing (ramming our heads into the code/wall) while spending more time driving the front (and sometimes even the head) of the robot into the wall and attempting to understand exactly why it did what it did. We found this rapid-prototyping approach to be most effective, particularly since we were only given a month to work on this as a class group assignment, which did not seem like quite enough time in hindsight.

Since this robot is heavily dependent on sensor readings from an Ultrasonic sensor, we determined the robot will be spending a significant amount of time in a measurement state which disables interrupts.

Therefore, we decided to avoid using any interrupts in our code to avoid issues.

Since we decided not to use interrupts, there was very little else we could do with the hardware timers, so we left them alone. This has the added benefit of also avoiding potentially interfering with the PWM signals.

To turn, we ended up using a turn-on-the-spot approach by running the wheels in opposite directions.

For our measurements, we utilized a total of 3 independent Ultrasonic sensors to track our position in the course. We poll the left and right sensors every cycle and use that data to determine which state we should be in. We poll the front sensor less often, it is currently configured to be every second cycle, since we utilize that data less and not polling it as frequently gives us a slight increase in cycle speed half the time. We end up polling the sensors nearly as fast as they allow for, and we utilize a rather short timeout distance of only 40 inches to keep everything moving fast.

We chose to have 3 Ultrasonic sensors so that we could reliably perform corrective measures to keep the robot from driving into walls and instead stay generally centered within the track.

## Hardware
The project is based on this (https://a.co/d/3Qj6L7B) kit, with some modifications as described in a later section. Included below is a brief summary of our final electronic components:
- 1x Arduino Uno
- 1x Sensor Shield
- 1x L298N Motor Driver
- 2x Geared Hobby Motor and Wheel
- 1x Dual 18650 Battery Holder
- 1x RGB LED
- 3x HC-SR04 Ultrasonic Sensor


## States Used
Below is a diagram showing all the states the robot can be in as well as the logical conditions we use to move between states:
![State Transition Diagram](https://github.com/jordankooyman/EmbeddedRobotCar/blob/main/Pictures/Final%20State%20Diagram.jpg)

## Changes Made
We decided to make a few modifications to the original robot we were provided.
We tried to stick to the original spirit of the competition this project is for, but we wanted to be the fastest to run the course, so we:
1. Added a left Ultrasonic sensor above the left wheel using a 3D Printed Mount
2. Added a right Ultrasonic sensor above the right wheel using a 3D Printed Mount
3. Removed the servo on the front holding the original front Ultrasonic sensor
4. Rigidly mounted the front Ultrasonic sensor using a 3D Printed Mount
5. Added an RGB Status LED
6. Screwed the wheels on using M2 screws and heat-set threaded inserts (recommend using M2.3x20mm self-tapping screws)

<!--## Demo
Below is a video of one of our final test runs, demonstrating how we tested our robot throughout development.
[![Video Thumbnail](https://example.com/thumbnail.jpg)](https://www.youtube.com/watch?v=VIDEO_ID)

Below is the video of our robot running the actual course during the final competition.
[![Video Thumbnail](https://example.com/thumbnail.jpg)](https://www.youtube.com/watch?v=VIDEO_ID)-->

## Challenges
We ran into numerous issues with this project, the biggest challenge being driving in a straight line down the track. We solved this by finding a decent equilibrium point for both motors to run at the same actual speed by using different software speeds. For our specific robot, the left motor had to be run at 20 less than the right motor, but we also could not exceed a speed of about 240 (on a 0-255 scale) before the difference in motor speeds was decreased dramatically, otherwise, the difference followed a nearly linear scale. We have ours calibrated around the 220 speed value.

Another challenge we faced was that the motors could not self-start or even just stalled when driven at less than 100 speed. For reliability, we never drive them below 120 speed. However, we needed to slow this down further for our turning sequence, so we implemented a software PWM that pulses the PWM signal to the motor, leveraging the fact that the motors will continue to coast a little bit after they stop being driven.

A challenge we had not anticipated was getting our robot angled between the walls and the mathematical fact that would increase our sensor distance readings. This caused our first iterations of our course correction system many problems and forced us to increase our initial sensor timeout distance from 20 inches to eventually 40 inches.

The single largest challenge in development was our course correction system. Between fighting the inherent right-turning drift from our left motor running faster and bad sensor readings from a damaged Ultrasonic sensor, as well as struggling with increasing measurements when angled, we easily spent close to 75% of our total development time on this function alone. With all these outside or unconsidered variables, what we expected to be a fairly simple system grew out of control very rapidly and had to be redesigned from the ground up twice. The final system we have is not perfect, but it is rather successful in avoiding hitting walls and getting the robot going down the track parallel to the walls while remaining relatively centered in the track overall.

A surprising challenge we faced was whether an Ultrasonic sensor timeout reading should be 0 or the max distance value. Initially it was 0, but it became more convenient to make it the max distance. This also forced us to realize that our initial Ultrasonic sensor driver code was not quite written correctly.

Lastly, the turn-on-the-spot approach was not what we had initially planned. We started with the goal of maximum speed, so we started with an arcing turn as fast as we could move, but after we had so many issues getting the robot to drive how we wanted it to, we realized that the arcing turn would not reliably function from an unknown starting position and angle when entering a turn. We then switched to a one-wheel turn (keeping the other wheel locked), but also ran into difficulties there since we had little idea where the robot might land when entering the corner. So we finally landed on our turn-on-the-spot method and even had to slow that down, so that we could reliably sense when we had turned far enough to finally get around the inconsistent starting location challenge.
