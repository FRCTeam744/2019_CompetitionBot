# 2019_CompetitionBot
The official code for team 744's 2019 robot.

The documentation for the code can be found by opening /2019_CompetitionBot/SharkAttack/html/index.html

An example of the robot running with the latest code can be seen in [Hopper Qualification Match 106](https://www.youtube.com/watch?v=RF7pf5sehvA), where the robot is the one in the red bumpers with the long, black arm. And primarily in the lower left corner of the field. 

The full results for this competition season can be found at our [The Blue Alliance page](https://www.thebluealliance.com/team/744/2019)

#### Features include
##### Autonomous:
* 2 hatch panel Cargo Bay Auto Mode [Hopper SF1-2](https://www.youtube.com/watch?time_continue=1&v=2D1KJ2R4SQM) and [Hopper SF1-3](https://www.youtube.com/watch?v=QXwQmYuDCyw) (blue bumpers)
* 1.5 hatch panel Rocket Auto Mode [Hopper Q106](https://www.youtube.com/watch?v=RF7pf5sehvA) (red bumpers)

##### Drivetrain:
* Tank style drive, with PIDF speed control on both drive sides, for use in vision tracking and autonomous driving.
* NAvX gyro correction of driveing when in autonomous path following mode.
* Limelight camera used for feedback to track drivetrain to reflective targets for automatic hatch panel and cargo pick-up and placement
* PID control on distance and angle for vision tracking

##### Arm:
* 1 button auto pick-up and placement of hatch panels
* 1 button auto scoring of Cargo in cargo ship
* 330 degree swing to enable pick-up and placement of hatch panels and cargo on the front and back of the robot
* state-machine ensuring that the arm passes through the robot as quickly and safely as possible
* PID controller on arm and wrist

##### Fourbar:
* 6 second lift from floor to level 3 [South Florida Q47](https://www.youtube.com/watch?time_continue=153&v=otYqbHrW5Ck) (red bumpers)
* automatic stop at max extensions
* automatic homing feature

##### Driver Station/Operator Interface
* Smart Dashboard used for testing of features
* ShuffleBoard used for in competition dashboard, relaying important robot and field info 
* 2-joystick, tank-style inputs for driving
* x-box controller for arm-presets and fourbar extention

##### LEDs:
* Used to relay robot state information to drivers (such as front and back or if auto-driving), without having to look down at the driver station

![](sharky.gif)
