___

### Aerial Robotics

MATLAB Library to simulate the motion of a quadcopter drone through
user customizable trajectories, control laws and dynamical models.

Author(s): UPenn School of Engineering and Applied Science
Contirbutor: Omar Kadhim - Spring 2018

IMPORTANTLY: this is an educational library and the vast majority of the
code, from the equations of motion, to the differential equation solver and
the visualisation libraries are provided by the UPenn course here: 
https://www.coursera.org/learn/robotics-flight/ 

Students are asked to write their own controllers - I have written 
two, a linearised and non-linear controller, and make them available for
anyone to use for any purpose. Students are also asked to write their
own minimum-jerk trajectory generators, which I have also done. My files
start with OK_ eg. OK_3Dcontroller.m

IP Disclaimer: I am not sure whether I'm allowed to re-use and make public
this modified software library. I genuinely only want to do so to showcase
my work, advertised my interest in the area and further my hobby/DIY drone
ambitions NOT FOR COMMERCIAL GAIN. Coursera and UPenn: if this is a problem
please let me know and I'll happily comply with whatever you want.

___

### Update Log:

Rev1: May 2018. Minimum jerk trajectory generator for a 5-waypoint path, choice
of linear and non-linear controllers.

Rev2: UNDER CONSTRUCTION: generalised n-point trajectory.
____

### Getting Started:

Ensure all files are in the same directory. Specify the trajectory coordinates
you would like in runsim.m and choose between the linear (OK_controller3D.m)
and non-linear (OK_controller3DNL.m) libraries to play around.

Files: 
Will update descriptions when I get some more time.

_____
