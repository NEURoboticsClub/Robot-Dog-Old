# PID Tuning Setup Guide for Leg on Test Stand Simulation

copy legv1_description package to src folder of your catkin workspace

Dependencies: 


Compile catkin workspace

    catkin make

    roslaunch legv1_description gazebo.launch 
    roslaunch legv1_description controller_temp.launch


launch rqt - control  user interface

    rqt

plugins -> topics-> message publisher 

in dropdown for topic : 

    select /legv1/j1_position_controller/command

select arrow to left of command 

in expression column, enter desires position in radians and hit enter

to execute : click check box to left

repeat from j1 to j3

    rosrun rqt_reconfigure rqt_reconfigure 

