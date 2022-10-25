INSTALLATION/CONFIG:
Copy this ENTIRE folder into your catkin_ws/src
	- git clone the repo

From the root of your catkin workspace (/catkin_ws/), install all deps
	- rosdep install -i --from-path src --rosdistro noetic -y
From the root of your catkin workspace, (/catkin_ws/), run catkin_make

RUNNING THE SIMULATION:
roslaunch simple_drive_practice simple_drive_gazebo.launch

SAMPLE TEST TRACK FOR USE:
There is a sample test track included in simple_drive_practice/worlds for you to test your obstacle avoidance algorithm in. Click on the insert tab in gazebo, and then add the worlds folder as a file path (only simple_drive_practice/worlds, if you have simple_drive_practice/worlds/test_track as the path you've gone too deep), and you should be able to set up the sample test track.

EXAMPLE SCRIPT:
Run chmod +x sample_script_lvov.py in simple_drive_practice/scripts
Then the example script should run with rosrun simple_drive_practice sample_script_lvov.py when the sim is running

RUNNING SCRIPTS:
Either make your python scripts executable via command line (chmod +x {NAMEOFSCRIPTHERE}.py from simple_drive_practice/scripts)

OR add them to the CMakeLists
	- You will need to add all C++ scripts to the CMakeLists, and run catkin_make for every change
	- See http://wiki.ros.org/catkin/CMakeLists.txt 
		-Section 11.1 for Python
		-Section 7.4 Build Targets for C++
