# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

# 2. install 
RUN apt-get update
RUN apt-get install -y python-yaml


# 3. Copy your src/ directory into the container
# should not copy the build/, devel/, or install/ directories from your host machine to your Docker container.
# These directories are generated when you build your Catkin workspace,
# and their contents are specific to the system where you did the build.
COPY ./catkin_ws/src/champ ./src/champ
COPY ./catkin_ws/src/champ_teleop ./src/champ_teleop
COPY ./catkin_ws/src/cpu/package.xml ./src/cpu/

# 4. CHAMP install dependencies thats defined in packages.xml in src folder
RUN rosdep install --from-paths src --ignore-src -r -y

COPY ./catkin_ws/src/cpu ./src/cpu
# 5. Run catkin_make to build your workspace
# - /opt/ros/melodic/setup.bash
# This command sources the setup script for the ROS environment.
# This script sets a number of environment variables that ROS uses,
# such as ROS_ROOT, ROS_PACKAGE_PATH, and ROS_MASTER_URI, among others.
# It also adds various ROS tools to your PATH.
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /app; catkin_make clean; catkin_make'

# 6. This command sources the setup script for your catkin workspace
# - This will be overriden by command in docker-compose file if you build the container using compose
# - source /app/devel/setup.bash
# This sets up the environment variables for the packages in your catkin workspace.
# For example, it modifies ROS_PACKAGE_PATH to include the packages in your workspace,
# and it adds any executables in your workspace to your PATH.
CMD ["bash", "-c", "source /app/devel/setup.bash && roslaunch cpu all.launch"]
