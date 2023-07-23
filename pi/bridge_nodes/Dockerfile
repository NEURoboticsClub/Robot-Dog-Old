# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

ENV ROS_PYTHON_VERSION=3

# 2 RUN
# 2.1 updates
RUN apt-get update && apt-get install -y \
    && apt-get update 

# 2.2 Install python3-rospkg and to remove ROS packages (already installed).
# Select Yes for that prompt. This will remove ROS packages and we will have to re-install them.
RUN apt-get install -y python3-all-dev python3-rospkg python3-pip

# 2.3 omplete installation
RUN apt-get install -y ros-melodic-desktop-full --fix-missing
# RUN apt-get install python-rosdep
# RUN rosdep update

# Ensure that Python 3.7 is used
RUN ln -sf /usr/bin/python3 /usr/bin/python

COPY mc_sub.py ./mc_sub.py
COPY cpu_sub.py ./cpu_sub.py
COPY run.sh ./run.sh

RUN chmod a+x ./run.sh