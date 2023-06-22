# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

ENV ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -y python3.7 python3.7-dev python3-pip \
    && python3.7 -m pip install pyyaml rospkg

# Ensure that Python 3.7 is used
RUN ln -sf /usr/bin/python3.7 /usr/bin/python

COPY mc_sub_thread.py ./mc_sub_thread.py
COPY cpu_sub_thread.py ./cpu_sub_thread.py
COPY run.sh ./run.sh

RUN chmod a+x ./run.sh