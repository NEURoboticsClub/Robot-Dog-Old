# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python-pip \
    && pip2 install pyyaml

COPY cpu.py ./cpu.py

# prepares your shell session to use the ROS commands and tools
# run the publisher node
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && python cpu_pi.py"]
