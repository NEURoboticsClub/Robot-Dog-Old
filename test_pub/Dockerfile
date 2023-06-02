# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python-pip \
    && pip2 install pyyaml

COPY test_pub.py ./test_pub.py

# prepares your shell session to use the ROS commands and tools
# run the publisher node
CMD ["bash", "-c", "source /opt/ros/melodic/setup.bash && python test_pub.py"]
