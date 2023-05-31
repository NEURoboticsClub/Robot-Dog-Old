# Dockerfile_test_node
FROM ros:noetic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY test_pub.py ./test_pub.py


# prepares your shell session to use the ROS commands and tools
# run the publisher node
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 test_pub.py"]
