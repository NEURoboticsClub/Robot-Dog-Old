# Dockerfile_server
FROM ros:noetic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY test_server_sub.py ./test_server_sub.py

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 test_server_sub.py"]
