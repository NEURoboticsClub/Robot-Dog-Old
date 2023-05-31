# Dockerfile_server
FROM ros:noetic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY cpu_sub.py ./cpu_sub.py

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && python3 cpu_sub.py"]
