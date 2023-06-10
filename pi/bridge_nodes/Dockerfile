
# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && pip3 install pyyaml

COPY mc_sub.py ./mc_sub.py
COPY cpu_sub.py ./cpu_sub.py
COPY run.sh ./run.sh

RUN chmod a+x ./run.sh
