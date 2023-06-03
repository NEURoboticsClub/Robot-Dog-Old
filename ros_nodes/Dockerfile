
# Dockerfile_test_node
FROM ros:melodic

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python-pip \
    && pip2 install pyyaml

COPY mc_sub.py ./mc_sub.py
COPY cpu_sub.py ./cpu_sub.py
COPY run.sh /run.sh

# RUN chmod a+x /run.sh
