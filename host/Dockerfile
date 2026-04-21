FROM ros:humble

# Set HOME to /tmp so ROS2 logging (rcl_logging_spdlog) writes to /tmp/.ros/log
# instead of /root/.ros/log.  /root is mode 700 and not accessible to the
# non-root uid that the container runs as (--user flag in docker run).
ENV HOME=/tmp

# Pre-create the ROS log directory so it exists at startup
RUN mkdir -p /tmp/.ros/log && chmod -R 777 /tmp/.ros

# Enable ROS2 Super Client mode for better DDS discovery
ENV ROS_SUPER_CLIENT=true

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

COPY host/requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

# ── Build mecanumbot_msgs so ROS2 service types are available ────────────
# Build context is the project root, so shared/ is reachable
#COPY shared/mecanumbot_msgs/ /ws/src/mecanumbot_msgs/
WORKDIR /ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select mecanumbot_msgs && \
    echo 'mecanumbot_msgs built OK'"

WORKDIR /app
COPY host/app.py /app/app.py
COPY shared/database_interface.py /app/database_interface.py
COPY shared/database.py /app/database.py
COPY host/static/ /app/static/
COPY host/templates/ /app/templates/
COPY host/docker_node.py /app/docker_node.py

EXPOSE 8080

# Create a startup script — Flask embeds the DockerNode, only one process needed
RUN echo '#!/bin/bash\n\
export ROS_SUPER_CLIENT=true\n\
source /opt/ros/humble/setup.bash\n\
source /ws/install/setup.bash\n\
python3 /app/app.py\n\
' > /app/start.sh && chmod +x /app/start.sh

# Run the startup script
CMD ["/bin/bash", "/app/start.sh"]
