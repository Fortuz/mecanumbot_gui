FROM ros:humble

ENV HOME=/tmp
RUN mkdir -p /tmp/.ros/log && chmod -R 777 /tmp/.ros
ENV ROS_SUPER_CLIENT=true

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy python requirements and install
COPY host/requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

# Copy GUI app files
COPY host/app.py /app/app.py
COPY shared/database_interface.py /app/database_interface.py
COPY shared/database.py /app/database.py
COPY host/static/ /app/static/
COPY host/templates/ /app/templates/
COPY host/docker_node.py /app/docker_node.py

EXPOSE 8080

# The startup script runs EVERY TIME the container starts.
# It navigates to the mounted /ws, builds it, sources it, and runs the Flask app.
RUN echo '#!/bin/bash\n\
export ROS_SUPER_CLIENT=true\n\
source /opt/ros/humble/setup.bash\n\
cd /ws\n\
colcon build\n\
source /ws/install/setup.bash\n\
cd /app\n\
python3 app.py\n\
' > /app/start.sh && chmod +x /app/start.sh

CMD ["/bin/bash", "/app/start.sh"]
