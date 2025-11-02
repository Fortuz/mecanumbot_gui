FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3 python3-pip \
    libsdl2-dev libsdl2-mixer-dev libsdl2-image-dev libsdl2-ttf-dev \
    libportmidi-dev libswscale-dev libavformat-dev libavcodec-dev \
    libasound2-dev && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

WORKDIR /app
COPY app.py /app/app.py
COPY static/ /app/static/
COPY templates/ /app/templates/

EXPOSE 8080
CMD ["python3", "app.py"]
