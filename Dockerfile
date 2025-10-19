FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir flask

WORKDIR /app
COPY app.py /app/app.py
COPY static/ /app/static/

EXPOSE 8080
CMD ["python3", "app.py"]
