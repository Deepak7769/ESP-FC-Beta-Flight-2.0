FROM ubuntu:24.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3 python3-pip build-essential mc git curl wget clang-format-20 clang-tidy-20 \
&& apt-get clean && rm -rf /var/lib/apt/lists/* \
&& mkdir -p /home/ubuntu/.platformio \
&& chown ubuntu:ubuntu /home/ubuntu/.platformio

RUN pip3 install -U platformio --break-system-packages

WORKDIR /app
