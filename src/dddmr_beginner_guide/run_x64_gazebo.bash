#!/bin/bash

xhost +local:docker

# docker run -it \
#     --privileged \
#     --network=host \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp:/tmp" \
#     --volume="/dev:/dev" \
#     --name="dddmr_x64_gazebo" \
#     dddmr_gz:x64

# server X11 setting
docker run -it \
  --privileged \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/root/.Xauthority \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v /tmp:/tmp \
  -v /dev:/dev \
  --name dddmr_x64_gazebo \
  dddmr_gz:x64