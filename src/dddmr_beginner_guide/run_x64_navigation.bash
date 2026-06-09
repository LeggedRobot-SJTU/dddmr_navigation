#!/bin/bash

xhost +local:docker

# docker run -it \
#     --privileged \
#     --network=host \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp:/tmp" \
#     --volume="/dev:/dev" \
#     --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
#     --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
#     --name="dddmr_x64_navigation" \
#     dddmr:x64

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
    -v "${HOME}/dddmr_navigation:/root/dddmr_navigation" \
    -v "${HOME}/dddmr_bags:/root/dddmr_bags" \
    --name dddmr_x64_navigation \
    dddmr:x64