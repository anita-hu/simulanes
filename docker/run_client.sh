#!/bin/bash

version=$(cat $(dirname $(realpath -s $0))/"image_version.txt")

cd .. && \
docker run --gpus all \
           --net=host \
           -e DISPLAY=$DISPLAY \
           -e SDL_VIDEODRIVER=x11 \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -it \
	   --user carla \
           -v $PWD/src:/home/carla/PythonAPI/simulanes \
           simulanes/dev:$version \
	   python3.7 automatic_control.py
