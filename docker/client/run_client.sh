#!/bin/bash

parent=$(dirname $(realpath -s $0))
version=$(cat $parent/"image_version.txt")
root=$(dirname $(dirname $parent))

docker run --gpus all \
           --net=host \
           -e DISPLAY=$DISPLAY \
           -e SDL_VIDEODRIVER=x11 \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -it \
	       --user carla \
           -v $root/src:/home/carla/PythonAPI/simulanes \
           simulanes/dev:$version