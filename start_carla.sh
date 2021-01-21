#!/bin/bash

docker run -p 2000-2002:2000-2002 \
           --gpus all \
           -e DISPLAY=$DISPLAY \
           -e SDL_VIDEODRIVER=x11 \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -it \
           carlasim/carla \
           ./CarlaUE4.sh -opengl
