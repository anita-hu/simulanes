#!/bin/bash

docker run -p 2000-2002:2000-2002 \
           --gpus all \
           -e SDL_VIDEODRIVER=offscreen \
           -it \
           carlasim/carla \
           ./CarlaUE4.sh -opengl -nosound -carla-server
