#!/bin/bash

version=$(cat $(dirname $(realpath -s $0))/"image_version.txt")

docker run -p 2000-2002:2000-2002 \
           --gpus all \
           -e SDL_VIDEODRIVER=offscreen \
           -it \
           simulanes/carla:$version \
           ./CarlaUE4.sh -opengl -nosound -carla-server
