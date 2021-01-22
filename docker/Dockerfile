FROM ubuntu:18.04

RUN apt-get update && \
    apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && \
    apt-get install -y python3.7 python3-pip libpng16-16 libtiff5-dev libjpeg-dev libgtk-3-0

WORKDIR /home/carla
COPY --from=carlasim/carla /home/carla/PythonAPI ./PythonAPI
RUN python3.7 -m easy_install ./PythonAPI/carla/dist/carla*py3.7*.egg
RUN python3.7 -m pip install pygame numpy networkx

WORKDIR /home/carla/PythonAPI/simulanes

RUN adduser carla
USER carla
