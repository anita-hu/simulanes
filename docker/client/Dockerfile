FROM ubuntu:18.04

RUN apt-get update && \
    apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && \
    apt-get install -y python3.7 python3-pip libpng16-16 libtiff5-dev libjpeg-dev libgtk-3-0

RUN useradd -m carla

WORKDIR /home/carla
COPY --from=carlasim/carla:0.9.11 /home/carla/PythonAPI ./PythonAPI
RUN python3.7 -m easy_install ./PythonAPI/carla/dist/carla*py3.7*.egg
RUN python3.7 -m pip install pygame numpy networkx

USER carla
WORKDIR /home/carla/PythonAPI/simulanes
