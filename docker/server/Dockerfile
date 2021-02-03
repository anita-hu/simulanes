FROM carlasim/carla:0.9.11

USER root
RUN apt-get update && apt-get install -y wget
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.11.tar.gz
RUN mv AdditionalMaps_0.9.11.tar.gz Import/
RUN ./ImportAssets.sh

USER carla
WORKDIR /home/carla
