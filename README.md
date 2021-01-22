# Simulanes

## Usage

### Carla server
Install [docker](https://docs.docker.com/engine/install/) and [nvidia-container-runtime](https://nvidia.github.io/nvidia-container-runtime/). 
Then pull the carla docker image:
```
docker pull carlasim/carla:latest
```
Run container and start carla server:
```
bash docker/start_carla.sh
```

### Carla client
Build docker image
```
bash docker/build.sh
```
Run script in client server
```
bash docker/run.sh
```
