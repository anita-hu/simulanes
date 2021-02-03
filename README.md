# Simulanes

## Usage

### Carla server
Install [docker](https://docs.docker.com/engine/install/) and [nvidia-container-runtime](https://nvidia.github.io/nvidia-container-runtime/). 
Then pull the carla docker image:
```
docker pull carlasim/carla:0.9.10
```
Build the container (downloads additional assets)
```
bash docker/build_server.sh
```
Run container and start carla server:
```
bash docker/run_server.sh
```

### Carla client
Build docker image
```
bash docker/build_client.sh
```
Run script in client server
```
bash docker/run_client.sh
```
