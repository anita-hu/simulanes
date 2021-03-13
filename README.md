# Simulanes

## Usage

### Carla server
Install [docker](https://docs.docker.com/engine/install/) and [nvidia-container-runtime](https://nvidia.github.io/nvidia-container-runtime/). 
Then pull the carla docker image:
```
docker pull carlasim/carla:0.9.11
```
Build the server image (downloads additional assets):
```
bash docker/server/build_server.sh
```
Run the server container:
```
bash docker/server/run_server.sh
```

### Carla client
Build the client image:
```
bash docker/client/build_client.sh
```
Run the client container:
```
bash docker/client/run_client.sh
```
Run the Python script from within the client container:
```
python3.7 automatic_control.py
```
