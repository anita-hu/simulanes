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
```
usage: automatic_control.py [-h] [-v] [--host H] [-p P] [--res WIDTHxHEIGHT]
                            [--filter PATTERN] [--gamma GAMMA]
                            [-b {cautious,normal,aggressive}]
                            [-a {Behavior,Roaming,Basic}] [-s SEED]
                            [-i NUM_IMAGES] [-r] [-n N] [-w W] [--safe]
                            [--filterv PATTERN] [--filterw PATTERN]
                            [--tm_port P] [--sync] [--hybrid]
                            [--car_lights_on]

CARLA Automatic Control Client

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         Print debug information
  --host H              IP of the host server (default: 127.0.0.1)
  -p P, --port P        TCP port to listen to (default: 2000)
  --res WIDTHxHEIGHT    Window resolution (default: 1280x720)
  --filter PATTERN      Actor filter (default: "vehicle.tesla.model3")
  --gamma GAMMA         Gamma correction of the camera (default: 2.2)
  -b {cautious,normal,aggressive}, --behavior {cautious,normal,aggressive}
                        Choose one of the possible agent behaviors (default:
                        normal)
  -a {Behavior,Roaming,Basic}, --agent {Behavior,Roaming,Basic}
                        select which agent to run
  -s SEED, --seed SEED  Set seed for repeating executions (default: None)
  -i NUM_IMAGES, --num_images NUM_IMAGES
                        Set total number of image to generate (default: 1000)
  -r, --resume          Continue from previous session resume_progress.json
                        file
  -n N, --num_npc_vehicles N
                        Max number of NPC vehicles (default: 20)
  -w W, --num_npc_walkers W
                        Max number of NPC walkers (default: 10)
  --safe                Avoid spawning vehicles prone to accidents
  --filterv PATTERN     Vehicles filter (default: "vehicle.*")
  --filterw PATTERN     Pedestrians filter (default: "walker.pedestrian.*")
  --tm_port P           Port to communicate with TM (default: 8000)
  --sync                Synchronous mode execution
  --hybrid              Enable hybrid physics mode
  --car_lights_on       Enable car lights at spawn
```
For manual control, run this Python script instead:
```
python3.7 manual_control.py [-t TOWN]
```

## Reference

Our code is built upon PythonAPI/examples from [carla](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples) git repository.

If it is helpful for your work, please cite:

_CARLA: An Open Urban Driving Simulator_<br>Alexey Dosovitskiy, German Ros,
Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16
[[PDF]](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)
[[talk]](https://www.youtube.com/watch?v=xfyK03MEZ9Q&feature=youtu.be&t=2h44m30s)

```
@inproceedings{Dosovitskiy17,
  title = {{CARLA}: {An} Open Urban Driving Simulator},
  author = {Alexey Dosovitskiy and German Ros and Felipe Codevilla and Antonio Lopez and Vladlen Koltun},
  booktitle = {Proceedings of the 1st Annual Conference on Robot Learning},
  pages = {1--16},
  year = {2017}
}
```
