## Overview
The following sections provide info for some important files

### Data collection
`automatic_control.py` and `manual_control.py` share the same code for lane extraction in `lane_utils.py`.
- `automatic_control.py` is used for automatic data collection
- `manual_control.py` is used for debugging and gathering map info where images are not saved

### Data format
Lane labels are saved in the TuSimple format. 
`lane_config.py` specifies the row anchors used which can be changed to follow the CULane format.

### Map info
`map_info.py` contains the following info for most available towns in CARLA
1. the max number of lanes (including curbs) expected for each road id
2. bad road ids where there are unfixable issues with the lane labels

This info is used to ensure the labels saved have no error. 
If a town is not in map info, it will not be used for data collection.

To add a new map to the collection, 
1. Generate the map using `python manual_control.py -t {TownID} -m`
2. With the generated map as reference, use `manual_control.py` to drive around the town and add the required 
information in `map_info.py` using the same class structure as the other towns.

### Dynamic weather
`dynamic_weather.py` is used to control how the weather changes in the simulation. The most important parameter is 
`altitude`, the equation that determines how the time of day changes over time [here](https://github.com/anita-hu/simulanes/blob/main/src/dynamic_weather.py#L52).
Modify the sine function to change the range of time and ratio of day/night in the generated dataset. 
