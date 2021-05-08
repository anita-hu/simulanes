## Overview
The following sections provide info for some important files

### Data collection
`automatic_control.py` and `manual_control.py` share the same code for lane extraction in `lane_utils.py`.
- `automatic_control.py` is used for automatic data collection
- `manual_control.py` is used for debugging

### Data format
Lane labels are saved in the TuSimple format. 
`lane_config.py` specifies the row anchors used which can be changed to follow the CULane format.

### Map info
`map_info.py` contains the max number of lanes (including curbs) expected for each road id.
This info is used to ensure the labels saved have no error. 
If a town is not in map info, it will not be used for data collection.
