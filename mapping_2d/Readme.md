# 2D Mapping with Hokuyo LIDAR
# TO BE UPDATED

### Existing LIDAR in ROBIN Lab

| LIDAR         | Interface     | IP Address setting  |
| ------------- |:-------------:| -------------------:|
| UTM-10LX      | Ethernet      | 192.168.0.10        |
| UTM-20LX      | Ethernet      | 192.168.2.11        |


### LIDAR Setting

ROS node required: urg_node

Setting ip_address 
1. Use windows platform
2. Set ip address of the Ethernet to the same network as the laser network
3. Connect to the device
4. Hit "Change IP Address " button

### Mapping 2D with Hokuyo LIDAR

Required node:

1. urg_node `sudo apt-get install ros-indigo-urg-node`
2. hector_mapping `sudo apt-get install ros-indigo-hector-mapping`

Run launch file:

`roslaunch mapping_2d mapping_2d.launch`
