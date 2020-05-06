# mrs_simulation repository
Support for spawning vehicles into gazebo simulation.

## 1. Start of the Gazebo simulator

To start the prepared example of Gazebo world call:

```bash
roslaunch simulation mrs_simulation.launch world_file:='$(find mrs_gazebo_common)/worlds/grass_plane.world' gui:=true
```

At this point the Gazebo world will only contain the environment with grass plane but with no vehicles yet.

## 2. Spawning of UAVs 
The command `spawn` can be used to perform the following tasks:

* Spawn the vehicle models in the Gazebo simulation (ids from 1 - 250). This is done internally by calling service `/gazebo/spawn_sdf_model`.
  
* For each vehicle a custom init script for PX4 and mavros is being generated which contains specific port numbers depending on the selected id.

* For the set of spawned vehicles a single ROS launch file could be generated.
  This launch configuration starts PX4 and mavros for each vehicle.

For the capability of the script see help:

```bash
spawn --help
```

Not all sensors have to be available for selected type of uav (DJI f450, DJI f550, Tarot t650 and Eagle.one mk2). Please check possible settings for
the selected type of UAV by calling command: 

```bash
spawn --$UAV_TYPE --available-sensors
```

#### Spawning on single PC

An example for spawning the DJI F550 vehicle with id 1 and additional sensors, running generated launch file directly, and deleting models from gazebo simulation after closing the script.

```bash
spawn 1 --f550 --enable-bluefox-camera --enable-rangefinder --run --delete
```

To spawn multiple vehicles use additional terminal windows/panels or chain the spawn commands by running the previous commands in background.

```bash
spawn 1 --f550 --run --delete --enable-rangefinder & spawn 2 --f550 --run --delete --enable-rangefinder && fg
```

Note that this approach will hide the output of all but one spawn script. A recommended practise to using multiple terminal panels.

#### distributed simulation
In order to run a simulation distributed to multiple machines, these prerequisities have to be met:
* A central machine with IP address `IP_ADDR_MASTER` must be running ROS master, i.e. perform `roslaunch mrs_simulation simulation.launch` to start Gazebo.
* Following variables have to be set in the environment on all used machines. On a machine with IP address `IP_ADDR`, add following lines to the `~/.bashrc` script (do not forget to source the script after).

```bash
    export ROS_IP=IP_ADDR
    export ROS_MASTER_URI=http://IP_ADDR_MASTER:11311
    export GAZEBO_IP=IP_ADDR
    export GAZEBO_MASTER_URI=http://IP_ADDR_MASTER:12345
```

* To spawn a vehicle from non-central machine with IP address `IP_ADDR`, extend the spawning example with parameter `--mavlink-address`, i.e.

```bash
    spawn 1 --enable-bluefox-camera --enable-rangefinder --run --delete --mavlink-address IP_ADDR
```

## 3. Running the MRS control pipeline
Check out the wiki on [how to run complete simulation](https://mrs.felk.cvut.cz/gitlab/uav/uav_core/wikis/how_to_start_simulation#3-run-mrs_uav_manager-node).
