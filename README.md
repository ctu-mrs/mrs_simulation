# MRS simulation

| Build status | [![Build Status](https://github.com/ctu-mrs/mrs_simulation/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_simulation/actions) |
|--------------|--------------------------------------------------------------------------------------------------------------------------------------------|

## Overview
Support for spawning vehicles into Gazebo simulation, where user can select from multiple UAV platforms.
This platforms can be additionaly equipped with several sensors (rangefinders, 2d lidars, stereo cameras etc.).

## Usage

> :warning: **New drone spawning mechanism**:
>
> * From now on, the drone spawning mechanism will be controlled by a ROS node
> * Spawning new vehicles is now done by calling ROS services
> * The spawner node is launched automatically with the simulation
> * The arguments for vehicle configuration remain mostly unchanged, just use 'rosservice call /mrs_drone_spawner/spawn "args"' instead of 'spawn_uav args'
> * Distributed simulation (running gazebo on one computer and mavros/PX4 on another is currently not supported


### Start of the Gazebo simulator

To start the prepared example of Gazebo world call:

```bash
roslaunch mrs_simulation simulation.launch world_file:='$(find mrs_gazebo_common)/worlds/grass_plane.world' gui:=true
```

At this point the Gazebo world will only contain the environment with grass plane but with no vehicles yet.

### Spawning of UAVs ([NEW](https://ctu-mrs.github.io/docs/simulation/drone_spawner.html))
The `simulation.launch` will automatically start the `mrs_drone_spawner` python node. If you use a custom launch file to start the simulation, you can start it separately:

```bash
roslaunch mrs_simulation mrs_drone_spawner.launch
```

The `mrs_drone_spawner` will perform the following tasks:

* Spawn vehicle models in the Gazebo simulation (ids from 0 to 250). This is done internally by calling the command `rosrun gazebo_ros spawn_model`.

* For each vehicle, PX4 firmware and mavros is started at specific port numbers depending on the vehicle ID.

Vehicles are added to the simulation by calling the `spawn` service of the `mrs_drone_spawner`. The service takes one string argument, which specifies the vehicle ID, type and sensor configuration. Example: spawn a single vehicle with a down-facing laser rangefinder:

```bash
rosservice call /mrs_drone_spawner/spawn "1 --enable-rangefinder"
```

To display the manual containing a list of all available arguments, perform a dry-run of the script:

```bash
rosrun mrs_simulation mrs_drone_spawner
```

The arguments are also listed in the `mrs_simulation/config/spawner_params.yaml` file. Note that not all sensors are available for all the vehicle types. The config file stores the available configurations in the following format: `parameter: [default_value, help_description, [compatible_vehicles]]`

#### New features
Multiple vehicles may be spawned with one service call:

```bash
rosservice call /mrs_drone_spawner/spawn "1 2 3 4 5 --t650 --enable-bluefox-camera --enable-rangefinder"
```

Spawn position may be specified by a command line argument `--pos x y z heading`: [m, m, m, rad]
```bash
rosservice call /mrs_drone_spawner/spawn "1 --f550 --enable-rangefinder --pos 10 -15 0.3 0.7"
```
For multiple vehicles, `--pos` defines the spawn point of the first vehicle. Following vehicles will be spawned in a line, with an x-offset of 2 meters from the previous vehicle.

Spawn position may be specified by a `.csv` or a `.yaml` file using `--file absolute_path_to_file`:
```bash
rosservice call /mrs_drone_spawner/spawn "1 --f450 --enable-rangefinder --enable-ouster --use-gpu-ray --ouster-model OS1-64 --file `pwd`/spawn_poses.yaml"
```

Use a whitespace instead of an ID to get an available ID automatically assigned by the spawner.
```bash
rosservice call /mrs_drone_spawner/spawn " --f450 --enable-rangefinder"
```

<!-- THIS SECTION IS OUTDATED
Not all sensors have to be available for selected type of uav (DJI f450, DJI f550, Tarot t650 and Eagle.one mk2). Please check possible settings for
the selected type of UAV by calling command:

```bash
spawn_uav --$UAV_TYPE --available-sensors
```

#### Spawning on single PC

An example for spawning the DJI F550 vehicle with id 1 and additional sensors, running generated launch file directly, and deleting models from gazebo simulation after closing the script.

```bash
spawn_uav 1 --f550 --enable-bluefox-camera --enable-rangefinder --run --delete
```

To spawn multiple vehicles use additional terminal windows/panels or chain the spawn commands by running the previous commands in background.

```bash
spawn_uav 1 --f550 --run --delete --enable-rangefinder & spawn_uav 2 --f550 --run --delete --enable-rangefinder && fg
```

Note that this approach will hide the output of all but one spawn script. A recommended practise to using multiple terminal panels.

#### Distributed simulation
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
    spawn_uav 1 --enable-bluefox-camera --enable-rangefinder --run --delete --mavlink-address IP_ADDR
```
-->

### Running the MRS control pipeline
Check out the wiki on [how to run control core](https://ctu-mrs.github.io/docs/simulation/howto.html#3-run-the-control-core).
