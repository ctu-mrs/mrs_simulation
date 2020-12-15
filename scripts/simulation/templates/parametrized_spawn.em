<?xml version="1.0"?>
<launch>
  <!-- MAVROS posix SITL environment launch script -->
  <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
  <arg name="ID" value="@(mav_sys_id)"/>
  <arg name="vehicle" default="@(vehicle_type)"/>
  <arg name="est" default="ekf2"/>
  <arg name="interactive" default="true"/>

  <!-- UAV namespace -->
  <group ns="uav$(arg ID)">

    <!-- Mavlink and Gazebo communication ports -->
    <!-- This has to match the rcS file!! -->
    <arg name="udp_offboard_port_remote" value="@(udp_offboard_port_remote)"/>
    <arg name="udp_offboard_port_local" value="@(udp_offboard_port_local)"/>
    <arg name="mavlink_tcp_port" value="@(mavlink_tcp_port)"/>
    <arg name="mavlink_udp_port" value="@(mavlink_udp_port)"/>

    <!-- PX4 params -->
    <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
    <env name="PX4_ESTIMATOR" value="$(arg est)" />

    <!-- MAVROS and vehicle configs -->
    <arg name="fcu_url" value="udp://:$(arg udp_offboard_port_remote)@@localhost:$(arg udp_offboard_port_local)"/>
    <arg name="x" value="@(x)"/>
    <arg name="y" value="@(y)"/>
    <arg name="z" value="@(z)"/>
    <arg name="R" value="0"/>
    <arg name="P" value="0"/>
    <arg name="Y" value="@(heading)"/>
  
    <!-- PX4 SITL -->
    <arg unless="$(arg interactive)" name="px4_command_arg1" value=""/>
    <arg     if="$(arg interactive)" name="px4_command_arg1" value="-d"/>
    <node name="sitl_$(arg ID)" pkg="px4" type="px4" output="screen" args="$(find mrs_simulation)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -i $(arg ID) -w sitl_$(arg vehicle)_$(arg ID) $(arg px4_command_arg1)">
    </node>

    <!-- spawn vehicle -->
    <!-- <arg name="cmd" value="$(find xacro)/xacro $(find mrs_simulation)/models/mrs_robots_description/urdf/$(arg vehicle).xacro mrs_robots_description_dir:=$(find mrs_simulation)/models/mrs_robots_description namespace:=uav$(arg ID) mavlink_udp_port:=$(arg mavlink_udp_port) mavlink_tcp_port:=$(arg mavlink_tcp_port) enable_rangefinder:=true"/> -->
    <!-- <param name="robot_description" command="$(arg cmd)"/> -->
    <!-- <node name="$(arg vehicle)_$(arg ID)_spawn" output="screen" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model $(arg vehicle)_$(arg ID) -package_to_model -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/> -->

    <!-- MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
      <arg name="fcu_url" value="$(arg fcu_url)"/>
      <arg name="gcs_url" value=""/>
      <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
      <arg name="tgt_component" value="1"/>
    </include>

  </group>
</launch>
