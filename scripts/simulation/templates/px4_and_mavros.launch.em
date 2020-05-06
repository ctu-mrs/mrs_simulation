<?xml version="1.0"?>
<launch>
  <group ns="uav@(mav_sys_id)">

    <!-- PX4 SITL parameters -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="@(vehicle_type)"/>
    <arg name="ID" default="@(mav_sys_id)"/>
    <arg name="interactive" default="false"/>

    <!-- MAVROS parameters -->
    <arg name="fcu_url" value="udp://:@(ros_interface_port4)@@localhost:@(ros_interface_port3)" />
    <arg name="tgt_system" value="@(mav_sys_id)" />
    <arg name="respawn_mavros" value="false"/>

    <!-- PX4 SITL -->
    <include file="$(find mrs_simulation)/launch/px4_sitl.launch">
      <arg name="est" value="$(arg est)"/>
      <arg name="vehicle" value="$(arg vehicle)"/>
      <arg name="ID" value="$(arg ID)"/>
      <arg name="interactive" value="$(arg interactive)"/>
    </include>

    <!-- MAVROS -->
    <include file="$(find mrs_simulation)/launch/mavros/px4.launch">
      <!-- GCS link is provided by SITL -->
      <arg name="fcu_url" value="$(arg fcu_url)"/>
      <arg name="gcs_url" value=""/>
      <arg name="tgt_system" value="$(arg tgt_system)"/>
      <arg name="tgt_component" value="1"/>
      <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
      <arg name="uav_name" value="uav@(mav_sys_id)" />
    </include>

  </group>
</launch>
