from __future__ import division
from __future__ import print_function

import argparse
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import math
import os
import re
import sys
import tempfile
import csv
import re

from em import Interpreter
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import DeleteModelRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from rosgraph import ROS_MASTER_URI
from rospy import ServiceProxy
from rospy import ServiceException
from rospy import logerr
from rospy import loginfo

import rospkg

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

bashCommand = "rosversion -d"
import subprocess
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()

## checking the ROS version
if output=="indigo\n":
    # for ROS indigo
    import xacro_jade as xacro
else:
    # for newest version of ROS
    import xacro

VEHICLE_BASE_PORT = 14000
VEHICLE_TCP_BASE_PORT = 4560

def print_error(string):
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    print(FAIL + string + ENDC) 

def print_info(string):
    BOLD = '\033[1m'
    ENDC = '\033[0m'
    print(BOLD + string + ENDC) 

def print_ok(string):
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'
    print(OKGREEN + string + ENDC) 

def get_vehicle_base_port(mav_sys_id):
    return VEHICLE_BASE_PORT + mav_sys_id * 4


def get_vehicle_pose(mav_sys_id):
    inteam_id = mav_sys_id % 100
    x = 0     
    y = 29.45
    z = 0.3
    
    if ( inteam_id > 1 ):
        y -= 8*((inteam_id)//2)
        
        if( inteam_id % 2 == 0 ):
            x += 4
        if( inteam_id % 2 == 1 ):
            x -= 4

    yaw = 0
    return (x, y, z, yaw)


def get_vehicle_pose_from_file(fname, uav_id):
    if not os.path.isfile(fname):
        print("File '%s' does not exist" %fname, file=sys.stderr)
        sys.exit(0)

    array_string = list(csv.reader(open(fname)))
    for row in array_string:
        if (len(row)!=5):
            print("Incorrect data in file '%s'! Data should be in format [id, x, y, z, heading] (example: int, float, float, float, float)" %fname)
            sys.exit(1)
        print(row)
        if int(row[0]) == uav_id:
            return [uav_id], [[float(row[i]) for i in range(1,len(row))]]

    # ids = [int(array_string[i][0]) for i in range(0,len(array_string))]
    # array = [[float(array_string[j][i]) for i in range(1,len(array_string[j]))] for j in range(0,len(array_string))]
    raise Exception('UAV_ID %d not found in the provided file' %uav_id)
    # return ids, array

def mav_sys_id_type(value):
    value = int(value)
    if value < 1 or value > 250:
        raise argparse.ArgumentTypeError('MAV_SYS_ID must be in [1, 250]')
    return value

def get_model_xacro_file(vehicle_type):
    if vehicle_type == "f550":
        model_pkg = "mrs_simulation"
        model_file = "models/mrs_robots_description/urdf/f550.xacro"
    
    if vehicle_type == "f450":
        model_pkg = "mrs_simulation"
        model_file = "models/mrs_robots_description/urdf/f450.xacro"
    
    if vehicle_type == "t650":
        model_pkg = "mrs_simulation"
        model_file = "models/mrs_robots_description/urdf/t650.xacro"

    if vehicle_type == "eaglemk2":
        model_pkg = "eagle_gazebo_resources"
        model_file = "models/eagle_robots_description/urdf/eaglemk2.xacro"

    model_pathname = os.path.join(
        rospack.get_path('mrs_simulation') , 'models', 'mrs_robots_description'
    )
    try:
        model_pkg_path = rospack.get_path(model_pkg)
    except rospkg.common.ResourceNotFound, e:
         print_error("===================================================================================")
         print_error("   Package \'%s\' was not found. " % model_pkg)
         print_error("   Note: The selected model doesn't have to be publicly available.")
         print_error("===================================================================================")
         sys.exit(3)

    model_filename = os.path.join(
        model_pkg_path, '%s' % model_file
    )

    with open(model_filename, 'r') as h:
        model_xml = h.read()

    return model_pathname, model_xml 

def check_for_uvdar_package():

    uvdar_pkg = "uvdar_gazebo_plugin"

    try:
        model_pkg_path = rospack.get_path(uvdar_pkg)
    except rospkg.common.ResourceNotFound, e:
         print_error("===================================================================================")
         print_error("   Package \'%s\' was not found. " % uvdar_pkg)
         print_error("   Note: This package is specific to the UVDAR project.")
         print_error("===================================================================================")
         sys.exit(3)

    return

def spawn_model(
        mav_sys_id, vehicle_type, tcp_port, udp_port, pose,
        ros_master_uri=None,
        mavlink_address=None,
        enable_rangefinder=False,
        enable_teraranger=False,
        enable_rangefinder_up=False,
        enable_ground_truth=False,
        enable_mobius_camera_down=False,
        enable_mobius_camera_front=False,
        enable_mobius_camera_back_right=False,
        enable_mobius_camera_back_left=False,
        enable_realsense_front_pitched=False,
        enable_realsense_down=False,
        enable_realsense_top=False,
        enable_realsense_front=False,
        use_realistic_realsense=False,
        enable_whycon_box=False,
        enable_bluefox_camera=False,
        enable_bluefox_camera_reverse=False,
        enable_pendulum=False,
        enable_timepix=False,
        enable_magnetic_gripper=False,
        enable_scanse_sweep=False,
        enable_rplidar=False,
        enable_teraranger_tower_evo=False,
        enable_velodyne=False,
        enable_ouster=False,
        ouster_model="OS1-16",
        use_gpu_ray=False,
        enable_ball_holder=False,
        enable_thermal_camera=False,
        enable_water_gun=False,
        enable_parachute=False,
        enable_light=False,
        enable_servo_camera=False,
        wall_challenge=False,
        fire_challenge=False,
        fire_challenge_blanket=False,
        gps_indoor_jamming = False,

        enable_uv_leds=False,
        uvled_fr_l=6,
        uvled_fr_r=15,
        enable_uv_leds_beacon=False,
        uvled_beacon_f=30,
        enable_uv_camera=False, 
        uvcam_calib_file="~/calib_results.txt",
        debug=False
    ):
    x, y, z, yaw = pose

    if ros_master_uri:
        original_uri = os.environ[ROS_MASTER_URI]
        os.environ[ROS_MASTER_URI] = ros_master_uri

    model_pathname, model_xml = get_model_xacro_file(vehicle_type)

    if enable_uv_camera or enable_uv_leds or enable_uv_leds_beacon:
        check_for_uvdar_package()

    kwargs = {
        'mappings': {
            'mavlink_udp_port': str(udp_port),
            'mavlink_tcp_port': str(tcp_port),
        },
    }
    
    # some default args for simulation (see sitl_gazebo cmake file for list)
    kwargs['mappings']['namespace'] = "uav" + str(mav_sys_id % 100) 
    kwargs['mappings']['enable_ground_truth'] = "true" if enable_ground_truth else "false"
    kwargs['mappings']['enable_bluefox_camera'] = "true" if enable_bluefox_camera else "false"
    kwargs['mappings']['enable_bluefox_camera_reverse'] = "true" if enable_bluefox_camera_reverse else "false"
    kwargs['mappings']['enable_mobius_camera_down'] = "true" if enable_mobius_camera_down else "false"
    kwargs['mappings']['enable_mobius_camera_front'] = "true" if enable_mobius_camera_front else "false"
    kwargs['mappings']['enable_mobius_camera_back_left'] = "true" if enable_mobius_camera_back_left else "false"
    kwargs['mappings']['enable_mobius_camera_back_right'] = "true" if enable_mobius_camera_back_right else "false"
    kwargs['mappings']['enable_realsense_front_pitched'] = "true" if enable_realsense_front_pitched else "false"
    kwargs['mappings']['enable_realsense_down'] = "true" if enable_realsense_down else "false"
    kwargs['mappings']['enable_realsense_top'] = "true" if enable_realsense_top else "false"
    kwargs['mappings']['enable_realsense_front'] = "true" if enable_realsense_front else "false"
    kwargs['mappings']['use_realistic_realsense'] = "true" if use_realistic_realsense else "false"
    kwargs['mappings']['enable_rangefinder'] = "true" if enable_rangefinder else "false"
    kwargs['mappings']['enable_teraranger'] = "true" if enable_teraranger else "false"
    kwargs['mappings']['enable_rangefinder_up'] = "true" if enable_rangefinder_up else "false"
    kwargs['mappings']['enable_whycon_box'] = "true" if enable_whycon_box else "false"
    kwargs['mappings']['enable_magnetic_gripper'] = "true" if enable_magnetic_gripper else "false"
    kwargs['mappings']['enable_scanse_sweep'] = "true" if enable_scanse_sweep else "false"
    kwargs['mappings']['enable_rplidar'] = "true" if enable_rplidar else "false"
    kwargs['mappings']['enable_teraranger_tower_evo'] = "true" if enable_teraranger_tower_evo else "false"
    kwargs['mappings']['enable_pendulum'] = "true" if enable_pendulum else "false"
    kwargs['mappings']['enable_timepix'] = "true" if enable_timepix else "false"
    kwargs['mappings']['enable_velodyne'] = "true" if enable_velodyne else "false"
    kwargs['mappings']['enable_ouster'] = "true" if enable_ouster else "false"
    kwargs['mappings']['ouster_model'] = ouster_model
    kwargs['mappings']['use_gpu_ray'] = "true" if use_gpu_ray else "false"
    kwargs['mappings']['enable_ball_holder'] = "true" if enable_ball_holder else "false"
    kwargs['mappings']['enable_water_gun'] = "true" if enable_water_gun and not fire_challenge else "false"
    kwargs['mappings']['enable_parachute'] = "true" if enable_parachute else "false"
    kwargs['mappings']['enable_light'] = "true" if enable_light else "false"
    kwargs['mappings']['enable_servo_camera'] = "true" if enable_servo_camera else "false"
    kwargs['mappings']['enable_thermal_camera'] = "true" if enable_thermal_camera else "false"
    kwargs['mappings']['wall_challenge'] = "true" if wall_challenge else "false"
    kwargs['mappings']['fire_challenge'] = "true" if fire_challenge else "false"
    kwargs['mappings']['fire_challenge_blanket'] = "true" if fire_challenge_blanket else "false"
    kwargs['mappings']['gps_indoor_jamming'] = "true" if gps_indoor_jamming else "false"

    kwargs['mappings']['enable_uv_leds'] = "true" if enable_uv_leds else "false"
    kwargs['mappings']['uvled_fr_l'] = uvled_fr_l
    kwargs['mappings']['uvled_fr_r'] = uvled_fr_r

    kwargs['mappings']['enable_uv_leds_beacon'] = "true" if enable_uv_leds_beacon else "false"
    kwargs['mappings']['uvled_beacon_f'] = uvled_beacon_f

    kwargs['mappings']['enable_uv_camera'] = "true" if enable_uv_camera else "false"
    kwargs['mappings']['uvcam_calib_file'] = uvcam_calib_file

    # ros commandline arguments
    kwargs['mappings']['mrs_robots_description_dir'] = model_pathname
    # additional xacro params for uctf
    if mavlink_address:
        kwargs['mappings']['mavlink_addr'] = mavlink_address
    
    if not detect_unused_arguments_in_xacro(model_xml, vehicle_type, **kwargs):
         print_error("===================================================================================")
         print_error("   UAV cannot be spawned with these settings !!!!")
         print_error("   Please check possible settings for this type of UAV by calling command:")
         print_error("           spawn --%s --available-sensors" % vehicle_type)
         print_error("===================================================================================")
         sys.exit(1)


    model_xml = parse_xacro(model_xml, **kwargs)

    if debug:
        print(model_xml)

    req = SpawnModelRequest()
    unique_name = 'uav' + str(mav_sys_id)
    req.model_name = unique_name
    req.model_xml = model_xml
    req.robot_namespace = unique_name
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    req.initial_pose.position.z = z
    req.initial_pose.orientation.x = 0.0
    req.initial_pose.orientation.y = 0.0
    req.initial_pose.orientation.z = math.sin(yaw / 2.0)
    req.initial_pose.orientation.w = math.cos(yaw / 2.0)
    req.reference_frame = ''

    try:
        srv = ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = srv(req)
    except ServiceException, e:
         print_error("===================================================================================")
         print_error("   Service call failed: %s" % e)
         print_error("   Please run gazebo_ros (for example: \"roslaunch mrs_simulation simulation.launch\")")
         print_error("===================================================================================")
         sys.exit(1)

    if ros_master_uri:
        os.environ[ROS_MASTER_URI] = original_uri

    if resp.success:
        msg = resp.status_message + ' (%s)' % unique_name;
        print_info(msg)
        return 0
    else:
        print_error(resp.status_message)
        return 1

def detect_available_arguments_in_xacro(template_xml):
    # find all expression that match "$(arg enable...."
    matches = re.findall(r"(?<=\$\(arg )(enable\w+)", template_xml)
    unigue_matches = list(set(matches))

    return sorted(unigue_matches)

def detect_unused_arguments_in_xacro(template_xml, vehicle_type, mappings=None, **kwargs):
    all_arguments_available = True
    # evaluate the attributes
    for key, value in mappings.items():
        if "enable" in key:
            try:
                eval_value = xacro.get_boolean_value(value, None) 
            except xacro.XacroException, e:
                continue
            if eval_value:
                searchtext = "$(arg {})".format(key)

                if searchtext not in template_xml:
                    print_error("Setting --{} is not specified for {}".format(key, vehicle_type))
                    all_arguments_available = False
    return all_arguments_available


def parse_xacro(template_xml, **kwargs):
    doc = xacro.parse(template_xml)
    xacro.process_doc(doc, **kwargs)
    xml = doc.toprettyxml(indent='  ')
    xml = xml.replace(' xmlns:xacro="http://ros.org/wiki/xacro"', '', 1)
    return xml


def generate_launch_file(
    mav_sys_id, vehicle_type, baseport, config_path
):
    launch_snippet = get_launch_snippet(
        mav_sys_id, vehicle_type, baseport, config_path)
    return write_launch_file(launch_snippet)


def get_launch_snippet(
    mav_sys_id, vehicle_type, vehicle_base_port
):
    data = {
        'ros_interface_port3': vehicle_base_port + 2,
        'ros_interface_port4': vehicle_base_port + 3,
        'vehicle_type': vehicle_type,
        'mav_sys_id': mav_sys_id,
    }
    return empy('px4_and_mavros.launch.em', data)


def write_launch_file(launch_snippet):
    fd, path = tempfile.mkstemp(prefix='simulation_', suffix='.launch')
    with os.fdopen(fd, 'w') as h:
        h.write(launch_snippet)
    return path


def empy(template_name, data, options=None):
    template_path = os.path.join(
        os.path.dirname(__file__), 'templates', template_name)
    output = StringIO()
    try:
        interpreter = Interpreter(output=output, options=options)
        with open(template_path, 'r') as h:
            content = h.read()
        interpreter.string(content, template_path, locals=data)
        value = output.getvalue()
        return value
    except Exception as e:
        print("%s processing template '%s'" %
              (e.__class__.__name__, template_name), file=sys.stderr)
        raise
    finally:
        interpreter.shutdown()


def spawn_launch_file(launch_file):
    print('roslaunch %s' % launch_file)


def delete_model(mav_sys_id, vehicle_type, ros_master_uri=None):
    if ros_master_uri:
        original_uri = os.environ[ROS_MASTER_URI]
        os.environ[ROS_MASTER_URI] = ros_master_uri
    try:
        srv = ServiceProxy('/gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        unique_name = 'uav' + str(mav_sys_id)
        req.model_name = unique_name
        resp = srv(req)
    except ServiceException, e:
         print_error("===================================================================================")
         print_error("   Service call failed: %s" % e)
         print_error("===================================================================================")
         sys.exit(2)


    if ros_master_uri:
        os.environ[ROS_MASTER_URI] = original_uri

    if resp.success:
        msg = resp.status_message + ' (%s)' % unique_name
        print_info(msg)
        return 0
    else:
        print_error(resp.status_message)
        return 1

