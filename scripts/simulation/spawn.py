import argparse
import subprocess
import sys

from simulation import get_launch_snippet
from simulation import get_vehicle_pose
from simulation import get_vehicle_pose_from_file
from simulation import delete_model
from simulation import spawn_model
from simulation import VEHICLE_BASE_PORT, VEHICLE_TCP_BASE_PORT
from simulation import write_launch_file
from simulation import print_ok
from simulation import detect_available_arguments_in_xacro 
from simulation import get_model_xacro_file 


def vehicle_id_type(value):
    """Validate the vehicle_id_type from string to int."""
    value = int(value)
    if value < 0 or value > 250:
        raise argparse.ArgumentTypeError('Vehicle id must be in [1, 250]')
    return value

def spawn():
    parser = argparse.ArgumentParser(description = 'Spawn vehicles into gazebo simulation.', 
                                     epilog = """Not all sensors have to be available for selected type of uav.
                                     Please check possible settings for the selected type of UAV by calling command: spawn --$UAV_TYPE --available_sensors""",
                                     formatter_class=lambda prog: argparse.HelpFormatter(prog,max_help_position=27, width= 100))
    parser.add_argument(
        'vehicle_id', nargs='*', metavar='VEHICLE_ID', type=vehicle_id_type,
        default=(1,),
        help='The id of a vehicle to spawn, when the load file is not specified (default: 1)')
    type_group = parser.add_mutually_exclusive_group()
    type_group.add_argument(
        '--f450', action = 'store_true',
        help='Change the spawned vehicle to DJI F450 (default is DJI F550).'
    )
    type_group.add_argument(
        '--f550',action = 'store_true',
        help='Force the spawned vehicle to DJI F550.'
    )
    type_group.add_argument(
        '--t650', action = 'store_true',
        help='Change the spawned vehicle to Tarot 650 (default is DJI F550)'
    )
    type_group.add_argument(
        '--eaglemk2', action = 'store_true',
        help='Change the spawned vehicle to Eagle.One Mk2 (default is DJI F550).'
    )
    parser.add_argument(
        '--available-sensors', action = 'store_true',
        help='Display all available sensor equipment that is possible to use for selected type of uav.'
    )
    parser.add_argument(
        '--enable-ground-truth', action = 'store_true',
        help='Enable topic with ground true odometry (default: false)')
    parser.add_argument(
        '--run', action='store_true',
        help='run generated launch file (default: false)')
    parser.add_argument(
        '--delete', action='store_true',
        help='Despawn when killed (default: false)')
    parser.add_argument(
        '--file',
        help='Load positions and ids from file (format: [id, x, y, z, heading])')
    bluefox_down_group = parser.add_mutually_exclusive_group()
    bluefox_down_group.add_argument(
        '--enable-bluefox-camera', action ='store_true',
        help='Add bluefox camera into vehicle [752x480 50hz] (default: false)')
    bluefox_down_group.add_argument(
        '--enable-bluefox-camera-reverse', action ='store_true',
        help='Add rotated bluefox camera into vehicle [752x480 50hz] (default: false)')
    parser.add_argument(
        '--enable-whycon-box', action ='store_true',
        help='Add whycon box for relative localization (default: false)')
    parser.add_argument(
        '--enable-mobius-camera-down', action = 'store_true',
        help='Add mobius camera to the vehicle [1280x720 30hz], pointed to ground (default: false)')
    parser.add_argument(
        '--enable-mobius-camera-front', action = 'store_true',
        help='Add mobius camera to the vehicle [1280x720 30hz], pointed to the front (default: false)')
    parser.add_argument(
        '--enable-mobius-camera-back-left', action = 'store_true',
        help='Add mobius camera to the vehicle [1280x720 30hz], pointed to the back left (default: false)')
    parser.add_argument(
        '--enable-mobius-camera-back-right', action = 'store_true',
        help='Add mobius camera to the vehicle [1280x720 30hz], pointed to the back right (default: false)')
    realsense_group = parser.add_mutually_exclusive_group()
    realsense_group.add_argument(
        '--enable-realsense-front-pitched', action = 'store_true',
        help='Add Intel Realsense D435 depth camera to the vehicle [1280x720 30hz], pointed to the front pitched down by 10 up to 45 deg. Depends on the type of vehicle (default: false)')
    realsense_group.add_argument(
        '--enable-realsense-down', action = 'store_true',
        help='Add Intel Realsense D435 depth camera to the vehicle [1280x720 30hz], pointed down (default: false)')
    realsense_group.add_argument(
        '--enable-realsense-top', action = 'store_true',
        help='Add Intel Realsense D435 depth camera to the vehicle [1280x720 30hz], pointed forward placed on the aluminum frame (default: false)')
    realsense_group.add_argument(
        '--enable-realsense-front', action = 'store_true',
        help='Add Intel Realsense D435 depth camera to the vehicle [1280x720 30hz], pointed forward placed on the front holder between the legs (default: false)')
    parser.add_argument(
        '--use-realistic-realsense', action = 'store_true',
        help='Enable a more realistic simulation of the Realsense D435 dept image (default: false, only takes effect if some Realsense is enabled)')
    parser.add_argument(
        '--enable-rangefinder', action = 'store_true',
        help='Add rangefinder into vehicle (default: false)')
    parser.add_argument(
        '--enable-teraranger', action = 'store_true',
        help='Add teraranger rangefinder into vehicle (default: false)')
    parser.add_argument(
        '--enable-rangefinder-up', action = 'store_true',
        help='Add rangefinder measuring upwards onto vehicle (default: false)')
    parser.add_argument(
        '--enable-gripper', action = 'store_true',
        help='Add magnetic gripper (default: false)')
    lidar_group = parser.add_mutually_exclusive_group()
    lidar_group.add_argument(
        '--enable-scanse', action = 'store_true',
        help='Add the Scanse Sweep laser scanner to the vehicle (default: false)')
    lidar_group.add_argument(
        '--enable-rplidar', action = 'store_true',
        help='Add the RPlidar A3 laser scanner to the vehicle (default: false)')
    lidar_group.add_argument(
        '--enable-teraranger-tower-evo', action = 'store_true',
        help='Add the Teraranger Tower Evo laser scanner to the vehicle (default: false)')
    lidar_group.add_argument(
        '--enable-velodyne', action = 'store_true',
        help='Add the Velodyne PUCK Lite laser scanner to the vehicle (default: false)')
    lidar_group.add_argument(
        '--enable-ouster', action = 'store_true',
        help='Add the Ouster laser scanner to the vehicle (default: false)')
    parser.add_argument(
        '--ouster-model', nargs=1, type=str, 
        default=('OS1-16'), 
        choices =['OS0-32', 'OS0-64', 'OS0-128', 'OS1-16', 'OS1-32', 'OS1-64', 'OS1-64', 'OS2-32', 'OS2-64', 'OS2-128'], 
        help='Choose the Ouster model (default: %(default)s)')
    parser.add_argument(
        '--use-gpu-ray', action = 'store_true',
        help='Laser ray casting will be handled by GPU shaders (default: false)')
    parser.add_argument(
        '--enable-timepix', action = 'store_true',
        help='Add Timepix detector to the drone (default: false)')
    parser.add_argument(
        '--enable-thermal-camera', action = 'store_true',
        help='Add thermal camera to the drone (default: false)')
    parser.add_argument(
        '--enable-ball-holder', action = 'store_true',
        help='Add the ball holder to the vehicle (default: false)')
    parser.add_argument(
        '--enable-light', action = 'store_true',
        help='Add the light to the vehicle (default: false)')
    parser.add_argument(
        '--enable-servo-camera', action = 'store_true',
        help='Add the camera on virtual servo to the vehicle (default: false)')
    parser.add_argument(
        '--enable-water-gun', action = 'store_true',
        help='Add water gun for fire fighting (default: false)')
    parser.add_argument(
        '--enable-parachute', action = 'store_true',
        help='Add a parachute for emergency flight termination (default: false)')
    parser.add_argument(
        '--gps-indoor-jamming', action = 'store_true',
        help='Enable gps jamming when drone is indoors (default:false)')
    parser.add_argument(
        '--enable-pendulum', action = 'store_true',
        help='Add pendulum to the drone (default: false)')
    parser.add_argument(
        '--enable-uv-leds', action = 'store_true',
        help='Add UV LEDs on the vehicle (default: false)')
    parser.add_argument(
        '--led-frequencies', nargs=2, type=int, default=(6,15),
        help='Specify UV LEDs frequencies (default: (6,15))')
    parser.add_argument(
        '--enable-uv-leds-beacon', action = 'store_true',
        help='Add UV LED beacon on the top of the vehicle (default: false)')
    parser.add_argument(
        '--beacon-frequency', nargs=1, type=int, default=[30],
        help='The frequency of blinking of the UV Beacon')
    parser.add_argument(
        '--enable-uv-camera', action = 'store_true',
        help='Add UV camera on the vehicle (default: false)')
    parser.add_argument(
        '--uv-camera-calibration-file', nargs=1, type=str, default=("~/calib_results.txt"),
        help='Specify UV camera calibration different than default one')
    mbzirc_group = parser.add_mutually_exclusive_group()
    mbzirc_group.add_argument(
        '--wall-challenge', action = 'store_true',
        help='Configuration for the MBZIRC wall challenge (T650 with 2x bluefox down, realsense down pitched, rangefinder down) (default: false)')
    mbzirc_group.add_argument(
        '--fire-challenge', action = 'store_true',
        help='Configuration for the MBZIRC fire challenge (T650 with bluefox down, realsense forward, rp lidar, 3x thermal camera, water gun) (default: false)')
    mbzirc_group.add_argument(
        '--fire-challenge-blanket', action = 'store_true',
        help='Configuration for the MBZIRC fire challenge (blanket dropping) (T650 with bluefox down, realsense forward, rp lidar, 2x thermal camera, maybe blanket dropper) (default: false)')
    parser.add_argument(
        '--gazebo-ros-master-uri',
        help='The uri used to spawn the models')
    parser.add_argument(
        '--mavlink-address',
        help='The IP address for mavlink (default: INADDR_ANY)')
    parser.add_argument(
        '--generate-launch-file', action='store_false',
        help='Generate launch file (default: true)')
    parser.add_argument('--debug', action='store_true',
        help='Print generated xml model file (default: false)')

    args = parser.parse_args()

    vehicle_id = args.vehicle_id

    n_vehicles = len(vehicle_id)
    # do not remove functionality (it may get fixed someday), just clip the nubmer of vehicles to 1
    if (n_vehicles > 1):
        print('\033[31;1m' + '[WARN]: Spawning multiple vehicles in one command is currently not supported.\n[WARN]: Only the first vehicle will spawn.\n' + '\033[32;1m' + '[HINT]: Create a separate tmux pane (ctrl+s) and call the script again to spawn more vehicles' + '\x1b[0m')
        n_vehicles = 1
    
    if args.f450:
        vehicle_type='f450'
    elif args.f550:
        vehicle_type='f550'
    elif args.t650:
        vehicle_type='t650'
    elif args.eaglemk2:
        vehicle_type='eaglemk2'
    else:
        vehicle_type='f550'

    if args.wall_challenge or args.fire_challenge or args.fire_challenge_blanket:
        vehicle_type='t650'

    if args.available_sensors:
        _, model_xml = get_model_xacro_file(vehicle_type)
        for setting in detect_available_arguments_in_xacro(model_xml):
            print('--{}'.format(setting.replace('_','-')))
        return

    if args.file:
        vehicle_id, poses = get_vehicle_pose_from_file(args.file, vehicle_id[0])
        n_vehicles = len(vehicle_id)

    ouster_model = args.ouster_model[0].upper()

    uvled_fr = args.led_frequencies
    uvled_beacon_fr = args.beacon_frequency
    uvcam_calib = args.uv_camera_calibration_file

    if args.enable_uv_camera:
        print("Calibration file parameter is %s" %uvcam_calib)
    
    launch_snippet = ''

    # spawn vehicles
    if n_vehicles == 1:
        print("Spawning vehicle!")
    else:
        print("Spawning %i vehicles!" %n_vehicles)

    for i in range(0,n_vehicles):

        mav_sys_id = vehicle_id[i]

        # each vehicle uses 4 ports
        # VEHICLE_BASE_PORT + MAV_SYS_IDs used for UDP communication
        # VEHICLE_TCP_PORT + MAV_SYS_IDs used for TCP communication
        # two are used between the px4 and and the mavros node ang QGC

        vehicle_base_port = VEHICLE_BASE_PORT + 4 * mav_sys_id
        vehicle_tcp_port = VEHICLE_TCP_BASE_PORT + mav_sys_id

        if args.file:
            vehicle_pose = poses[i]
        else:
            vehicle_pose = get_vehicle_pose(mav_sys_id)
        
        # spawn the vehicle model in gazebo
        spawn_model(
            mav_sys_id = mav_sys_id,
            vehicle_type = vehicle_type, 
            udp_port = vehicle_base_port, 
            tcp_port = vehicle_tcp_port,
            pose = vehicle_pose,
            ros_master_uri = args.gazebo_ros_master_uri,
            mavlink_address = args.mavlink_address,
            enable_rangefinder = args.enable_rangefinder,
            enable_teraranger = args.enable_teraranger,
            enable_rangefinder_up = args.enable_rangefinder_up,
            enable_ground_truth = args.enable_ground_truth,
            enable_mobius_camera_front = args.enable_mobius_camera_front,
            enable_mobius_camera_down = args.enable_mobius_camera_down,
            enable_mobius_camera_back_left = args.enable_mobius_camera_back_left,
            enable_mobius_camera_back_right = args.enable_mobius_camera_back_right,
            enable_realsense_front_pitched = args.enable_realsense_front_pitched,
            enable_realsense_down = args.enable_realsense_down,
            enable_realsense_top = args.enable_realsense_top,
            enable_realsense_front = args.enable_realsense_front,
            use_realistic_realsense = args.use_realistic_realsense,
            enable_whycon_box = args.enable_whycon_box,
            enable_bluefox_camera = args.enable_bluefox_camera,
            enable_bluefox_camera_reverse = args.enable_bluefox_camera_reverse,
            enable_magnetic_gripper = args.enable_gripper,
            enable_scanse_sweep = args.enable_scanse,
            enable_pendulum = args.enable_pendulum,
            enable_rplidar = args.enable_rplidar,
            enable_teraranger_tower_evo = args.enable_teraranger_tower_evo,
            enable_timepix = args.enable_timepix,
            enable_thermal_camera = args.enable_thermal_camera,
            enable_velodyne = args.enable_velodyne,
            enable_ouster = args.enable_ouster,
            ouster_model = ouster_model,
            use_gpu_ray = args.use_gpu_ray,
            enable_light = args.enable_light,
            enable_servo_camera = args.enable_servo_camera,
            enable_ball_holder = args.enable_ball_holder,
            enable_water_gun = args.enable_water_gun,
            enable_parachute = args.enable_parachute,
            wall_challenge = args.wall_challenge,
            fire_challenge = args.fire_challenge,
            fire_challenge_blanket = args.fire_challenge_blanket,
            gps_indoor_jamming = args.gps_indoor_jamming,
            enable_uv_leds = args.enable_uv_leds,
            uvled_fr_l = "{}".format(uvled_fr[0]),
            uvled_fr_r = "{}".format(uvled_fr[1]),
            enable_uv_leds_beacon = args.enable_uv_leds_beacon,
            uvled_beacon_f = "{}".format(uvled_beacon_fr[0]),
            enable_uv_camera = args.enable_uv_camera,
            uvcam_calib_file = uvcam_calib[0].strip(),
            debug=args.debug)

        if args.generate_launch_file or args.run:
            launch_snippet += get_launch_snippet(
                mav_sys_id, vehicle_type, vehicle_base_port)
    print("----------------------------------------------")

    if args.generate_launch_file or args.run:
        launch_path = write_launch_file(launch_snippet)
        cmd = ['roslaunch', launch_path]
        print_ok(' '.join(cmd))

    retcode = 0
    if args.run:
        try:
            retcode = subprocess.call(cmd)
        except KeyboardInterrupt:
            pass
    if args.delete:
        for i in range(0, n_vehicles):
            mav_sys_id = vehicle_id[i]
            delete_model(
                mav_sys_id,
                vehicle_type,
                ros_master_uri=args.gazebo_ros_master_uri)
    return retcode
