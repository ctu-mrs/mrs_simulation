#!/usr/bin/python3

import rospy
import os
import re
import roslaunch
import rospkg
import copy
import tempfile

from mrs_msgs.srv import String as StringSrv
from mrs_msgs.srv import StringResponse as StringSrvResponse

VEHICLE_BASE_PORT = 14000
MAVLINK_TCP_BASE_PORT = 4560
MAVLINK_UDP_BASE_PORT = 14560
DEFAULT_VEHICLE_TYPE = 't650'

# #{
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
# #}


class MrsDroneSpawner:

    def __init__(self):
        rospy.init_node('mrs_drone_spawner', anonymous=True)
        self.model_params = rospy.get_param('~model_params')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mrs_simulation')
        self.path_to_launch_file = pkg_path + os.sep + 'launch' + os.sep + 'parametrized_spawn.launch'

        rospy.loginfo('[DroneSpawner]: Loaded the following params:')
        for param, value in self.model_params.items():
            print('\t\t' + str(param) + ': ' + str(value))
        
        print('Launchfile: ' + self.path_to_launch_file)
        s = rospy.Service('spawn', StringSrv, self.callback_spawn_drone)
        rospy.spin()
 
    def get_id(self, params_list):
        for p in params_list:
            if p.isnumeric():
                return int(p)

    def get_comm_ports(self, ID):
        ports = {}
        ports['udp_offboard_port_remote'] = VEHICLE_BASE_PORT + (4 * ID) + 2
        ports['udp_offboard_port_local'] = VEHICLE_BASE_PORT + (4 * ID) + 1
        ports['mavlink_tcp_port'] = MAVLINK_TCP_BASE_PORT + ID
        ports['mavlink_udp_port'] = MAVLINK_UDP_BASE_PORT + ID
        return ports

    def parse_params_sequence(self, data):
        sequence_elements = data.split()
        custom_model_params = copy.deepcopy(self.model_params)
        
        if '--f450' in sequence_elements:
            custom_model_params['vehicle_type'] = 'f450' 
        elif '--f550' in sequence_elements:
            custom_model_params['vehicle_type'] = 'f550' 
        elif '--eaglemk2' in sequence_elements:
            custom_model_params['vehicle_type'] = 'eaglemk2' 
        else:
            custom_model_params['vehicle_type'] = DEFAULT_VEHICLE_TYPE 

        for i, element in enumerate(sequence_elements):

            # # TODO:
            # NEW --run no longer necessary (will be automatic)
            # NEW --delete no longer necessary:
            #   1) can be called manually through service
            #   2) will be called automatically if this node is killed

            # update parsing procedure:
            # loop through sequence until '--' is found
            # everything without '--' should be id number
            # if no id number is provided, assign one (internally remember used isd)
            # DO NOT ALLOW creation of uav with already existing id

            if '--' in element:
                param_name = element[2:]
                param_name = param_name.replace('-', '_')
                if param_name not in custom_model_params.keys():
                    rospy.logwarn('Spawn param \'' + str(element) + '\' not recognized. Check mrs_simulation/config/model_params.yaml')
                children = []
                for j in range(i+1, len(sequence_elements)):
                    if '--' not in sequence_elements[j]:
                        children.append(sequence_elements)
                    else:
                        break
                if len(children) >= 1:
                    custom_model_params[param_name] = children
                else:
                    custom_model_params[param_name] = True
            elif element.isnumeric() and 'id' not in custom_model_params.keys():
                custom_model_params['id'] = int(element)

        print('PARAMS:')
        for pname, pvalue in custom_model_params.items():
            print(str(pname) + ': ' +  str(pvalue))
        return custom_model_params 

    def generate_launch_args(self, params_dict):
        args_sequence = []
        
        # get vehicle ID number
        args_sequence.append('ID:=' + str(params_dict['id']))

        # get vehicle type
        args_sequence.append('vehicle:=' + params_dict['vehicle_type'])

        # setup communication ports
        comm_ports = self.get_comm_ports(params_dict['id'])
        for name,value in comm_ports.items():
            args_sequence.append(str(name) + ':=' + str(value))

        # setup vehicle spawn pose
        args_sequence.append('x:=' + str(2 * params_dict['id']))
        args_sequence.append('y:=' + str(0))
        args_sequence.append('z:=' + str(0))
        args_sequence.append('heading:=' + str(0))

        # generate a yaml file for the custom model config
        fd, path = tempfile.mkstemp(prefix='simulation_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            for pname, pvalue in params_dict.items():
                if 'enable' in pname or 'use' in pname:
                    f.write(str(pname) + ': ' + str(pvalue).lower() + '\n')
        args_sequence.append('model_params_file:=' + path)


        # active_params = copy.deepcopy(self.model_params)

        # for p in params_list:
        #     if p[0:2] != '--' and p.isnumeric():
        #         active_params['ID'] = p
        #     if p[2:8] == 'enable' or p[2:6] == 'use':
        #         active_params[p] = True

        # for pname, value in active_params.items():
        #     if value in (True,False):
        #         args_sequence.append(str(pname) + ':=' + str(value).lower())
        #     else:
        #         args_sequence.append(str(pname) + ':=' + str(value))

        print('ARGS_SEQUENCE:')
        print(args_sequence)
        return args_sequence


    def callback_spawn_drone(self, req):
        orig_signal_handler = roslaunch.pmon._init_signal_handlers
        roslaunch.pmon._init_signal_handlers = self.dummy_function
        params = self.parse_params_sequence(req.value)

        roslaunch_args = self.generate_launch_args(params)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        rospy.loginfo('Service called, generated uuid:' + str(uuid))

        id_str = 'ID:=' + req.value
        id_str = str(id_str)
        print(id_str)

        roslaunch_sequence = [(self.path_to_launch_file, roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_sequence)
        launch.start()
        res = StringSrvResponse()
        res.success = True
        res.message = 'Success'
        roslaunch.pmon._init_signal_handlers = orig_signal_handler
        return res

    def dummy_function(self):
        pass

if __name__ == '__main__':
    try:
        spawner = MrsDroneSpawner()
    except rospy.ROSInterruptException:
        pass
