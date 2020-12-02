#!/usr/bin/python3

import rospy
import os
import re
import roslaunch
import rospkg
import copy
import tempfile
import signal
import time

from mrs_msgs.srv import String as StringSrv
from mrs_msgs.srv import StringResponse as StringSrvResponse
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel

VEHICLE_BASE_PORT = 14000
MAVLINK_TCP_BASE_PORT = 4560
MAVLINK_UDP_BASE_PORT = 14560
LAUNCH_BASE_PORT = 14900
DEFAULT_VEHICLE_TYPE = 't650'
VEHICLE_TYPES = ['f450', 'f550', 't650', 'eaglemk2']
SPAWNING_DELAY_SECONDS = 6

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

def rinfo(message):
    rospy.loginfo('[DroneSpawner]: ' + message)

def rwarn(message):
    rospy.logwarn('[DroneSpawner]: ' + message)

def rerr(message):
    rospy.logerr('[DroneSpawner]: ' + message)


class MrsDroneSpawner:

    def __init__(self):
        rospy.init_node('mrs_drone_spawner', anonymous=True)
        self.model_params = rospy.get_param('~model_params')
        self.assigned_ids = {} # id: process_handle

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mrs_simulation')
        self.path_to_launch_file = pkg_path + os.sep + 'launch' + os.sep + 'parametrized_spawn.launch'

        rinfo('Loaded the following params:')
        for param, value in self.model_params.items():
            print('\t\t' + str(param) + ': ' + str(value))

        print('Launchfile: ' + self.path_to_launch_file)
        spawn_server = rospy.Service('~spawn', StringSrv, self.callback_spawn)
        delete_server = rospy.Service('~delete', StringSrv, self.callback_delete)
        self.delete_gazebo_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        rospy.spin()

    # #{ assign_free_id
    def assign_free_id(self):
        for i in range(0,251):
            if i not in self.assigned_ids.keys():
                return i
        raise Exception('Cannot assign a free ID to the vehicle!')
    # #}

    # #{ get_ids
    def get_ids(self, params_list):
        requested_ids = []

        # read params until non-numbers start comming
        for p in params_list:
            if p.isnumeric():
                requested_ids.append(int(p))
            else:
                break

        if len(requested_ids) < 1:
            free_id = self.assign_free_id()
            requested_ids.append(free_id)
            rwarn('Vehicle ID not specified. Number ' + str(free_id) + ' assigned automatically.')
            return requested_ids

        rinfo('Requested vehicle IDs: ' + str(requested_ids))

        ids = []
        # remove all IDs that are already assigned or out of range
        for ID in requested_ids:
            if ID > 249:
                rwarn('Cannot spawn uav' + str(ID) + ', ID out of range <0, 250>!')
                continue

            if ID in self.assigned_ids.keys():
                rwarn('Cannot spawn uav' + str(ID) + ', ID already assigned!')
                continue
            ids.append(ID)

        if len(ids) < 1:
            raise Exception('No valid ID provided')

        return ids
    # #}

    # #{ get_vehicle_type
    def get_vehicle_type(self, params_list):
        vehicle_type = DEFAULT_VEHICLE_TYPE
        for p in params_list:
            for v in VEHICLE_TYPES:
                if v in p:
                    vehicle_type = v
                    break
        return vehicle_type
    # #}

    # #{ get_params_dict
    def get_params_dict(self, params_list):
        params_dict = copy.deepcopy(self.model_params)
        custom_params = {}
        for i, p in enumerate(params_list):
            if '--' in p:
                param_name = p[2:]
                param_name = param_name.replace('-', '_')
                if param_name not in self.model_params.keys() and param_name not in VEHICLE_TYPES:
                    raise Exception('Param \'' + str(param_name) + '\' not recognized!')
                children = []
                for j in range(i+1, len(params_list)):
                    if '--' not in params_list[j]:
                        children.append(params_list[j])
                    else:
                        break
                if len(children) < 1:
                    custom_params[param_name] = True
                else:
                    custom_params[param_name] = children
        if len(custom_params.keys()) > 0:
            rinfo('Customized params:')
            for pname, pval in custom_params.items():
                print('\t' + str(pname) + ': ' + str(pval))
            params_dict.update(custom_params)
        return params_dict
    # #}

    # #{ get_comm_ports
    def get_comm_ports(self, ID):
        '''
        NOTE
        ports have to match with values assigned in
        mrs_simulation/ROMFS/px4fmu_common/init.d-posix/rcS
        '''
        ports = {}
        ports['udp_offboard_port_remote'] = VEHICLE_BASE_PORT + (4 * ID) + 2
        ports['udp_offboard_port_local'] = VEHICLE_BASE_PORT + (4 * ID) + 1
        ports['mavlink_tcp_port'] = MAVLINK_TCP_BASE_PORT + ID
        ports['mavlink_udp_port'] = MAVLINK_UDP_BASE_PORT + ID
        ports['fcu_url'] = 'udp://:' + str(ports['udp_offboard_port_remote']) + '@localhost:' + str(ports['udp_offboard_port_local'])
        return ports
    # #}

    # #{ parse_input_params
    def parse_input_params(self, data):
        params_list = data.split()

        try:
            uav_ids = self.get_ids(params_list)
            vehicle_type = self.get_vehicle_type(params_list)
            params_dict = self.get_params_dict(params_list)
        except Exception as e:
            rerr('Exception raised while parsing user input:')
            rerr(str(e.args[0]))
            raise Exception('Cannot spawn vehicle. Reason: ' + str(e.args[0]))

        for ID in uav_ids:
            self.assigned_ids[ID] = None

        rinfo('Spawning ' + str(len(uav_ids)) + ' vehicles of type \'' + vehicle_type + '\'')

        params_dict['uav_ids'] = uav_ids
        params_dict['vehicle_type'] = vehicle_type
        return params_dict
    # #}

    # #{ generate_launch_args
    def generate_launch_args(self, params_dict):

        args_sequences = []

        for n in range(len(params_dict['uav_ids'])):
            uav_args_sequence = []
            ID = params_dict['uav_ids'][n]
            # get vehicle ID number
            uav_args_sequence.append('ID:=' + str(ID))

            # get vehicle type
            uav_args_sequence.append('vehicle:=' + params_dict['vehicle_type'])

            # setup communication ports
            comm_ports = self.get_comm_ports(ID)
            for name,value in comm_ports.items():
                uav_args_sequence.append(str(name) + ':=' + str(value))

            # setup vehicle spawn pose
            uav_args_sequence.append('x:=' + str(ID))
            uav_args_sequence.append('y:=' + str(0))
            uav_args_sequence.append('z:=' + str(0.4))
            uav_args_sequence.append('heading:=' + str(0))

            # generate a yaml file for the custom model config
            fd, path = tempfile.mkstemp(prefix='simulation_', suffix='.yaml')
            with os.fdopen(fd, 'w') as f:
                for pname, pvalue in params_dict.items():
                    if 'enable' in pname or 'use' in pname:
                        f.write(str(pname) + ': ' + str(pvalue).lower() + '\n')
            uav_args_sequence.append('model_params_file:=' + path)


            print('UAV' + str(ID) + ' ARGS_SEQUENCE:')
            print(uav_args_sequence)
            args_sequences.append(uav_args_sequence)
        return args_sequences
    # #}

    # #{ callback_spawn
    def callback_spawn(self, req):
        params_dict = None
        try:
            params_dict = self.parse_input_params(req.value)
        except Exception as e:
            res = StringSrvResponse()
            res.success = False
            res.message = str(e.args[0])
            return res

        if params_dict is None:
            res = StringSrvResponse()
            res.success = False
            res.message = str('Cannot process input parameters')
            return res

        roslaunch_args = self.generate_launch_args(params_dict)

        orig_signal_handler = roslaunch.pmon._init_signal_handlers
        roslaunch.pmon._init_signal_handlers = self.dummy_function

        successful_spawns = 0
        unsuccessful_spawns = 0
        launched = []
        # iterate to get sequences for individual UAVs
        for i, uav_roslaunch_args in enumerate(roslaunch_args):
            ID = params_dict['uav_ids'][i]
            rinfo('Spawning uav' + str(ID) + '...')
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            rospy.loginfo('Service called, generated uuid:' + str(uuid))
            roslaunch_sequence = [(self.path_to_launch_file, uav_roslaunch_args)]
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_sequence)
            try:
                launch.start()
            except:
                rerr('Error occured while spawning uav' + str(ID) + '!')
                del self.assigned_ids[ID]
                unsuccessful_spawns += 1
                continue
            self.assigned_ids[ID] = launch
            time.sleep(SPAWNING_DELAY_SECONDS)
            successful_spawns += 1

        res = StringSrvResponse()
        res.success = unsuccessful_spawns == 0
        res.message = 'Spawned ' + str(successful_spawns) + ' vehicles'
        roslaunch.pmon._init_signal_handlers = orig_signal_handler
        return res
        # #}

    # #{ kill_plugins
    def kill_plugins(self, ID):
        if ID not in self.assigned_ids.keys():
            rerr('Cannot kill plugins of \'uav' + str(ID) + '\'. Vehicle not found')
            return False

        rinfo('Killing plugins of \'uav' + str(ID) + '\'...')
        try:
            result = self.assigned_ids[ID].shutdown()
        except:
            rerr('Cannot kill plugins of \'uav' + str(ID) + '\'')
            return False
        rinfo('Plugins for \'uav' + str(ID) + '\' killed')
        return True
    # #}

    # #{ delete_model
    def delete_model(self, ID):
        delete_result = self.delete_gazebo_proxy('uav' + str(ID))
        if not delete_result.success:
            rerr('Cannot delete model \'uav' + str(ID) + '\'. Reason: ' + str(delete_result.status_message))
        else:
            rinfo('Model \'uav' + str(ID) + '\' deleted')
        return delete_result.success
    # #}

    # #{ callback_delete
    def callback_delete(self, req):
        params_list = req.value.split()
        num_errors = 0
        ids_to_delete = []
        for p in params_list:
            if p.isnumeric():
                ids_to_delete.append(int(p))
            else:
                res = StringSrvResponse()
                res.success = False
                res.message = 'Invalid vehicle ID: ' + str(p)
                return res

        successfully_deleted = 0
        rinfo('Will delete these vehicles: ' + str(ids_to_delete))
        for ID in ids_to_delete:
            plugins_killed = self.kill_plugins(ID)
            model_deleted = self.delete_model(ID)
            if plugins_killed and model_deleted:
                successfully_deleted += 1
                del self.assigned_ids[ID]
        
        res = StringSrvResponse()
        res.success = successfully_deleted == len(ids_to_delete)
        res.message = str('Deleted ' + str(successfully_deleted) + ' vehicles')
        return res
    # #}

    def dummy_function(self):
        pass

if __name__ == '__main__':
    try:
        spawner = MrsDroneSpawner()
    except rospy.ROSInterruptException:
        pass
