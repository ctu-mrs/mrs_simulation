import os
import signal
from subprocess import Popen, PIPE

class DroneSoftware:

    def __init__(self, px4_pid, mavros_pid):
        self.px4_pid = px4_pid
        self.mavros_pid = mavros_pid


    def kill(self):
        print('Killing mavros')
        try:
            os.kill(self.mavros_pid, signal.SIGKILL)
        except:
            print('Process not found!')
        print('Killing px4 firmware')
        try:
            os.kill(self.px4_pid, signal.SIGKILL)
        except:
            print('Process not found!')
