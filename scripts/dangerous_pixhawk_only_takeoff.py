import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import SetMode
from geographic_msgs.msg import GeoPoseStamped

NAV_WAYPOINT = 16
NAV_LOITER_UNLIM = 17 

class PixhawkTakeoff:
    
    def callback_global_pos(self, msg):
        if not self.ready:
            return
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        rospy.loginfo_once('Got GPS position')
        self.got_position = True

    def call_takeoff(self):
        rospy.loginfo('Calling takeoff...')
        latitude = self.latitude
        longitude = self.longitude
        altitude = self.takeoff_height
        min_pitch = 0.1
        yaw = 0
        resp = self.service_takeoff.call(min_pitch, yaw, latitude, longitude, altitude)

    def publish_setpoint(self, latitude, longitude, altitude):
        msg = GeoPoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.latitude = self.latitude
        msg.pose.position.longitude = self.longitude
        msg.pose.position.altitude = self.takeoff_height
        self.publisher_setpoint.publish(msg)

    def switch_to_offboard(self):
        rospy.loginfo('Switching to offboard mode')
        msg = SetMode()
        self.service_set_mode.call(5, 'OFFBOARD')
        rospy.loginfo('Takeoff')


    def call_arming(self):
        rospy.loginfo('Arming...')
        resp = self.service_arming.call(True)

    def __init__(self):
        self.ready = False
        self.got_position = False
        self.success = False

        rospy.init_node('dangerous_takeoff', anonymous=True)

        self.takeoff_height = rospy.get_param('~takeoff_height')

        self.subscriber_pos = rospy.Subscriber('~global_pos_in', NavSatFix, self.callback_global_pos)
        self.publisher_setpoint = rospy.Publisher('~global_setpoint_out', GeoPoseStamped)

        self.service_arming = rospy.ServiceProxy('~arming_out', CommandBool)
        self.service_set_mode = rospy.ServiceProxy('~mode_out', SetMode)
        
        self.ready = True

        while not self.got_position:
            rospy.sleep(0.1)
            rospy.loginfo_once('Waiting for GPS position...')
        rospy.sleep(0.1)
        self.call_arming()

        i = 0
        while not rospy.is_shutdown():
            rospy.loginfo_once('Begin setpoint stream @ 20 Hz...')
            if i < 100:
                i+=1
            if i == 100:
                i+=1
                self.switch_to_offboard()
            self.publish_setpoint(self.latitude, self.longitude, self.takeoff_height)
            rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        takeoff_node = PixhawkTakeoff()
    except rospy.ROSInterruptException:
        pass
