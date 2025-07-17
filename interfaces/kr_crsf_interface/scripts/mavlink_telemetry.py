import rospy
from pymavlink import mavutil
from sensor_msgs.msg import BatteryState

def mavlink_connect(port='/dev/ttyUSB0', baudrate=115200):
    """Connect to a MAVLink source"""
    return mavutil.mavlink_connection(port, baud=baudrate)

def battery_status_publisher():
    rospy.init_node('battery_status_publisher', anonymous=True)
    battery_pub = rospy.Publisher('battery_status', BatteryState, queue_size=10)
    rate = rospy.Rate(100)

    # Connect to MAVLink (adjust port and baudrate as necessary)
    mav = mavlink_connect(port='/dev/ttyUSB0', baudrate=115200)

    while not rospy.is_shutdown():
        message = mav.recv_match(blocking=True)
        if message:
            if message.get_type() == "SYS_STATUS":
                battery_msg = BatteryState()
                battery_msg.voltage = message.voltage_battery / 1000.0 / 4.0
                battery_msg.current = message.current_battery / 100.0
                battery_msg.percentage = message.battery_remaining
                
                battery_msg.header.stamp = rospy.Time.now()
                battery_pub.publish(battery_msg)
                
        rate.sleep()

if __name__ == '__main__':
    try:
        battery_status_publisher()
    except rospy.ROSInterruptException:
        pass
