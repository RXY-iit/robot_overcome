import sys
import rospy
# Ensure the path is added
sys.path.append('/home/ruan-x/midTask/devel/lib/python3/dist-packages')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# from std_srvs.srv import Empty, EmptyResponse  # Replace with your actual service if needed
from openai_test.srv import SelectTopic, SelectTopicResponse
# from openai_test.srv import SelectTopic
# from std_srvs.srv import SelectTopic, SelectTopicResponse 






# Global variable to track if no obstacle is detected
no_obstacle_detected = False

# Publisher initialization
twist_pub = None

def scan_callback(scan_data):
    global no_obstacle_detected, twist_pub
    twist = Twist()

    # Check if scan data is available
    if not scan_data.ranges:
        rospy.logwarn("Scan data is empty!")
        return

    # Initialize variables for obstacle detection
    min_distance_front = float('inf')
    found_obstacle = False

    num_ranges = len(scan_data.ranges)

    # Calculate the indices for the front 60 degrees of the scan
    front_min_index = num_ranges // 3
    front_max_index = num_ranges * 2 // 3

    # Clamp indices to valid range
    front_min_index = max(0, front_min_index)
    front_max_index = min(num_ranges - 1, front_max_index)

    # Scan through the range around the center
    for i in range(front_min_index, front_max_index + 1):
        distance = scan_data.ranges[i]
        if 0.2 <= distance < 0.5:
            if distance < min_distance_front:
                min_distance_front = distance
                found_obstacle = True

    # Obstacle avoidance logic based on the front range
    if found_obstacle and min_distance_front < 0.5:
        twist.linear.x = 0.0
        twist.angular.z = 0.2  # Rotate to avoid obstacle
        no_obstacle_detected = False
    else:
        twist.linear.x = 0.1  # Move forward if no obstacle within 50 cm
        twist.angular.z = 0.0
        if min(scan_data.ranges) >= 0.5:
            no_obstacle_detected = True

    twist_pub.publish(twist)


def switch_control(source_topic):
    """Call the service to switch the control source of /cmd_vel."""
    rospy.wait_for_service('/cmd_vel_mux/select')
    try:
        # Create a service proxy for the /cmd_vel_mux/select service
        select_srv = rospy.ServiceProxy('/cmd_vel_mux/select', SelectTopic)
        # Call the service with the desired topic
        resp = select_srv(source_topic)
        if resp.success:
            rospy.loginfo(f"Successfully switched to {source_topic}")
        else:
            rospy.logwarn(f"Failed to switch to {source_topic}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def main():
    global twist_pub
    global no_obstacle_detected

    rospy.init_node('avoid_obstacle')

    # Initialize the publisher here
    twist_pub = rospy.Publisher('/cmd_vel_source_avoidance', Twist, queue_size=10)
    
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Switch to obstacle avoidance control
    switch_control('/cmd_vel_source_avoidance')
    
    rate = rospy.Rate(10)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        if no_obstacle_detected:  # Terminate if nothing is within 50 cm
            rospy.signal_shutdown("No obstacle within 50 cm. Avoidance complete.")
        rate.sleep()

if __name__ == '__main__':
    main()
