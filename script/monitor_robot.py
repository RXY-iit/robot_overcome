import os
import time
import subprocess
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from topic_tools.srv import MuxSelect
import sys
from openai_test.srv import SelectTopic, SelectTopicResponse

def start_roslaunch():
    """Start the ROS launch with the necessary nodes."""
    os.chdir('/home/ruan-x/midTask')
    return subprocess.Popen(["roslaunch", "openai_test", "dynamic_openai_test.launch"])

def velocity_callback(msg):
    """Callback to get velocity updates."""
    global last_velocity_time, has_stopped, avoidance_node, normal_control
    if avoidance_node is None:  # Only monitor movement if avoidance node is not running
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            if normal_control and not has_stopped:
                # last_velocity_time = time.time()
                print("Time count started")
                has_stopped = True
        else:
            last_velocity_time = time.time()
            has_stopped = False

def check_robot_movement(timeout=20):
    """Check if the robot has been stationary for more than the specified timeout."""
    if last_velocity_time and (time.time() - last_velocity_time) > timeout:
        return True
    return False

def print_time_count():
    """Print the current time count since last recorded movement."""
    if last_velocity_time:
        current_time = time.time()
        elapsed_time = current_time - last_velocity_time
        print(f"Current time count since last movement: {elapsed_time:.2f} seconds")

def write_avoidance_script():
    """Dynamically write the obstacle avoidance code to a new Python file."""
    avoidance_code = """
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

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
        twist.angular.z = 0.2
        if min(scan_data.ranges) >= 0.5:
            no_obstacle_detected = True

    twist_pub.publish(twist)

def main():
    global twist_pub
    global no_obstacle_detected

    rospy.init_node('avoid_obstacle')

    # Initialize the publisher here
    twist_pub = rospy.Publisher('/cmd_vel_source_avoidance', Twist, queue_size=10)
    
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    rate = rospy.Rate(10)  # 10 Hz loop rate
    while not rospy.is_shutdown():
        if no_obstacle_detected:  # Terminate if nothing is within 50 cm
            rospy.signal_shutdown("No obstacle within 50 cm. Avoidance complete.")
        rate.sleep()

if __name__ == '__main__':
    main()
    """

    with open('/home/ruan-x/midTask/src/openai_test/script/avoid_obstacle.py', 'w') as f:
        f.write(avoidance_code)

def run_avoidance_node():
    """Run the dynamically written obstacle avoidance node."""
    return subprocess.Popen(["python3", "/home/ruan-x/midTask/src/openai_test/script/avoid_obstacle.py"])

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
    global last_velocity_time, has_stopped, avoidance_node, normal_control, firsttime
    last_velocity_time = None  # Initialize the global variable
    has_stopped = False  # Initialize the global variable
    avoidance_node = None  # Initialize the flag for the avoidance node
    normal_control = True  # Initialize the flag for normal control
    firsttime = True  # Flag to check if avoidance script has been written

    # Start the ROS launch process
    launch_process = start_roslaunch()

    # Initialize ROS node
    rospy.init_node('velocity_monitor')
    rospy.Subscriber('/ypspur_ros/cmd_vel', Twist, velocity_callback)

    while not rospy.is_shutdown():
        if check_robot_movement():  # Only check if no avoidance process is running
            if avoidance_node is None:
                print("Robot stopped for more than 20seconds. Writing and running obstacle avoidance script...")
                if firsttime:
                    write_avoidance_script()
                    print("write AI code finished")
                    firsttime = False
                avoidance_node = run_avoidance_node()

                # add circle here to confirm and regenearate new code

                
                print("run new code finished")

                # Switch to obstacle avoidance control
                switch_control('/cmd_vel_source_avoidance')
                print("switch cmd_vel control finished")
                normal_control = False
        
        # Check if the avoidance node is still running
        # if not normal_control:
        #     if avoidance_node and avoidance_node.poll() is not None:  # Check if the node has finished
        #         print("Avoidance finished. Switching control back to normal.")
        #         avoidance_node = None  # Reset to allow future executions

        #         # Switch back to the normal run control
        #         switch_control('/cmd_vel_source_normal')
        #         normal_control = True
        
        # Print the current time count
        print_time_count()

        time.sleep(1)  # Check every second for robot movement

if __name__ == "__main__":
    main()


# timeout
# not failure of state but timeout

# ChatGPT O1-preview
# 17 Tuesday 18:30