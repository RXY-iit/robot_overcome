import os
import time
import subprocess
import rospy
from geometry_msgs.msg import Twist

def start_roslaunch():
    """Start the ROS launch with the necessary nodes."""
    os.chdir('/home/ruan-x/midTask')
    return subprocess.Popen(["roslaunch", "openai_test", "openai_test.launch"])

def velocity_callback(msg):
    """Callback to get velocity updates."""
    global last_velocity_time, has_stopped, code_replaced
    if msg.linear.x == 0.0 and msg.angular.z == 0.0:
        if not has_stopped:
            last_velocity_time = time.time()
            has_stopped = True
            print("Time count started")
    else:
        has_stopped = False

def check_robot_movement(timeout=3):
    """Check if the robot has been stationary for more than the specified timeout."""
    if last_velocity_time and (time.time() - last_velocity_time) > timeout:
        return True
    return False

def replace_code(src, dst):
    """Replace the code in the project."""
    try:
        with open(src, 'r') as src_file:
            code = src_file.read()

        with open(dst, 'w') as dst_file:
            dst_file.write(code)
        print(f"Replaced code in {dst} with {src}")
    except FileNotFoundError as e:
        print(f"Error replacing code: {e}")

def execute_ros_project():
    """Execute the ROS project by rebuilding it."""
    try:
        # Define the base path of your catkin workspace
        base_path = "/home/ruan-x/midTask"
        
        # Change the current working directory to the base path
        os.chdir(base_path)
        
        # Run catkin_make in the correct directory
        print("Rebuilding the project using catkin_make...")
        subprocess.run(["catkin_make"], check=True)
        print("Rebuild successful.")
    except subprocess.CalledProcessError as e:
        print(f"Error in executing the project: {e}")
    except FileNotFoundError as e:
        print(f"File or directory not found: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def restart_ros_project(launch_process):
    """Restart the ROS project by killing and starting the launch file."""
    print("Terminating the current ROS launch process...")
    launch_process.terminate()  # Terminate the existing ROS launch
    launch_process.wait()  # Wait for the termination to complete
    
    time.sleep(1)  # Brief pause before restarting
    
    # Optional: Check and kill any lingering nodes with the same name
    try:
        nodes = subprocess.check_output(["rosnode", "list"]).decode().split()
        for node in nodes:
            if "openai_test" in node or "hokuyo" in node:
                subprocess.run(["rosnode", "kill", node])
    except subprocess.CalledProcessError as e:
        print(f"Error killing nodes: {e}")

    print("Restarting the ROS launch process...")
    return start_roslaunch()  # Start the launch again

def main():
    global last_velocity_time, has_stopped, code_replaced
    last_velocity_time = None  # Initialize the global variable
    has_stopped = False  # Initialize the global variable
    code_replaced = False  # Initialize the flag for code replacement

    src_code_path = "/home/ruan-x/midTask/src/openai_test/src/openai_task_2.cpp"  # Replace with actual path
    dst_code_path = "/home/ruan-x/midTask/src/openai_test/src/openai_task.cpp"  # Replace with actual path

    # Start the ROS launch process
    launch_process = start_roslaunch()

    # Initialize ROS node
    rospy.init_node('velocity_monitor')
    rospy.Subscriber('/ypspur_ros/cmd_vel', Twist, velocity_callback)

    while not rospy.is_shutdown():
        if check_robot_movement():
            if not code_replaced:
                print("Robot has been stationary for too long. Replacing code and rebuilding ROS project...")
                replace_code(src_code_path, dst_code_path)
                execute_ros_project()
                code_replaced = True
                # Optionally restart the ROS project here if necessary
                launch_process = restart_ros_project(launch_process)
        else:
            # Reset the code replacement flag if needed
            if code_replaced:
                code_replaced = False

        time.sleep(1)  # Check every second for robot movement

if __name__ == "__main__":
    main()
