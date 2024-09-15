#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>

// Current position obtained from odometry
double robot_x, robot_y;
double roll, pitch, yaw;
bool get_obs_flag = false;  // Flag to get obstacle data
// Laser scan data
sensor_msgs::LaserScan scan;
bool scan_received = false; // Flag to check if scan data is received
// Odometry robot rotation data
geometry_msgs::Quaternion robot_r;
// Final goal data
geometry_msgs::PoseStamped goal;
// Obstacle point
geometry_msgs::Point obstacle_point;

// Rotation
int flag = 0, count = 0;

geometry_msgs::Twist twist; // Commanded velocity and angular velocity
// Initial velocity
double v0 = 0.0;
// Initial angular velocity
double w0 = 0.0;

std::map<std::string, std::string> params_;

// Time tracking variables for stop state timeout
auto stop_start_time = std::chrono::steady_clock::now();
bool in_stop_state = false;

// Odometry callback
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;
}

// Laser scan callback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    scan = *scan_;
    scan_received = true;
}

// Convert Quaternion to Euler angles
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// Calculate the commanded velocity to follow a line and store it in twist
void follow_line(double x, double y, double th)
{
    // Gains for line following
    double k_eta = 0.09;
    double k_phai = 0.04;
    double k_w = 0.03;

    // Maximum velocity and angular velocity of the robot
    double v_max = 0.5;
    double w_max = 1.0;

    // Distance between the robot and the line (clipped if above a certain distance)
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(robot_x - x);
    else if (th == -M_PI / 2.0)
        eta = robot_x - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);

    if (eta > 0.4)
        eta = 0.4;
    else if (eta < -0.4)
        eta = -0.4;

    // Robot's orientation relative to the line (kept within -M_PI to M_PI)
    double phai = yaw - th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai += 2 * M_PI;
        else
            phai -= 2 * M_PI;
    }

    // Difference between the target angular velocity and current angular velocity
    double w_diff = w0;

    // Angular velocity
    double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w_diff);
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;

    // Translational velocity
    double v = v0;
    if (v > v_max)
        v = v_max;
    else if (v < -v_max)
        v = -v_max;

    twist.linear.x = v;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = w;

    // Use the current angular velocity for the next time step calculation
    w0 = twist.angular.z;
}

// Check if the robot is near the goal position
bool near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_x - goal.pose.position.x;
    double dify = robot_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.1);
}

// Obstacle avoidance behavior
void avoid_obstacle()
{
    // Parameters for obstacle avoidance
    double obstacle_distance_threshold = 0.5;
    double safe_distance = 0.6;
    double angular_speed = 0.5;
    double linear_speed = 0.1;

    // Find the direction with the maximum clearance
    int min_index = -1;
    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double distance = scan.ranges[i];
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    // Calculate the angle of the closest obstacle
    double angle_of_closest = scan.angle_min + min_index * scan.angle_increment;

    // Determine turning direction (turn away from obstacle)
    if (angle_of_closest >= 0)
    {
        // Obstacle is on the left, turn right
        twist.angular.z = -angular_speed;
    }
    else
    {
        // Obstacle is on the right, turn left
        twist.angular.z = angular_speed;
    }

    // Move forward slowly
    twist.linear.x = linear_speed;
}

// Update obstacle detection flag
void update_obstacle_flag()
{
    double obstacle_distance_threshold = 0.5;
    get_obs_flag = false;

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double distance = scan.ranges[i];
        if (distance >= 0.2 && distance < obstacle_distance_threshold)
        {
            get_obs_flag = true;
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_gpt_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_source_avoidance", 100);

    // Wait 2 seconds for sensors to initialize
    ros::Duration(2.0).sleep();
    ROS_INFO("Waited 2s.");

    robot_x = 0.0;
    robot_y = 0.0;
    robot_r.x = 0.0;
    robot_r.y = 0.0;
    robot_r.z = 0.0;
    robot_r.w = 1.0;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    ros::Rate loop_rate(50);

    // Velocity
    v0 = 0.2;
    // Current angular velocity
    w0 = twist.angular.z;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool obstacle_detected = false;
    bool first_get=true;

    // Final Goal
    goal.pose.position.x = 5.0;
    goal.pose.position.y = 0.0;

    auto last_move_log_time = std::chrono::steady_clock::now();

    while (ros::ok())
    {
        ros::spinOnce();

        if (!scan_received)
        {
            // Wait until scan data is received
            loop_rate.sleep();
            continue;
        }

        geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

        update_obstacle_flag();
        obstacle_detected = get_obs_flag;
        get_obs_flag = false;

        if (near_position(goal))
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            twist_pub.publish(twist);
            if(first_get){
                ROS_INFO("Arrived at goal.");
                first_get=false;
            }
            break; // Exit the loop
        }

        if (!obstacle_detected)
        {
            // When the robot starts to move after encountering an obstacle
            if (in_stop_state)
            {
                ROS_INFO("Obstacle avoided. Resuming navigation.");
                in_stop_state = false; // Reset stop state flag
            }

            // Log "robot moving" every 2 seconds while the robot is moving
            auto current_time = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_move_log_time).count() >= 2)
            {
                ROS_INFO("Robot moving.");
                last_move_log_time = current_time; // Update the last log time
            }

            x = goal.pose.position.x;
            y = goal.pose.position.y;
            theta = atan2(y - robot_y, x - robot_x);
            follow_line(x, y, theta);
        }
        else
        {
            // Log when the robot encounters an obstacle and starts avoidance
            if (!in_stop_state)
            {
                ROS_WARN("Obstacle detected. Initiating avoidance maneuver.");
                in_stop_state = true;
            }

            // Obstacle avoidance behavior
            avoid_obstacle();
        }

        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    ROS_INFO("Navigation completed.");
    return 0;
}
