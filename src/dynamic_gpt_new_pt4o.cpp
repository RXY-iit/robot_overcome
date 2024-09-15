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
#include <chrono> // Added for time tracking

// Current position obtained from odometry
double robot_x, robot_y;
double roll, pitch, yaw;
bool get_obs_flag = false;  // Flag to get obstacle data
// Laser scan data
sensor_msgs::LaserScan scan;
// Odometry robot rotation data
geometry_msgs::Quaternion robot_r;
// Final goal data
geometry_msgs::PoseStamped goal;

// Velocity and angular velocity
geometry_msgs::Twist twist; 
double v0 = 0.2;  // Initial velocity
double w0 = 0.0;  // Initial angular velocity

// Time tracking variables for stop state timeout
auto stop_start_time = std::chrono::steady_clock::now();
bool in_stop_state = false;
bool obstacle_avoidance_mode = false;

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
    double k_eta = 0.09;
    double k_phai = 0.04;
    double k_w = 0.03;
    double v_max = 1.0;
    double w_max = 6.28;

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

    double phai = yaw - th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai += 2 * M_PI;
        else
            phai -= 2 * M_PI;
    }

    double w_diff = w0;
    double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w_diff);
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;

    double v = v0;
    if (v > v_max)
        v = v_max;
    else if (v < -v_max)
        v = -v_max;

    twist.linear.x = v;
    twist.angular.z = w;
    w0 = twist.angular.z;
}

int near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_x - goal.pose.position.x;
    double dify = robot_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.1);
}

// Function to avoid obstacles by moving sideways
void avoid_obstacle()
{
    twist.linear.x = 0.0;
    twist.angular.z = 0.5;  // Turn to avoid obstacle
    ROS_WARN("Avoiding obstacle...");
}

// Detect obstacles and decide whether to avoid or stop
void detect_and_avoid_obstacles()
{
    double min_distance = std::numeric_limits<double>::infinity();
    bool found_obstacle = false;

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double distance = scan.ranges[i];
        if (distance >= 0.2 && distance < 0.5)
        {
            found_obstacle = true;
            min_distance = std::min(min_distance, distance);
        }
    }

    if (found_obstacle)
    {
        obstacle_avoidance_mode = true;
        avoid_obstacle();
    }
    else
    {
        obstacle_avoidance_mode = false;
        ROS_WARN("obstacle avoided...");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openai_task_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_source_avoidance", 100);

    ros::Rate loop_rate(10);

    goal.pose.position.x = 5.0;
    goal.pose.position.y = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();
        geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

        detect_and_avoid_obstacles();

        if (!obstacle_avoidance_mode)
        {
            if (near_position(goal))
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                ROS_INFO("Arrived at goal.");
            }
            else
            {
                ROS_INFO("FOLLOWING line.");
                follow_line(0, 0, 0);
            }
        }
        
        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    return 0;
}
