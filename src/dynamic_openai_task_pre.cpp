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
// Obstacle point1, point2
geometry_msgs::Point point1, point2;

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
    float nandeyanen = scan_->range_max;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
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
    double k_eta = 900.00000000 / 10000;
    double k_phai = 400.00000000 / 10000;
    double k_w = 300.00000000 / 10000;

    // Maximum velocity and angular velocity of the robot
    double v_max = 1.00000000;
    double w_max = 6.28000000;

    // Distance between the robot and the line (actually clipped if above a certain distance)
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(robot_x - x);
    else if (th == -M_PI / 2.0)
        eta = robot_x - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * robot_x + robot_y - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    if (eta > 0.40000000)
        eta = 0.40000000;
    else if (eta < -0.40000000)
        eta = -0.40000000;

    // Robot's orientation relative to the line (kept within -M_PI to M_PI)
    double phai = yaw - th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai = phai + 2 * M_PI;
        else
            phai = phai - 2 * M_PI;
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

int near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_x - goal.pose.position.x;
    double dify = robot_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.1);
}

void rotation()
{
    // Calculate the current roll, pitch, yaw of the robot
    geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    ROS_INFO("rotate in");

    if (-0.1 < yaw && yaw < -0.01)
    {
        flag = 1;
    }
    else if (-0.01 <= yaw && flag == 1)
    {
        flag = 0;
        count++; // Number of rotations
        ROS_INFO("count++");
    }

    if (count >= 1)
    {
        twist.angular.z = 0.0;
    }
}

void get_obj_data()
{
    // Update objects obstacle data in the world
    double min_distance1 = std::numeric_limits<double>::infinity();
    double angle_of_closest1;
    bool found1 = false;

    for (int i = 0; i < scan.ranges.size(); ++i)
    {
        double distance1 = scan.ranges[i];
        if (distance1 >= 0.2 && distance1 < 0.5)
        {
            if (distance1 < min_distance1)
            {
                min_distance1 = distance1;
                angle_of_closest1 = scan.angle_min + i * scan.angle_increment;
                found1 = true;
            }
        }
    }

    if (found1)
    {
        double obstacle1_x = min_distance1 * cos(angle_of_closest1);
        double obstacle1_y = min_distance1 * sin(angle_of_closest1);

        double world_obstacle1_x = 0;
        double world_obstacle1_y = 0;
        // Obstacle1 in the world (fixed)
        world_obstacle1_x = robot_x + obstacle1_x * cos(yaw) - obstacle1_y * sin(yaw);
        world_obstacle1_y = robot_y + obstacle1_x * sin(yaw) + obstacle1_y * cos(yaw);

        point1.x = world_obstacle1_x;
        point1.y = world_obstacle1_y;
        point1.z = 0.0;

        // Two point found flag
        get_obs_flag = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openai_task_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_source_normal", 100);

    // Wait 2 seconds for UGE sensor to work
    int wait_start = 2;
    ros::Time ros_start_time = ros::Time::now();
    ros::Duration start_time(wait_start);
    while (ros::Time::now() - ros_start_time < start_time)
        ;
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

    ros::Rate loop_rate(100);

    // Velocity
    v0 = 0.2;
    // Current angular velocity
    w0 = twist.angular.z;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    bool obstacle_detected = false;

    // Final Goal
    goal.pose.position.x = 5.0;
    goal.pose.position.y = 0.0;
    
    auto last_move_log_time = std::chrono::steady_clock::now();

    while (ros::ok())
    {
        ros::spinOnce();
        geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

        get_obj_data();
        if (get_obs_flag)
        {
            obstacle_detected = true;
        }
        else
        {
            obstacle_detected = false;
        }
        get_obs_flag = false;

        if (!obstacle_detected)
        {
            // When the robot starts to move after encountering an obstacle
            if (in_stop_state)
            {
                ROS_INFO("Robot starts moving.");
                in_stop_state = false; // Reset stop state flag
            }

            // Log "robot moving" every 2 seconds while the robot is moving
            auto current_time = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_move_log_time).count() >= 2)
            {
                ROS_INFO("Robot moving.");
                last_move_log_time = current_time; // Update the last log time
            }

            x = 0;
            y = 0;
            theta = 0;
            follow_line(x, y, theta);
        }
        else
        {
            // Log when the robot encounters an obstacle and stops
            if (!in_stop_state)
            {
                ROS_WARN("Obstacle detected. Robot stopping.");
                in_stop_state = true;
                stop_start_time = std::chrono::steady_clock::now(); // Start timing stop state
            }
            else
            {
                // Check if stop duration exceeds 30 seconds
                auto stop_duration = std::chrono::steady_clock::now() - stop_start_time;
                if (std::chrono::duration_cast<std::chrono::seconds>(stop_duration).count() > 30)
                {
                    ROS_WARN("Robot stop timeout");
                    ROS_WARN("Robot Current Position: (%.2f, %.2f)", robot_x, robot_y);
                }
            }

            // Stop state
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
        }

        if (near_position(goal))
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            ROS_INFO("Arrived at goal.");
        }

        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    return 0;
}