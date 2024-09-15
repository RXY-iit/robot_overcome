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

// オドメトリから得られる現在の位置
double robot_x, robot_y;
double roll, pitch, yaw;
bool get_obs_flag = false;  // Obstacle detection flag

double safe_distance = 0.5; // Safe distance from obstacles
sensor_msgs::LaserScan scan; // Laser scan data
geometry_msgs::Quaternion robot_r; // Robot rotation data
geometry_msgs::PoseStamped goal; // Final goal data
geometry_msgs::Point point1, point2; // Obstacle points
geometry_msgs::Twist twist; // Velocity and angular velocity

// Initial velocity
double v0 = 0.0;
// Initial angular velocity
double w0 = 0.0;
std::map<std::string, std::string> params_;

// オドメトリのコールバック
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

// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// 直線追従をするために指令する速度を計算してtwistに格納
void follow_line(double x, double y, double th)
{
    double k_eta = 900.00000000 / 10000;
    double k_phai = 400.00000000 / 10000;
    double k_w = 300.00000000 / 10000;
    double v_max = 1.00000000;
    double w_max = 6.28000000;
    
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

    double phai = yaw - th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai = phai + 2 * M_PI;
        else
            phai = phai - 2 * M_PI;
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
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = w;

    w0 = twist.angular.z;
}



void get_obj_data()
{
    if (scan.ranges.empty()) // Ensure scan data is available
    {
        ROS_WARN("Scan data is empty!");
        return;
    }

    // Initialize variables for obstacle detection
    double min_distance1 = std::numeric_limits<double>::infinity();
    double angle_of_closest1;
    bool found1 = false;

    // Scan through the range around the center
    for (int i = (scan.ranges.size())/3; i < (scan.ranges.size())*2/3; ++i) {
    // for (int i = min_index; i <= max_index; ++i) {
        double distance1 = scan.ranges[i];
        if (distance1 >= 0.2 && distance1 < 0.5) {
            if (distance1 < min_distance1) {
                min_distance1 = distance1;
                angle_of_closest1 = scan.angle_min + i * scan.angle_increment;
                found1 = true;
            }
        }
    }

    // Two point found flag
    if (found1) {
        ROS_INFO("Obs found");
        get_obs_flag = true;
        std::cout << "P1:(dis,angle):" << min_distance1 << ", " << angle_of_closest1 << std::endl;
    } else {
        ROS_INFO("Obs not found");
        get_obs_flag = false;
    }
}

int near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_x - goal.pose.position.x;
    double dify = robot_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openai_task_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 10, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);

    int wait_start = 2;
    ros::Time ros_start_time = ros::Time::now();
    ros::Duration start_time(wait_start);
    while (ros::Time::now() - ros_start_time < start_time);

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

    ros::Rate loop_rate(10);
    v0 = 0.3;
    w0 = twist.angular.z;
    double x = 0.0, y = 0.0, theta = 0.0;
    
    goal.pose.position.x = 5.0;
    goal.pose.position.y = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();
        geometry_quat_to_rpy(roll, pitch, yaw, robot_r);
        get_obj_data();

        if (get_obs_flag)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            ROS_INFO("NOW STOP");
        }
        else
        {
            x = 0;
			y = 0;
			theta = 0;
            follow_line(x, y, theta);
            ROS_INFO("NOW RUNNING.");
        }

        if (near_position(goal))
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            ROS_INFO("Arrived at the goal.");
        }

        twist_pub.publish(twist);
        loop_rate.sleep();
    }

    return 0;
}
