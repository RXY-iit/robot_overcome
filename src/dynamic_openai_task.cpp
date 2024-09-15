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
bool get_obs_flag = false;  //get obs data flag
// laser scan data
sensor_msgs::LaserScan scan;
// odom robot rotation data
geometry_msgs::Quaternion robot_r;
// final goal data
geometry_msgs::PoseStamped goal;
// obstacle ponit1, point2
geometry_msgs::Point point1, point2;

// rotaion 
int flag=0,count=0;

geometry_msgs::Twist twist; // 指令する速度、角速度
// 初期速度
double v0 = 0.0;
// 初期角速度
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
	// 現在のロボットのroll, pitch, yawを計算
	// geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	// 直線追従のゲイン
	double k_eta = 900.00000000 / 10000;
	double k_phai = 400.00000000 / 10000;
	double k_w = 300.00000000 / 10000;

	// ロボットの最大速度、最大角速度
	double v_max = 1.00000000;
	double w_max = 6.28000000;

	// ロボットと直線の距離(実際には一定以上の距離でクリップ)
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

	// 直線に対するロボットの向き(-M_PIからM_PIに収まるように処理)
	double phai = yaw - th;
	while (phai <= -M_PI || M_PI <= phai)
	{
		if (phai <= -M_PI)
			phai = phai + 2 * M_PI;
		else
			phai = phai - 2 * M_PI;
	}

	// 目標となるロボットの角速度と現在の角速度の差
	double w_diff = w0;

	// 角速度
	double w = w0 + (-k_eta * eta - k_phai * phai - k_w* w_diff);
	if (w > w_max)
		w = w_max;
	else if (w < -w_max)
		w = -w_max;

	// 並進速度
	double v = v0;
//	if (v0 != 0)
//		v = v0 - v0 / abs(v0) * abs(w0);
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

	// 現在の角速度を次の時間の計算に使用
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
	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;
	ROS_INFO("rotate in");

	// デバック用のプリント
	// std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
	// std::cout << "v: " << twist.linear.x << "   w: " << twist.angular.z << std::endl;
	// std::cout << "(x,y) = (" << robot_x << "," << robot_y << ")" << std::endl;
	// std::cout << "------------------------------" << std::endl;
	if(-0.1<yaw && yaw<-0.01){
		flag=1;

	}else if(-0.01<=yaw && flag==1){
		flag=0;
		count++;	// 回転数
		ROS_INFO("count++");
	}

	if(count>=1){
		twist.angular.z=0.0;
	}
}

void get_obj_data()
{
	// upada objects obstacle data in world
	double min_distance1 = std::numeric_limits<double>::infinity();
	double angle_of_closest1;
	bool found1 = false;
	// double min_distance2 = std::numeric_limits<double>::infinity();
	// double angle_of_closest2;
	// double angle_now;
	// bool found2 = false;

	for (int i = 0; i < scan.ranges.size(); ++i) {
		double distance1 = scan.ranges[i];
		// std::cout << "dis1 now = " << distance1 << std::endl;
		if (distance1 >= 0.2 && distance1 < 0.5) {
			if (distance1 < min_distance1) {
				min_distance1 = distance1;
				angle_of_closest1 = scan.angle_min + i * scan.angle_increment;
				// std::cout << "P1:(dis,angle):" << min_distance1 << ", " << angle_of_closest1 << std::endl;
				found1 = true;
			}
		}
	}
	// min_distance1 = 1.75;

	// for (int k = 0; k < scan.ranges.size(); ++k) {
	// 	// std::cout << "get in P2 , k=" << k << std::endl;
	// 	// angle_now = scan.angle_min + i * scan.angle_increment;
	// 	double distance2 = scan.ranges[k];
	// 	// std::cout << "dis2 now = " << distance2 << std::endl;
	// 	// if ((angle_now > (angle_of_closest1 + (scan.angle_increment*6))) || (angle_now < (angle_of_closest1 - (scan.angle_increment*6)))){
	// 	if(distance2 > (min_distance1 + 0.11)){
	// 		// std::cout << "dis2 now = " << distance2 << std::endl;
	// 		if (distance2 >= 0.1 && distance2 < 3.5) {
	// 			if (distance2 < min_distance2) {
	// 				min_distance2 = distance2;
	// 				angle_of_closest2 = scan.angle_min + k * scan.angle_increment;
	// 				// std::cout << "P2:(dis,angle):" << min_distance2 << ", " << angle_of_closest2 << std::endl;
	// 				found2 = true;
	// 			}
	// 		}
	// 	}
	// }

	if (found1) {
		double obstacle1_x = min_distance1 * cos(angle_of_closest1);
		double obstacle1_y = min_distance1 * sin(angle_of_closest1);

		double world_obstacle1_x = 0;
		double world_obstacle1_y = 0;
		//obstacle1 in world (fixed)
		world_obstacle1_x = robot_x + obstacle1_x*cos(yaw)-obstacle1_y*sin(yaw);
        world_obstacle1_y = robot_y + obstacle1_x*sin(yaw)+obstacle1_y*cos(yaw);

		point1.x = world_obstacle1_x;
		point1.y = world_obstacle1_y;
		point1.z = 0.0;

		// if (found2) {
		// double obstacle2_x = min_distance2 * cos(angle_of_closest2);
		// double obstacle2_y = min_distance2 * sin(angle_of_closest2);

		// double world_obstacle2_x = 0;
		// double world_obstacle2_y = 0;
		// //obstacle1 in world (fixed)
		// world_obstacle2_x = robot_x + obstacle2_x*cos(yaw)-obstacle2_y*sin(yaw);
        // world_obstacle2_y = robot_y + obstacle2_x*sin(yaw)+obstacle2_y*cos(yaw);

		// point2.x = world_obstacle2_x;
		// point2.y = world_obstacle2_y;
		// point2.z = 0.0;

		//two point found flag
		get_obs_flag = true;
		// }
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "openai_task_node");
	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 100, odom_callback);
	ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_source_normal", 100);

	// wait 4s for UGE sensor work
	int wait_start = 2;
	ros::Time ros_start_time = ros::Time::now();
	ros::Duration start_time(wait_start);
	while(ros::Time::now()-ros_start_time < start_time);
	ROS_INFO("Waitted 2s.");

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

	// 速度
	v0 = 0.2;
	// 現在の角速度
	w0 = twist.angular.z;
	
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	bool ob_avio_fnish = false;
    
	//Final Goal
	goal.pose.position.x = 5.0;
    goal.pose.position.y = 0.0;

	while (ros::ok())
	{
		ros::spinOnce();
		// std::cout << " whiel in"  << std::endl;
		// 現在のロボットのroll, pitch, yawを計算
		geometry_quat_to_rpy(roll, pitch, yaw, robot_r);
		
		get_obj_data();
        if (get_obs_flag)
        {
			ob_avio_fnish = true;
        }else{
			ob_avio_fnish = false;
		}
		get_obs_flag = false;
		
		if (!ob_avio_fnish)
        {
			x = 0;
			y = 0;
			theta = 0;
			follow_line(x, y, theta);
            // std::cout << "now follow line"  << std::endl;
		
        }else{
			//stop state
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.linear.z = 0.0;
			twist.angular.x = 0.0;
			twist.angular.y = 0.0;
			twist.angular.z = 0.0;
            // std::cout << "now stop"  << std::endl;
		}

		if (near_position(goal))
        {
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			std::cout << "arrived goal"  << std::endl;
        }
		twist_pub.publish(twist);
		loop_rate.sleep();
	}

	return 0;
}