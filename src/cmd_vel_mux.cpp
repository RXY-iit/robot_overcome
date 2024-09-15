#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <openai_test/SelectTopic.h>  // Include the custom service header
#include <string>

class CmdVelMux
{
public:
    CmdVelMux()
    {
        // Initialize node parameters
        nh_.param("initial_topic", current_topic_, std::string("/cmd_vel_source_normal"));

        // Subscribers for different velocity sources
        sub_normal_ = nh_.subscribe("/cmd_vel_source_normal", 10, &CmdVelMux::normalCmdVelCallback, this);
        sub_avoidance_ = nh_.subscribe("/cmd_vel_source_avoidance", 10, &CmdVelMux::avoidanceCmdVelCallback, this);

        // Publisher to the final cmd_vel topic
        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);

        // Service to manually switch between control sources
        select_service_ = nh_.advertiseService("/cmd_vel_mux/select", &CmdVelMux::selectSource, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_normal_;
    ros::Subscriber sub_avoidance_;
    ros::Publisher pub_cmd_vel_;
    ros::ServiceServer select_service_;

    std::string current_topic_;
    geometry_msgs::Twist last_normal_twist_;
    geometry_msgs::Twist last_avoidance_twist_;

    // Callback for normal cmd_vel source
    void normalCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_normal_twist_ = *msg;
        if (current_topic_ == "/cmd_vel_source_normal")
        {
            pub_cmd_vel_.publish(last_normal_twist_);
        }
    }

    // Callback for obstacle avoidance cmd_vel source
    void avoidanceCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_avoidance_twist_ = *msg;
        if (current_topic_ == "/cmd_vel_source_avoidance")
        {
            pub_cmd_vel_.publish(last_avoidance_twist_);
        }
    }

    // Service to switch between cmd_vel sources
    bool selectSource(openai_test::SelectTopic::Request& req, openai_test::SelectTopic::Response& res)
    {
        if (req.topic == "/cmd_vel_source_normal" || req.topic == "/cmd_vel_source_avoidance")
        {
            current_topic_ = req.topic;
            ROS_INFO("Switched to topic: %s", current_topic_.c_str());
            res.success = true;
            return true;
        }
        else
        {
            ROS_WARN("Invalid topic selected: %s", req.topic.c_str());
            res.success = false;
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_mux");
    CmdVelMux mux;
    ros::spin();
    return 0;
}
