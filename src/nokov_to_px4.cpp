#include "rclcpp/rclcpp.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <chrono>
#include <memory>
using std::placeholders::_1;
using namespace std::chrono_literals;

class NokovToPx4 : public rclcpp::Node
{
    public:
    NokovToPx4() : Node("nokov_to_px4"){
        this->declare_parameter("nokov_topic","/px/px4");
        this->declare_parameter("px4_topic","/fmu/in/vehicle_visual_odometry");
        const std::string nokov_topic = this->get_parameter("nokov_topic").as_string();
        const std::string px4_topic   = this->get_parameter("px4_topic").as_string();
        RCLCPP_INFO(this->get_logger(), ("nokov_topic:" + nokov_topic).c_str());
        RCLCPP_INFO(this->get_logger(), ("px4_topic:  " + px4_topic).c_str());
        poseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(nokov_topic, 10, std::bind(&NokovToPx4::poseCallback, this, _1));
        odomPub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 10);
    }

    private:
        void poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr);
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;
	    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odomPub_;
};

void NokovToPx4::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr poseMsg_){
    RCLCPP_INFO_ONCE(this->get_logger(), "Recived first msg from Nokov.");
	RCLCPP_INFO_ONCE(this->get_logger(), "P: %f, %f, %f", poseMsg_->pose.position.x, 
					poseMsg_->pose.position.y, poseMsg_->pose.position.z);
	RCLCPP_INFO_ONCE(this->get_logger(), "q: %f, %f, %f, %f", poseMsg_->pose.orientation.w,
					poseMsg_->pose.orientation.x, poseMsg_->pose.orientation.y, poseMsg_->pose.orientation.z);
    px4_msgs::msg::VehicleOdometry odomMsg_;

    odomMsg_.pose_frame = odomMsg_.POSE_FRAME_FRD;
    odomMsg_.timestamp  = uint64_t(poseMsg_->header.stamp.sec)*1000000 + uint64_t(poseMsg_->header.stamp.nanosec)/1000;
    odomMsg_.timestamp_sample = odomMsg_.timestamp;
    odomMsg_.position[0] = poseMsg_->pose.position.x;
    odomMsg_.position[1] = -poseMsg_->pose.position.y;
    odomMsg_.position[2] = -poseMsg_->pose.position.z;
    odomMsg_.q[0] = poseMsg_->pose.orientation.w;
    odomMsg_.q[1] = poseMsg_->pose.orientation.x;
    odomMsg_.q[2] = -poseMsg_->pose.orientation.y;
    odomMsg_.q[3] = -poseMsg_->pose.orientation.z;

    odomPub_->publish(odomMsg_);
    RCLCPP_INFO_ONCE(this->get_logger(), "Sent to PX4 as:");
	RCLCPP_INFO_ONCE(this->get_logger(), "P: %f, %f, %f", odomMsg_.position[0], odomMsg_.position[1], odomMsg_.position[2]);
	RCLCPP_INFO_ONCE(this->get_logger(), "q: %f, %f, %f, %f", odomMsg_.q[0],
					odomMsg_.q[1], odomMsg_.q[2], odomMsg_.q[3]);
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<NokovToPx4>());
	rclcpp::shutdown();
	return 0;
}