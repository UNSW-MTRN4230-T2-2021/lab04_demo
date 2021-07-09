/* Author: Luke Dennis l.dennis@unsw.edu.au */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <lab04_example/key_control.h>
#include <lab04_example/KeyToCmdVel.h>

namespace Lab04 {
	/* Inspiration from:
	 *  https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h
	 *  https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
	 */
	class VelocityController {
		public:
			VelocityController(ros::NodeHandle nh) : nh_(nh) {
				key_control_sub_ = nh_.subscribe("KeyControl", 10,
						&Lab04::VelocityController::keyCommandCallback_, this);
				twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
				cmd_vel_timer_ = nh.createTimer(ros::Duration(1.0 / 10.0),
						std::bind(&Lab04::VelocityController::cmdVelCallback_, this));
				client_ = nh.serviceClient<lab04_example::KeyToCmdVel>("key_to_cmd_vel");
			}
		private:
			ros::NodeHandle nh_;

			ros::Subscriber key_control_sub_;
			ros::Publisher 	twist_pub_;
			ros::Timer cmd_vel_timer_;
			ros::ServiceClient client_;

			KeyControl::key_press current_cmd_{KeyControl::NULL_KEY};

			auto keyCommandCallback_(std_msgs::Int8::ConstPtr const& msg) -> void {
				switch(msg->data) {
					case(KeyControl::NULL_KEY):
					case(KeyControl::W_KEY):
					case(KeyControl::A_KEY):
					case(KeyControl::S_KEY):
					case(KeyControl::D_KEY):
					case(KeyControl::Q_KEY):
					case(KeyControl::E_KEY):
					case(KeyControl::SPACE_KEY):
						current_cmd_ = static_cast<KeyControl::key_press>(msg->data);
						break;
					default:
						ROS_ERROR("Recieved invalid command: %d", msg->data);
				}
			}

			auto cmdVelCallback_() -> void {
				auto cmd_vel_msg = geometry_msgs::Twist{};
				auto req = lab04_example::KeyToCmdVel::Request{};
				auto res = lab04_example::KeyToCmdVel::Response{};
				req.key = current_cmd_;
				if(client_.call(req, res)){
					cmd_vel_msg = res.cmd_vel;
				} else {
					ROS_WARN("Service returned false");
					cmd_vel_msg.linear.x = 0;
					cmd_vel_msg.linear.y = 0;
					cmd_vel_msg.linear.z = 0;
					cmd_vel_msg.angular.x = 0;
					cmd_vel_msg.angular.y = 0;
					cmd_vel_msg.angular.z = 0;
				}
				ROS_INFO("(%lf, %lf) (%lf, %lf)", cmd_vel_msg.linear.x, cmd_vel_msg.linear.y,
						cmd_vel_msg.angular.x, cmd_vel_msg.angular.y);
				twist_pub_.publish(cmd_vel_msg);
				current_cmd_ = KeyControl::NULL_KEY; /* Set to zero in case subscriber does not recieve fresh data */
			}
	};
}
