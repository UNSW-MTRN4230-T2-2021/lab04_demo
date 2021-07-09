/* Author: Luke Dennis l.dennis@unsw.edu.au */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <lab04_example/key_control.h>

namespace Lab04 {
    /* Inspiration from: https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.h */
    class VelocityController {
        public:
            VelocityController(ros::NodeHandle nh) : nh_(nh) {
                key_control_sub_ = nh_.subscribe("KeyControl", 10,
                        &Lab04::VelocityController::keyCommandCallback_, this);
                twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            }
        private:
            ros::NodeHandle nh_;

            ros::Subscriber key_control_sub_;
            ros::Publisher 	twist_pub_;

            auto keyCommandCallback_(std_msgs::Int8::ConstPtr const& msg) -> void {
                ROS_INFO("CMD: %d", msg->data);
            }
    };
}
