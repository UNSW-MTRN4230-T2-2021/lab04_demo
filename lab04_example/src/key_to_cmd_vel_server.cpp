/* http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2FTutorials.2FWritingServiceClient.Writing_a_Service_Node */

/* Author: Luke Dennis l.dennis@unsw.edu.au */

#include <ros/ros.h>
#include <lab04_example/KeyToCmdVel.h>
#include <lab04_example/key_control.h>


class KeyCmdSrv {
    public:
        KeyCmdSrv(ros::NodeHandle nh) : nh_(nh), last_vel_(geometry_msgs::Twist{}) {
            srv_ = nh.advertiseService("key_to_cmd_vel", &KeyCmdSrv::convert_to_vel_, this);
        }
    private:
        static double constexpr lin_speed = 2.4;
        static double constexpr ang_speed = 0.7;

        ros::NodeHandle nh_;
        ros::ServiceServer srv_;

        geometry_msgs::Twist last_vel_;

        auto convert_to_vel_(lab04_example::KeyToCmdVel::Request & req,
                lab04_example::KeyToCmdVel::Response & res) -> bool {

            ROS_INFO("(%lf, %lf) (%lf, %lf)", last_vel_.linear.x, last_vel_.linear.y,
                    last_vel_.angular.x, last_vel_.angular.y);

            /* VERY basic service, this can be extended to do basic DR based on
             * previous velocity and current command to smooth out changes.
             * E.G. cmd can be taken as acc and applied accordingly
             */
            last_vel_.linear.x = 0;
            last_vel_.linear.y = 0;
            last_vel_.angular.x = 0;
            last_vel_.angular.y = 0;
            switch(req.key) {
                case(KeyControl::W_KEY):
                    last_vel_.linear.x = lin_speed;
                    break;
                case(KeyControl::A_KEY):
                    last_vel_.linear.y = lin_speed;
                    break;
                case(KeyControl::S_KEY):
                    last_vel_.linear.x = -lin_speed;
                    break;
                case(KeyControl::D_KEY):
                    last_vel_.linear.y = -lin_speed;
                    break;
                case(KeyControl::Q_KEY):
                    last_vel_.angular.y = ang_speed;
                    break;
                case(KeyControl::E_KEY):
                    last_vel_.angular.y = -ang_speed;
                    break;
                case(KeyControl::SPACE_KEY):
                case(KeyControl::NULL_KEY):
                    break;
            }
            res.cmd_vel = last_vel_;
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_to_cmd_vel_service");
    ros::NodeHandle nh;

    auto srv = KeyCmdSrv(nh);

    ROS_INFO("Ready to convert commands to velocities.");
    ros::spin();

    return 0;
}
