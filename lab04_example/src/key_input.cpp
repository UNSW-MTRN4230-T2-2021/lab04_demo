/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Luke Dennis l.dennis@unsw.edu.au */

#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <lab04_example/key_control.h>


int main(int argc, char **argv)
{

    // Extension: Take hzs from ros param
    auto const spin_hz = static_cast<int>(100);

    ros::init(argc, argv, "key_input");

    ros::NodeHandle n;

    ros::Publisher key_control_pub = n.advertise<std_msgs::Int8>("KeyControl", spin_hz*0.5);

    ros::Rate loop_rate(spin_hz);

    auto key_msg = std_msgs::Int8{};
    auto key = KeyControl::key_press{};

    auto c = int8_t{};
    auto no_key_count = 0;
    auto const max_no_key_count = static_cast<int>(0.5*spin_hz); /* Time out every 0.5 seconds */

    ROS_INFO("Enter WASD for linear control, QE for angular control, Space to exit");

    while (ros::ok())
    {
        c = KeyControl::getch();

        switch(c) {
            case('w'):
                key = KeyControl::W_KEY;
                break;
            case('a'):
                key = KeyControl::A_KEY;
                break;
            case('s'):
                key = KeyControl::S_KEY;
                break;
            case('d'):
                key = KeyControl::D_KEY;
                break;
            case('q'):
                key = KeyControl::Q_KEY;
                break;
            case('e'):
                key = KeyControl::E_KEY;
                break;
            case(' '):
                key = KeyControl::SPACE_KEY;
                break;
            default:
                key = KeyControl::NULL_KEY;
        }

        if(key != KeyControl::NULL_KEY) {
            /* send every non-null key */
            no_key_count = 0;

            key_msg.data = key;
            key_control_pub.publish(key_msg);
        } else {
            ROS_INFO_DELAYED_THROTTLE(10,"Press Space to quit");
            if(++no_key_count == max_no_key_count) {
                /* Reached time out */
                /* send null key after 0.5 sec time out*/
                key_msg.data = key;
                key_control_pub.publish(key_msg);
                ROS_INFO("Key Control Time Out");
            }
        }

        ros::spinOnce();

        if(key == KeyControl::SPACE_KEY) {
            /* exit */
            break;
        }

        loop_rate.sleep();
    }

    return 0;
}
