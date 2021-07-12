/* Author: Luke Dennis l.dennis@unsw.edu.au */

#include <ros/ros.h>
#include <termios.h>

namespace KeyControl {
    /* https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp*/
    char getch() {
        fd_set set;
        struct timeval timeout;
        int rv;
        char buff = 0;
        int len = 1;
        int filedesc = 0;
        FD_ZERO(&set);
        FD_SET(filedesc, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;

        rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

        struct termios old = {0};
        if (tcgetattr(filedesc, &old) < 0)
            ROS_ERROR("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(filedesc, TCSANOW, &old) < 0)
            ROS_ERROR("tcsetattr ICANON");

        if(rv > 0)
            read(filedesc, &buff, len );

        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
            ROS_ERROR ("tcsetattr ~ICANON");
        return (buff);
    }

    enum key_press {
        NULL_KEY,
        W_KEY,
        A_KEY,
        S_KEY,
        D_KEY,
        Q_KEY,
        E_KEY,
        SPACE_KEY,
    };

}
