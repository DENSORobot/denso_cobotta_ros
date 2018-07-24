#include "ros/ros.h"


//bool set_motor_state

int main(int argc, char **argv)
{
    ros::init(argc, argv, "denso_cobotta_driver");

    ros::NodeHandle n;

    ros::Rate loop_rate(125);

    //ros::ServiceServer srv_motor = n.advertiseService("set_motor_state", 

    while(ros::ok()){

    }

    return 0;
}
