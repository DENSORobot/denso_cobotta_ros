#include "ros/ros.h"
#include "denso_cobotta_driver/set_motor_state.h"

bool set_motor_state(denso_cobotta_driver::set_motor_state::Request &req,
                      denso_cobotta_driver::set_motor_state::Response &res){
    if(req.state == 1){
        ROS_INFO("Motor ON\n");
        res.success = true;
    }else{
        ROS_INFO("Motor OFF\n");
        res.success = true;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "denso_cobotta_driver");

    ros::NodeHandle n;
    ros::Rate loop_rate(125);

    ros::ServiceServer srv_motor = n.advertiseService("set_motor_state", &set_motor_state);

    // TODO: Init driver

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
