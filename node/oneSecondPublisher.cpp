#include "ros/ros.h"
#include "std_msgs/String.h"



void main(int argc, char **argv){
    
    ros::init(argc, argv, "oneSecPublisher");

    ros::NodeHandle n;
    std::string oneSecInfo, key_topic;

    n.getParam("oneSecondInfo", oneSecInfo);
    n.getParam("keyboard_topic", key_topic)

    ros::Publisher oneSec_pub = n.advertise<std_msgs::String>(oneSecInfo, 1000);
    ros::Subscriber key_sub = n.subscribe(key_topic, 1000)


    ros::Rate loopRate = 1;
    
    ROS_INFO("Keys pressed/s: %s", '2');
    ROS_INFO("Last key pressed: %s", 'a')
    
    loopRate.sleep();
}

