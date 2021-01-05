#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int keyPresses = 0;
char lastKey;

void keyCallback(const std_msgs::String::ConstPtr& msg)
{
  keyPresses++;
  lastKey = *(msg->data.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "oneSecInfo");

    ros::NodeHandle n = ros::NodeHandle("~");

    ros::Publisher oSI_pub = n.advertise<std_msgs::String>("oneSecInfo", 1000);
    
    std::string keyboard_topic;
    n.getParam("keyboard_topic", keyboard_topic);
    ros::Subscriber oSI_sub = n.subscribe(keyboard_topic, 1000, keyCallback);

    ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std_msgs::String msg;

    ros::spinOnce();

    std::stringstream ss;
    ss << "Keys Pressed/s: " << keyPresses << " Last Key Pressed: " << lastKey;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    oSI_pub.publish(msg);

    loop_rate.sleep();

    keyPresses = 0;
    lastKey = ' ';
  }

    return 0;

}


