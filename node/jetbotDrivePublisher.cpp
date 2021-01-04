#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

class jetbotDriveCmd
{
private:
    ros::NodeHandle n;

    ros::Subscriber key_sub;

    ros::Publisher diff_drive_pub;

    double prev_key_velocity = 0.0;
    double keyboard_max_speed = 1.0;
    double rotationWheelSpeedScale;
    double yawVel;
    double speedInc;
    double maxSpeed;
    double leftWheelSpeed;
    double rightWheelSpeed;    
    double radius = 1.0;
    bool mode = false; // false = mode 1 true = mode2

public:
    jetbotDriveCmd()
    {
        n = ros::NodeHandle("~");

        std::string diff_drive_topic, mux_topic, joy_topic, key_topic;
	    n.getParam("diff_drive_topic", diff_drive_topic);
        n.getParam("keyboard_topic", key_topic);
        n.getParam("jetbot_rotation_wheel_speed_scale", rotationWheelSpeedScale);
        n.getParam("yaw_vel", yawVel);
        n.getParam("speedInc", speedInc);
        n.getParam("jetbot_max_wheel_speed", maxSpeed);

        diff_drive_pub = n.advertise<std_msgs::Float64MultiArray>(diff_drive_topic, 10);

        key_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::key_callback, this);

    }


    void publish_to_diff_drive(double rightWheelTrq,double leftWheelTrq)
    {
        std_msgs::Float64MultiArray diffDriveMsg;
	    diffDriveMsg.data.clear();
	    diffDriveMsg.data.push_back(rightWheelTrq);
	    diffDriveMsg.data.push_back(leftWheelTrq);
        diff_drive_pub.publish(diffDriveMsg);
    }

    void key_callback(const std_msgs::String & msg){
        double track = 0.2;

        bool publish = true;

        if (msg.data == "w"){
            if (!mode){
                leftWheelSpeed = 1.0;
                rightWheelSpeed = 1.0;
            }
            else{
                if(leftWheelSpeed <= maxSpeed){
                    leftWheelSpeed += speedInc;
                    rightWheelSpeed += speedInc;
                }                
            }
            
        }else if(msg.data=="s"){
           if (!mode){
                leftWheelSpeed = -1.0;
                rightWheelSpeed = -1.0;
            }
            else{
                if(leftWheelSpeed >= -maxSpeed){
                    leftWheelSpeed -= speedInc;
                    rightWheelSpeed -= speedInc;
                }                
            }

        }else if(msg.data == "a"){
            leftWheelSpeed = -1.0*rotationWheelSpeedScale;
            rightWheelSpeed = 1.0*rotationWheelSpeedScale;
       
	    }else if(msg.data == "d") {
            leftWheelSpeed = 1.0*rotationWheelSpeedScale;
            rightWheelSpeed = -1.0*rotationWheelSpeedScale;

        }else if(msg.data=="t"){
            leftWheelSpeed = (yawVel)*radius*rotationWheelSpeedScale;
            rightWheelSpeed = (yawVel)*(radius+track)*rotationWheelSpeedScale;
        
        }else if(msg.data=="i"){
            radius = radius+0.1;
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
            if (radius>=0 && yawVel<=0){
                yawVel = -yawVel;
            }
        
        }else if(msg.data=="r"){
            radius = radius-0.1;
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
            if(radius<0 && yawVel>0){
                yawVel = -yawVel;
            }
        
        }else if(msg.data=="m"){
            mode ^= true;
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
        
        }else if (msg.data ==" "){
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
        
        }else {
            publish = false;
        }
        if (publish){
            publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);

        }
    }
   
};
int main(int argc, char ** argv){
  ros::init(argc, argv, "jetbotDriveCmd");
  jetbotDriveCmd jetDriver;
  ros::spin();
  return 0;
  }


