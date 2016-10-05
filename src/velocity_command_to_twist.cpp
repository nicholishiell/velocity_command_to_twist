#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "bupimo_msgs/VelocityCommand.h"
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
float currentHeading = 0.;
float targetHeading = 0.;
float targetLinearVel = 0.;
    
bool commandRecieved = false;

void VelocityCommandCallBack(const bupimo_msgs::VelocityCommand::ConstPtr& msg){
  commandRecieved = true;

  targetHeading = msg->bearing;
  targetLinearVel = msg->linearSpeed;
}


void CurrentHeadingCallback(const std_msgs::Float64::ConstPtr& msg){
  currentHeading = msg->data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "behaviour_controller");

  ros::NodeHandle n;
 
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber command_Sub = n.subscribe("dns_command", 1000, VelocityCommandCallBack);
  ros::Subscriber currentHeading_Sub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);
  
  ros::Rate loop_rate(15);

  while (ros::ok()){

    if(!commandRecieved){
      printf("Waiting for commands...\n");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

    //printf("currentHeading = %f\n", currentHeading);
    //printf("targetHeading = %f\n", targetHeading);
    //printf("targetLinearVel = %f\n", targetLinearVel);

    geometry_msgs::Twist msg;

    float headingError = currentHeading - targetHeading;
    if( headingError > 180.){
      headingError = headingError - 360.;
    }
    else if(headingError < -180.){
      headingError = headingError + 360;
    }
    else{
      headingError = headingError;
    }
    
    // Now convert to radians (per second)
    headingError = headingError * M_PI/180.;

    // Minimum threshold on linear speed to stop drift
    if(fabs(targetLinearVel) < 0.2) targetLinearVel = 0.;
    
    // set values of twist msg.
    msg.linear.x = 0.075*targetLinearVel;
    msg.angular.z = 2.*headingError;
    msg.angular.y = -1.;

    // Need to remove excess digits for transmission over serial.
    msg.linear.x = roundf(msg.linear.x * 100.) / 100.;
    msg.angular.z = roundf(msg.angular.z * 100.) / 100.; 
    
    // publish twist msg.
    twist_pub.publish(msg);
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
};
