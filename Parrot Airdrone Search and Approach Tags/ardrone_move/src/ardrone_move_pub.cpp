#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h" //Header for Empty msgs
#include "geometry_msgs/Twist.h" //Header for control the ardrone Twist

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardrone_move_pub");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Duration(1.0).sleep();

  ros::Publisher launch = n.advertise<std_msgs::Empty>("ardrone/takeoff",5);
  ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land",5);
  ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset",5);
  ros::Publisher twist = n.advertise<geometry_msgs::Twist>("cmd_vel",10);

  ROS_INFO("Moving ardrone!!! Watch your head!!");
    
  if (ros::ok())
  {
    launch.publish(std_msgs::Empty()); // Take off the ardrone
	ros::Duration(5.0).sleep();    // Mantain the ardrone 5 sec in the position
	// Declare the msg to send
	geometry_msgs::Twist msgz;
		//msg.linear.x=12.0;
		msgz.angular.z=0.5;
	twist.publish(geometry_msgs::Twist(msgz)); //Twist
	//ros::Duration(15.0).sleep();    // Mantain the ardrone 5 sec in the position
	//geometry_msgs::Twist msgx;
	//	msgx.linear.x=1.0;
		//msg.angular.z=1.0;
	//twist.publish(geometry_msgs::Twist(msgx)); //Twist
	//ros::Duration(5.0).sleep();    // Mantain the ardrone 5 sec in the position
	land.publish(std_msgs::Empty()); // Land the ardrone
    //ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
