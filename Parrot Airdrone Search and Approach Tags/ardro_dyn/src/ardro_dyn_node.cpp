#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>    //Header for dynamic reconfiguration
#include <ardro_dyn/ardroparamsConfig.h>   //Header of parameters used by D. Reconfiguration
#include <std_msgs/Empty.h>                //Header for Empty msgs
#include <ar_pose/ARMarkers.h>		   //Header for tag handle 
#include <std_srvs/Empty.h>		   //Header for Empty srvs
#include <geometry_msgs/Twist.h>	   //Header for control the drone movement

#include <unistd.h>			   //Header for Delay handle	
#include <iostream>			   //Header for Handle input/output from terminal c++	
#include <math.h>			   //Header for Math computations C++
#include <cmath>			   //std::abs 	

#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>
#include<tf/transform_listener.h>

/* *************************************************************************
VARIABLES
***************************************************************************/
using namespace std;
std_msgs::Empty empty_msg; 		//Empty msgs variable
std_srvs::Empty empty_srv;		//Empty srvs variable
geometry_msgs::Twist twist_msg_hover;   //Variable for hover functionality
geometry_msgs::Twist twist_msg;
std::vector<double> desired_pose_;
bool start_;				//Start control
double x, y, z;
double xd, yd, zd;
double ex, ey, ez, enorm, eyy;
double roll_ar,pitch_ar,yaw_ar;
double start_time;
float end_time = 5.0;
bool is_marker_ok = false;
int counter=0;
/* *************************************************************************
CALLBACK BLOCK DEFINITION
***************************************************************************/
void dynrec_callback(ardro_dyn::ardroparamsConfig &config, uint32_t level)
{
	ROS_INFO("New Desired position: (%f,%f,%f). START: %s", config.new_x, config.new_y, config.new_z, config.start ? "True" : "False");

	// Store new acquired values
	
	xd=config.new_x;
	yd=config.new_y;
	zd=config.new_z;
	start_ = config.start;
}

void ARtag_CBfloor(const ar_pose::ARMarker& msg)
{
   ROS_INFO("ArtagPbvsAlgNode::input_ARtag_callback: New Message Received");
  
   bool is_marker_ok = true; // flag for mark detection
   x = msg.pose.pose.position.x;
   y = msg.pose.pose.position.y;
   z = msg.pose.pose.position.z;
   // Convert quaternion to RPY

   tf::Quaternion qt;
   double roll,pitch,yaw;
   tf::quaternionMsgToTF(msg.pose.pose.orientation, qt);
   tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
   
   roll_ar=roll;
   pitch_ar=pitch;
   yaw_ar=yaw;

   if(std::isnan(roll)  || !std::isfinite(roll) || (msg.pose.pose.position.z<0))
   is_marker_ok = false;
   if (start_)
  	ROS_INFO("STARTING ...");

  
}
void ARtag_CBwall(const ar_pose::ARMarkers::ConstPtr& msg)
{
  ROS_INFO("ArtagPbvsAlgNode::input_ARtag_callback: New Message Received");

  if (msg->markers.empty()){

  	// No marker detected
    ROS_INFO("Nor marker detected");
  }
  else {

  	// Get current detected position
  	std::vector<double> curr_pose_wall(3);
  	curr_pose_wall[0] = msg->markers[0].pose.pose.position.x;
  	curr_pose_wall[1] = msg->markers[0].pose.pose.position.y;
  	curr_pose_wall[2] = msg->markers[0].pose.pose.position.z;

  	// Compute error between desired and current positions
  	std::vector<double> error_wall(3);
  	for (int ii = 0; ii < curr_pose_wall.size(); ++ii)
  		error_wall[ii] = curr_pose_wall[ii]-desired_pose_[ii];

  	// Compute the norm of the error
  	double norm_wall = sqrt(pow(error_wall[0],2)+pow(error_wall[1],2)+pow(error_wall[2],2));

  	// SHOW START INFO
  	if (start_)
  		ROS_INFO("STARTING ...");

  }
}
/***************************************************************************
SUBRUTINES
***************************************************************************/
void hover()				//Subrutine for hover functionality
{
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0;  	
}

double compu_error (double x, double y, double z, double xd, double yd, double zd, int index) 

{							  	
	//int ex,ey,ez,enorm;
	ex = xd-x;
        ey = yd-y;
        ez = zd-z;   

        enorm = sqrt(pow(ex,2) + pow(ey,2) + pow(ez,2));  

	switch (index)

   	{
		case 1:
			return ex;
			break;
		case 2:
			return ey;
			break;
		case 3:
			return ez;
			break;
		case 4:
			return enorm;
			break;
		default:
			printf("Index error on void error function");
			break;
	}	
}

/* *************************************************************************
MAIN PROGRAM
***************************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardro_dyn");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	//Variables for pose handling
	desired_pose_.resize(3);
	start_ = false;
	//Subscriber ar_pose_marker configuration
	ros::Subscriber subf = n.subscribe("/tagfloor", 1000, ARtag_CBfloor);
	ros::Subscriber subw = n.subscribe("/tagwall", 1000, ARtag_CBwall);
	//Dynamic reconfigure Configuration for call back
	dynamic_reconfigure::Server <ardro_dyn::ardroparamsConfig> server;
	dynamic_reconfigure::Server <ardro_dyn::ardroparamsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Pose error node Spinning");
	// Taking off and Landing setup
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);		
	ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land",1);
	//Setup for togglecam
	ros::ServiceClient toggleCam = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
	//Setup for control velocity
	ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel",1); 
	while (ros::ok())
	{
			
		if(start_==true)
		{
			takeoff.publish(empty_msg); // launch the drone			
			toggleCam.call(empty_srv);  // toggle teh camera to the bottom
                                                    // the (default = frontal camera
			//hover();                    // Keep the drone flat
			start_time =(double)ros::Time::now().toSec(); // Start time for 5 sec
			while((double)ros::Time::now().toSec()< start_time+end_time) // aling for 5 sec
			{
				
				// Compute the error
				ex = xd-x;
        			ey = yd-y;
       				ez = zd-z;   
        			enorm = sqrt(pow(ex,2) + pow(ey,2) + pow(ez,2)); 
				ROS_INFO("xd: %f yd: %f zd: %f", xd, yd, zd);
				ROS_INFO("x: %f y: %f z: %f", x, y, z);	
				eyy=compu_error(x,y,z,xd,yd,zd,2);	
				ROS_INFO("RESULTADO: %f",eyy);	
				
				// Aling the drone 1m over the mark
				/* the values are based on the camera axis, */
				if (std::abs(ey)>0.1)  //if the error on x is higher than 0.5 we correct
				{
					twist_msg.linear.x =0.5*ex; 
					twist_msg.linear.y =0.5*ey;
					twist_msg.linear.z =0.5*ez;
					twist_msg.angular.x =0.5; 
					twist_msg.angular.y =0.5; 
					twist_msg.angular.z =1; 
					ROS_INFO("velocidad: %f", twist_msg.linear.y);	
				       
				}// if std::abs				
			}// while timer
			//ROTATE DRONE COUNTERCLOCK WISE
					
					twist_msg.linear.x =0; 
					twist_msg.linear.y =0;
					twist_msg.linear.z =0;
					twist_msg.angular.x =0; 
					twist_msg.angular.y =0; 
					twist_msg.angular.z =1; 
					ROS_INFO("velocidad: %f", twist_msg.linear.y);
				//usleep(200000); //sleep 5sec (expresed in nanoseconds)
		}//if start==true
		
		ros::spinOnce();
		loop_rate.sleep();
		
	}//ros::ok loop

	return 0;
}//main program
