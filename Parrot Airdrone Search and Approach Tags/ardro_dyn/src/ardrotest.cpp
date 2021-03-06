#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>    //Header for dynamic reconfiguration
#include <ardro_dyn/ardroparamsConfig.h>   //Header of parameters used by D. Reconfiguration
#include <std_msgs/Empty.h>                //Header for Empty msgs
#include <ar_pose/ARMarkers.h>		   //Header for tag handle 
#include <std_srvs/Empty.h>		   //Header for Empty srvs
#include <unistd.h>			   //Header for Delay handle	
#include <iostream>			   //Header for Handle input/output from terminal c++	
#include <math.h>			   //Header for Math computations C++	

#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>
#include<tf/transform_listener.h>
using namespace std;
/* *************************************************************************
VARIABLES
***************************************************************************/
 using namespace std;

 std_msgs::Empty empty_msg; 		//Empty msgs variable
 std_srvs::Empty empty_srv;		//Empty srvs variable
 geometry_msgs::Twist hover_msg;   //Variable for hover functionality
 geometry_msgs::Twist velo_msg;
 geometry_msgs::Twist velAng_msg;
 ros::Publisher cmd_vel;
 std::vector<double> desired_pose_;
 bool start=false;				//Start control for dynamic reconfiguration
 bool stop=false;				//Stop control 
 double vel_x, vel_y, vel_z, Roll, Pitch, Yaw;	//Variables for dynamic reconfiguration
 double x, y, z, xw, yw, zw;
 double xd=0, yd=0, zd=1, yawd=1.57;
 double ex, ey, ez, enorm, ex_act,ey_act,ez_act;
 double ex_ini=0, ey_ini=0, ez_ini=0;
 double roll_ar,pitch_ar,yaw_ar,roll_arw,pitch_arw,yaw_arw;
 double start_timer,stop_timer;
 float end_time = 0;
 bool is_marker_ok = false, is_marker_okw = false;
 int counter=0;
 bool tkflag = false, hoflag=false, tgflagb = false, tgflagf= false, step2=false; //flags to keep control on the execution of S.W.
 bool markflagb, markflagf, landflag=false; 
 float errorx=0, errory=0, errorz=0;
 float P_errorx=0, P_errory=0, P_errorz=0;  
 float I_errorx=0, I_errory=0, I_errorz=0;
 float D_errorx=0, D_errory=0, D_errorz=0; 

/* *************************************************************************
CALLBACK BLOCK DEFINITION
***************************************************************************/
// CALLBACK FOR DYNAMIC RECONFIGURATION 
void dynrec_callback(ardro_dyn::ardroparamsConfig &config, uint32_t level)
{
	ROS_INFO("START: %s New Position: (%f,%f,%f). Vxyz: (%f,%f,%f) RPY: (%f,%f,%f)", config.start ? "True" : "False", config.new_x, config.new_y, config.new_z, vel_x, vel_y, vel_z, Roll, Pitch, Yaw);

	// Store new acquired values
	desired_pose_.clear();
	desired_pose_.push_back(config.new_x);
	desired_pose_.push_back(config.new_y);
	desired_pose_.push_back(config.new_z);
	start = config.start;
	vel_x = config.vel_x;
	vel_y = config.vel_y;
	vel_z = config.vel_z;	
	Roll = config.Roll;
	Pitch = config.Pitch;
	Yaw = config.Yaw;
}
// CALLBACK FOR FLOOR MARKERS DETECTION AND POSITION MEASUREMENT
void ARtag_CBfloor(const ar_pose::ARMarkers::ConstPtr& msg)
{ 

    if (!msg->markers.empty())
    {	
	x = msg->markers.at(0).pose.pose.position.x;
	y = msg->markers.at(0).pose.pose.position.y;
	z = msg->markers.at(0).pose.pose.position.z;
	
	tf::Quaternion qt;
    	double roll,pitch,yaw;
    	tf::quaternionMsgToTF(msg->markers.at(0).pose.pose.orientation, qt);
    	tf::Matrix3x3(qt).getRPY(roll,pitch,yaw);
	
    	roll_ar=roll;
    	pitch_ar=pitch;
    	yaw_ar=yaw;
	markflagb=true;  //mark detected flag
	if(std::isnan(roll) || !std::isfinite(roll) || (msg->markers.at(0).pose.pose.position.z<0))
	{
			markflagb = false;
	}
    }
    else    
    {
	ROS_INFO("NO MARK ERROR");
	markflagb=false;	
    }   
}
// CALLBACK FOR WALL MARKERS DETECTION AND POSITION MEASUREMENT
void ARtag_CBwall(const ar_pose::ARMarkers::ConstPtr& msg)
{
   if (!msg->markers.empty())
    {	
	xw = msg->markers.at(0).pose.pose.position.x;		
	yw = msg->markers.at(0).pose.pose.position.y;		
	zw = msg->markers.at(0).pose.pose.position.z;
	
	tf::Quaternion qtw;
    	double rollw,pitchw,yaww;
    	tf::quaternionMsgToTF(msg->markers.at(0).pose.pose.orientation, qtw);
    	tf::Matrix3x3(qtw).getRPY(rollw,pitchw,yaww);
	
    	roll_arw=rollw;
    	pitch_arw=pitchw;
    	yaw_arw=yaww;
	markflagf=true;  //mark detected flag
	if(std::isnan(rollw) || !std::isfinite(rollw) || (msg->markers.at(0).pose.pose.position.z<0))
	{
			markflagf = false;
	}
    }
    else    
    {
	ROS_INFO("NO MARK ERROR");
	markflagf=false;	
    }   

  
}
/***************************************************************************
SUBRUTINES
***************************************************************************/
/* compu_error subrutine return the error between the desired position and the actual position of the drone, the inputs of 
   this function are the desired position on the real world (xd,yd,zd), the position given by the ar_pose (x,y,z) and finally
   an integer which tells to the function which error should be returned (1=returns ex, 2=ey and so on). It is important to take into account the orientation of the cameras in order to asign proper values of x,y,z)*/
double compu_error (double x, double y, double z, double xd, double yd, double zd, int index) 
{							  	
	//int ex,ey,ez,enorm;
	ex = xd-x;
        ey = yd-y;
        ez = zd-z;   
	//eyaw = yawd-Yaw;
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
/* The following functions returns the computed values of the PID control for every position each axis*/
float pidx (float ex_ini)
{

	float errorx= ex_act;
	float P_errorx = errorx;
	float I_errorx =I_errorx + ex_ini;
	float D_errorx = errorx - ex_ini;
                 
        if (errorx<0.05 && errorx>-0.05){
	P_errorx=0;
	I_errorx=0;
	D_errorx=0;
	errorx=0;
	}  
		
	return 0.08*P_errorx +0.04*I_errorx+ 0.08*D_errorx ;
                
}

float pidy (float ey_ini)
{

	float errory= ey_act;
	float P_errory = errory;
	float I_errory =I_errory+ ey_ini;
	float D_errory = errory - ey_ini;
                 
        if (errory<0.05 && errory>-0.05){
	P_errory=0;
	I_errory=0;
	D_errory=0;
	errory=0;
	}  
		
	return 0.08*P_errory +0.04*I_errory+ 0.08*D_errory ;
                
}

float pidz (float ez_ini)
		{

				
		float errorz = ez_act;
		float P_errorz = errorz;
		float I_errorz =I_errorz+ ez_ini;
		float D_errorz = errorz - ez_ini;
                 
                if (errorz<0.05 && errorz>-0.05){
		P_errorz=0;
		I_errorz=0;
		D_errorz=0;
		errorz=0;
		}  
		
		return 0.08*P_errorz +0.04*I_errorz+ 0.08*D_errorz ;
                
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
	start = false;
	
	// HOVER MODE VARIABLES
	/* By publishing values of zero on each linear and angular value we achieve the hover mode on the drone, 
	   this help us to stop undesired movements along the execution of the control program */
	hover_msg.linear.x=0.0; 
	hover_msg.linear.y=0.0;
	hover_msg.linear.z=0.0;
	hover_msg.angular.x=0.0; 
	hover_msg.angular.y=0.0;
	hover_msg.angular.z=0.0;

	//Subscriber ar_pose_marker configuration
	ros::Subscriber subf = n.subscribe("/ar_pose_marker", 1000, ARtag_CBfloor);
	ros::Subscriber subw = n.subscribe("/ar_pose_marker", 1000, ARtag_CBwall);
	//Dynamic reconfigure Configuration for call back
	dynamic_reconfigure::Server <ardro_dyn::ardroparamsConfig> server;
	dynamic_reconfigure::Server <ardro_dyn::ardroparamsConfig>::CallbackType f;

	f = boost::bind(&dynrec_callback, _1, _2);
	server.setCallback(f);

	// Taking off and Landing setup
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff",5);		
	ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land",5);
	//Setup for togglecam
	ros::ServiceClient toggleCam = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
	//Setup for control velocity
	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1); 
	
	while (ros::ok())
	{
		/* The following program will run only if the value of start is set to true, this can be done on the panel of the   			   dynamic reconfiguration or just uncomment the line start=true */		
		ROS_INFO("Please start the routine...");	
		//start = true;
		while(start==true)
		{
			/* Take of the drone */			
			if(tkflag==false){						
				takeoff.publish(empty_msg); // launch the drone
				ROS_INFO("Taking off, watch your head!!!");
				//usleep(1000000); 	    // wait for 1 sec
				cmd_vel.publish(hover_msg);	// Put the drone on hover mode
				tkflag=	true;		    // turn on the tkflag 						
			}//end tkflag

			/* If the drone is on the air keep hovering for stability (drone flat)*/
			if (tkflag==true && markflagb==false && step2==false){	
				cmd_vel.publish(hover_msg);	// publishing hover msg 
				ROS_INFO("Hovering ...");
				//usleep(1000000); 
				hoflag=true;
			}//end hoflag

			/* If the drone is hovering toggle to bottom camera */
			if (tkflag==true && hoflag==true && tgflagb==false && step2==false){
				toggleCam.call(empty_srv);  // toggle the camera to the bottom
                                                            // the (default = frontal camera
				ROS_INFO("Set Bottom Camera");
				tgflagb=true;
			}// end toggle bottom camera

			/* If the camera is set to bottom start to look for the floor marker */
			if(tgflagb==true && step2==false){
				ROS_INFO("Looking for Marker ...");

				/* If the mark is detected start the control */
				if(markflagb==true && step2==false){
					ROS_INFO("Marker Detected ...");
					start_timer =(double)ros::Time::now().toSec(); // Start timer for 5 sec
					
					/* Control loop for maintain the drone and the tag for 5 sec */
					while(end_time<=5.0 && markflagb==true){
						ROS_INFO("Control loop 1...");
											
						//ROS_INFO("x: %f y: %f z: %f ",x,y,z);
						/* Publishing the velocities on cdm_vel based on the pid control */
						velo_msg.linear.x = pidx(ex_ini);						
						velo_msg.linear.y = pidy(ey_ini);
						velo_msg.linear.z = pidz(ez_ini);
						velo_msg.angular.x = 0.08*roll_ar;
						velo_msg.angular.y = 0.08*pitch_ar;
						velo_msg.angular.z = 0.08*yaw_ar;
						cmd_vel.publish(velo_msg);
						/* wait for 0.5 sec to measure the actual error since the initial movement */
						usleep(500000);		
						cmd_vel.publish(hover_msg);
						/* Using the compu_error function we obtain the error */
						ex_act=compu_error(x,y,z,xd,yd,zd,1);	
						ey_act=compu_error(x,y,z,xd,yd,zd,2);
						ez_act=compu_error(x,y,z,xd,yd,zd,3); 
						/* The actual error will be the initial error for the next loop */
						ex_ini = ex_act;						
						ey_ini = ey_act;
						ez_ini = ez_act;
						/* Compute the time since the control loop start*/
						stop_timer = (double)ros::Time::now().toSec(); // Stop time for 5 sec
						end_time = stop_timer - start_timer;
						ROS_INFO("Timer: %f",end_time);
						//ROS_INFO("eza = %f ",ez_act);	
						ros::spinOnce();
						loop_rate.sleep();
					}// end control loop

					/* If the time is bigger than 5 sec start step 2 */
					if (end_time>=5.0){
						step2 = true;
							
					}// end of step2 switch					
				}//end of floor mark detected
				
				/* If there is no mark detected go up and remaing hovering */
				if(markflagb==false && step2==false){
					ROS_INFO("Error no Mark GO UP...");
					
						velo_msg.linear.x = 0;						
						velo_msg.linear.y = 0;
						velo_msg.linear.z = 0.5;
						velo_msg.angular.x = 0;
						velo_msg.angular.y = 0;
						velo_msg.angular.z = 0;
						cmd_vel.publish(velo_msg);				
				}//end of no mark detected
			}//end of control routine
			// reset timers			
			start_timer=0;
			stop_timer=0;
			end_time=0;

			/* Routine for driving the drone to the mark on th wall (step2==true) */
			if(step2==true){
				ROS_INFO("STEP 2");
				ex_ini=0;
				ey_ini=0;
				ez_ini=0;
				cmd_vel.publish(hover_msg);
				ROS_INFO("Hover ...");
				usleep(1000000);
				if(tgflagf==false){				
					toggleCam.call(empty_srv);  // toggle the camera to the front
                                                                    // the (default = frontal camera)
					ROS_INFO("Set Front Camera");
					tgflagf=true;
				}
				if (tgflagf==true && landflag==false){
					/* If there is no mark detected rotate and hover until ar_pose return a valid flag */
					if(markflagf==false){  
						ROS_INFO(" No wall tag"); 
						velo_msg.angular.z = 1;
						//usleep(500000);
						//cmd_vel.publish(hover_msg); 						
					
					}// end no wall mark
					/* Routine to drive the drone to the wall mark and remains on that position */
					if(markflagf==true){
						ROS_INFO("Wall tag detected");
						
						start_timer =(double)ros::Time::now().toSec(); // Start time for 5 sec
						while(end_time<=10.0 && markflagf==true){
							ROS_INFO("Going to wall marker...");
							/* Here we use a function pidx and pidz but the input values are switching
							   between them because the orientation of the frontal camera, we also have 								   to consider the movement of the drone, if we want to reach the desired 
							   position we need to publish negative values on linear x, linear y, 
							   angular x, angular y*/
							velo_msg.linear.x = -pidx(ex_ini); 					
							velo_msg.linear.y= -pidy(ey_ini);
							velo_msg.linear.y = pidz(ez_ini);
							velo_msg.angular.x = -0.08*roll_arw;
							velo_msg.angular.y = -0.08*pitch_arw;
							velo_msg.angular.z = 0.08*yaw_arw;
							cmd_vel.publish(velo_msg);
							usleep(500000);
							//Compute error
							/* Since we are changing the camera and its orientation is different
							   from the world is necessary to change the axis to compute the error
							   and drive the drone */
							ex_act=compu_error(zw,yw,xw,zd,yd,xd,1); 
							ey_act=compu_error(zw,yw,xw,zd,yd,xd,2);
							ez_act=compu_error(zw,yw,xw,zd,yd,xd,3); 
							ex_ini = ex_act;						
							ey_ini = ey_act;
							ez_ini = ez_act;
							stop_timer = (double)ros::Time::now().toSec(); // Stop time for 5 sec
							end_time = stop_timer - start_timer;
							ROS_INFO("Timer: %f",end_time);
							ros::spinOnce();
							loop_rate.sleep();
						}// end control2 while
						if(end_time>10.0){
							landflag=true;
						}// end of land set to true
					}//end of mark detected
					
				}//end tgflagf
				/* If the control routine already finish, land the drone */
				if(landflag==true){
					land.publish(empty_msg);    // Land drone
					ROS_INFO("Landing the drone ..."); 	
				
				}//end of landing
				
			}// end step2
			
			ros::spinOnce();
			loop_rate.sleep();
		}//while start 
		
		//usleep(4000000);
		/* The following code will stop the execution of the program if the start bottom is unselected*/
		land.publish(empty_msg);    // Land drone
		ROS_INFO("EMERGENCY STOP"); 		
		ros::spinOnce();
		loop_rate.sleep();
		
	}//whileros::ok loop
	
	return 0;

}//main program
