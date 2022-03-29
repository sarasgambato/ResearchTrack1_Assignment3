#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

std_srvs::Empty rst;

// declaration of publisher and message
ros::Publisher pub;
geometry_msgs::Twist my_vel;

// variable to set the modality
bool assisted = false;

// variable for the threshold of the wall
float wall_th = 1;

// variables to temporarily store the velocities
float lin;
float ang;

std::string menu = R"(
***********************************************
You chose to drive the robot manually!
The default modality is the non-assisted keyboard.

Press:	a/A to drive with the assisted keyboard

Press:	b/B to go back to the main menu
***********************************************
)";

std::string assisted_menu = R"(
***********************************************
You are in the assisted modality!

Press:	n/N to go back to the default modality
***********************************************
)";

char chooseMod()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

float getMinimum(int start, int end, float distances[])
{
    	/* Function to get the minimum distance
        Arguments:  - distances(float[]): array of distances from the obstacles, divided in subsections
                   - start(int): first point of the subsection
                   - end(int): last point of the subsection
        Returns:    - min(float): minimum value of the subsection */
    
    	float min = 50;
    	for(int i = start; i < end; i++) 
    	{
        	if (distances[i] < min)
            		min = distances[i];
    	}
    	
    	return min;
}

void avoidCollision(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(assisted)
	{
	    	/* Callback to avoid hitting the walls */

	    	// we know that the robot can see obstacles in a range of 180 degrees, and that it stores
	    	// distances from obstacles in a vector with 720 elements, where the first element
	    	// represents the distance on the far right and the last element represents the distance
	    	// on the far left. So we get the minimum value of three subsections from this vector: right, left, front.
	    	
	    	
	    	float distances[720];

	    	for(int i = 0; i < 720; i++)
			distances[i] = msg->ranges[i];

	    	float min_right = getMinimum(0, 69, distances);
	    	float min_front = getMinimum(300, 419, distances);
	    	float min_left = getMinimum(650, 719, distances);
	  
    		my_vel.linear.x = lin;
    		my_vel.angular.z = ang;

		// if the wall in front of the robot is too close, it cannot move forward
		if (min_front < wall_th)
		{
			my_vel.linear.x = 0;
    			system("clear");
			std::cout << "Detected wall in front of the robot. Turn left or right!\n";
		}
		
		// if the wall on the left of the robot is too close, the robot turns right
	    	else if (min_left < wall_th)
	    	{
	    		my_vel.angular.z = -1;
    			system("clear");
	    		std::cout << "Detected wall on the left of the robot! Turning right...\n";
	    	}
	    	
	    	// if the wall on the right of the robot is too close, the robot turns left
	    	else if (min_right < wall_th)
	    	{
	    		my_vel.angular.z = 1;
    			system("clear");
	    		std::cout << "Detected wall on the right of the robot! Turning left...\n";
	    	}
	    	
	    	else
	    	{
    			system("clear");
    			
	    		// prompt the user
    			std::cout << assisted_menu;
    		}
    		
	    	pub.publish(my_vel);
    	}
    	
    	return;
}

void updateVel(const geometry_msgs::Twist::ConstPtr& msg)
{
	/* Callback to update the velocity of the robot */
	
	lin = msg -> linear.x;
	ang = msg -> angular.z;
	
	my_vel.linear.x = lin;
	my_vel.angular.z = ang;
	
	pub.publish(my_vel);
}

int main(int argc, char **argv)
{
	// initialize the node, set up the NodeHandle for handling the communication with the ROS system
    	ros::init(argc, argv, "keyboard");
    	ros::NodeHandle nh;

    	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    	
    	ros::Subscriber subC = nh.subscribe("/scan", 1, avoidCollision);
    	ros::Subscriber subV = nh.subscribe("/cmd_key_vel", 1, updateVel);
    	
    	ros::AsyncSpinner spinner(4);
  	spinner.start();
	
	while(1)
	{
		switch(chooseMod())
		{
			case 'a':
			case 'A':
				assisted = true;
				break;
				
			case 'n':
			case 'N':
				assisted = false;
				break;
				
			case 'b':
			case 'B':
				my_vel.linear.x = 0;
				my_vel.angular.z = 0;
				pub.publish(my_vel);
				return 0;
				break;
				
			default:
				std::cout << "Wrong key pressed, please try again.\n";
				sleep(2);
        			break;
		}
	}
	
	spinner.stop();
		
    	return 0;
}
