#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

// variables to store the coordinates of the point I want to reach
float xCoord, yCoord;

// declaration of publishers to set and cancel the goal
ros::Publisher pubG;
ros::Publisher pubC;

// variable to store the goal
move_base_msgs::MoveBaseActionGoal goal;

std::string menu = R"(
Press: \tp/P to set the coordinates of the goal
Press: \tc/C to cancel the current goal
)";

char choice()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

void setGoal()
{
	system("clear");
	
	std::cout << "Enter a point (x, y) that the robot will try to reach\n\n";
	
	std::cout << "x: \n";
	std::cin >> xCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: \n";
		std::cin >> xCoord;
    	}
    	
    	std::cout << "y: \n";
	std::cin >> yCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: \n";
		std::cin >> yCoord;
    	}
    	
    	// set the goal parameters
    	goal.goal.target_pose.pose.position.x = xCoord;
    	goal.goal.target_pose.pose.position.y = yCoord;
    	goal.goal.target_pose.header.frame_id = "map";
	goal.goal.target_pose.pose.orientation.w = 1;
	
	pubG.publish(goal);
}

void cancelGoal()
{

}


int main(int argc, char **argv)
{
    	ros::init(argc, argv, "reachPoint");
    	ros::NodeHandle nh;

    	pubG = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    	pubC = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    	
    	while(1)
    	{
    		switch(choice())
    		{
    			case 'p':
    			case 'P':
    				setGoal();
    				break;
    				
    			case 'c':
    			case 'C':
    				cancelGoal();
    				break;
    				
    			default:
    				std::cout << "Wrong key pressed, please try again.\n";
        			sleep(2);
        			break;
    		}
    	}

    	return 0;
}
