#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

std_srvs::Empty rst;

// variables to store the coordinates of the point I want to reach
float xCoord, yCoord;

// declaration of publishers to set and cancel the goal
ros::Publisher pubG;
ros::Publisher pubC;

// variable to store the goal
move_base_msgs::MoveBaseActionGoal goal;

// variable for the goal id
std::string goal_id;

// variable for the goal to cancel
actionlib_msgs::GoalID id_cancel;

// flag for the goal
bool flag = false;

std::string menu = R"(
***********************************************
Press:	p/P to set the coordinates of the goal
Press:	c/C to cancel the current goal

Press:	r/R to reset the simulation
Press:	b/B to go back to the main menu
***********************************************
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
{	/* Function to set the goal */

	// Firstly the function prompts the user to insert a valid point to
	// be reached; then, after a valid input is detected, the goal parameters
	// are set and the flag of the goal is set to true.
	
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
	
	flag = true;
	
	pubG.publish(goal);
}


void cancelGoal()
{	/* Function to cancel the goal */

    	// Function checks whether a goal has been set: if not, a message is printed
    	// warning the user that no goal has been set yet, otherwise the function cancels the 
    	// current goal, sets the flag to false and informs the user that the operation succeded.
    	
	if(flag)
	{
		id_cancel.id = goal_id;
		pubC.publish(id_cancel);
		std::cout << "The goal has been correclty cancelled!\n";
		flag = false;
	}
	else
		std::cout << "No goal has been set yet!.\n";
		
	sleep(2);
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
    				
    			case 'r':
        		case 'R':
        			ros::service::call("gazebo/reset_simulation", rst);
        			if(flag)
    					cancelGoal();
        			break;
        			
    			case 'b':
    			case 'B':
    				if(flag)
    					cancelGoal();
    				return 0;
    				break;
    				
    			default:
    				std::cout << "Wrong key pressed, please try again.\n";
        			sleep(2);
        			break;
    		}
    	}

    	return 0;
}
