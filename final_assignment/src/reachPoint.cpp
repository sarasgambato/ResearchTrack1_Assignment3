#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "time.h"

std_srvs::Empty rst;

// variables to store the coordinates of the point I want to reach
float xCoord, yCoord;

// declaration of publishers to set and cancel the goal
ros::Publisher pubG;
ros::Publisher pubC;

// declaration of subscribers
ros::Subscriber subG;
ros::Subscriber subS;

// variable to store the goal
move_base_msgs::MoveBaseActionGoal goal;

// variable for the goal id
std::string goal_id;

// variable for the goal to cancel
actionlib_msgs::GoalID id_cancel;

// variable to store the status of the goal
int goal_status;

// flag for the goal
bool flag = false;

std::string menu = R"(
***********************************************
You chose to let the robot drive on its own!
Press:	p/P to set the coordinates of the goal
Press:	c/C to cancel the current goal

Press:	b/B to go back to the main menu
***********************************************
)";

void getID(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

char choice()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    	
    	// show the current goal
    	if(flag)
    	{
    		std::cout << "Current goal: (" << xCoord << ", " << yCoord << ")\n";
    	}
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

void setGoal()
{	
	/* Function to set the goal */

	// Firstly the function prompts the user to insert a valid point to
	// be reached; then, after a valid input is detected, the goal parameters
	// are set and the flag of the goal is set to true.

	system("clear");
	
	std::cout << "Enter a point (x, y) that the robot will try to reach\n\n";
	
	std::cout << "x: ";
	std::cin >> xCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: ";
		std::cin >> xCoord;
    	}
    	
    	std::cout << "y: ";
	std::cin >> yCoord;
	while(std::cin.fail())
    	{
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		std::cout << "Invalid input, enter a number: ";
		std::cin >> yCoord;
    	}
    	
    	// set the goal parameters
    	goal.goal.target_pose.pose.position.x = xCoord;
    	goal.goal.target_pose.pose.position.y = yCoord;
    	goal.goal.target_pose.header.frame_id = "map";
	goal.goal.target_pose.pose.orientation.w = 1;
	
	pubG.publish(goal);
	
	// I need to subscribe to these 2 topics to get the id of the goal and its status
	ros::NodeHandle nh;
	subG = nh.subscribe("/move_base/feedback", 1, getID);
    	subS = nh.subscribe("/move_base/status", 1, getStatus);
    	
	flag = true;
}

void cancelGoal()
{	
	/* Function to cancel the goal */

    	// The function checks whether a goal has been set: if not, a message is printed
    	// warning the user that no goal has been set yet, otherwise the function cancels the 
    	// current goal, sets the flag to false and informs the user that the operation succeded.
    	
	if(flag)
	{
		id_cancel.id = goal_id;
		subS.shutdown();
		subG.shutdown();
		pubC.publish(id_cancel);
		
		std::cout << "The goal has been correctly cancelled!\n";
		flag = false;
	}
	else
		std::cout << "No goal has been set yet!\n";
		
	sleep(2);
}

void getID(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
	/* Callback to get the id of the current goal */
	
	goal_id = msg -> status.goal_id.id;
}

void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	/* Callback to get the status of the current goal */
	
	if(goal_id == msg -> status_list[0].goal_id.id)
		goal_status = msg -> status_list[0].status;
		
	// goal SUCCEDED
	if(goal_status == 3)
	{
		std::cout << "The goal was successfully reached!\n";
		goal_status = -1;
		cancelGoal();
	}
	
	// goal ABORTED
	if(goal_status == 4)
	{
		std::cout << "The goal was aborted!";
		goal_status = -1;
		cancelGoal();
	}
	
	// goal REJECTED
	if(goal_status == 5)
	{
		std::cout << "The goal was rejected!";
		goal_status = -1;
		cancelGoal();
	}
	
}

int main(int argc, char **argv)
{
	// initialize the node, set up the NodeHandle for handling the communication with the ROS system
    	ros::init(argc, argv, "reachPoint");
	ros::NodeHandle nh;

    	pubG = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    	pubC = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    	
    	ros::AsyncSpinner spinner(4);
  	spinner.start();
	
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
        			
    			case 'b':
    			case 'B':
    				if(flag)
    					cancelGoal();
    				return 0;
    				break;
    				
    			default:
    				std::cout << "Wrong key pressed, please try again.";
        			sleep(2);
        			break;
    		}
    	}

	spinner.stop();
	
    	return 0;
}
