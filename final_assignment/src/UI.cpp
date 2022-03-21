#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "stdio.h"
#include "time.h"

std_srvs::Empty rst;

std::string menu = R"(
Choose how to control the robot:
Press: \t1 to make the robot reach a point autonomously
Press: \t2 to drive the robot with the keyboard
				
Press: \tr/R to reset the simulation
Press: \te/E to exit the program
)";

char getInput()
{
	char input;

    	system("clear");
    
    	// prompt the user
    	std::cout << menu;
    
   	// get the keybord input
  	std::cin >> input;
    
   	return input;
}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "UI");

    	while(1) 
    	{
        	switch(getInput()) 
        	{
        		case '1':
        			system("rosrun final_assignment auto_controller");
        			break;
        			
        		case '2':
        			system("rosrun final_assignment keyboard_controller");
        			break;
        			
        		case 'r':
        		case 'R':
        			ros::service::call("gazebo/reset_simulation", rst);
        			break;
        			
        		case 'e':
        		case 'E':
        			std::cout << "Exiting the program...";
        			sleep(2);
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
