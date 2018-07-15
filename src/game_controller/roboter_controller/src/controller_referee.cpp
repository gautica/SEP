#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <roboter_controller/Status.h>
#include <string>
#include <time.h>

using namespace ros;
using namespace std;

struct resource {
    string name;
    double x;
    double y;
};

vector<resource> resources;

Subscriber link_sub;       //to get all links in gazebo
Publisher link_pose_publisher; //to put the resource far away
Publisher status_pub;          // user information
roboter_controller::Status status;  // msg for status

double gripper_0_x;     // all gripper stats
double gripper_0_y;
double gripper_1_x;
double gripper_1_y;

double goal_x = -0.863439;   //goal stats
double goal_y = -4.151809;
double goal_radius = 0.953929;


bool winner;         //winning conditions
bool robot_0_task_1;
bool robot_0_task_2;
bool robot_1_task_1;
bool robot_1_task_2;

string task[2];  // 2 products


void link_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg) {
   resources.clear();
   for(int i  = 0; i < msg->name.size(); i++) {  //for all links
        if (msg->name[i].find("robot_0::gripper_link") != std::string::npos) { // get the gripper link pose of robot 0
            gripper_0_x = msg->pose[i].position.x;
            gripper_0_y = msg->pose[i].position.y;
        }
        if (msg->name[i].find("robot_1::gripper_link") != std::string::npos) { // get the gripper link pose of robot 1
            gripper_1_x = msg->pose[i].position.x;
            gripper_1_y = msg->pose[i].position.y;
        }
        if(msg->name[i].find("Resource") != std::string::npos) //find resource links and save them if they are in goal radius
        {  
            if (sqrt( pow(msg->pose[i].position.x - goal_x, 2) + pow(msg->pose[i].position.y - goal_y, 2)) < goal_radius ) { 
                resource new_resource;
                new_resource.name = msg->name[i];
                new_resource.x = msg->pose[i].position.x; 
                new_resource.y = msg->pose[i].position.y; 
                resources.push_back(new_resource);
            }
        }
    } 
}

//random product essay
void get_produkt() {
    string produkte[] = {"Blue", "Yellow", "Red", "Violet", "Orange", "Green", "Black"};
    srand (time(NULL));
    int random_num = rand() % 7;
    task[0] = produkte[random_num];
    status.tasks[0] =(int) random_num;
    int tmp= random_num;
    while (random_num == tmp) {
      random_num = rand() % 7;
    }
    task[1] = produkte[random_num];
    status.tasks[1] =(int) random_num;
}

int check_win() {
    for(int i  = 0; i < resources.size(); i++) { // for every resource in goal
        if (resources[i].name.find(task[0])  != std::string::npos) { // if the resource is task[0]
            double dist_to_robot_0 = sqrt( pow(resources[i].x - gripper_0_x, 2) + pow(resources[i].y - gripper_0_y, 2));
	    double dist_to_robot_1 = sqrt( pow(resources[i].x - gripper_1_x, 2) + pow(resources[i].y - gripper_1_y, 2));    
            if(dist_to_robot_0 > 0.05 && dist_to_robot_0 < 0.15) { //if the ressource is dropped from robot 0  //hier ist noch ein bug, wenn man beim fahren abgibt wird es nicht registriert
                gazebo_msgs::LinkState set_resource;          // "despawn" it
                set_resource.link_name = resources[i].name;
                set_resource.pose.position.x = 1000;
                link_pose_publisher.publish(set_resource);
                Duration(0.1).sleep(); // to pub status once and not 100 times
                robot_0_task_1 = true;
                status.status = "Player 1 created the " + task[0] + " product";
                status.player1_tasks[0] = true;
                status_pub.publish(status);
                if (robot_0_task_1 == true && robot_0_task_2 == true) { // if both tasks are finished
                     status.status = "Player 1 wins!";
					 status.go =10;
                     status_pub.publish(status);
                     winner = true;
                }   
	    } 
            else if (dist_to_robot_1 > 0.05 && dist_to_robot_1 < 0.15) {
	        status.status = "Spieler 2 hat gewonnen!";
                gazebo_msgs::LinkState set_resource;
                set_resource.link_name = resources[i].name;
                set_resource.pose.position.x = 1000;
                link_pose_publisher.publish(set_resource);
                Duration(0.1).sleep(); // to pub status once and not 100 times
                robot_1_task_1 = true;
                status.status = "Player 2 created the" + task[0] + " product";
                status.player2_tasks[0] = true;
                status_pub.publish(status);
                if (robot_1_task_1 == true && robot_1_task_2 == true) {
                     status.status = "Player 2 wins!";
					 status.go =20;
                     status_pub.publish(status);
                     winner = true;
                }
	    }
        }
       if (resources[i].name.find(task[1])  != std::string::npos) {
            double dist_to_robot_0 = sqrt( pow(resources[i].x - gripper_0_x, 2) + pow(resources[i].y - gripper_0_y, 2));
	    double dist_to_robot_1 = sqrt( pow(resources[i].x - gripper_1_x, 2) + pow(resources[i].y - gripper_1_y, 2));    
            if(dist_to_robot_0 > 0.05 && dist_to_robot_0 < 0.15) {
                gazebo_msgs::LinkState set_resource;
                set_resource.link_name = resources[i].name;
                set_resource.pose.position.x = 1000;
                link_pose_publisher.publish(set_resource);
                Duration(0.1).sleep(); // to pub status once and not 100 times
                robot_0_task_2 = true;
                status.status = "Player 1 created the" + task[1] + " product";
                status.player1_tasks[1] = true;
                status_pub.publish(status);
                if (robot_0_task_1 == true && robot_0_task_2 == true) {
                     status.status = "Player 1 wins!";
					 status.go =10;
                     status_pub.publish(status);
                     winner = true;
                }   
	    } 
            else if (dist_to_robot_1 > 0.05 && dist_to_robot_1 < 0.15) {
	        status.status = "Spieler 2 hat gewonnen!";
                gazebo_msgs::LinkState set_resource;
                set_resource.link_name = resources[i].name;
                set_resource.pose.position.x = 1000;
                link_pose_publisher.publish(set_resource);
                Duration(0.1).sleep(); // to pub status once and not 100 times
                robot_1_task_2 = true;
                status.player2_tasks[1] = true;
                status.status = "Player 2 created the" + task[1] + " product";
                status_pub.publish(status);
                if (robot_1_task_1 == true && robot_1_task_2 == true) {
                     status.status = "Player 2 wins!";
					status.go =20;
                     status_pub.publish(status);
                     winner = true;
                }
	    }
        }        
    }
    return 0;
}







int main(int argc, char** argv)
{
    init(argc, argv, "roboter_controller_1");
    NodeHandle n;

    link_sub = n.subscribe("/gazebo/link_states", 1, link_Callback);
    status_pub = n.advertise<roboter_controller::Status>("/status", 1);
    link_pose_publisher = n.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);

    sleep(1); // wait for connection
    status.go = 7;
    get_produkt(); // create product essay
    status.status = "Task: Create a " + task[0] + " and a " + task[1] + " product";
    status_pub.publish(status);
    //
    sleep(3);
	status.status = "The match will start in 5 seconds";
    status.go = 6;
    status_pub.publish(status);
    sleep(1);

    status.status = "The match will start in 5 seconds";
    status.go = 5;
    status_pub.publish(status);
    sleep(1);

    status.status = "The match will start in 4 seconds";
    status.go = 4;
    status_pub.publish(status);
    sleep(1);

    status.status = "The match will start in 3 seconds";
    status.go = 3;
    status_pub.publish(status);
    sleep(1);

    status.status = "The match will start in 2 seconds";
    status.go = 2;
    status_pub.publish(status);
    sleep(1);

    status.status = "The match will start in 1 seconds";
    status.go = 1;
    status_pub.publish(status);
    sleep(1);
	
	status.status = "Go!";
    status.go = 0;
    status_pub.publish(status);

   
	

    while (ok() && winner == false) {    //check winning conditions and callback simulation
        spinOnce();
        check_win();
    }    
   //
}

