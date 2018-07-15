#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <roboter_controller/Status.h>
#include "roboter_controller/Attach.h"
#include "roboter_controller/got_resource.h"

using namespace ros;
using namespace std;

struct resource {
    string name;
    double distance_to_gripper;
    double x;
    double y;
};

vector<resource> resources;
resource nearest_resource;

double robot_gripper_0_x;
double robot_gripper_0_y;
double robot_gripper_1_x;
double robot_gripper_1_y;
double robot_base_x;
double robot_base_y;

Subscriber controller; //get controller
Subscriber status_sub; //get controller
Publisher status_pub; //
Publisher twist_publisher;     //to set robot velocity
geometry_msgs::Twist velocity; //linear and angular with x,y,z in script
Subscriber link_sub;       //to get all links in gazebo
Publisher link_pose_publisher; // to set ressource link in gripper link
Publisher machine_publisher; //Publish Message To Referee -> Start Machine

ServiceClient attach_client;
ServiceClient detach_client;

bool grabbed; //because bug that robot drives and drops abnd instanty he can drop again

bool L2;
double L2_axis_multiplier; //0-1  
bool R2;
double R2_axis_multiplier; //0-1
bool L3_left;
bool L3_right;
bool R1;
bool L1;
bool Share;
bool R3_press;

bool camera_top_view;

roboter_controller::Status local_status;

void controller_Callback(const sensor_msgs::Joy::ConstPtr &msg) {
  if (local_status.go == 0) {
        if (msg->buttons[7] == 1) { //R2 Button is pressed and how strong
           R2 = true;
           R2_axis_multiplier = ((-1*msg->axes[5])/2)+ 0.5;// define multiplier
        } else {
            R2 = false;
        }
        if (msg->buttons[6] == 1) { //L2 Button is pressed and how strong
            L2 = true;
            L2_axis_multiplier = (-1*msg->axes[2]/2)+ 0.5;
        } else {
            L2 = false;
        }
        if (msg->axes[0] == 1) {                         // Left Rotate Stick is left
            L3_left = true;
        } else {
            L3_left= false;
        }
        if (msg->axes[0] == -1) {                      // Left Rotate Stick is right
            L3_right = true;
        } else {
            L3_right = false;
        }
        if (msg->buttons[5] == 1) {                      //R1 Button i pressed
            R1 = true;
        } else {
            R1 = false;
        }
        if (msg->buttons[4] == 1) {                      //Left Rotate Stick is right
            L1 = true;
        } else {
            L1 = false;
        }
        if (msg->buttons[8] == 1) {                      //Share button is pressed
            Share = true;
        } else {
            Share = false;
        }
        if (msg->buttons[12] == 1) {                      //Share button is pressed
              camera_top_view = !camera_top_view;
              local_status.player1_topview = camera_top_view;
              status_pub.publish(local_status);
        }
 }
 if (msg->buttons[1] == 1) {                      //Share button is pressed
     local_status.sim_closed = true;
     status_pub.publish(local_status);
 }
}


void status_Callback(const roboter_controller::Status::ConstPtr &msg) {
    local_status = *msg;
}

void link_Callback(const gazebo_msgs::LinkStates::ConstPtr &msg) {
   resources.clear();
   nearest_resource.distance_to_gripper = 1000;
   for(int i  = 0; i < msg->name.size(); i++) {
        if (msg->name[i].find("robot_0::gripper_link") != std::string::npos) { // get the gripper link pose of robot 
            robot_gripper_0_x = msg->pose[i].position.x;
            robot_gripper_0_y = msg->pose[i].position.y;
        }
        if (msg->name[i].find("robot_1::gripper_link") != std::string::npos) { // get the gripper link pose of robot 
            robot_gripper_1_x = msg->pose[i].position.x;
            robot_gripper_1_y = msg->pose[i].position.y;
        }
        if (msg->name[i].find("robot_0::base_footprint") != std::string::npos) { // get the base link pose of robot 
            robot_base_x = msg->pose[i].position.x;
            robot_base_y = msg->pose[i].position.y;
        }
        if(msg->name[i].find("Resource") != std::string::npos) //find resource links and save name and the distance to the gripper link
        {   
            resource new_resource;
            new_resource.name = msg->name[i];
            new_resource.x = msg->pose[i].position.x; 
            new_resource.y = msg->pose[i].position.y; 
            new_resource.distance_to_gripper = sqrt( pow(msg->pose[i].position.x - robot_gripper_0_x, 2) + pow(msg->pose[i].position.y - robot_gripper_0_y, 2) ); // save distance to gripper of robot
            if (new_resource.distance_to_gripper < nearest_resource.distance_to_gripper) {
                nearest_resource = new_resource;
            } 
            resources.push_back(new_resource); // save the resource information locally
        }
    } 
}
int grab() {
    if (nearest_resource.distance_to_gripper < 0.1 && grabbed == false ) { // if its close enough to gripper
        if (sqrt( pow(nearest_resource.x - robot_gripper_1_x, 2) + pow(nearest_resource.y - robot_gripper_1_y, 2)) > 0.05) { // and not in the other gripper
            ROS_ERROR("case");
	    gazebo_msgs::LinkState set_resource;
            set_resource.link_name = nearest_resource.name;
            ROS_ERROR("Reesss: %s", nearest_resource.name.c_str());
            set_resource.pose.position.x = robot_gripper_0_x;
            set_resource.pose.position.y = robot_gripper_0_y;
	    set_resource.pose.position.z = 0.03;
            link_pose_publisher.publish(set_resource);

            Duration(0.01).sleep();

	    //Get Model-Name
            stringstream ss(nearest_resource.name);
            vector<string> result;

            while(ss.good())
            {
                string substr;
                getline( ss, substr, ':');
                result.push_back(substr);
            }

            //Ros Service Call
            roboter_controller::Attach attach_srv;
            attach_srv.request.model_name_1 = result.at(0).c_str();
            attach_srv.request.link_name_1 = "link";
            attach_srv.request.model_name_2 = "robot_0";
            attach_srv.request.link_name_2 = "gripper_link";
            attach_client.call(attach_srv);
            grabbed = true;
        } 
    }
    return 0;
} 

int drop() {
    if (nearest_resource.distance_to_gripper > 0 && nearest_resource.distance_to_gripper < 0.05 && grabbed == true) {
        //Get Model-Name
        stringstream ss(nearest_resource.name);
        vector<string> result;
        while(ss.good())
        {
            string substr;
            getline( ss, substr, ':');
            result.push_back(substr);
        }

        //Publish Machine Messages To Referee
        roboter_controller::got_resource resource_msg;
        resource_msg.roboter_name = "robot_0";
        resource_msg.resource_name = result.at(0).c_str();

        //Ros Service Call
        roboter_controller::Attach dettach_srv;
        dettach_srv.request.model_name_1 = result.at(0).c_str();
        dettach_srv.request.link_name_1 = "link";
        dettach_srv.request.model_name_2 = "robot_0";
        dettach_srv.request.link_name_2 = "gripper_link";
        
           
        machine_publisher.publish(resource_msg);
        detach_client.call(dettach_srv);
        Duration(0.01).sleep();

        gazebo_msgs::LinkState set_resource;
        set_resource.link_name = nearest_resource.name;
        set_resource.pose.position.x = (robot_gripper_0_x - robot_base_x)*2 + robot_base_x;
        set_resource.pose.position.y = (robot_gripper_0_y - robot_base_y)*2 + robot_base_y;
        set_resource.pose.position.z = 0.03;
        link_pose_publisher.publish(set_resource);
        
        grabbed = false;
        
        ROS_INFO("hab abgelegt");
    }
    return 0;
} 

int main(int argc, char** argv)
{
    init(argc, argv, "roboter_controller_1");
    NodeHandle n;
    Duration(0.01).sleep();
    controller = n.subscribe("/joy_1", 1, controller_Callback);
    status_sub = n.subscribe("/status", 1, status_Callback);
    link_sub = n.subscribe("/gazebo/link_states", 1, link_Callback);
    twist_publisher = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1);
    link_pose_publisher = n.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);
    attach_client = n.serviceClient<roboter_controller::Attach>("/link_attacher_node/attach");
    detach_client = n.serviceClient<roboter_controller::Attach>("/link_attacher_node/detach");
    machine_publisher = n.advertise<roboter_controller::got_resource>("/roboter/resource", 1);
    status_pub = n.advertise<roboter_controller::Status>("/status", 1);
	local_status.go = 1000;
    
    while (ok()) {
        spinOnce();
        if ( R2 == true) {
            ROS_INFO("you pressed drive forward");
            if (velocity.linear.x < 0) {
                velocity.linear.x = 0;
            } 
            else if (velocity.linear.x <= 1){
                if (velocity.linear.x + R2_axis_multiplier*0.05 > 1) {
                    velocity.linear.x = 1;
                } else {
                    velocity.linear.x = velocity.linear.x + R2_axis_multiplier*0.05;
                }
            } 
            if (L2 == true) {
                velocity.linear.x = 0;     
            }
        }
        if ( L2 == true) {
            ROS_INFO("you pressed drive backwards");
            if (velocity.linear.x > 0) {
                velocity.linear.x = 0;
            }
            else if (velocity.linear.x > -1){
                if (velocity.linear.x - L2_axis_multiplier*0.05 < -1) {
                    velocity.linear.x = -1;
                } else {
                    velocity.linear.x = velocity.linear.x - L2_axis_multiplier*0.05;
                }
            } 
            if (R2 == true) {
                velocity.linear.x = 0;   
            }
        }
        if ( L3_left == true) {
            ROS_INFO("you rotate left");
            if (velocity.angular.z < 0) {
                velocity.angular.z = 0;
            } 
            else if (velocity.angular.z <= 3) {
                velocity.angular.z = velocity.angular.z + 0.3;
            }
        }
        if ( L3_right == true) {
            ROS_INFO("you rotate right");
            if (velocity.angular.z > 0) {
                velocity.angular.z = 0;
            } 
            else if (velocity.angular.z >= -3) {
                velocity.angular.z = velocity.angular.z - 0.3;
            }
        }
        if (L3_right == false && L3_left == false) { // if no rotation, set rotate to 0
            velocity.angular.z = 0;
        }
        if (R2 == false && L2 == false) { // if no rotation, set rotate to 0
            if (velocity.linear.x > 0) {
                if (velocity.linear.x - 0.05 < 0) {
                    velocity.linear.x = 0;
                } else {
                    velocity.linear.x =  velocity.linear.x- 0.05;
                }
            }
            if (velocity.linear.x < 0) {
                if (velocity.linear.x + 0.05 > 0) {
                    velocity.linear.x = 0;
                } else {
                    velocity.linear.x = velocity.linear.x + 0.05;
                }
            }
        }
        if (R1 == true) {
            grab();
        }
        if (L1 == true) {
            drop();
        }
        twist_publisher.publish(velocity);
        Duration(0.1).sleep();
    }
}
    

    
