#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <pwd.h>
#include <string>

#include<gazebo_msgs/ApplyBodyWrench.h>

#include "referee_node/Spawn.h"

using namespace std;
using namespace ros;

int spawned_objects = 0;
vector<referee_node::Spawn> spawn_tasks;
bool is_working;

void spawn_Model(string name, string obj_name, float pos_x, float pos_y, float pos_z, bool add_index, int apply_force_orientation)
{ 
    ROS_ERROR("SPAWN %s", obj_name.c_str());
    spawn_tasks.pop_back();
    passwd* pw = getpwuid(getuid());
    string path(pw->pw_dir);
    path += "/.gazebo/models/";
    path += name;
    path += "/model.sdf";
        
    ifstream ifs;
    ifs.open (path.c_str());

    string content((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
        
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    
    //Model and its Pose
    gazebo_msgs::SpawnModel new_model;
    geometry_msgs::Pose new_models_pose;

    //Model name
    string object_name(obj_name);

    if(add_index)
    {
        object_name += "_";

        ostringstream convert;   // stream used for the conversion
        convert << spawned_objects;
        object_name += convert.str();
        spawned_objects++;
    }

    
    new_models_pose.position.x = pos_x;
    new_models_pose.position.y = pos_y;
    new_models_pose.position.z = pos_z;
    new_models_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    new_model.request.model_name = object_name;
    new_model.request.model_xml = content;
    new_model.request.initial_pose = new_models_pose;
    new_model.request.reference_frame = "world";
    
    cout << "Loading from path: \"" << path << "\n" << endl;
    
    if (client.call(new_model))
    {
        ROS_INFO("success");
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }

    if (name.find("Resource") != std::string::npos)
    {
       ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
       gazebo_msgs::ApplyBodyWrench apply_force;

       string body_name(object_name);
       body_name += "::link";

       apply_force.request.body_name= body_name;
       apply_force.request.reference_frame= body_name;
       apply_force.request.reference_point.x = pos_x;
       apply_force.request.reference_point.x = pos_y;
       apply_force.request.reference_point.x = pos_z;
       apply_force.request.start_time = ros::Time(0);
       apply_force.request.duration = ros::Duration(0.001);

       ROS_ERROR("FORCE: %d", apply_force_orientation);

       switch(apply_force_orientation)
       {
         case 1:
           apply_force.request.wrench.force.x = 180;  //x
         break;

         case 2:
           apply_force.request.wrench.force.y = 180;   //y
         break;

         case 3:
           apply_force.request.wrench.force.x = -180;  //-x
         break;

         case 4:
           apply_force.request.wrench.force.y = -180;    //-y
         break;

         default:
         break;
       }

       client.call(apply_force);
   }

   is_working = false;
}

void spawn_callback(const referee_node::Spawn::ConstPtr& msg)
{
    referee_node::Spawn new_task;
    new_task.name = msg->name;
    new_task.object_name = msg->object_name;
    new_task.x = msg->x;
    new_task.y = msg->y;
    new_task.z = msg->z;
    new_task.add_index = msg->add_index;
    new_task.apply_force_orientation = msg->apply_force_orientation;

    spawn_tasks.push_back(new_task);
}

int main(int argc, char** argv)
{
    init(argc, argv, "referee_receive_spawn");
    
    NodeHandle n;
    Subscriber spawn_sub = n.subscribe("referee_spawn", 100, spawn_callback);

    while(ok())
    {
       if(spawn_tasks.size() > 0)
       {
           if(!is_working)
           {
              is_working = true;
              spawn_Model(spawn_tasks.at(spawn_tasks.size()-1).name, spawn_tasks.at(spawn_tasks.size()-1).object_name, spawn_tasks.at(spawn_tasks.size()-1).x, spawn_tasks.at(spawn_tasks.size()-1).y, spawn_tasks.at(spawn_tasks.size()-1).z, spawn_tasks.at(spawn_tasks.size()-1).add_index, spawn_tasks.at(spawn_tasks.size()-1).apply_force_orientation);
           }
       }

       spinOnce();
    }
    return 0;
}
