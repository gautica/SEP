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

#include "referee_node/despawn.h"

using namespace std;
using namespace ros;

vector<string> despawn_tasks;
bool is_working;

void despawn_Model(string name)
{
    despawn_tasks.pop_back();
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel deleted_model;
    deleted_model.request.model_name= (std::string) name; 
    client.call(deleted_model);
    is_working = false;
}

void despawn_callback(const referee_node::despawn::ConstPtr& msg)
{
    despawn_tasks.push_back(msg->name);
}

int main(int argc, char** argv)
{
    init(argc, argv, "referee_receive_despawn");
    
    NodeHandle n;
    Subscriber spawn_sub = n.subscribe("referee_despawn", 100, despawn_callback);

    while(ok())
    {
      if(despawn_tasks.size() > 0)
      {
        if(!is_working)
        {
          is_working = true;
          despawn_Model(despawn_tasks.at(despawn_tasks.size()-1));
        }
      }
      Duration(0.1).sleep();
      spinOnce();
    }
    return 0;
}
