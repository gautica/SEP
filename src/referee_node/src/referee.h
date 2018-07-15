#ifndef REFEREE_H
#define REFEREE_H

#include "ros/ros.h"
#include "referee_node/update_machine.h"
#include <vector>
#include "referee_node/referee_machines.h"
#include "referee_node/referee_machine.h"
#include "referee_node/referee_resources.h"
#include "referee_node/referee_resource.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "referee_node/Spawn.h"
#include "referee_node/despawn.h"
#include "referee_node/machine_work_state.h"
#include "robot_navigation/got_resource.h"
#include "machine.h"

using namespace std;
using namespace ros;
using namespace machine;

typedef pair<float, float> Vector2;

namespace referee {
class Referee 
{
    public:
      struct Machine_Struct
      {
          int id;
          float pos_x;
          float pos_y;
          float time_working;
          float radius;
          float distance_to_roboter;
          bool is_working;
          bool is_publishing;
          string wanted_resource[4];
          string product[4];
          float deposition_x;
          float deposition_y;
          float deposition_z;
          int apply_force_orientation; //0: x, 1: y, 2: -x, 3:-y
      };

    private:
        NodeHandle n;
        int last_sub_id_machine;
        int last_sub_count_resources_robot_0;
        int last_sub_count_resources_robot_1;

        Subscriber machine_sub;
        Subscriber resource_sub;
        Subscriber roboter_pose_sub;
        Subscriber got_resource_sub;

        Publisher resource_pub;
        Publisher machine_pub;
        Publisher workstate_pub;
        Publisher spawn_publisher;

        vector<Machine_Struct> machines;
        vector<referee_node::referee_resource> resources;
        Vector2 roboter_0_pos;
        Vector2 roboter_1_pos;

    public:
        vector<Machine*> machine_node;
        
    public:
        Referee();
        
    private:
        void machine_Callback(const referee_node::update_machine::ConstPtr& msg);
        void resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void spawnMachineVisuals(int machine_id, int publish_id, Publisher spawn_publisher);
        void create_machines();
        void add_machine  (int id, float pos_x, float pos_y, float deposition_x, float deposition_y, float deposition_z,
                          int apply_force_orientation, string product_0, string product_1, string product_2, string product_3,
                          string wanted_resource_0, string wanted_resource_1, string wanted_resource_2, string wanted_resource_3);
        void roboter_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void resource_callback(const robot_navigation::got_resource::ConstPtr& msg);
        bool isContained(string array[4], string str);
};
}
#endif
