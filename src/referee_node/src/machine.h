#ifndef MACHINE_H
#define MACHINE_H

#include <string>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "referee_node/update_machine.h"
#include "referee_node/Spawn.h"
#include "referee_node/despawn.h"
#include "robot_navigation/got_resource.h"
#include "referee_node/machine_work_state.h"


using namespace std;
using namespace ros;

namespace machine
{

class Machine 
{
    private:
      NodeHandle n;
      Subscriber work_sub;
      Publisher despawn_publisher;
      Publisher spawn_publisher;
      Publisher referee_publisher;

      int last_sub_id_work_robot_0;
      int last_sub_id_work_robot_1;
      int last_pub_id_spawn;
      int last_pub_id_resource;

      int machine_id;
      bool is_working;
      vector<referee_node::machine_work_state> assignments;

      referee_node::machine_work_state machine_is_working;
      referee_node::machine_work_state machine_is_waiting;

    public:
        Machine();
        void init(int id);

    private:
        void work_Callback(const referee_node::machine_work_state::ConstPtr& msg);
        bool isContained(string array[4], string str);
        void set_Machine_To_Working(referee_node::machine_work_state assignment);

        //bool isContained(string array[4], string str);
        //void resource_callback(const robot_navigation::got_resource::ConstPtr& msg);
};
}
#endif
