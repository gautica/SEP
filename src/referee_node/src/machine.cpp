#include "machine.h"

namespace machine {
//void* waiting_machine_thread();
//void* set_Machine_To_Working(void* m_info);

Machine::Machine()
{
    work_sub = n.subscribe("/referee/machine_workState", 1, &Machine::work_Callback, this);

    despawn_publisher = n.advertise<referee_node::despawn>("referee_despawn", 10);
    spawn_publisher = n.advertise<referee_node::Spawn>("referee_spawn", 10);
    referee_publisher = n.advertise<referee_node::machine_work_state>("/machine_info", 1);

    machine_is_working.machine_id = machine_is_waiting.machine_id = machine_id;
    machine_is_working.is_working = true;
    machine_is_waiting.is_working = false;
}

void Machine::init(int id)
{
    machine_id = id;

    while(ok())
    {
        if(assignments.size() > 0)
        {
            if(!is_working)
            {
              is_working = true;
              set_Machine_To_Working(assignments.at(assignments.size()-1));
            }
        }

        Duration(0.1).sleep();
        spinOnce();
    }
}

void Machine::work_Callback(const referee_node::machine_work_state::ConstPtr& msg)
{
  if(msg->machine_id == machine_id) //Check if this machine should start working
  {
      referee_node::machine_work_state new_assignment;
      new_assignment.machine_id = msg->machine_id;
      new_assignment.machine_work_pos_x = msg->machine_work_pos_x;
      new_assignment.machine_work_pos_y = msg->machine_work_pos_y;
      new_assignment.resource_name = msg->resource_name;
      new_assignment.working_time = msg->working_time;
      new_assignment.deposition_x = msg->deposition_x;
      new_assignment.deposition_y = msg->deposition_y;
      new_assignment.deposition_z = msg->deposition_z;
      new_assignment.apply_force = msg->apply_force;
      new_assignment.product = msg->product;
      new_assignment.roboter_name = msg->roboter_name;

      assignments.push_back(new_assignment);
  }
}

void Machine::set_Machine_To_Working(referee_node::machine_work_state assignment)
{
    //Say Referee that machine is working
    referee_publisher.publish(machine_is_working);

    //Delete Assignment
    assignments.pop_back();

    //Convert Machine_ID to string
    ostringstream convert;
    convert << machine_id;

    //Build Red_Visual name
    string vs_red_name("visual_signal_red_");
    vs_red_name += convert.str();

    //Build Green_Visual name
    string vs_green_name("visual_signal_green_");
    vs_green_name += convert.str();

    //Create Spawn and Despawn messages
    referee_node::despawn despawn_visual_red_msg;
    referee_node::Spawn spawn_visual_red_msg;
    referee_node::despawn despawn_visual_green_msg;
    referee_node::Spawn spawn_visual_green_msg;
    referee_node::despawn despawn_resource_msg;
    referee_node::Spawn spawn_resource_msg;

    //Define Spawn Message [RED Visual]
    spawn_visual_red_msg.name = "visual_signal_red";
    spawn_visual_red_msg.x = assignment.machine_work_pos_x;
    spawn_visual_red_msg.y = assignment.machine_work_pos_y;
    spawn_visual_red_msg.object_name = vs_red_name;

    //Define Spawn Message [GREEN Visual]
    spawn_visual_green_msg.name = "visual_signal_green";
    spawn_visual_green_msg.x = assignment.machine_work_pos_x;
    spawn_visual_green_msg.y = assignment.machine_work_pos_y;
    spawn_visual_green_msg.object_name = vs_green_name;

    //Define Despawn Message [RED Visual]
    despawn_visual_red_msg.name = vs_red_name;

    //Define Despawn Message [GREEN Visual]
    despawn_visual_green_msg.name = vs_green_name;

    //Define Despawn Resource Message
    despawn_resource_msg.name = assignment.resource_name;

    //Define Spawn Resource Message
    spawn_resource_msg.name = assignment.product;
    spawn_resource_msg.object_name = assignment.product;
    spawn_resource_msg.x = assignment.deposition_x;
    spawn_resource_msg.y = assignment.deposition_y;
    spawn_resource_msg.z = assignment.deposition_z;
    spawn_resource_msg.apply_force_orientation = assignment.apply_force;
    spawn_resource_msg.add_index = true;

    //Update Visuals & Delete Resource
    spawn_publisher.publish(spawn_visual_red_msg);
    despawn_publisher.publish(despawn_visual_green_msg);
    despawn_publisher.publish(despawn_resource_msg);

    //Wait for machines working time
    Duration(assignment.working_time-0.1).sleep();
   
    //Update Visuals & Create Product
    spawn_publisher.publish(spawn_visual_green_msg);
    despawn_publisher.publish(despawn_visual_red_msg);
    spawn_publisher.publish(spawn_resource_msg);

    //Reste Machine
    referee_publisher.publish(machine_is_waiting);
    is_working = false;
}
}
