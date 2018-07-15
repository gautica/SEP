#include "referee.h"

namespace referee
{ 
    Referee::Referee()
    {
        machine_sub = n.subscribe("/machine_info", 2, &Referee::machine_Callback, this);
        machine_pub = n.advertise<referee_node::referee_machines>("referee_machines", 10);
        resource_pub = n.advertise<referee_node::referee_resources>("referee_resources", 10);
        workstate_pub = n.advertise<referee_node::machine_work_state>("/referee/machine_workState", 10);
        resource_sub = n.subscribe("/gazebo/model_states", 10, &Referee::resource_pose_Callback, this);
        roboter_pose_sub = n.subscribe("/gazebo/model_states", 10, &Referee::roboter_pose_Callback, this);
        got_resource_sub = n.subscribe("/roboter/resource", 2, &Referee::resource_callback, this);
        spawn_publisher = n.advertise<referee_node::Spawn>("referee_spawn", 10);

        create_machines();

        while(ok())
        {
            referee_node::referee_machines referee_machines_msg;

            for(int i = 0; i < machines.size(); i++)
            {
                referee_node::referee_machine machine_msg;
                machine_msg.machine_id = machines[i].id;
                machine_msg.pos_x = machines[i].pos_x;
                machine_msg.pos_y = machines[i].pos_y;
                machine_msg.radius = machines[i].radius;
                machine_msg.is_working = machines[i].is_working;
                for (int j = 0; j < 4; j++)
                {
                    machine_msg.wanted_resource[j] = machines[i].wanted_resource[j];        //%%%%%%%%
                    machine_msg.product[j] = machines[i].product[j];
                }
                referee_machines_msg.machines.push_back(machine_msg);
            }
            machine_pub.publish(referee_machines_msg);

            Duration(0.1).sleep();
            spinOnce();
        }
    }
    
    void Referee::create_machines()
    {
        //Machine 0 Blue
        add_machine(0, -4.689391, 3.076189, -4.688817, 3.297257, 0.200258, 2, "Resource_Blue", "Resource_Violet", "Resource_Green", "Resource_Black",
                                                                       "Resource_White", "Resource_Red", "Resource_Yellow", "Resource_Orange");

        //Machine 1 Red
        add_machine(1, -4.695904, -4.723634, -4.687050, -4.463008, 0.200258, 2, "Resource_Red", "Resource_Violet", "Resource_Orange", "Resource_Black",
                                                                          "Resource_White", "Resource_Blue", "Resource_Yellow", "Resource_Green");

        //Machine 2 Yellow
        add_machine(2, -0.348833, 3.076202, -0.562188, 3.041248, 0.200258, 3, "Resource_Yellow", "Resource_Green", "Resource_Orange", "Resource_Black",
                                                                       "Resource_White", "Resource_Blue", "Resource_Red", "Resource_Violet");

        //Machine 3 Blue
        add_machine(3, 2.909260, -3.303037, 2.909241, -3.538039, 0.200258, 4, "Resource_Blue", "Resource_Violet", "Resource_Green", "Resource_Black",
                                                                    "Resource_White", "Resource_Red", "Resource_Yellow", "Resource_Orange");

        //Machine 4 Red
        add_machine(4, 2.909241, 3.04, 3.076211, 3.044755, 0.200258, 1, "Resource_Red", "Resource_Violet", "Resource_Orange", "Resource_Black",
                                                                    "Resource_White", "Resource_Blue", "Resource_Yellow", "Resource_Green");

        //Machine 5 Yellow
        add_machine(5, 2.909251, -0.178594, 2.704879, -0.199812, 0.200258, 3, "Resource_Yellow", "Resource_Green", "Resource_Orange", "Resource_Black",
                                                                        "Resource_White", "Resource_Blue", "Resource_Red", "Resource_Violet");

        //Spawn machine visualization
        long unsigned int size = machines.size();
        int i = 0;

        Duration(0.5).sleep();

        for(i = 0; i < size; i++)
        {
            spawnMachineVisuals(i, i+1, spawn_publisher);
        }
    }

    void Referee::add_machine(int id, float pos_x, float pos_y, float deposition_x, float deposition_y, float deposition_z,
                              int apply_force_orientation, string product_0, string product_1, string product_2, string product_3,
                              string wanted_resource_0, string wanted_resource_1, string wanted_resource_2, string wanted_resource_3)
    {
        Machine_Struct machine;
        machine.id = id;
        machine.pos_x = pos_x;
        machine.pos_y = pos_y;
        machine.deposition_x = deposition_x;
        machine.deposition_y = deposition_y;
        machine.deposition_z = deposition_z;
        machine.apply_force_orientation = apply_force_orientation;
        machine.product[0] = product_0;
        machine.wanted_resource[0] = wanted_resource_0;
        machine.product[1] = product_1;
        machine.wanted_resource[1] = wanted_resource_1;
        machine.product[2] = product_2;
        machine.wanted_resource[2] = wanted_resource_2;
        machine.product[3] = product_3;
        machine.wanted_resource[3] = wanted_resource_3;
        machine.is_working = false;
        machine.radius = 1.42;
        machine.time_working = 5;
        machines.push_back(machine);
    }

    void Referee::spawnMachineVisuals(int machine_id, int publish_id, Publisher spawn_publisher)
    {
        ostringstream convert;
        convert << machines[machine_id].id;

        referee_node::Spawn spawn_visual_msg;
        string vs_green_name("visual_signal_green_");
        vs_green_name += convert.str();
        spawn_visual_msg.id = publish_id;
        spawn_visual_msg.name = "visual_signal_green";
        spawn_visual_msg.object_name = vs_green_name;
        spawn_visual_msg.x = machines[machine_id].pos_x;
        spawn_visual_msg.y = machines[machine_id].pos_y;

        spawn_publisher.publish(spawn_visual_msg);
    }
    
    void Referee::machine_Callback(const referee_node::update_machine::ConstPtr& msg)
    {
        machines[msg->machine_id].is_working = !machines[msg->machine_id].is_working;
    }   
    
    void Referee::resource_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        long unsigned int size = msg->name.size();
        int i;
        referee_node::referee_resources referee_resources_msg;
        vector<referee_node::referee_resource> temp_resources;

        for(int i = 0; i < resources.size(); i++)
        {
           temp_resources.push_back(resources[i]);
        }

        resources.clear();

        for(i = 0; i < (int) size; i++)     //;
        {
            if(msg->name[i].find("Resource") != std::string::npos)
            {       
                referee_node::referee_resource resource_msg;
                resource_msg.name = msg->name[i];
                resource_msg.pos_x = msg->pose[i].position.x;
                resource_msg.pos_y = msg->pose[i].position.y;
                
                for(int j = 0; j < temp_resources.size(); j++)
                {
                    if(msg->name[i].compare(temp_resources[j].name) == 0)
                    {
                        resource_msg.blocked = temp_resources[j].blocked;
                    }
                }

                resources.push_back(resource_msg);
                referee_resources_msg.resources.push_back(resource_msg);
            }
        }

        resource_pub.publish(referee_resources_msg); 
    }

    void Referee::roboter_pose_Callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        long unsigned int size = msg->name.size();
        int i;

        for(i = 0; i < (int) size; i++)
        {
            if(msg->name[i].find("robot_0") != std::string::npos)
            {
                roboter_0_pos.first = msg->pose[i].position.x;
                roboter_0_pos.second = msg->pose[i].position.y;
            }

            if(msg->name[i].find("robot_1") != std::string::npos)
            {
                roboter_1_pos.first = msg->pose[i].position.x;
                roboter_1_pos.second = msg->pose[i].position.y;
            }
        }
    }

    void Referee::resource_callback(const robot_navigation::got_resource::ConstPtr& msg)
    {
      ROS_ERROR("KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK");
        for(int i = 0; i < resources.size(); i++)
        {
            ROS_ERROR("NNNNNNNNNNNNNNNNNNNNNNNNNNN %s ||||||||||||||| %s", msg->resource_name.c_str(), resources[i].name.c_str());
            if(msg->resource_name.compare(resources[i].name) == 0)
            {
                //resources[i].blocked = msg->resource_blocked; //Would be right but the other roboter could find the resource for like one second if we set it to false
                resources[i].blocked = true;
            }
        }

        if(!msg->resource_blocked)
        {
            Vector2 current_roboter = (msg->roboter_name).compare("robot_0") == 0 ? roboter_0_pos : roboter_1_pos;

            for(int i = 0; i < machines.size(); i++)
            {
                float distance = sqrt(pow(current_roboter.first - machines[i].pos_x, 2) + pow(current_roboter.second - machines[i].pos_y, 2));

                if(distance < machines[i].radius)
                {
                  referee_node::machine_work_state work_msg;
                  work_msg.machine_id = i;
                  work_msg.roboter_name = msg->roboter_name;
                  work_msg.machine_work_pos_x = machines[i].pos_x;
                  work_msg.machine_work_pos_y = machines[i].pos_y;
                  work_msg.resource_name = msg->resource_name;
                  work_msg.working_time = machines[i].time_working;
                  work_msg.deposition_x = machines[i].deposition_x;
                  work_msg.deposition_y = machines[i].deposition_y;
                  work_msg.deposition_z = machines[i].deposition_z;
                  work_msg.apply_force = machines[i].apply_force_orientation;

                  int j = 0;
                  while (msg->resource_name.find(machines[i].wanted_resource[j]) == std::string::npos)
                  {
                      j++;
                  }

                  work_msg.product = machines[i].product[j];
                  ROS_ERROR("REFEREE____________________ %s", work_msg.roboter_name.c_str());

                  workstate_pub.publish(work_msg);
                  break;
                }
            }
        }
    }
}
