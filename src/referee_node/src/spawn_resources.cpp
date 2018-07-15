#include <string>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "referee_node/Spawn.h"

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "machine_manager");
    
    NodeHandle spawn_node;
    Publisher spawn_publisher = spawn_node.advertise<referee_node::Spawn>("referee_spawn", 10);
    referee_node::Spawn spawn_resource_msg;
    	
    Duration(3).sleep();
    for(int i = 0; i < 4; i++)
    {
        string resource_name("Resource_White_");
        ostringstream convert;   // stream used for the conversion
        convert << (i+1);
        resource_name += convert.str();
        spawn_resource_msg.name = "Resource_White"; 
        spawn_resource_msg.object_name = resource_name;
        spawn_resource_msg.x = 0.13352 - 1 * (i%2) - 0.5; //-0.866480
        spawn_resource_msg.y = i < 2 ? -3.660052 : -4.660052; //-4.160052

        spawn_publisher.publish(spawn_resource_msg);
        Duration(0.1).sleep();
    }
    
    return 0;
}
