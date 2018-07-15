#include "machine.h"

using namespace ros;
using namespace machine;

int main(int argc, char** argv)
{
    init(argc, argv, "machine_node_yellow_2");
    Machine machine;
    machine.init(5);
    return 0;
}
