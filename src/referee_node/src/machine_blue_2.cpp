#include "machine.h"

using namespace ros;
using namespace machine;

int main(int argc, char** argv)
{
    init(argc, argv, "machine_node_blue_2");
    Machine machine;
    machine.init(3);
    return 0;
}
