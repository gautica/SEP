#include "machine.h"

using namespace ros;
using namespace machine;

int main(int argc, char** argv)
{
    init(argc, argv, "machine_node_red_1");
    Machine machine;
    machine.init(1);
    return 0;
}
