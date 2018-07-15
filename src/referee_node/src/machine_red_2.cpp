#include "machine.h"

using namespace ros;
using namespace machine;

int main(int argc, char** argv)
{
    init(argc, argv, "machine_node_red_2");

    Machine machine;
    machine.init(4);

    return 0;
}
