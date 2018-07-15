#include "referee.h"

using namespace ros;
using namespace referee;
using namespace machine;

int main(int argc, char** argv)
{
    init(argc, argv, "referee_node");

    Referee referee;
    return 0;
}
