#ifndef MUTEX_H
#define MUTEX_H
#include <QMutex>
#include <QWaitCondition>
#include <queue>
#include "referee.h"
#include "referee_node/referee_resource.h"

typedef struct goal
{
  int task;
  std::pair<int, int> goal_pos;
  double yaw;
  int distance_precision;
} goal_t;

enum Task
{
  GRAB = 0,
  DROP = 1
};

enum Product
{
  NO_PRODUCT = -1,
  BLUE_PRODUCT = 0,
  YELLOW_PRODUCT = 1,
  RED_PRODUCT = 2,
  VIOLET_PRODUCT = 3,
  ORANGE_PRODUCT = 4,
  GREEN_PRODUCT = 5,
  BLACK_PRODUCT = 6
};

enum Location
{
  WAREHOUSE = 0,
  BLUE_MACHINE_1 = 1,
  RED_MACHINE_1 = 2,
  YELLOW_MACHINE_1 = 3,
  BLUE_MACHINE_2 = 4,
  RED_MACHINE_2 = 5,
  YELLOW_MACHINE_2 = 6
};

enum ResourceColor
{
  WHITE = 0,
  BLUE = 1,
  YELLOW = 2,
  RED = 3,
  VIOLET = 4,
  ORANGE = 5,
  GREEN = 6,
  BLACK = 7
};

extern QMutex mutex;
extern QWaitCondition condition;
extern bool stopSim;

extern std::vector<std::vector<int> > gridMap;
extern std::vector<std::pair<int, int> > path;
extern std::queue<goal_t> goals;
extern goal_t curr_goal;

extern int curr_product;
extern int curr_location;

extern std::vector<std::vector<int> > roboter_local_field;
extern std::pair<int, int> roboter_pos;

extern bool is_dest_reachable;
extern bool is_job_finished;
extern bool is_roboter_pos_init;
extern bool is_map_init;
extern bool is_path_init;
extern bool is_job_finished;
extern bool request_new_plan;
extern bool grabbed;
extern bool is_reach_goal;
extern bool is_goal_ready;

extern int ROW;
extern int COL;
extern double mapResolution;

extern std::vector<global_planner::Referee::Machine_Struct> machines;
extern std::vector<referee_node::referee_resource> resources;

extern const int costMap_area;
extern const int dist_to_resource;
extern const int dist_to_machine;

extern std::vector<int> products;

extern std::string will_grab;
extern bool is_navigate;

extern std::string roboter_name;
extern int machine_id;
extern int curr_color;

extern std::vector<int> machine_chain;
#endif // MUTEX_H
