#ifndef HTNPLANNER_H
#define HTNPLANNER_H
#include <vector>

class HTNPlanner
{
public:
  HTNPlanner();

  void make_plan(int product);
private:
  void create_violet();
  void create_orange();
  void create_green();
  void create_black();
  bool find_free(const int &row1, const int &col1, const int &row2, const int &col2);
  void update_table();
private:
  std::vector<std::vector<bool> > timeTable;
  unsigned int wait_time;
};



#endif // HTNPLANNER_H
