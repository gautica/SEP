#ifndef COSTMAPTHREAD_H
#define COSTMAPTHREAD_H
#include <QThread>

namespace global_planner {

class CostMapThread : public QThread
{
public:
  CostMapThread();

  void run();
private:
  void calc_costMap();
  void draw_costMap(int row, int col, int n, int value);
};

}



#endif // COSTMAPTHREAD_H
