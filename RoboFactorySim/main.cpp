#include "mainwindow.h"
#include <QApplication>
#include "param.h"

bool gamepadEnable = true;
std::vector<unsigned int> products;
int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
