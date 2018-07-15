#ifndef COUNTDOWN_H
#define COUNTDOWN_H

#include <QWidget>
#include "ui_countdown.h"

namespace Ui {
class CountDown;
}

namespace gui {
class CountDown : public QWidget
{
  Q_OBJECT

public:
  explicit CountDown(QWidget *parent = 0);
  ~CountDown();

  void start();
private:
  Ui::CountDown *ui;
};

}


#endif // COUNTDOWN_H
