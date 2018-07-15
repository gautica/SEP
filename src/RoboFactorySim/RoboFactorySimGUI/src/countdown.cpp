#include "../include/gui/countdown.hpp"
#include <unistd.h>

namespace gui {

CountDown::CountDown(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CountDown)
{
  ui->setupUi(this);

}

CountDown::~CountDown()
{
  delete ui;
}

void CountDown::start()
{
  ui->label->setText("1");
  sleep(1);
  ui->label->setText("2");
  sleep(1);
  ui->label->setText("3");
  sleep(1);
  ui->label->setText("4");
  sleep(1);
  ui->label->setText("5");
}

}
