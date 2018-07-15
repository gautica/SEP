#include "../include/gui/imageWidget.hpp"
#include "../include/gui/param.h"
#include <QImage>
#include <QLabel>
#include <QHBoxLayout>
#include <QTimer>
#include <iostream>
#include <string>
#include <unistd.h>

namespace gui {

ImageDisplay::ImageDisplay(QWidget *parent, int modell_id) :
  QWidget(parent), modell_id(modell_id),
  ui(new Ui::ImageDisplay)
{
  ui->setupUi(this);


  label = new QLabel(ui->label);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);
  label->setGeometry(200, 200, 500, 500);
  label->setAttribute(Qt::WA_TranslucentBackground);

  //image = new Image;

  timer = new QTimer;
  timer->setInterval(1);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start();
}

ImageDisplay::~ImageDisplay()
{
  delete ui;
}

void ImageDisplay::update()
{
  if (modell_id == 0) {
    currentImage = QImage(robot0_buffer, robot0_width, robot0_height, QImage::Format_RGB888);
  } else if (modell_id == 1) {
    currentImage = QImage(robot1_buffer, robot1_width, robot1_height, QImage::Format_RGB888);
  } else if(modell_id == 2) {
    currentImage = QImage(robot0_camera_up_buffer, robot0_camera_up_width, robot0_camera_up_height, QImage::Format_RGB888);
  } else if (modell_id == 3) {
    currentImage = QImage(robot1_camera_up_buffer, robot1_camera_up_width, robot1_camera_up_height, QImage::Format_RGB888);
  }else {
    std::cout << "unvailid robot_id for image viewer\n";
    exit(-1);
  }

  ui->label->setPixmap(QPixmap::fromImage(currentImage));
  if (curr_gamemode == PLAYER_VS_PLAYER) {
    switch (go) {
      case 0:
        label->hide();
        break;
      case 1:
        label->setPixmap(QPixmap(":/images/GO.png"));
        break;
      case 2:
        label->setPixmap(QPixmap(":/images/1.png"));
        break;
      case 3:
        label->setPixmap(QPixmap(":/images/2.png"));
        break;
      case 4:
        label->setPixmap(QPixmap(":/images/3.png"));
        break;
      case 5:
        label->setPixmap(QPixmap(":/images/4.png"));
        break;
      case 6:
        label->setPixmap(QPixmap(":/images/5.png"));
        break;
      case 10:
        if (modell_id == 0 || modell_id == 2) {
          label->show();
          label->setPixmap(QPixmap(":/images/winner.png"));
        }
        break;
      case 20:
        if (modell_id == 1 || modell_id == 3) {
          label->show();
          label->setPixmap(QPixmap(":/images/winner.png"));
        }
        break;
      default:
        break;
      }
  }
  if (curr_gamemode == KI_VS_KI) {
    if (who_wins == ROBOT_0) {
      if (modell_id == 0 || modell_id == 2) {
        label->show();
        label->setPixmap(QPixmap(":/images/winner.png"));
      }
    } else if (who_wins == ROBOT_1) {
      if (modell_id == 1 || modell_id == 3) {
        label->show();
        label->setPixmap(QPixmap(":/images/winner.png"));
      }
    }
  }

}

}


