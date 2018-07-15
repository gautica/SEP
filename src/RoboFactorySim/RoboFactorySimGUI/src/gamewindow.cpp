#include "../include/gui/gamewindow.hpp"
#include "../include/gui/assignment.hpp"
#include "../include/gui/param.h"
#include "../include/gui/imageWidget.hpp"
#include "../include/gui/teamwidget.hpp"
#include "../include/gui/kiwidget.hpp"
#include "../include/gui/mapviewer.hpp"
#include "../include/gui/playwidget.hpp"
#include "../include/gui/countdown.hpp"
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QRgb>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QProcess>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QTimer>
#include <QMessageBox>
#include <unistd.h>


namespace gui {
GameWindow::GameWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::GameWindow)
{
  ui->setupUi(this);
  viewer_id_robot0 = 0;
  viewer_id_robot1 = 1;
  is_first_start = true;

  robot0_layout = new QHBoxLayout();
  ui->robot0_viewer->setLayout(robot0_layout);
  robot1_layout = new QHBoxLayout();
  ui->robot1_viewer->setLayout(robot1_layout);

  QHBoxLayout* status_layout = new QHBoxLayout();
  ui->GameStatus->setLayout(status_layout);

  while (!init_image_robot0 || !init_image_robot1 || !init_camera_up_robot0 || !init_camera_up_robot1) {
    sleep(1);
  }
  image_viewer_robot0 = new ImageDisplay(0, 0);
  camera_up_robot0 = new ImageDisplay(0, 2);
  image_viewer_robot1 = new ImageDisplay(0, 1);
  camera_up_robot1 = new ImageDisplay(0, 3);

  robot0_layout->addWidget(image_viewer_robot0);
  robot0_layout->addWidget(camera_up_robot0);
  camera_up_robot0->hide();
  robot1_layout->addWidget(image_viewer_robot1);
  robot1_layout->addWidget(camera_up_robot1);
  camera_up_robot1->hide();

  KI_Widget = new KIWidget;
  teamWidget = new TeamWidget;
  playerWidget = new PlayWidget;
  switch (curr_gamemode) {
  case KI_VS_KI:
    status_layout->addWidget(KI_Widget);
    break;
  case AS_TEAM:
    status_layout->addWidget(teamWidget);
    break;
  case PLAYER_VS_PLAYER:
    status_layout->addWidget(playerWidget);
    break;
  default:
    break;
  }
  connect(ui->change_viewer_robot0, SIGNAL(clicked(bool)), this, SLOT(change_viewer_robot0()));
  connect(ui->change_viewer_robot1, SIGNAL(clicked(bool)), this, SLOT(change_viewer_robot1()));
  connect(ui->map_button, SIGNAL(clicked(bool)), this, SLOT(open_mapViewer()));
  timer = new QTimer();
  timer->setInterval(50);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
  timer->start();
}

GameWindow::~GameWindow()
{
  delete ui;
}

void GameWindow::change_viewer_robot0()
{
  if (viewer_id_robot0 == 0) {
    image_viewer_robot0->hide();
    camera_up_robot0->show();
    viewer_id_robot0 = 2;
  } else if(viewer_id_robot0 == 2) {
    camera_up_robot0->hide();
    image_viewer_robot0->show();
    viewer_id_robot0 = 0;
  }
}

void GameWindow::change_viewer_robot1()
{
  if (viewer_id_robot1 == 1) {
    image_viewer_robot1->hide();
    camera_up_robot1->show();
    viewer_id_robot1 = 3;
  } else if(viewer_id_robot1 == 3) {
    camera_up_robot1->hide();
    image_viewer_robot1->show();
    viewer_id_robot1 = 1;
  }
}

void GameWindow::update_window()
{
  update_viewer();
  update_status_window();
  is_game_finished();
  if (is_first_start) {

    //cout_down->start();
    //cout_down->close();
    //delete cout_down;
    is_first_start = false;
  }
}

void GameWindow::open_mapViewer()
{
  is_mapViewr_active = true;
  MapViewer* mapViewer = new MapViewer(this);
  mapViewer->setAttribute(Qt::WA_DeleteOnClose);
  mapViewer->show();
}

void GameWindow::update_viewer()
{
  if (curr_gamemode == PLAYER_VS_PLAYER) {
    if (player1_topview) {
      image_viewer_robot0->hide();
      camera_up_robot0->show();
      viewer_id_robot0 = 2;
    } else {
      image_viewer_robot0->show();
      camera_up_robot0->hide();
      viewer_id_robot0 = 0;
    }
    if (player2_topview) {
      image_viewer_robot1->hide();
      camera_up_robot1->show();
      viewer_id_robot1 = 3;
    } else {
      image_viewer_robot1->show();
      camera_up_robot1->hide();
      viewer_id_robot1 = 1;
    }
  }
}

void GameWindow::update_status_window()
{
  if (update_status) {
    switch (curr_gamemode) {
      case KI_VS_KI:
          KI_Widget->update_window();
          update_status = false;
        break;
      case AS_TEAM:
          teamWidget->update_window();
          update_status = false;
        break;
      case PLAYER_VS_PLAYER:
        playerWidget->update_window();
        update_status = false;
        break;
      default:
        break;
    }
  }

}

void GameWindow::is_game_finished()
{
  bool is_finished = false;
  bool is_player1_wins = true;
  bool is_player2_wins = true;
  if (sim_closed) {
    this->close();
  }
  switch (curr_gamemode) {
  case KI_VS_KI:
    is_finished = is_finished_robot0 || is_finished_robot1 ? true : false;
    break;
  case AS_TEAM:
    is_finished = is_finished_robot0 && is_finished_robot1 ? true : false;
    break;
  case PLAYER_VS_PLAYER:
    for (int i = 0; i < products.size(); i++) {
      if (products[i] != -1) {
        is_player1_wins = false;
      }
      if (products_copy[i] != -1) {
        is_player2_wins = false;
      }
    }
    is_finished = is_player1_wins || is_player2_wins;
  default:
    break;
  }
  if (is_finished) {
    QMessageBox msgBox;
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setInformativeText("Do you want to close the window and play again?");
    switch (curr_gamemode) {
    case KI_VS_KI:
      return;
    case PLAYER_VS_PLAYER:
      return;
      break;
    case AS_TEAM:
      msgBox.setText("Congratulstions! Robots have Job finished.");
      break;
    default:
      break;
    }
    int ret = msgBox.exec();
    switch (ret) {
    case QMessageBox::Ok:
      this->close();
      break;
    case QMessageBox::Cancel:

      break;
    default:
      break;
    }
  }
}

}

