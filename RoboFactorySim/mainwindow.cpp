#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtGamepad/QGamepad>
#include <iostream>
#include <QDebug>
#include <assignment.h>
#include <QProcess>
#include <QFile>
#include<cstdlib>
#include <unistd.h>
#include "param.h"
#include <QEvent>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  simProcess = new QProcess;
  auto gamepads = QGamepadManager::instance()->connectedGamepads();
  if (gamepads.isEmpty()) {
      qDebug() << "Did not find any connected gamepads";
      //return;
  } else {
    m_gamepad = new QGamepad(*gamepads.begin(), this);
    connect(m_gamepad, SIGNAL(buttonAChanged(bool)), this, SLOT(buttonA_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonBChanged(bool)), this, SLOT(buttonB_pressed(bool)));

    connect(m_gamepad, SIGNAL(buttonDownChanged(bool)), this, SLOT(buttonDown_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonUpChanged(bool)), this, SLOT(buttonUp_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonLeftChanged(bool)), this, SLOT(buttonLeft_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonRightChanged(bool)), this, SLOT(buttonRight_pressed(bool)));

  }


  connect(ui->gamemode, SIGNAL(clicked(bool)), this, SLOT(choose_gamemode()));
  connect(ui->assignments, SIGNAL(clicked(bool)), this, SLOT(create_assignments()));
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  connect(ui->quit, SIGNAL(clicked(bool)), this, SLOT(quit_simulation()));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::buttonA_pressed(bool pressed)
{
  if (pressed && gamepadEnable) {
    qDebug() << "Button B" << pressed;
    switch (counter % 4) {
    case 0:
      choose_gamemode();
      break;
    case 1:
      create_assignments();
      break;
    case 2:
      start_simulation();
      gamepadEnable = false;
      break;
    case 3:
      quit_simulation();
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }
}

void MainWindow::buttonB_pressed(bool pressed)
{
  sleep(2);
  quit_simulation();
  gamepadEnable = true;
}

void MainWindow::buttonUp_pressed(bool pressed)
{
  if (pressed && gamepadEnable) {
    unselected(counter);
    if (counter < 2) counter = 4 + counter;
    counter = counter - 2;
    selected(counter);
    //ui->gamemode->setFocus();
  }
}

void MainWindow::buttonDown_pressed(bool pressed)
{
  if (pressed && gamepadEnable) {
    unselected(counter);
    counter = counter + 2;
    selected(counter);
    //ui->gamemode->setFocus();
  }
}

void MainWindow::buttonLeft_pressed(bool pressed)
{
  if (pressed && gamepadEnable) {
    unselected(counter);
    counter--;
    selected(counter);
    //ui->gamemode->setFocus();
  }
}

void MainWindow::buttonRight_pressed(bool pressed)
{
  if (pressed && gamepadEnable) {
    unselected(counter);
    counter++;
    selected(counter);
    //ui->gamemode->setFocus();
  }
}

void MainWindow::selected(int counter)
{
  switch (counter%4) {
    case 0:
      switch (gamemode%3) {
        case 0:
          ui->gamemode->setStyleSheet("border-image:url(:/images/images/Ki_O.png)");
          break;
        case 1:
          ui->gamemode->setStyleSheet("border-image:url(:/images/images/Player_O.png)");
          break;
        case 2:
          ui->gamemode->setStyleSheet("border-image:url(:/images/images/As_team_O.png)");
          break;
        default:
          break;
      }
      break;
    case 1:
      ui->assignments->setStyleSheet("border-image:url(:/images/images/assignment_O.png)");
      break;
    case 2:
      ui->start->setStyleSheet("border-image:url(:/images/images/start_O.png)");
      break;
    case 3:
      ui->quit->setStyleSheet("border-image:url(:/images/images/quit_O.png)");
      break;
    default:
      qDebug() << "unvalid conter";
      break;
  }
}

void MainWindow::unselected(int counter)
{
  switch (counter%4) {
  case 0:
    switch (gamemode%3) {
      case 0:
        ui->gamemode->setStyleSheet("border-image:url(:/images/images/Ki.png)");
        break;
      case 1:
        ui->gamemode->setStyleSheet("border-image:url(:/images/images/Player.png)");
        break;
      case 2:
        ui->gamemode->setStyleSheet("border-image:url(:/images/images/As_team.png)");
        break;
      default:
        break;
    }
    break;
  case 1:
    ui->assignments->setStyleSheet("border-image:url(:/images/images/assignment.png)");
    break;
  case 2:
    ui->start->setStyleSheet("border-image:url(:/images/images/Start.png)");
    break;
  case 3:
    ui->quit->setStyleSheet("border-image:url(:/images/images/quit.png)");
    break;
  default:
    qDebug() << "unvalid conter";
    break;
  }
}

void MainWindow::create_assignments()
{
  Assignment* assignment = new Assignment();
  assignment->setAttribute(Qt::WA_DeleteOnClose);
  gamepadEnable = false;
  assignment->show();
}

void MainWindow::start_simulation()
{
  // write game mode
  write_gamemode("gamemode.txt");
  // open launch file
  if (gamemode%3 == KI_VS_KI) {
    simProcess->start("xterm", QStringList() << "-e" << "roslaunch RoboFactorySimGUI RoboFactoryGUI.launch");
  } else if (gamemode%3 == PLAYER_VS_PLAYER) {
    simProcess->start("xterm", QStringList() << "-e" << "roslaunch roboter_controller start_competetive.launch");
  } else if (gamemode%3 == AS_TEAM) {
    simProcess->start("xterm", QStringList() << "-e" << "roslaunch RoboFactorySimGUI RoboFactoryGUI.launch");
  }

  //simProcess->write("-e roslaunch RoboFactorySimGUI RoboFactoryGUI.launch");
  qDebug() << "debug";
}

void MainWindow::quit_simulation()
{
  simProcess->close();
}

void MainWindow::choose_gamemode()
{
  gamemode++;
  switch (gamemode%3) {
    case 0:
      ui->gamemode->setStyleSheet("border-image:url(:/images/images/Ki_O.png)");
      break;
    case 1:
      ui->gamemode->setStyleSheet("border-image:url(:/images/images/Player_O.png)");
      break;
    case 2:
      ui->gamemode->setStyleSheet("border-image:url(:/images/images/As_team_O.png)");
      break;
    default:
      break;
  }
}


void MainWindow::write_gamemode(QString filename)
{
  QFile file(filename);
  if(!file.open(QFile::WriteOnly |
                QFile::Text))
  {
      qDebug() << " Could not open file for writing";
      return;
  }
  QTextStream out(&file);
  qDebug() << "game mode: " << gamemode%3;
  out << QString::number(gamemode%3);
  file.flush();
  file.close();
}

