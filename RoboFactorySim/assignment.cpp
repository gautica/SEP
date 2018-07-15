#include "assignment.h"
#include "ui_assignment.h"
#include "param.h"
#include <iostream>
#include <QFile>
#include <QDebug>
#include <QGamepad>

Assignment::Assignment(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Assignment)
{
  ui->setupUi(this);
  this->setWindowTitle("Assignment");
  auto gamepads = QGamepadManager::instance()->connectedGamepads();
  if (!gamepads.isEmpty()) {
    m_gamepad = new QGamepad(*gamepads.begin(), this);
    connect(m_gamepad, SIGNAL(buttonAChanged(bool)), this, SLOT(buttonA_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonBChanged(bool)), this, SLOT(buttonB_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonXChanged(bool)), this, SLOT(buttonX_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonYChanged(bool)), this, SLOT(buttonY_pressed(bool)));

    connect(m_gamepad, SIGNAL(buttonDownChanged(bool)), this, SLOT(buttonDown_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonUpChanged(bool)), this, SLOT(buttonUp_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonLeftChanged(bool)), this, SLOT(buttonLeft_pressed(bool)));
    connect(m_gamepad, SIGNAL(buttonRightChanged(bool)), this, SLOT(buttonRight_pressed(bool)));
  }


  connect(ui->ok_button, SIGNAL(clicked(bool)), this, SLOT(click_ok()));
  connect(ui->cancel_button, SIGNAL(clicked(bool)), this, SLOT(click_cancel()));
}

Assignment::~Assignment()
{
  gamepadEnable = true;
  delete ui;
}

void Assignment::click_ok()
{
  create_products();
  write_products("products.txt");
  std::cout << "Products is initialized \n";
  close();
  gamepadEnable = true;
}

void Assignment::click_cancel()
{
  close();
  gamepadEnable = true;
}

void Assignment::create_products()
{
  //products.clear();

  if (ui->blue_checkBox->isChecked()) {
    int count = ui->blue_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(BLUE_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->red_checkBox->isChecked()) {
    int count = ui->red_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(RED_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->yellow_checkBox->isChecked()) {
    int count = ui->yellow_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(YELLOW_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->violet_checkBox->isChecked()) {
    int count = ui->violet_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(VIOLET_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->orange_checkBox->isChecked()) {
    int count = ui->orange_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(ORANGE_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->green_checkBox->isChecked()) {
    int count = ui->green_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(GREEN_PRODUCT) + " ";
      count--;
    }
  }
  if (ui->black_checkBox->isChecked()) {
    int count = ui->black_lineEdit->text().toInt();
    while (count > 0) {
      products += QString::number(BLACK_PRODUCT) + " ";
      count--;
    }
  }
}

void Assignment::write_products(QString filename)
{
  QFile file(filename);
  if(!file.open(QFile::WriteOnly |
                QFile::Text))
  {
      qDebug() << " Could not open file for writing";
      return;
  }
  QTextStream out(&file);
  qDebug() << products;
  out << products;
  file.flush();
  file.close();
}

void Assignment::buttonDown_pressed(bool pressed)
{
  if (pressed) {
    unselected(counter);
    counter++;
    selected(counter);
  }
}

void Assignment::buttonUp_pressed(bool pressed)
{
  if (pressed) {
    unselected(counter);
    if (counter == 0) counter = 9;
    counter--;
    selected(counter);
  }
}

void Assignment::buttonLeft_pressed(bool pressed)
{
  if (pressed) {
    qDebug() << "Button left" << pressed;
    switch (counter % 9) {
    qDebug() << "Button left" << counter % 9;
    case 0:
      if (ui->blue_checkBox->isChecked()) {
        ui->blue_lineEdit->setText(QString::number(ui->blue_lineEdit->text().toInt() - 1));
      }
      if (ui->blue_lineEdit->text().toInt() == 0) {
        ui->blue_checkBox->setChecked(false);
      }
      break;
    case 1:
      if (ui->red_checkBox->isChecked()) {
        ui->red_lineEdit->setText(QString::number(ui->red_lineEdit->text().toInt() - 1));
      }
      if (ui->red_lineEdit->text().toInt() == 0) {
        ui->red_checkBox->setChecked(false);
      }
      break;
    case 2:
      if (ui->yellow_checkBox->isChecked()) {
        ui->yellow_lineEdit->setText(QString::number(ui->yellow_lineEdit->text().toInt() - 1));
      }
      if (ui->yellow_lineEdit->text().toInt() == 0) {
        ui->yellow_checkBox->setChecked(false);
      }
      break;
    case 3:
      if (ui->violet_checkBox->isChecked()) {
        ui->violet_lineEdit->setText(QString::number(ui->violet_lineEdit->text().toInt() - 1));
      }
      if (ui->violet_lineEdit->text().toInt() == 0) {
        ui->violet_checkBox->setChecked(false);
      }
      break;
    case 4:
      if (ui->orange_checkBox->isChecked()) {
        ui->orange_lineEdit->setText(QString::number(ui->orange_lineEdit->text().toInt() - 1));
      }
      if (ui->orange_lineEdit->text().toInt() == 0) {
        ui->orange_checkBox->setChecked(false);
      }
      break;
    case 5:
      if (ui->green_checkBox->isChecked()) {
        ui->green_lineEdit->setText(QString::number(ui->green_lineEdit->text().toInt() - 1));
      }
      if (ui->green_lineEdit->text().toInt() == 0) {
        ui->green_checkBox->setChecked(false);
      }
      break;
    case 6:
      if (ui->black_checkBox->isChecked()) {
        ui->black_lineEdit->setText(QString::number(ui->black_lineEdit->text().toInt() - 1));
      }
      if (ui->black_lineEdit->text().toInt() == 0) {
        ui->black_checkBox->setChecked(false);
      }
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }
}

void Assignment::buttonRight_pressed(bool pressed)
{
  if (pressed) {
    qDebug() << "Button right" << pressed;
    switch (counter % 9) {
    case 0:
      if (ui->blue_checkBox->isChecked() == false) {
        ui->blue_checkBox->setChecked(true);
      }
      ui->blue_lineEdit->setText(QString::number(ui->blue_lineEdit->text().toInt() + 1));
      break;
    case 1:
      if (ui->red_checkBox->isChecked() == false) {
        ui->red_checkBox->setChecked(true);
      }
      ui->red_lineEdit->setText(QString::number(ui->red_lineEdit->text().toInt() + 1));
      break;
    case 2:
      if (ui->yellow_checkBox->isChecked() == false) {
        ui->yellow_checkBox->setChecked(true);
      }
      ui->yellow_lineEdit->setText(QString::number(ui->yellow_lineEdit->text().toInt() + 1));
      break;
    case 3:
      if (ui->violet_checkBox->isChecked() == false) {
        ui->violet_checkBox->setChecked(true);
      }
      ui->violet_lineEdit->setText(QString::number(ui->violet_lineEdit->text().toInt() + 1));
      break;
    case 4:
      if (ui->orange_checkBox->isChecked() == false) {
        ui->orange_checkBox->setChecked(true);
      }
      ui->orange_lineEdit->setText(QString::number(ui->orange_lineEdit->text().toInt() + 1));
      break;
    case 5:
      if (ui->green_checkBox->isChecked() == false) {
        ui->green_checkBox->setChecked(true);
      }
      ui->green_lineEdit->setText(QString::number(ui->green_lineEdit->text().toInt() + 1));
      break;
    case 6:
      if (ui->black_checkBox->isChecked() == false) {
        ui->black_checkBox->setChecked(true);
      }
      ui->black_lineEdit->setText(QString::number(ui->black_lineEdit->text().toInt() + 1));
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }

}

void Assignment::buttonA_pressed(bool pressed)
{
  if (pressed) {
    qDebug() << "Button B" << pressed;
    switch (counter % 9) {
    case 7:
      click_cancel();
      break;
    case 8:
      click_ok();
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }

}

/**
 * @brief MainWindow::buttonB_pressed implements click events
 * @param pressed
 */
void Assignment::buttonB_pressed(bool pressed)
{

  if (pressed) {
    qDebug() << "Button B" << pressed;
    switch (counter % 9) {
    case 0:
      if (ui->blue_checkBox->isChecked()) {
        ui->blue_lineEdit->setText(QString::number(ui->blue_lineEdit->text().toInt() - 1));
      }
      if (ui->blue_lineEdit->text().toInt() == 0) {
        ui->blue_checkBox->setChecked(false);
      }
      break;
    case 1:
      if (ui->red_checkBox->isChecked()) {
        ui->red_lineEdit->setText(QString::number(ui->red_lineEdit->text().toInt() - 1));
      }
      if (ui->red_lineEdit->text().toInt() == 0) {
        ui->red_checkBox->setChecked(false);
      }
      break;
    case 2:
      if (ui->yellow_checkBox->isChecked()) {
        ui->yellow_lineEdit->setText(QString::number(ui->yellow_lineEdit->text().toInt() - 1));
      }
      if (ui->yellow_lineEdit->text().toInt() == 0) {
        ui->yellow_checkBox->setChecked(false);
      }
      break;
    case 3:
      if (ui->violet_checkBox->isChecked()) {
        ui->violet_lineEdit->setText(QString::number(ui->violet_lineEdit->text().toInt() - 1));
      }
      if (ui->violet_lineEdit->text().toInt() == 0) {
        ui->violet_checkBox->setChecked(false);
      }
      break;
    case 4:
      if (ui->orange_checkBox->isChecked()) {
        ui->orange_lineEdit->setText(QString::number(ui->orange_lineEdit->text().toInt() - 1));
      }
      if (ui->orange_lineEdit->text().toInt() == 0) {
        ui->orange_checkBox->setChecked(false);
      }
      break;
    case 5:
      if (ui->green_checkBox->isChecked()) {
        ui->green_lineEdit->setText(QString::number(ui->green_lineEdit->text().toInt() - 1));
      }
      if (ui->green_lineEdit->text().toInt() == 0) {
        ui->green_checkBox->setChecked(false);
      }
      break;
    case 6:
      if (ui->black_checkBox->isChecked()) {
        ui->black_lineEdit->setText(QString::number(ui->black_lineEdit->text().toInt() - 1));
      }
      if (ui->black_lineEdit->text().toInt() == 0) {
        ui->black_checkBox->setChecked(false);
      }
      break;
    case 7:
      click_cancel();
      break;
    case 8:
      click_ok();
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }

}

void Assignment::buttonX_pressed(bool pressed)
{
  if (pressed) {
    qDebug() << "Button X" << pressed;
    switch (counter % 9) {
    case 0:
      if (ui->blue_checkBox->isChecked() == false) {
        ui->blue_checkBox->setChecked(true);
      }
      ui->blue_lineEdit->setText(QString::number(ui->blue_lineEdit->text().toInt() + 1));
      break;
    case 1:
      if (ui->red_checkBox->isChecked() == false) {
        ui->red_checkBox->setChecked(true);
      }
      ui->red_lineEdit->setText(QString::number(ui->red_lineEdit->text().toInt() + 1));
      break;
    case 2:
      if (ui->yellow_checkBox->isChecked() == false) {
        ui->yellow_checkBox->setChecked(true);
      }
      ui->yellow_lineEdit->setText(QString::number(ui->yellow_lineEdit->text().toInt() + 1));
      break;
    case 3:
      if (ui->violet_checkBox->isChecked() == false) {
        ui->violet_checkBox->setChecked(true);
      }
      ui->violet_lineEdit->setText(QString::number(ui->violet_lineEdit->text().toInt() + 1));
      break;
    case 4:
      if (ui->orange_checkBox->isChecked() == false) {
        ui->orange_checkBox->setChecked(true);
      }
      ui->orange_lineEdit->setText(QString::number(ui->orange_lineEdit->text().toInt() + 1));
      break;
    case 5:
      if (ui->green_checkBox->isChecked() == false) {
        ui->green_checkBox->setChecked(true);
      }
      ui->green_lineEdit->setText(QString::number(ui->green_lineEdit->text().toInt() + 1));
      break;
    case 6:
      if (ui->black_checkBox->isChecked() == false) {
        ui->black_checkBox->setChecked(true);
      }
      ui->black_lineEdit->setText(QString::number(ui->black_lineEdit->text().toInt() + 1));
      break;
    default:
      qDebug() << "Unvailid counter id";
      break;
    }
  }

}

void Assignment::buttonY_pressed(bool pressed)
{
  if (pressed) {
    qDebug() << "Button Y" << pressed;
    unselected(counter);
    if (counter == 0) counter = 9;
    counter--;
    selected(counter);
    //ui->gamemode->setFocus();
  }
}

void Assignment::selected(int counter)
{
  QPalette p = palette();
  p.setBrush(QPalette::Button, Qt::red);
  switch (counter%9) {
  case 0:
    ui->blue_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->blue_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 1:
    ui->red_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->red_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 2:
    ui->yellow_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->yellow_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 3:
    ui->violet_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->violet_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 4:
    ui->orange_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->orange_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 5:
    ui->green_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->green_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 6:
    ui->black_checkBox->setStyleSheet("background-color: rgb(206, 92, 0)");
    ui->black_lineEdit->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  case 7:
    ui->cancel_button->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;

  case 8:
    ui->ok_button->setStyleSheet("background-color: rgb(206, 92, 0)");
    break;
  default:
    qDebug() << "unvalid conter";
    break;
  }
}

void Assignment::unselected(int counter)
{
  QPalette p = palette();
  p.setBrush(QPalette::Button, Qt::gray);
  switch (counter%9) {
  case 0:
    ui->blue_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->blue_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 1:
    ui->red_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->red_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 2:
    ui->yellow_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->yellow_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 3:
    ui->violet_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->violet_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 4:
    ui->orange_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->orange_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 5:
    ui->green_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->green_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 6:
    ui->black_checkBox->setStyleSheet("background-color: rgb(56, 63, 84)");
    ui->black_lineEdit->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;
  case 7:
    ui->cancel_button->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;

  case 8:
    ui->ok_button->setStyleSheet("background-color: rgb(56, 63, 84)");
    break;default:
    qDebug() << "unvalid conter";
    break;
  }
}
