#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <QDialog>

namespace Ui {
class Assignment;
}

class QGamepad;
class QString;
class Assignment : public QDialog
{
  Q_OBJECT

public:
  explicit Assignment(QWidget *parent = 0);
  ~Assignment();
private slots:
  void click_ok();
  void click_cancel();
  void buttonA_pressed(bool pressed = false);
  void buttonB_pressed(bool pressed = false);
  void buttonX_pressed(bool pressed = false);
  void buttonY_pressed(bool pressed = false);

  void buttonUp_pressed(bool pressed = false);
  void buttonDown_pressed(bool pressed = false);
  void buttonLeft_pressed(bool pressed = false);
  void buttonRight_pressed(bool pressed = false);
private:
  void selected(int counter);
  void unselected(int counter);
  void create_products();
  void write_products(QString filename);

private:
  Ui::Assignment *ui;
  QString products;
  QGamepad* m_gamepad;
  int counter = 0;
};

#endif // ASSIGNMENT_H
