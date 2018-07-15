#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class QGamepad;
class QProcess;
class QEvent;
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();
private:
  void selected(int counter);
  void unselected(int counter);
  void write_gamemode(QString filename);

private slots:
  void buttonA_pressed(bool pressed = false);
  void buttonB_pressed(bool pressed = false);
  void buttonUp_pressed(bool pressed = false);
  void buttonDown_pressed(bool pressed = false);
  void buttonLeft_pressed(bool pressed = false);
  void buttonRight_pressed(bool pressed = false);
  void create_assignments();
  void start_simulation();
  void quit_simulation();
  void choose_gamemode();
private:
  Ui::MainWindow *ui;
  QGamepad* m_gamepad;
  QProcess* simProcess;
  int counter = 0;
  int gamemode = 0;
};

#endif // MAINWINDOW_H
