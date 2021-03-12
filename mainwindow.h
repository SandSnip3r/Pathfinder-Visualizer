#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "navmeshrenderarea.hpp"

#include <QMainWindow>
#include <QScrollArea>

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  NavmeshRenderArea *navmeshRenderArea_;
  QScrollArea *navmeshRenderScrollArea_;

  void createToolbar();
  void createNavmeshRenderArea();
private slots:
  void openFile();
};

#endif // MAINWINDOW_H
