#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "navmeshrenderarea.hpp"
#include "zoomablescrollarea.hpp"

#include <QMainWindow>
#include <QString>

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  const QString kSampleNavmeshFileName_{tr("./sample.poly")};
  NavmeshRenderArea *navmeshRenderArea_;
  ZoomableScrollArea *navmeshRenderScrollArea_;

  void createToolbar();
  void createNavmeshRenderArea();
private slots:
  void openFile();
};

#endif // MAINWINDOW_H
