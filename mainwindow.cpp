#include "mainwindow.h"

#include <QMenuBar>

#include <iostream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  createToolbar();
  createNavmeshRenderArea();
  setWindowTitle(tr("Pathfinder Visualization"));
}

MainWindow::~MainWindow() {
}

void MainWindow::createToolbar() {
  auto fileMenuPtr = menuBar()->addMenu(tr("&File"));
  QAction *openAct = new QAction(tr("&Open..."), this);
  openAct->setShortcuts(QKeySequence::Open);
  openAct->setStatusTip(tr("Open an existing file"));
  connect(openAct, &QAction::triggered, this, &MainWindow::open);
  fileMenuPtr->addAction(openAct);
}

void MainWindow::createNavmeshRenderArea() {
  navmeshRenderArea_ = new NavmeshRenderArea;

  navmeshRenderScrollArea_ = new QScrollArea;
  navmeshRenderScrollArea_->setBackgroundRole(QPalette::ColorRole::Dark);
  navmeshRenderScrollArea_->setWidget(navmeshRenderArea_);

  setCentralWidget(navmeshRenderScrollArea_);
}

void MainWindow::open() {
    std::cout << "Opening a file!" << std::endl;
}
