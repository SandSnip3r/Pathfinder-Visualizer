#include "mainwindow.h"

#include <QFileDialog>
#include <QMenuBar>
#include <QString>

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
  connect(openAct, &QAction::triggered, this, &MainWindow::openFile);
  fileMenuPtr->addAction(openAct);
}

void MainWindow::createNavmeshRenderArea() {
  navmeshRenderArea_ = new NavmeshRenderArea;

  navmeshRenderScrollArea_ = new QScrollArea;
  navmeshRenderScrollArea_->setBackgroundRole(QPalette::ColorRole::Dark);
  navmeshRenderScrollArea_->setWidget(navmeshRenderArea_);

  setCentralWidget(navmeshRenderScrollArea_);
}

void MainWindow::openFile() {
  QString fileName = QFileDialog::getOpenFileName(this,tr("Choose Planar Straight Line Graph file"),QString(),tr("Poly (*.poly)"));
  if (!fileName.isEmpty()) {
    navmeshRenderArea_->openFile(fileName);
  }
}
