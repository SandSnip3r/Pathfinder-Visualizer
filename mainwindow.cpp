#include "mainwindow.h"

#include <QFileDialog>
#include <QMenuBar>
#include <QMessageBox>

#include <iostream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  createToolbar();
  createNavmeshRenderArea();
  setWindowTitle(tr("Pathfinder Visualization"));

  // Window is built, now lets try to open and display the sample navmesh file
  // const QString kSampleNavmeshFileName{tr("./sample.poly")};
  try {
    navmeshRenderArea_->openFile(kSampleNavmeshFileName_);
  } catch (std::exception &ex) {
    // Could not open the sample, pop up the open prompt for the user to open a poly file
    std::cout << "Could not open poly file \"" << kSampleNavmeshFileName_.toStdString() << "\"" << std::endl;
    openFile();
  }
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

  navmeshRenderScrollArea_ = new ZoomableScrollArea;
  navmeshRenderScrollArea_->setBackgroundRole(QPalette::ColorRole::Dark);
  navmeshRenderScrollArea_->setWidget(navmeshRenderArea_);

  setCentralWidget(navmeshRenderScrollArea_);
}

void MainWindow::openFile() {
  QString fileName = QFileDialog::getOpenFileName(this,tr("Choose Planar Straight Line Graph file"),QString(),tr("Poly (*.poly)"));
  if (!fileName.isEmpty()) {
    try {
      navmeshRenderArea_->openFile(fileName);
    } catch (std::exception &ex) {
      // Could not open the file
      QMessageBox msgBox;
      msgBox.setText("Could not open file");
      msgBox.exec();
    }
  }
}
