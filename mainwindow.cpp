#include "mainwindow.h"
#include "navmeshdisplay.h"

#include <QDockWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QTabWidget>
#include <QToolBar>
#include <QIcon>

#include <iostream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  createMenubar();
  createToolbar();
  createNavmeshDisplay();
  createConfigDock();

  createConnectionsToNavmeshDisplay();

  setWindowTitle(tr("Pathfinder Visualization"));

  // Window is built, now lets try to open and display the sample navmesh file
  initializeNavmeshTriangleData();
  openNavmeshFile(kSampleNavmeshFileName_);
}

MainWindow::~MainWindow() {
}

void MainWindow::createMenubar() {
  auto fileMenuPtr = menuBar()->addMenu(tr("&File"));
  QAction *openAct = new QAction(tr("&Open..."), this);
  openAct->setShortcuts(QKeySequence::Open);
  openAct->setStatusTip(tr("Open an existing file"));
  connect(openAct, &QAction::triggered, this, &MainWindow::openNavmeshFilePrompt);
  fileMenuPtr->addAction(openAct);
}

void MainWindow::createToolbar() {
  auto toolBar = addToolBar(tr("Toolbar!"));
  toolbarActionGroup_ = new QActionGroup(this);
  toolBar->setIconSize(QSize{40,40});

  dragAction_ = toolBar->addAction(tr("Drag"));
  dragAction_->setCheckable(true);
  dragAction_->setChecked(true);
  dragAction_->setIcon(QIcon(":/icons/move.png"));
  toolbarActionGroup_->addAction(dragAction_);

  movePathStartAction_ = toolBar->addAction(tr("Move Path Start"));
  movePathStartAction_->setCheckable(true);
  movePathStartAction_->setIcon(QIcon(":/icons/start.png"));
  toolbarActionGroup_->addAction(movePathStartAction_);

  movePathGoalAction_ = toolBar->addAction(tr("Move Path Goal"));
  movePathGoalAction_->setCheckable(true);
  movePathGoalAction_->setIcon(QIcon(":/icons/goal.png"));
  toolbarActionGroup_->addAction(movePathGoalAction_);

  connect(movePathStartAction_, &QAction::toggled, this, &MainWindow::setMovePathStartEnabled);
  connect(movePathGoalAction_, &QAction::toggled, this, &MainWindow::setMovePathGoalEnabled);
}

void MainWindow::createNavmeshDisplay() {
  navmeshDisplay_ = new NavmeshDisplay;
  setCentralWidget(navmeshDisplay_);

  navmeshDisplay_->getNavmeshRenderArea()->setAgentRadius(agentRadius_);
}

void MainWindow::createConfigDock() {
  // ====================================================================
  // ========================Pathfinding settings========================
  // ====================================================================
  QLabel *pathfindingCharacterRadiusLabel = new QLabel(tr("Character Radius: "));
  agentRadiusLineEdit_ = new QLineEdit;
  // TODO: Add an input filter to the QLineEdit (that also matches the possibilities of the slider)
  agentRadiusSlider_ = new QSlider(Qt::Horizontal);
  // TODO: It might make sense to scale the maximum and step based on the size of the given navmesh
  agentRadiusSlider_->setMaximum(1000);
  // Update both input widgets with our initial agentRadius_
  setAgentRadiusLineEdit();
  setAgentRadiusSlider();
  connect(agentRadiusSlider_, &QSlider::valueChanged, this, &MainWindow::agentRadiusSliderChanged);
  connect(agentRadiusLineEdit_, &QLineEdit::textChanged, this, &MainWindow::agentRadiusTextChanged);

  QGridLayout *pathfindingOptionsLayout = new QGridLayout;
  pathfindingOptionsLayout->addWidget(pathfindingCharacterRadiusLabel, 0, 0, 1, 1);
  pathfindingOptionsLayout->addWidget(agentRadiusLineEdit_, 0, 1, 1, 1);
  pathfindingOptionsLayout->setColumnStretch(pathfindingOptionsLayout->columnCount(), 1);
  pathfindingOptionsLayout->addWidget(agentRadiusSlider_, 1, 0, 1, 3);
  pathfindingOptionsLayout->setRowStretch(pathfindingOptionsLayout->rowCount(), 1);

  QWidget *pathfindingOptionsTabContent = new QWidget;
  pathfindingOptionsTabContent->setLayout(pathfindingOptionsLayout);

  // ====================================================================
  // =======================Visualization settings=======================
  // ====================================================================
  nonConstraintEdgesCheckBox_ = new QCheckBox(tr("Show Non-Constraint Edges"));
  triangleLabelsCheckBox_ = new QCheckBox(tr("Show Triangle Labels"));
  edgeLabelsCheckBox_ = new QCheckBox(tr("Show Edge Labels"));
  vertexLabelsCheckBox_ = new QCheckBox(tr("Show Vertex Labels"));

  QGridLayout *navmeshVisualizationOptionsLayout = new QGridLayout;
  navmeshVisualizationOptionsLayout->addWidget(nonConstraintEdgesCheckBox_, 0, 0, 1, 1);
  navmeshVisualizationOptionsLayout->addWidget(triangleLabelsCheckBox_, 1, 0, 1, 1);
  navmeshVisualizationOptionsLayout->addWidget(edgeLabelsCheckBox_, 2, 0, 1, 1);
  navmeshVisualizationOptionsLayout->addWidget(vertexLabelsCheckBox_, 3, 0, 1, 1);
  navmeshVisualizationOptionsLayout->setRowStretch(navmeshVisualizationOptionsLayout->rowCount(), 1);

  QWidget *navmeshVisualizationOptionsTabContent = new QWidget;
  navmeshVisualizationOptionsTabContent->setLayout(navmeshVisualizationOptionsLayout);

  // ====================================================================
  // ====================================================================
  // ====================================================================

  // Build the config UI
  QTabWidget *tabWidget = new QTabWidget(this);
  tabWidget->addTab(pathfindingOptionsTabContent, tr("Pathfinding"));
  tabWidget->addTab(navmeshVisualizationOptionsTabContent, tr("Navmesh Visualization"));

  // Now, build the dock for the config UI
  QDockWidget *dockWidget = new QDockWidget(tr("Settings"), this);
  dockWidget->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
  dockWidget->setWidget(tabWidget);
  addDockWidget(Qt::BottomDockWidgetArea, dockWidget);
}

void MainWindow::createConnectionsToNavmeshDisplay() {
  // Toolbar
  connect(dragAction_, &QAction::toggled, navmeshDisplay_, &NavmeshDisplay::setDragModeEnabled);

  // Configuration
  connect(nonConstraintEdgesCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderArea::setDisplayNonConstraintEdges);
  connect(triangleLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderArea::setDisplayTriangleLabels);
  connect(edgeLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderArea::setDisplayEdgeLabels);
  connect(vertexLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderArea::setDisplayVertexLabels);

  // Mouse event handling
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderArea::draggingMouseOnNavmesh, this, &MainWindow::draggingMouseOnNavmesh);
}

void MainWindow::openNavmeshFilePrompt() {
  QString filename = QFileDialog::getOpenFileName(this,tr("Choose Planar Straight Line Graph file"),QString(),tr("Poly (*.poly)"));
  if (!filename.isEmpty()) {
    openNavmeshFile(filename);
  }
}

void MainWindow::movePathStart(const Vector &pos) {
  startPoint_ = pos;
  navmeshDisplay_->setPathStartPoint(pos);
}

void MainWindow::movePathGoal(const Vector &pos) {
  goalPoint_ = pos;
  navmeshDisplay_->setPathGoalPoint(pos);
}

void MainWindow::draggingMouseOnNavmesh(const Vector &navmeshPoint) {
  if (movePathStartEnabled_) {
    movePathStart(navmeshPoint);
  }
  if (movePathGoalEnabled_) {
    movePathGoal(navmeshPoint);
  }
  if (startPoint_ && goalPoint_) {
    // Have both start and goal
    rebuildPath();
  }
}

void MainWindow::setMovePathStartEnabled(bool enabled) {
  movePathStartEnabled_ = enabled;
  navmeshDisplay_->getNavmeshRenderArea()->setHandleMouseDrag(!dragAction_->isChecked());
}

void MainWindow::setMovePathGoalEnabled(bool enabled) {
  movePathGoalEnabled_ = enabled;
  navmeshDisplay_->getNavmeshRenderArea()->setHandleMouseDrag(!dragAction_->isChecked());
}

void MainWindow::setAgentRadiusLineEdit() {
  agentRadiusLineEdit_->setText(QString::number(agentRadius_, 'f', 3));
}

void MainWindow::setAgentRadiusSlider() {
  agentRadiusSlider_->setValue(agentRadius_);
}

//===========================================================================================================
//================================================= Navmesh =================================================
//===========================================================================================================

void MainWindow::openNavmeshFile(const QString &filename) {
  try {
    // First, try to open the file and build the navmesh
    buildNavmeshFromFile(filename);

    // File must've opened successfully, reset path data
    startPoint_.reset();
    goalPoint_.reset();
    navmeshDisplay_->resetPathStart();
    navmeshDisplay_->resetPathGoal();
    navmeshDisplay_->resetPath();
  } catch (std::exception &ex) {
    // Could not open the file
    QMessageBox msgBox;
    msgBox.setText("Could not open file \""+filename+"\". The file may have invalid .poly format.");
    msgBox.exec();

    // Need to re-prompt user to open another file
    openNavmeshFilePrompt();
  }
}

void MainWindow::initializeNavmeshTriangleData() {
	triangle_initialize_triangleio(&savedTriangleData_);
	triangle_initialize_triangleio(&savedTriangleVoronoiData_);
}

void MainWindow::buildNavmeshFromFile(QString fileName) {
  triangleio inputData;
  triangle_initialize_triangleio(&inputData);

  int firstNode;
  // Call Triangle's file reading function
  int readFileResult = triangle_read_poly(fileName.toStdString().c_str(), &inputData, &firstNode);
  if (readFileResult < 0) {
    throw std::runtime_error("Unable to open .poly file, error "+std::to_string(readFileResult));
  }

  // Create a context
  context *ctx;
  ctx = triangle_context_create();
  // Set context's behavior
  *(ctx->b) = behaviorBuilder_.getBehavior();

  // Build the triangle mesh
  int meshCreateResult = triangle_mesh_create(ctx, &inputData);
  if (meshCreateResult < 0) {
    // Free memory
    triangle_free_triangleio(&inputData);
    throw std::runtime_error("Error creating navmesh ("+std::to_string(meshCreateResult)+")");
  }

  // Now, the context holds the mesh, lets extract this data
  // First, prepare data structures
  // Free in case this has already been used
  triangle_free_triangleio(&savedTriangleData_);
  triangle_free_triangleio(&savedTriangleVoronoiData_);
  initializeNavmeshTriangleData();

  // Copy data
  int copyResult = triangle_mesh_copy(ctx, &savedTriangleData_, 1, 1, &savedTriangleVoronoiData_);
  if (copyResult < 0) {
    // Free memory
    triangle_free_triangleio(&inputData);
    throw std::runtime_error("Error copying navmesh data ("+std::to_string(copyResult)+")");
  }

  // Done with input data
  triangle_free_triangleio(&inputData);

  // Done with context
  triangle_context_destroy(ctx);

  navmeshDisplay_->setNavmesh(savedTriangleData_);
}

void MainWindow::rebuildPath() {
  if (!startPoint_ || !goalPoint_) {
    // Trying to build a path but we dont have our start and goal points; nothing to do
    return;
  }
  try {
    Pathfinder pathfinder(savedTriangleData_, savedTriangleVoronoiData_);
    pathfinder.setCharacterRadius(agentRadius_);
    pathfindingResult_ = pathfinder.findShortestPath(*startPoint_, *goalPoint_);
    navmeshDisplay_->setPath(pathfindingResult_);
  } catch (std::exception &ex) {
    // Unable to build path
    navmeshDisplay_->resetPath();
  }
}

void MainWindow::agentRadiusTextChanged(const QString &text) {
  bool conversionSuccessful{false};
  double newRadius = text.toDouble(&conversionSuccessful);
  if (conversionSuccessful) {
    // Update value
    agentRadius_ = newRadius;
    // Update slider
    if (!matchingAgentRadiusLineEditAndSlider_) {
      // Someone manually changed this, update the other widget to match
      matchingAgentRadiusLineEditAndSlider_ = true;
      setAgentRadiusSlider();
      matchingAgentRadiusLineEditAndSlider_ = false;
    }
    // Update navmesh
    navmeshDisplay_->getNavmeshRenderArea()->setAgentRadius(agentRadius_);
    // New path needs to be created
    rebuildPath();
  }
}

void MainWindow::agentRadiusSliderChanged(int value) {
  // Update value
  agentRadius_ = value;
  // Update LineEdit
  if (!matchingAgentRadiusLineEditAndSlider_) {
    // Someone manually changed this, update the other widget to match
    matchingAgentRadiusLineEditAndSlider_ = true;
    setAgentRadiusLineEdit();
    matchingAgentRadiusLineEditAndSlider_ = false;
  }
  // Update navmesh
  navmeshDisplay_->getNavmeshRenderArea()->setAgentRadius(agentRadius_);
  // New path needs to be created
  rebuildPath();
}
