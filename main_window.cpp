#include "main_window.hpp"

#include <Pathfinder/triangle_lib_navmesh.h>

#include <QDockWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QLabel>
#include <QGroupBox>
#include <QMenuBar>
#include <QMessageBox>
#include <QTabWidget>
#include <QToolBar>
#include <QIcon>

#include <iostream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  createMenubar();
  createToolbar();
  createConfigDock();

  // Check some boxes
  nonConstraintEdgesCheckBox_->setChecked(true);

  setWindowTitle(tr("Pathfinder Visualization"));

  // Window is built. Start by opening an example TriangleLib .poly file.
  openPolyFile(kSampleNavmeshFileName_);
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
}

void MainWindow::createConfigDock() {
  // ====================================================================
  // ======================Pathfinding settings tab======================
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
  // =====================Visualization settings tab=====================
  // ====================================================================
  // Navmesh Triangulation
  verticesCheckBox_ = new QCheckBox(tr("Show Vertices"));
  nonConstraintEdgesCheckBox_ = new QCheckBox(tr("Show Non-Constraint Edges"));

  triangleCorridorCheckBox_ = new QCheckBox(tr("Show Triangle Corridor"));
  trianglesCompletelySearchedCheckBox_ = new QCheckBox(tr("Show Triangles Completely Searched"));
  trianglesVisitedCheckBox_ = new QCheckBox(tr("Show Triangles Visited"));

  triangleLabelsCheckBox_ = new QCheckBox(tr("Show Triangle Labels"));
  edgeLabelsCheckBox_ = new QCheckBox(tr("Show Edge Labels"));
  vertexLabelsCheckBox_ = new QCheckBox(tr("Show Vertex Labels"));

  QGridLayout *navmeshTriangulationVisualizationOptionsLayout = new QGridLayout;
  navmeshTriangulationVisualizationOptionsLayout->addWidget(verticesCheckBox_, 0, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(nonConstraintEdgesCheckBox_, 1, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(triangleCorridorCheckBox_, 2, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(trianglesCompletelySearchedCheckBox_, 3, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(trianglesVisitedCheckBox_, 4, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(triangleLabelsCheckBox_, 5, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(edgeLabelsCheckBox_, 6, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(vertexLabelsCheckBox_, 7, 0, 1, 1);

  // The triangles being displayed will require that the non-constraint edges are shown
  // Connect the checkboxes to prevent triangles being displayed without the edges being shown
  connect(nonConstraintEdgesCheckBox_, &QCheckBox::toggled, [this](bool checked) {
    if (!checked) {
      triangleCorridorCheckBox_->setChecked(false);
      trianglesCompletelySearchedCheckBox_->setChecked(false);
      trianglesVisitedCheckBox_->setChecked(false);
    }
  });
  connect(triangleCorridorCheckBox_, &QCheckBox::toggled, [this](bool checked){
    if (checked) {
      nonConstraintEdgesCheckBox_->setChecked(true);
    }
  });
  connect(trianglesCompletelySearchedCheckBox_, &QCheckBox::toggled, [this](bool checked){
    if (checked) {
      nonConstraintEdgesCheckBox_->setChecked(true);
    }
  });
  connect(trianglesVisitedCheckBox_, &QCheckBox::toggled, [this](bool checked){
    if (checked) {
      nonConstraintEdgesCheckBox_->setChecked(true);
    }
  });

  QGroupBox *navmeshTriangulationGroupbox = new QGroupBox("Navmesh Triangulation");
  navmeshTriangulationGroupbox->setLayout(navmeshTriangulationVisualizationOptionsLayout);

  QVBoxLayout *navmeshTabVBoxLayout = new QVBoxLayout;
  navmeshTabVBoxLayout->addWidget(navmeshTriangulationGroupbox);
  navmeshTabVBoxLayout->setAlignment(Qt::AlignTop);
  QWidget *navmeshVisualizationOptionsTabContent = new QWidget;
  navmeshVisualizationOptionsTabContent->setLayout(navmeshTabVBoxLayout);

  // ====================================================================
  // ====================================================================
  // ====================================================================

  // Build the config UI
  configTabWidget_ = new QTabWidget(this);
  configTabWidget_->addTab(pathfindingOptionsTabContent, tr("Pathfinding"));
  configTabWidget_->addTab(navmeshVisualizationOptionsTabContent, tr("Navmesh Visualization"));

  // Now, build the dock for the config UI
  QDockWidget *dockWidget = new QDockWidget(tr("Settings"), this);
  dockWidget->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::RightDockWidgetArea);
  dockWidget->setWidget(configTabWidget_);
  addDockWidget(Qt::RightDockWidgetArea, dockWidget);
}

void MainWindow::populateConfigDock() {
  // Get values for configuration from the render area.
  if (navmeshDisplay_ == nullptr) {
    throw std::runtime_error("No NavmeshDisplay");
  }
  verticesCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayVertices());
  nonConstraintEdgesCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayNonConstraintEdges());
  triangleCorridorCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayTriangleCorridor());
  trianglesCompletelySearchedCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayTrianglesCompletelySearched());
  trianglesVisitedCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayTrianglesVisited());
  triangleLabelsCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayTriangleLabels());
  edgeLabelsCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayEdgeLabels());
  vertexLabelsCheckBox_->setChecked(navmeshDisplay_->getNavmeshRenderArea()->getDisplayVertexLabels());
}

void MainWindow::createConnectionsToNavmeshDisplay() {
  // Toolbar
  connect(dragAction_, &QAction::toggled, navmeshDisplay_, &NavmeshDisplayBase::setDragModeEnabled);
  connect(movePathStartAction_, &QAction::toggled, this, &MainWindow::setMovePathStartEnabled);
  connect(movePathGoalAction_, &QAction::toggled, this, &MainWindow::setMovePathGoalEnabled);

  // Configuration
  connect(verticesCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayVertices);
  connect(nonConstraintEdgesCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayNonConstraintEdges);
  connect(triangleCorridorCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTriangleCorridor);
  connect(trianglesCompletelySearchedCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTrianglesCompletelySearched);
  connect(trianglesVisitedCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTrianglesVisited);
  connect(triangleLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTriangleLabels);
  connect(edgeLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayEdgeLabels);
  connect(vertexLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayVertexLabels);

  // Mouse event handling
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::draggingMouseOnNavmesh, this, &MainWindow::draggingMouseOnNavmesh);
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::movingMouseOnNavmesh, this, &MainWindow::movingMouseOnNavmesh);
}

void MainWindow::openNavmeshFilePrompt() {
  QString filename = QFileDialog::getOpenFileName(this,tr("Choose Navmesh file"),QString(),tr("Poly (*.poly)"));
  if (!filename.isEmpty()) {
    const auto fileInfo = QFileInfo(filename);
    if (fileInfo.suffix() == "poly") {
      openPolyFile(filename);
    } else {
      throw std::runtime_error("Unsupported file format "+fileInfo.suffix().toStdString());
    }
  }
}

void MainWindow::movePathStart(const pathfinder::Vector &pos) {
  navmeshDisplay_->setPathStartPoint(pos);
  startPoint_ = pos;
  if (startPoint_ && goalPoint_) {
    // Have both start and goal
    rebuildPath();
  }
}

void MainWindow::movePathGoal(const pathfinder::Vector &pos) {
  navmeshDisplay_->setPathGoalPoint(pos);
  goalPoint_ = pos;

  if (startPoint_ && goalPoint_) {
    // Have both start and goal
    rebuildPath();
  }
}

void MainWindow::draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint) {
  if (movePathStartEnabled_) {
    movePathStart(navmeshPoint);
  }
  if (movePathGoalEnabled_) {
    movePathGoal(navmeshPoint);
  }
}

void MainWindow::movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint) {
  navmeshDisplay_->setMousePosition(navmeshPoint);
}

// void MainWindow::mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint) {
// }

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

void MainWindow::updateAgentRadius(double newRadius) {
  // Update value
  agentRadius_ = newRadius;
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

void MainWindow::resetPathData() {
  startPoint_.reset();
  goalPoint_.reset();
  navmeshDisplay_->resetPathStart();
  navmeshDisplay_->resetPathGoal();
  navmeshDisplay_->resetPath();
}

void MainWindow::openPolyFile(const QString &filename) {
  try {
    // Try to open the file and build the navmesh triangulation.
    buildTriangleLibNavmeshTriangulationFromFile(filename);

    // Build the navmesh display for this type of navmesh triangulation.
    createNavmeshDisplay<NavmeshDisplay<TriangleLibNavmeshTriangulationType>>();

    // Give the navmesh triangulation to the display.
    auto &concreteNavmeshDisplay = dynamic_cast<NavmeshDisplay<TriangleLibNavmeshTriangulationType>&>(*navmeshDisplay_);
    concreteNavmeshDisplay.setNavmeshTriangulation(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_));

    // File opened successfully, navmesh display created and setup, reset path data
    resetPathData();
  } catch (std::exception &ex) {
    // Could not open the file
    QMessageBox msgBox;
    // See https://www.cs.cmu.edu/~quake/triangle.poly.html for .poly file format.
    msgBox.setText("Could not open file \""+filename+"\". The file may have invalid .poly format. Error: \""+ex.what()+"\"");
    msgBox.exec();

    // Need to re-prompt user to open another file
    openNavmeshFilePrompt();
  }
}

void MainWindow::buildTriangleLibNavmeshTriangulationFromFile(QString fileName) {
  triangle::triangleio inputData;
  triangle::triangle_initialize_triangleio(&inputData);

  int firstNode;
  // Call Triangle's file reading function
  int readFileResult = triangle::triangle_read_poly(fileName.toStdString().c_str(), &inputData, &firstNode);
  if (readFileResult < 0) {
    throw std::runtime_error("Unable to open .poly file, error "+std::to_string(readFileResult));
  }

  // Create a context
  triangle::context *ctx;
  ctx = triangle::triangle_context_create();
  // Set context's behavior
  *(ctx->b) = behaviorBuilder_.getBehavior();

  // Build the triangle mesh
  int meshCreateResult = triangle::triangle_mesh_create(ctx, &inputData);
  if (meshCreateResult < 0) {
    // Free memory
    triangle_free_triangleio(&inputData);
    throw std::runtime_error("Error creating navmesh ("+std::to_string(meshCreateResult)+")");
  }

  // Now, the context holds the mesh, lets extract this data
  // First, prepare data structures
  triangle::triangleio triangleData, triangleVoronoiData;
	triangle::triangle_initialize_triangleio(&triangleData);
	triangle::triangle_initialize_triangleio(&triangleVoronoiData);

  // Copy data
  int copyResult = triangle::triangle_mesh_copy(ctx, &triangleData, 1, 1, &triangleVoronoiData);
  if (copyResult < 0) {
    // Free memory
    triangle_free_triangleio(&inputData);
    throw std::runtime_error("Error copying navmesh data ("+std::to_string(copyResult)+")");
  }

  // Done with input data
  triangle_free_triangleio(&inputData);

  // Done with context
  triangle_context_destroy(ctx);

  // Build navmesh from triangle data
  navmeshTriangulation_.emplace<TriangleLibNavmeshTriangulationType>(TriangleLibNavmeshTriangulationType(triangleData, triangleVoronoiData));

  // Free
  triangle_free_triangleio(&triangleData);
  triangle_free_triangleio(&triangleVoronoiData);
}

void MainWindow::rebuildPath() {
  if (!startPoint_ || !goalPoint_) {
    // Trying to build a path but we dont have our start and goal points; nothing to do
    return;
  }
  if (!navmeshTriangulation_) {
    throw std::runtime_error("Trying to build path when there is no navmesh triangulation");
  }
  try {
    if (std::holds_alternative<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_)) {
      TriangleLibPathfinderType pathfinder(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_), agentRadius_);
      pathfindingResult_ = pathfinder.findShortestPath(*startPoint_, *goalPoint_);
      auto &concreteNavmeshDisplay = dynamic_cast<NavmeshDisplay<TriangleLibNavmeshTriangulationType>&>(*navmeshDisplay_);
      concreteNavmeshDisplay.setPath(std::get<TriangleLibPathfindingResult>(*pathfindingResult_));
    }
  } catch (std::exception &ex) {
    // Unable to build path
    std::cout << "Unable to build path. Exception: \"" << ex.what() << "\"" << std::endl;
    navmeshDisplay_->resetPath();
  }
}

void MainWindow::agentRadiusTextChanged(const QString &text) {
  bool conversionSuccessful{false};
  double newRadius = text.toDouble(&conversionSuccessful);
  if (conversionSuccessful) {
    updateAgentRadius(newRadius);
  }
}

void MainWindow::agentRadiusSliderChanged(int value) {
  updateAgentRadius(value);
}
