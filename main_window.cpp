#include "main_window.hpp"

#include <absl/strings/str_format.h>

#include <QDockWidget>
#include <QFileDialog>
#include <QGridLayout>
#include <QIntValidator>
#include <QLabel>
#include <QGroupBox>
#include <QMenuBar>
#include <QMessageBox>
#include <QTabWidget>
#include <QToolBar>
#include <QIcon>

#include <iostream>

#include <algorithm>
#include <array>
#include <functional>
#include <random>

namespace {

absl::string_view toString(MainWindow::AllPairsIsFor isFor) {
  if (isFor == MainWindow::AllPairsIsFor::kGoal) {
    return "goal";
  } else if (isFor == MainWindow::AllPairsIsFor::kStart) {
    return "start";
  } else {
    return "none";
  }
}

std::mt19937 createRandomEngine() {
  std::random_device rd;
  std::array<int, std::mt19937::state_size> seed_data;
  std::generate_n(seed_data.data(), seed_data.size(), std::ref(rd));
  std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
  return std::mt19937(seq);
}

} // anonymous namespace

class RandomPointGenerator : public PointGenerator {
public:
  RandomPointGenerator(std::optional<pathfinder::Vector> start, std::optional<pathfinder::Vector> goal, double minX, double maxX, double minY, double maxY) : PointGenerator(start, goal), eng_(createRandomEngine()) {
    xDist_ = std::uniform_real_distribution<double>(minX, maxX);
    yDist_ = std::uniform_real_distribution<double>(minY, maxY);
  }

  std::pair<pathfinder::Vector, pathfinder::Vector> getNext() override {
    ++current_;
    const double x = xDist_(eng_);
    const double y = yDist_(eng_);
    last_ = pathfinder::Vector(x,y);
    pathfinder::Vector start = (start_ ? *start_ : pathfinder::Vector(x,y));
    pathfinder::Vector goal = (goal_ ? *goal_ : pathfinder::Vector(x,y));
    return {start, goal};
  }

  int getCurrent() const override {
    return current_;
  }

  int getTotal() const override {
    return kPointCount;
  }
private:
  static constexpr const int kPointCount{1'000};
  std::mt19937 eng_;
  std::uniform_real_distribution<double> xDist_;
  std::uniform_real_distribution<double> yDist_;
  int current_{0};
};

class RegularPointGenerator : public PointGenerator {
public:
  RegularPointGenerator(std::optional<pathfinder::Vector> start, std::optional<pathfinder::Vector> goal, double minX, double maxX, double minY, double maxY) : PointGenerator(start, goal), minX_(minX), maxX_(maxX), minY_(minY), maxY_(maxY), widthIncrement_((maxX-minX) / (kHorizontalPointCount-1)), heightIncrement_((maxY-minY) / (kVerticalPointCount-1)) {
    currentX_ = minX_;
    currentY_ = minY_;
  }

  std::pair<pathfinder::Vector, pathfinder::Vector> getNext() override {
    ++current_;
    last_ = pathfinder::Vector(currentX_, currentY_);
    pathfinder::Vector start = (start_ ? *start_ : pathfinder::Vector(currentX_, currentY_));
    pathfinder::Vector goal = (goal_ ? *goal_ : pathfinder::Vector(currentX_, currentY_));
    currentY_ += heightIncrement_;
    if (currentY_ > maxY_) {
      currentY_ = minY_;
      currentX_ += widthIncrement_;
    }
    return {start, goal};
  }

  int getCurrent() const override {
    return current_;
  }

  int getTotal() const override {
    return kHorizontalPointCount*kVerticalPointCount;
  }
private:
  static constexpr int kHorizontalPointCount{100};
  static constexpr int kVerticalPointCount{100};
  const double minX_, maxX_, minY_, maxY_;
  const double widthIncrement_, heightIncrement_;
  double currentX_, currentY_;
  int current_{0};
};

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
  createMenubar();
  createToolbar();
  createConfigDock();

  // Set up core connections.
  createLocalConnections();

  // Check some boxes
  nonConstraintEdgesCheckBox_->setChecked(true);

  setWindowTitle(tr("Pathfinder Visualization"));

  // Check some boxes
  nonConstraintEdgesCheckBox_->setChecked(true);
  triangleLabelsCheckBox_->setChecked(true);
  edgeLabelsCheckBox_->setChecked(true);
  vertexLabelsCheckBox_->setChecked(true);
}

template<typename TriangulationType>
void MainWindow::generate(const TriangulationType &triangulation, std::unique_ptr<PointGenerator> generator) {
  // Turn off debug logging so the Pathfinder doesnt print.
  absl::SetGlobalVLogLevel(0);

  // If there's another thread running, stop it and wait for it to finish
  stopGenerating();
  navmeshDisplay_->getNavmeshRenderArea()->resetAllPairsDistanceMap();

  // Initialize
  allPairsIsFor_ = AllPairsIsFor::kStart;
  shouldStopGeneration_ = false;
  generationProgressBar_->setValue(0);
  generationProgressBar_->setEnabled(true);

  // Launch generation on a separate thread
  generationThread_ = std::thread([this](const TriangulationType &triangulation, std::unique_ptr<PointGenerator> generator){
    // Create the Pathfinder.
    using PathfinderType = pathfinder::Pathfinder<TriangulationType>;
    PathfinderType pathfinder(triangulation, agentRadius_);

    int current;
    while (!shouldStopGeneration_ && (current = generator->getCurrent()) < generator->getTotal()) {
      // Build path for this goal.
      const auto [start, goal] = generator->getNext();
      const auto last = generator->getLast();
      try {
        typename PathfinderType::PathfindingResult pathfindingResult;
        if constexpr (std::is_same_v<TriangulationType, TriangleLibNavmeshTriangulationType>) {
          pathfindingResult = pathfinder.findShortestPath(start, goal);
        }
        const double pathLength = pathfinder::calculatePathLength(pathfindingResult.shortestPath);
        if (pathfindingResult.shortestPath.empty()) {
          // No path.
          emit newPairsDistanceFound(last.x(), last.y(), std::numeric_limits<double>::max());
        } else {
          emit newPairsDistanceFound(last.x(), last.y(), pathLength);
        }
      } catch (std::exception &ex) {
        emit newPairsDistanceFound(last.x(), last.y(), std::numeric_limits<double>::lowest());
      }
      emit generationProgressUpdate(current, generator->getTotal());
    }
    emit generationProgressUpdate(generator->getTotal(), generator->getTotal());
  }, std::ref(triangulation), std::move(generator));
}

void MainWindow::generateSingleStartRegular() {
  if (!navmeshTriangulation_) {
    LOG(INFO) << "No triangulation";
    return;
  }
  if (!startPoint_) {
    LOG(INFO) << "No start point";
    return;
  }

  NavmeshRenderAreaBase *navmeshRenderAreaBase = navmeshDisplay_->getNavmeshRenderArea();
  const double navmeshMaxX = navmeshRenderAreaBase->getNavmeshMinX() + navmeshRenderAreaBase->getNavmeshWidth();
  const double navmeshMaxY = navmeshRenderAreaBase->getNavmeshMinY() + navmeshRenderAreaBase->getNavmeshHeight();
  std::unique_ptr<PointGenerator> generator(new RegularPointGenerator(*startPoint_, std::nullopt, navmeshRenderAreaBase->getNavmeshMinX(), navmeshMaxX, navmeshRenderAreaBase->getNavmeshMinY(), navmeshMaxY));
  if (std::holds_alternative<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_)) {
    generate(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_), std::move(generator));
  }
}

void MainWindow::generateSingleStartRandom() {
  if (!navmeshTriangulation_) {
    LOG(INFO) << "No triangulation";
    return;
  }
  if (!startPoint_) {
    LOG(INFO) << "No start point";
    return;
  }

  NavmeshRenderAreaBase *navmeshRenderAreaBase = navmeshDisplay_->getNavmeshRenderArea();
  const double navmeshMaxX = navmeshRenderAreaBase->getNavmeshMinX() + navmeshRenderAreaBase->getNavmeshWidth();
  const double navmeshMaxY = navmeshRenderAreaBase->getNavmeshMinY() + navmeshRenderAreaBase->getNavmeshHeight();
  std::unique_ptr<PointGenerator> generator(new RandomPointGenerator(*startPoint_, std::nullopt, navmeshRenderAreaBase->getNavmeshMinX(), navmeshMaxX, navmeshRenderAreaBase->getNavmeshMinY(), navmeshMaxY));
  if (std::holds_alternative<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_)) {
    generate(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_), std::move(generator));
  }
}

void MainWindow::generateSingleGoalRegular() {
  if (!navmeshTriangulation_) {
    LOG(INFO) << "No triangulation";
    return;
  }
  if (!goalPoint_) {
    LOG(INFO) << "No goal point";
    return;
  }

  NavmeshRenderAreaBase *navmeshRenderAreaBase = navmeshDisplay_->getNavmeshRenderArea();
  const double navmeshMaxX = navmeshRenderAreaBase->getNavmeshMinX() + navmeshRenderAreaBase->getNavmeshWidth();
  const double navmeshMaxY = navmeshRenderAreaBase->getNavmeshMinY() + navmeshRenderAreaBase->getNavmeshHeight();
  std::unique_ptr<PointGenerator> generator(new RegularPointGenerator(std::nullopt, *goalPoint_, navmeshRenderAreaBase->getNavmeshMinX(), navmeshMaxX, navmeshRenderAreaBase->getNavmeshMinY(), navmeshMaxY));
  if (std::holds_alternative<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_)) {
    generate(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_), std::move(generator));
  }
}

void MainWindow::generateSingleGoalRandom() {
  if (!navmeshTriangulation_) {
    LOG(INFO) << "No triangulation";
    return;
  }
  if (!goalPoint_) {
    LOG(INFO) << "No goal point";
    return;
  }

  NavmeshRenderAreaBase *navmeshRenderAreaBase = navmeshDisplay_->getNavmeshRenderArea();
  const double navmeshMaxX = navmeshRenderAreaBase->getNavmeshMinX() + navmeshRenderAreaBase->getNavmeshWidth();
  const double navmeshMaxY = navmeshRenderAreaBase->getNavmeshMinY() + navmeshRenderAreaBase->getNavmeshHeight();
  std::unique_ptr<PointGenerator> generator(new RandomPointGenerator(std::nullopt, *goalPoint_, navmeshRenderAreaBase->getNavmeshMinX(), navmeshMaxX, navmeshRenderAreaBase->getNavmeshMinY(), navmeshMaxY));
  if (std::holds_alternative<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_)) {
    generate(std::get<TriangleLibNavmeshTriangulationType>(*navmeshTriangulation_), std::move(generator));
  }
}

void MainWindow::stopGenerating() {
  if (generationThread_.joinable()){
    shouldStopGeneration_ = true;
    generationThread_.join();
  }
  QCoreApplication::instance()->processEvents();
  generationProgressBar_->setEnabled(false);
  generationProgressBar_->setValue(0);
  allPairsIsFor_ = AllPairsIsFor::kNone;
}

MainWindow::~MainWindow() {
  if (generationThread_.joinable()) {
    generationThread_.join();
  }
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

  switchStartAndGoalAction_ = toolBar->addAction(tr("Swap Start and Goal"));
  switchStartAndGoalAction_->setCheckable(false);
  switchStartAndGoalAction_->setIcon(QIcon(":/icons/switch.png"));
  toolbarActionGroup_->addAction(switchStartAndGoalAction_);
}

void MainWindow::createLocalConnections() {
  connect(this, &MainWindow::newPairsDistanceFound, this, &MainWindow::addPairsDistance);
  connect(this, &MainWindow::generationProgressUpdate, this, &MainWindow::updateGenerationProgress);
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
  pathfindingOptionsLayout->addWidget(agentRadiusSlider_, 1, 0, 1, -1);

  // All pairs
  QGridLayout *allPairsGridLayout = new QGridLayout;
  QPushButton *generateSingleStartRegular = new QPushButton("Generate Single-Start Regular");
  QPushButton *generateSingleStartRandom = new QPushButton("Generate Single-Start Random");
  connect(generateSingleStartRegular, &QPushButton::pressed, this, &MainWindow::generateSingleStartRegular);
  connect(generateSingleStartRandom, &QPushButton::pressed, this, &MainWindow::generateSingleStartRandom);
  allPairsGridLayout->addWidget(generateSingleStartRegular, 0, 0, 1, 1);
  allPairsGridLayout->addWidget(generateSingleStartRandom, 1, 0, 1, 1);
  QPushButton *generateSingleGoalRegular = new QPushButton("Generate Single-Goal Regular");
  QPushButton *generateSingleGoalRandom = new QPushButton("Generate Single-Goal Random");
  allPairsGridLayout->addWidget(generateSingleGoalRegular, 2, 0, 1, 1);
  allPairsGridLayout->addWidget(generateSingleGoalRandom, 3, 0, 1, 1);
  connect(generateSingleGoalRegular, &QPushButton::pressed, this, &MainWindow::generateSingleGoalRegular);
  connect(generateSingleGoalRandom, &QPushButton::pressed, this, &MainWindow::generateSingleGoalRandom);
  QPushButton *stopGeneratingButton = new QPushButton("Stop Generating");
  allPairsGridLayout->addWidget(stopGeneratingButton, 4, 0, 1, 1);
  connect(stopGeneratingButton, &QPushButton::pressed, this, &MainWindow::stopGenerating);
  generationProgressBar_ = new QProgressBar;
  // 0%-100%
  generationProgressBar_->setMinimum(0);
  generationProgressBar_->setMaximum(100);
  generationProgressBar_->setValue(0);
  generationProgressBar_->setEnabled(false);
  allPairsGridLayout->addWidget(generationProgressBar_, 5, 0, 1, 1);

  showNoPathToGoalCheckBox_ = new QCheckBox(tr("Show No Paths"));
  showExceptionCheckBox_ = new QCheckBox(tr("Show Exceptions"));
  showNoPathToGoalCheckBox_->setChecked(true);
  showExceptionCheckBox_->setChecked(true);
  allPairsGridLayout->addWidget(showNoPathToGoalCheckBox_, 6, 0, 1, 1);
  allPairsGridLayout->addWidget(showExceptionCheckBox_, 7, 0, 1, 1);

  QGroupBox *singleStartAllGoalsGroupBox = new QGroupBox("All Pairs");
  singleStartAllGoalsGroupBox->setLayout(allPairsGridLayout);

  pathfindingOptionsLayout->addWidget(singleStartAllGoalsGroupBox, 2, 0, 1, -1);
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

  // Polyanya animation widgets
  stepBackPlaybackButton_ = new QPushButton(tr("<"));
  startPlaybackButton_ = new QPushButton(tr("Play"));
  pausePlaybackButton_ = new QPushButton(tr("Pause"));
  stopPlaybackButton_ = new QPushButton(tr("Stop"));
  stepForwardPlaybackButton_ = new QPushButton(tr(">"));
  animationCurrentFrameLineEdit_ = new QLineEdit(tr("0"));
  animationFrameCountLabel_ = new QLabel(tr("0"));
  QHBoxLayout *playbackButtonsHorizontalLayout = new QHBoxLayout;
  playbackButtonsHorizontalLayout->addWidget(stepBackPlaybackButton_);
  playbackButtonsHorizontalLayout->addWidget(startPlaybackButton_);
  playbackButtonsHorizontalLayout->addWidget(pausePlaybackButton_);
  playbackButtonsHorizontalLayout->addWidget(stopPlaybackButton_);
  playbackButtonsHorizontalLayout->addWidget(stepForwardPlaybackButton_);
  QWidget *playbackButtonsWidget = new QWidget;
  playbackButtonsWidget->setLayout(playbackButtonsHorizontalLayout);
  QVBoxLayout *polyanyaVerticalLayout = new QVBoxLayout;
  polyanyaVerticalLayout->addWidget(playbackButtonsWidget);
  polyanyaVerticalLayout->addWidget(animationCurrentFrameLineEdit_);
  polyanyaVerticalLayout->addWidget(animationFrameCountLabel_);
  QGroupBox *polyanyaAnimationGroupbox = new QGroupBox("Polyanya Animation");
  polyanyaAnimationGroupbox->setLayout(polyanyaVerticalLayout);

  QGridLayout *navmeshTriangulationVisualizationOptionsLayout = new QGridLayout;
  navmeshTriangulationVisualizationOptionsLayout->addWidget(verticesCheckBox_, 0, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(nonConstraintEdgesCheckBox_, 1, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(triangleCorridorCheckBox_, 2, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(trianglesCompletelySearchedCheckBox_, 3, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(trianglesVisitedCheckBox_, 4, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(triangleLabelsCheckBox_, 5, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(edgeLabelsCheckBox_, 6, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(vertexLabelsCheckBox_, 7, 0, 1, 1);
  navmeshTriangulationVisualizationOptionsLayout->addWidget(polyanyaAnimationGroupbox, 8, 0, 1, 1);

  // Polyanya animation widget

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
  connect(switchStartAndGoalAction_, &QAction::triggered, this, &MainWindow::switchStartAndGoal);

  // Configuration
  connect(verticesCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayVertices);
  connect(nonConstraintEdgesCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayNonConstraintEdges);
  connect(triangleCorridorCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTriangleCorridor);
  connect(trianglesCompletelySearchedCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTrianglesCompletelySearched);
  connect(trianglesVisitedCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTrianglesVisited);
  connect(triangleLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayTriangleLabels);
  connect(edgeLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayEdgeLabels);
  connect(vertexLabelsCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setDisplayVertexLabels);

  // All Pairs
  connect(showNoPathToGoalCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setAllPairsShowNoPathToGoal);
  connect(showExceptionCheckBox_, &QCheckBox::toggled, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setAllPairsShowException);

  // Polyanya animation
  connect(stepBackPlaybackButton_, &QPushButton::pressed, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::stepBackPlaybackAnimation);
  connect(startPlaybackButton_, &QPushButton::pressed, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::startPlaybackAnimation);
  connect(pausePlaybackButton_, &QPushButton::pressed, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::pausePlaybackAnimation);
  connect(stopPlaybackButton_, &QPushButton::pressed, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::stopPlaybackAnimation);
  connect(stepForwardPlaybackButton_, &QPushButton::pressed, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::stepForwardPlaybackAnimation);
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::animationDataUpdated, this, &MainWindow::animationDataUpdated);
  connect(animationCurrentFrameLineEdit_, &QLineEdit::textEdited, navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::setFramePlaybackAnimation);

  // Mouse event handling
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::draggingMouseOnNavmesh, this, &MainWindow::draggingMouseOnNavmesh);
  connect(navmeshDisplay_->getNavmeshRenderArea(), &NavmeshRenderAreaBase::movingMouseOnNavmesh, this, &MainWindow::movingMouseOnNavmesh);
}

void MainWindow::addPairsDistance(double x, double y, double distance) {
  navmeshDisplay_->getNavmeshRenderArea()->addPairsDistance(x, y, distance);
}

void MainWindow::updateGenerationProgress(int countCompleted, int countTotal) {
  // Calculate the percentage.
  const double percentage = std::round(100.0 * static_cast<double>(countCompleted)/countTotal);
  if (generationProgressBar_->isEnabled()) {
    generationProgressBar_->setValue(percentage);
  }
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

void MainWindow::animationDataUpdated(int currentIndex, int frameCount) {
  animationCurrentFrameLineEdit_->setValidator(new QIntValidator(-1,frameCount, this));
  animationCurrentFrameLineEdit_->setText(QString::number(currentIndex));
  animationFrameCountLabel_->setText(QString::number(frameCount));
}

void MainWindow::movePathStart(const pathfinder::Vector &pos) {
  if (allPairsIsFor_ == AllPairsIsFor::kStart) {
    stopGenerating();
    resetAllPairsDistanceMap();
  }
  absl::SetGlobalVLogLevel(0);
  navmeshDisplay_->setPathStartPoint(pos);
  startPoint_ = pos;
  if (startPoint_ && goalPoint_) {
    // Have both start and goal
    rebuildPath();
  }
}

void MainWindow::movePathGoal(const pathfinder::Vector &pos) {
  if (allPairsIsFor_ == AllPairsIsFor::kGoal) {
    stopGenerating();
    resetAllPairsDistanceMap();
  }
  absl::SetGlobalVLogLevel(0);
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

void MainWindow::setMovePathStartEnabled(bool enabled) {
  movePathStartEnabled_ = enabled;
  navmeshDisplay_->getNavmeshRenderArea()->setHandleMouseDrag(!dragAction_->isChecked());
}

void MainWindow::setMovePathGoalEnabled(bool enabled) {
  movePathGoalEnabled_ = enabled;
  navmeshDisplay_->getNavmeshRenderArea()->setHandleMouseDrag(!dragAction_->isChecked());
}

void MainWindow::switchStartAndGoal() {
  LOG(INFO) << "Swap!";
  if (!startPoint_ || !goalPoint_) {
    // Need both in order to do anything.
    return;
  }
  // Save start & goal.
  const auto oldStart = *startPoint_;
  const auto oldGoal = *goalPoint_;
  // Clear path.
  resetPathData();
  movePathStart(oldGoal);
  movePathGoal(oldStart);
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

  // If any generation is running, stop it and clean it up
  stopGenerating();
  resetAllPairsDistanceMap();

  // Update navmesh
  navmeshDisplay_->setAgentRadius(agentRadius_);
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

void MainWindow::resetAllPairsDistanceMap() {
  if (navmeshDisplay_ != nullptr) {
    navmeshDisplay_->getNavmeshRenderArea()->resetAllPairsDistanceMap();
  }
}

void MainWindow::openPolyFile(const QString &filename) {
  resetAllPairsDistanceMap();
  try {
    // Try to open the file and build the navmesh triangulation.
    buildTriangleLibNavmeshTriangulationFromFile(filename);

    // Build the navmesh display for this type of navmesh triangulation.
    createNavmeshDisplay<NavmeshDisplay<TriangleLibNavmeshTriangulationType>>();

    // Give the navmesh triangulation to the display.
    auto &concreteNavmeshDisplay = dynamic_cast<NavmeshDisplay<TriangleLibNavmeshTriangulationType>&>(*navmeshDisplay_);
    concreteNavmeshDisplay.setNavmeshName(filename.toStdString());
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
  triangle::behavior_t behavior = behaviorBuilder_.getBehavior();
  int behaviorSetResult = triangle::triangle_context_set_behavior(ctx, &behavior);
  if (behaviorSetResult < 0) {
    // Free memory
    triangle_context_destroy(ctx);
    triangle_free_triangleio(&inputData);
    throw std::runtime_error("Error setting behavior ("+std::to_string(behaviorSetResult)+")");
  }

  // Build the triangle mesh
  int meshCreateResult = triangle::triangle_mesh_create(ctx, &inputData);
  if (meshCreateResult < 0) {
    // Free memory
    triangle_context_destroy(ctx);
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
    triangle_context_destroy(ctx);
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
