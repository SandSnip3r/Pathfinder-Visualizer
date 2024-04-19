#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include "navmesh_display_base.hpp"
#include "navmesh_display.hpp"
#include "zoomable_scroll_area.hpp"

#include <Pathfinder/behaviorBuilder.h>
#include <Pathfinder/pathfinder.h>
#include <Pathfinder/triangle_lib_navmesh.h>

#include <QAction>
#include <QActionGroup>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QString>

#include <atomic>
#include <memory>
#include <optional>
#include <thread>
#include <variant>
#include <vector>

class PointGenerator {
public:
  PointGenerator(std::optional<pathfinder::Vector> start, std::optional<pathfinder::Vector> goal) : start_(start), goal_(goal) {
    if (start_.has_value() == goal_.has_value()) {
      // Only supports receiving a start or a goal, not both nor neither.
      throw std::runtime_error("Given invalid start/goal pair");
    }
  }
  virtual std::pair<pathfinder::Vector, pathfinder::Vector> getNext() = 0;
  virtual int getCurrent() const = 0;
  virtual int getTotal() const = 0;
  pathfinder::Vector getLast() const {
    return last_;
  }
protected:
  const std::optional<pathfinder::Vector> start_;
  const std::optional<pathfinder::Vector> goal_;
  pathfinder::Vector last_;
};

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  using TriangleLibNavmeshTriangulationType = pathfinder::navmesh::TriangleLibNavmesh;
  using TriangleLibPathfinderType = pathfinder::Pathfinder<TriangleLibNavmeshTriangulationType>;
  using TriangleLibPathfindingResult = TriangleLibPathfinderType::PathfindingResult;

  // Toolbar UI elements
  QAction *dragAction_;
  QAction *movePathStartAction_;
  QAction *movePathGoalAction_;
  QAction *switchStartAndGoalAction_;
  QActionGroup *toolbarActionGroup_;

  // Config UI elements
  QTabWidget *configTabWidget_;
  std::optional<int> navmeshRegionTabIndex_;
  QLineEdit *agentRadiusLineEdit_;
  QSlider *agentRadiusSlider_;
  QProgressBar *generationProgressBar_;
  QCheckBox *showNoPathToGoalCheckBox_;
  QCheckBox *showExceptionCheckBox_;

  // Navmesh Triangulation config UI elements
  QCheckBox *verticesCheckBox_;
  QCheckBox *nonConstraintEdgesCheckBox_;
  QCheckBox *triangleLabelsCheckBox_;
  QCheckBox *edgeLabelsCheckBox_;
  QCheckBox *vertexLabelsCheckBox_;
  
  // Polyanya animation UI elements
  QPushButton *stepBackPlaybackButton_;
  QPushButton *startPlaybackButton_;
  QPushButton *pausePlaybackButton_;
  QPushButton *stopPlaybackButton_;
  QPushButton *stepForwardPlaybackButton_;
  QLineEdit *animationCurrentFrameLineEdit_;
  QLabel *animationFrameCountLabel_;

  NavmeshDisplayBase *navmeshDisplay_{nullptr};

  // UI creation functions
  void createMenubar();
  void createToolbar();
  void createLocalConnections();

  template<typename NavmeshTriangulationType>
  void createNavmeshDisplay();

  void generateSingleStartRegular();
  void generateSingleStartRandom();
  void generateSingleGoalRegular();
  void generateSingleGoalRandom();
  template<typename TriangulationType>
  void generate(const TriangulationType &triangulation, std::unique_ptr<PointGenerator> generator);

  void stopGenerating();

  void createConfigDock();
  void populateConfigDock();
  void createConnectionsToNavmeshDisplay();

  // UI maninpulation functions
  bool matchingAgentRadiusLineEditAndSlider_{false};
  bool matchingTriangulationMinimumAngleLineEditAndSlider_{false};
  void setAgentRadiusLineEdit();
  void setAgentRadiusSlider();

  // Navmesh data
  std::optional<std::variant<TriangleLibNavmeshTriangulationType>> navmeshTriangulation_;
  // TriangleLib-specific navmesh data
  pathfinder::BehaviorBuilder behaviorBuilder_;

  // Navmesh functions
  void updateAgentRadius(double newRadius);
  void resetPathData();
  void resetAllPairsDistanceMap();
  // TriangleLib-specific navmesh functions
  void openPolyFile(const QString &filename);
  void buildTriangleLibNavmeshTriangulationFromFile(QString fileName);

  // Path data
  double agentRadius_{7.5};
  bool movePathStartEnabled_{false};
  bool movePathGoalEnabled_{false};
  std::optional<pathfinder::Vector> startPoint_, goalPoint_;
  std::optional<std::variant<TriangleLibPathfindingResult>> pathfindingResult_;

  // All pairs generation
public:
  enum class AllPairsIsFor {
    kNone,
    kStart,
    kGoal
  };
private:
  AllPairsIsFor allPairsIsFor_{AllPairsIsFor::kNone};
  std::atomic_bool shouldStopGeneration_;
  std::thread generationThread_;

  // Path functions
  void rebuildPath();
  void movePathStart(const pathfinder::Vector &pos);
  void movePathGoal(const pathfinder::Vector &pos);

signals:
  void newPairsDistanceFound(double x, double y, double distance);
  void generationProgressUpdate(int countCompleted, int countTotal);

private slots:
  void addPairsDistance(double x, double y, double distance);
  void updateGenerationProgress(int countCompleted, int countTotal);
  void openNavmeshFilePrompt();
  void animationDataUpdated(int currentIndex, int frameCount);
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void setMovePathStartEnabled(bool enabled);
  void setMovePathGoalEnabled(bool enabled);
  void switchStartAndGoal();
  void agentRadiusTextChanged(const QString &text);
  void agentRadiusSliderChanged(int value);
};

template<typename NavmeshDisplayType>
void MainWindow::createNavmeshDisplay() {
  // Check if the navmesh display is already this type
  if (navmeshDisplay_ != nullptr) {
    if (dynamic_cast<NavmeshDisplayType*>(navmeshDisplay_) != nullptr) {
      // Already the desired type.
      return;
    }
  }
  navmeshDisplay_ = new NavmeshDisplayType;
  setCentralWidget(navmeshDisplay_);
  // When we call setCentralWidget, we pass ownership. No need to worry about deleting the previous one, if there is one.

  navmeshDisplay_->getNavmeshRenderArea()->setAgentRadius(agentRadius_);

  navmeshDisplay_->setDragModeEnabled(dragAction_->isChecked());
  setMovePathStartEnabled(movePathStartAction_->isChecked());
  setMovePathGoalEnabled(movePathGoalAction_->isChecked());

  createConnectionsToNavmeshDisplay();
  populateConfigDock();
}

#endif // MAIN_WINDOW_HPP_
