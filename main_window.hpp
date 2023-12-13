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
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QString>

#include <memory>
#include <optional>
#include <variant>
#include <vector>

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  using TriangleLibNavmeshTriangulationType = pathfinder::navmesh::TriangleLibNavmesh;
  using TriangleLibPathfinderType = pathfinder::Pathfinder<TriangleLibNavmeshTriangulationType>;
  using TriangleLibPathfindingResult = TriangleLibPathfinderType::PathfindingResult;

  const QString kSampleNavmeshFileName_{tr("./maze.poly")};

  // Toolbar UI elements
  QAction *dragAction_;
  QAction *movePathStartAction_;
  QAction *movePathGoalAction_;
  QActionGroup *toolbarActionGroup_;

  // Config UI elements
  QTabWidget *configTabWidget_;
  QLineEdit *agentRadiusLineEdit_;
  QSlider *agentRadiusSlider_;

  // Navmesh Triangulation config UI elements
  QCheckBox *verticesCheckBox_;
  QCheckBox *nonConstraintEdgesCheckBox_;
  QCheckBox *triangleCorridorCheckBox_;
  QCheckBox *trianglesCompletelySearchedCheckBox_;
  QCheckBox *trianglesVisitedCheckBox_;
  QCheckBox *triangleLabelsCheckBox_;
  QCheckBox *edgeLabelsCheckBox_;
  QCheckBox *vertexLabelsCheckBox_;

  NavmeshDisplayBase *navmeshDisplay_{nullptr};

  // UI creation functions
  void createMenubar();
  void createToolbar();

  template<typename NavmeshTriangulationType>
  void createNavmeshDisplay();

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
  // TriangleLib-specific navmesh functions
  void openPolyFile(const QString &filename);
  void buildTriangleLibNavmeshTriangulationFromFile(QString fileName);

  // Path data
  double agentRadius_{7.5};
  bool movePathStartEnabled_{false};
  bool movePathGoalEnabled_{false};
  std::optional<pathfinder::Vector> startPoint_, goalPoint_;
  std::optional<std::variant<TriangleLibPathfindingResult>> pathfindingResult_;

  // Path functions
  void rebuildPath();
  void movePathStart(const pathfinder::Vector &pos);
  void movePathGoal(const pathfinder::Vector &pos);

private slots:
  void openNavmeshFilePrompt();
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  // void mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void setMovePathStartEnabled(bool enabled);
  void setMovePathGoalEnabled(bool enabled);
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
