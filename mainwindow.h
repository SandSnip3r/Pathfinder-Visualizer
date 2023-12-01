#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "navmeshdisplay.h"
#include "zoomablescrollarea.hpp"

#include <Pathfinder/behaviorBuilder.h>

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
#include <vector>

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  using NavmeshTriangulationType = pathfinder::navmesh::TriangleLibNavmesh;
  using PathfinderType = pathfinder::Pathfinder<NavmeshTriangulationType>;
  using PathfindingResult = PathfinderType::PathfindingResult;

  const QString kSampleNavmeshFileName_{tr("./maze.poly")};

  // Toolbar UI elements
  QAction *dragAction_;
  QAction *movePathStartAction_;
  QAction *movePathGoalAction_;
  QActionGroup *toolbarActionGroup_;

  // Config UI elements
  QLineEdit *agentRadiusLineEdit_;
  QSlider *agentRadiusSlider_;

  QCheckBox *verticesCheckBox_;
  QCheckBox *nonConstraintEdgesCheckBox_;
  QCheckBox *triangleCorridorCheckBox_;
  QCheckBox *trianglesCompletelySearchedCheckBox_;
  QCheckBox *trianglesVisitedCheckBox_;
  QCheckBox *triangleLabelsCheckBox_;
  QCheckBox *edgeLabelsCheckBox_;
  QCheckBox *vertexLabelsCheckBox_;

  NavmeshDisplay *navmeshDisplay_;

  // UI creation functions
  void createMenubar();
  void createToolbar();
  void createNavmeshDisplay();
  void createConfigDock();
  void createConnectionsToNavmeshDisplay();

  // UI maninpulation functions
  bool matchingAgentRadiusLineEditAndSlider_{false};
  bool matchingTriangulationMinimumAngleLineEditAndSlider_{false};
  void setAgentRadiusLineEdit();
  void setAgentRadiusSlider();

  // Navmesh data
  pathfinder::BehaviorBuilder behaviorBuilder_;
  std::optional<pathfinder::navmesh::TriangleLibNavmesh> triangleLibNavmeshTriangulation_;
  // Navmesh functions
  void resetPathData();
  void openNavmeshFile(const QString &filename);
  void buildNavmeshFromFile(QString fileName);

  // Path data
  double agentRadius_{7.5};
  bool movePathStartEnabled_{false};
  bool movePathGoalEnabled_{false};
  std::optional<pathfinder::Vector> startPoint_, goalPoint_;
  PathfindingResult pathfindingResult_;

  // Path functions
  void rebuildPath();
  void movePathStart(const pathfinder::Vector &pos);
  void movePathGoal(const pathfinder::Vector &pos);

private slots:
  void openNavmeshFilePrompt();
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void setMovePathStartEnabled(bool enabled);
  void setMovePathGoalEnabled(bool enabled);
  void agentRadiusTextChanged(const QString &text);
  void agentRadiusSliderChanged(int value);
};

#endif // MAINWINDOW_H
