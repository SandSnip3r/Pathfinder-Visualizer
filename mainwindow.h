#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "navmeshdisplay.h"
#include "zoomablescrollarea.hpp"

#include <QAction>
#include <QActionGroup>
#include <QCheckBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QSlider>
#include <QString>


#include <optional>

class MainWindow : public QMainWindow {
  Q_OBJECT
public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
private:
  const QString kSampleNavmeshFileName_{tr("./sample.poly")};

  // Toolbar UI elements
  QAction *dragAction_;
  QAction *movePathStartAction_;
  QAction *movePathGoalAction_;
  QActionGroup *toolbarActionGroup_;

  // Config UI elements
  QLineEdit *agentRadiusLineEdit_;
  QSlider *agentRadiusSlider_;

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
  triangleio savedTriangleData_;
  triangleio savedTriangleVoronoiData_;
  BehaviorBuilder behaviorBuilder_;
  // Navmesh functions
  void initializeInputTriangleData();
  void initializeNavmeshTriangleData();
  void openNavmeshFile(const QString &filename);
  void buildNavmeshFromFile(QString fileName);

  // Path data
  double agentRadius_{5};
  bool movePathStartEnabled_{false};
  bool movePathGoalEnabled_{false};
  std::optional<Vector> startPoint_;
  std::optional<Vector> goalPoint_;
  PathfindingResult pathfindingResult_;

  // Path functions
  void rebuildPath();
  void movePathStart(const Vector &pos);
  void movePathGoal(const Vector &pos);

private slots:
  void openNavmeshFilePrompt();
  void draggingMouseOnNavmesh(const Vector &navmeshPoint);
  void setMovePathStartEnabled(bool enabled);
  void setMovePathGoalEnabled(bool enabled);
  void agentRadiusTextChanged(const QString &text);
  void agentRadiusSliderChanged(int value);
};

#endif // MAINWINDOW_H
