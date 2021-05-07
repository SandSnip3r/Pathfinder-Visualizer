#ifndef NAVMESHDISPLAY_H
#define NAVMESHDISPLAY_H

#include "navmeshrenderarea.hpp"
#include "zoomablescrollarea.hpp"

#include <QLabel>
#include <QWidget>

#include <optional>

class NavmeshDisplay : public QWidget {
  Q_OBJECT
public:
  explicit NavmeshDisplay(QWidget *parent = nullptr);
  NavmeshRenderArea* getNavmeshRenderArea();
  void setNavmesh(const triangle::triangleio &triangleData);
  void setPathStartPoint(const pathfinder::Vector &pos);
  void setPathGoalPoint(const pathfinder::Vector &pos);
  void setMousePosition(const pathfinder::Vector &pos);
  void setPath(const pathfinder::PathfindingResult &pathfindingResult);
  void resetPathStart();
  void resetPathGoal();
  void resetPath();
private:
  ZoomableScrollArea *navmeshRenderScrollArea_;
  NavmeshRenderArea *navmeshRenderArea_;
  QLabel *pathStartPositionLabel_;
  QLabel *pathGoalPositionLabel_;
  QLabel *pathLengthLabel_;
  QLabel *vertexCountLabel_;
  QLabel *triangleCountLabel_;
  QLabel *totalEdgeCountLabel_;
  QLabel *constrainedEdgeCountLabel_;
  QLabel *mousePositionLabel_;
  QWidget* createTextDisplayArea();
  QString pathStartPointLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;
  QString pathGoalPointLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;
  QString pathLengthLabelContents(const std::optional<double> &length = {}) const;
  QString vertexCountLabelContents(const std::optional<int> &count = {}) const;
  QString triangleCountLabelContents(const std::optional<int> &count = {}) const;
  QString totalEdgeCountLabelContents(const std::optional<int> &count = {}) const;
  QString constrainedEdgeCountLabelContents(const std::optional<int> &count = {}) const;
  QString mousePositionLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;

signals:
public slots:
  void setDragModeEnabled(bool enabled);
};

#endif // NAVMESHDISPLAY_H
