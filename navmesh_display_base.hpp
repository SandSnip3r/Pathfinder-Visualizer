#ifndef NAVMESH_DISPLAY_BASE_HPP_
#define NAVMESH_DISPLAY_BASE_HPP_

#include "navmesh_render_area.hpp"
#include "zoomable_scroll_area.hpp"

#include <Pathfinder/pathfinder.h>

#include <QWidget>

class NavmeshDisplayBase : public QWidget {
  Q_OBJECT

public:
  using QWidget::QWidget;
  virtual NavmeshRenderAreaBase* getNavmeshRenderArea() = 0;
  virtual void setPathStartPoint(const pathfinder::Vector &pos) = 0;
  virtual void setPathGoalPoint(const pathfinder::Vector &pos) = 0;
  virtual void setMousePosition(const pathfinder::Vector &pos) = 0;
  virtual void resetPathStart() = 0;
  virtual void resetPathGoal() = 0;
  virtual void resetPath() = 0;

protected:
  ZoomableScrollArea *navmeshRenderScrollArea_;

public slots:
  void setDragModeEnabled(bool enabled);
};

#endif // NAVMESH_DISPLAY_BASE_HPP_
