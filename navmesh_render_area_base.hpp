#ifndef NAVMESH_RENDER_AREA_BASE_HPP_
#define NAVMESH_RENDER_AREA_BASE_HPP_

#include <Pathfinder/pathfinder.h>
#include <Pathfinder/triangle_lib_navmesh.h>
#include <Pathfinder/vector.h>

#include <QColor>
#include <QWidget>

class NavmeshRenderAreaBase : public QWidget {
  Q_OBJECT
public:
  using QWidget::QWidget;

  void resetZoom();
  void zoomIn(double zoomDiff);
  void zoomOut(double zoomDiff);

  bool getDisplayVertices() const;
  bool getDisplayNonConstraintEdges() const;
  bool getDisplayTriangleCorridor() const;
  bool getDisplayTrianglesCompletelySearched() const;
  bool getDisplayTrianglesVisited() const;
  bool getDisplayTriangleLabels() const;
  bool getDisplayEdgeLabels() const;
  bool getDisplayVertexLabels() const;

  void setAgentRadius(double agentRadius);
  void setPathStartPoint(const pathfinder::Vector &point);
  void setPathGoalPoint(const pathfinder::Vector &point);
  void resetPathStart();
  void resetPathGoal();
  virtual void resetPath() = 0;

  void setHandleMouseDrag(bool enabled);
  void setHandleMouseClick(bool enabled);

protected:
  // View data
  int widgetBaseWidth_{0}, widgetBaseHeight_{0};
  double zoomLevel_{0};
  bool handleMouseDrag_{false};
  bool handleMouseClick_{false};

  // Display configurations
  bool displayVertices_{false};
  bool displayNonConstraintEdges_{true};

  bool displayTriangleCorridor_{false};
  bool displayTrianglesCompletelySearched_{false};
  bool displayTrianglesVisited_{false};

  bool displayTriangleLabels_{false};
  bool displayEdgeLabels_{false};
  bool displayVertexLabels_{false};

  // Pathfinding data
  double agentRadius_{0.0};
  std::optional<pathfinder::Vector> startPoint_;
  std::optional<pathfinder::Vector> goalPoint_;

  QSize currentSize() const;
  void resizeForNewZoom();
  double getScale() const;

  virtual QColor getColorForEdgeMarker(const int marker);

signals:
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  // void mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint);

public slots:
  void setDisplayVertices(bool shouldDisplay);
  void setDisplayNonConstraintEdges(bool shouldDisplay);
  void setDisplayTriangleCorridor(bool shouldDisplay);
  void setDisplayTrianglesCompletelySearched(bool shouldDisplay);
  void setDisplayTrianglesVisited(bool shouldDisplay);
  void setDisplayTriangleLabels(bool shouldDisplay);
  void setDisplayEdgeLabels(bool shouldDisplay);
  void setDisplayVertexLabels(bool shouldDisplay);
};

#endif // NAVMESH_RENDER_AREA_BASE_HPP_
