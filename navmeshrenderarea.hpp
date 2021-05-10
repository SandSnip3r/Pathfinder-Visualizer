#ifndef NAVMESHRENDERAREA_HPP
#define NAVMESHRENDERAREA_HPP

#include "Pathfinder/behaviorBuilder.h"
#include "Pathfinder/navmesh_interface.h"
#include "Pathfinder/pathfinder.h"
#include "Pathfinder/vector.h"

#include <QPainter>
#include <QString>
#include <QWheelEvent>
#include <QWidget>

class NavmeshRenderArea : public QWidget {
  Q_OBJECT
public:
  explicit NavmeshRenderArea(QWidget *parent = nullptr);
  QSize minimumSizeHint() const override;
  void mouseMoveEvent(QMouseEvent *event) override;
  QSize sizeHint() const override;

  void resetZoom();
  void zoomIn(double zoomDiff);
  void zoomOut(double zoomDiff);

  void setAgentRadius(double agentRadius);
  void setNavmesh(const pathfinder::navmesh::NavmeshInterface &navmesh);
  void setPathStartPoint(const pathfinder::Vector &point);
  void setPathGoalPoint(const pathfinder::Vector &point);
  void setPath(const pathfinder::PathfindingResult &pathfindingResult);
  void resetPathStart();
  void resetPathGoal();
  void resetPath();

  void setHandleMouseDrag(bool enabled);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  // View data
  static const int kMargin_{25};
  double navmeshMinX_, navmeshMinY_;
  double navmeshWidth_{1000}, navmeshHeight_{navmeshWidth_/1.618033988749895}; // TODO: How to init
  int widgetBaseWidth_{0}, widgetBaseHeight_{0};
  double zoomLevel_{0};
  bool handleMouseDrag_{false};

  // Display configurations
  bool displayNonConstraintEdges_{false};

  bool displayTriangleCorridor_{false};
  bool displayTrianglesCompletelySearched_{false};
  bool displayTrianglesVisited_{false};

  bool displayTriangleLabels_{false};
  bool displayEdgeLabels_{false};
  bool displayVertexLabels_{false};

  // Navmesh data
  const pathfinder::navmesh::NavmeshInterface *navmesh_{nullptr};

  // Pathfinding data
  double agentRadius_{0.0};
  std::optional<pathfinder::Vector> startPoint_;
  std::optional<pathfinder::Vector> goalPoint_;
  const pathfinder::PathfindingResult *pathfindingResult_{nullptr};

  void setSizeBasedOnNavmesh();
  QSize currentSize() const;
  void drawVertices(QPainter &painter);
  void drawEdges(QPainter &painter);

  void drawShortestPath(QPainter &painter);
  void drawPathfindingStartAndGoal(QPainter &painter);
  void drawTriangles(QPainter &painter, const std::vector<int> &triangles, const QColor &color);
  void drawTriangleCorridor(QPainter &painter);
  void drawTrianglesCompletelySearched(QPainter &painter);
  void drawTrianglesVisited(QPainter &painter);

  void drawVertexLabels(QPainter &painter);
  void drawEdgeLabels(QPainter &painter);
  void drawTriangleLabels(QPainter &painter);

  void resizeForNewZoom();
  double getScale() const;

  pathfinder::Vector transformWidgetCoordinateToNavmeshCoordinate(const pathfinder::Vector &v) const;
  pathfinder::Vector transformNavmeshCoordinateToWidgetCoordinate(const pathfinder::Vector &v) const;
  pathfinder::Vector transformVectorToRenderArea(const pathfinder::Vector &v) const;

signals:
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);

public slots:
  void setDisplayNonConstraintEdges(bool shouldDisplay);
  void setDisplayTriangleCorridor(bool shouldDisplay);
  void setDisplayTrianglesCompletelySearched(bool shouldDisplay);
  void setDisplayTrianglesVisited(bool shouldDisplay);
  void setDisplayTriangleLabels(bool shouldDisplay);
  void setDisplayEdgeLabels(bool shouldDisplay);
  void setDisplayVertexLabels(bool shouldDisplay);
};

#endif // NAVMESHRENDERAREA_HPP
