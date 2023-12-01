#ifndef NAVMESHRENDERAREA_HPP
#define NAVMESHRENDERAREA_HPP

#include <Pathfinder/pathfinder.h>
#include <Pathfinder/triangle_lib_navmesh.h>
#include <Pathfinder/vector.h>

#include <QPainter>
#include <QString>
#include <QWheelEvent>
#include <QWidget>

class NavmeshRenderArea : public QWidget {
  Q_OBJECT
public:
  using NavmeshTriangulationType = pathfinder::navmesh::TriangleLibNavmesh;
private:
  using PathfinderType = pathfinder::Pathfinder<NavmeshTriangulationType>;
  using PathfindingResult = PathfinderType::PathfindingResult;
  using IndexType = NavmeshTriangulationType::IndexType;
public:
  explicit NavmeshRenderArea(QWidget *parent = nullptr);
  QSize minimumSizeHint() const override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  QSize sizeHint() const override;

  void resetZoom();
  void zoomIn(double zoomDiff);
  void zoomOut(double zoomDiff);

  void setAgentRadius(double agentRadius);
  void setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation);
  void setPathStartPoint(const pathfinder::Vector &point);
  void setPathGoalPoint(const pathfinder::Vector &point);
  void setPath(const PathfindingResult &pathfindingResult);
  void resetPathStart();
  void resetPathGoal();
  void resetPath();

  void setHandleMouseDrag(bool enabled);
  void setHandleMouseClick(bool enabled);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  // View data
  double navmeshRenderAreaMargin_{0.0};
  double navmeshMinX_{0.0}, navmeshMinY_{0.0};
  double navmeshWidth_{0.0}, navmeshHeight_{0.0};
  int widgetBaseWidth_{0}, widgetBaseHeight_{0};
  double zoomLevel_{0};
  bool handleMouseDrag_{false};
  bool handleMouseClick_{false};

  // Display configurations
  bool displayVertices_{false};
  bool displayNonConstraintEdges_{false};

  bool displayTriangleCorridor_{false};
  bool displayTrianglesCompletelySearched_{false};
  bool displayTrianglesVisited_{false};

  bool displayTriangleLabels_{false};
  bool displayEdgeLabels_{false};
  bool displayVertexLabels_{false};

  // Navmesh data
  const NavmeshTriangulationType *navmeshTriangulation_{nullptr};

  // Pathfinding data
  double agentRadius_{0.0};
  std::optional<pathfinder::Vector> startPoint_;
  std::optional<pathfinder::Vector> goalPoint_;
  const PathfindingResult *pathfindingResult_{nullptr};

  void setSizeBasedOnNavmesh();
  QSize currentSize() const;

  void drawVertices(QPainter &painter);
  void drawEdges(QPainter &painter);
  QColor getColorForEdgeMarker(const int marker);

  void drawShortestPath(QPainter &painter);
  void drawPathfindingStartAndGoal(QPainter &painter);
  void drawTriangles(QPainter &painter, const std::vector<IndexType> &triangles, const QColor &color);
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
  void mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint);

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

#endif // NAVMESHRENDERAREA_HPP
