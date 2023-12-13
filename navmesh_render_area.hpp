#ifndef NAVMESH_RENDER_AREA_HPP_
#define NAVMESH_RENDER_AREA_HPP_

#include "navmesh_render_area_base.hpp"

#include <Pathfinder/pathfinder.h>
#include <Pathfinder/triangle_lib_navmesh.h>
#include <Pathfinder/vector.h>

#include <QPainter>
#include <QString>
#include <QWheelEvent>
#include <QWidget>

template<typename NavmeshTriangulationType>
class NavmeshRenderArea : public NavmeshRenderAreaBase {
private:
  using PathfinderType = pathfinder::Pathfinder<NavmeshTriangulationType>;
  using PathfindingResult = typename PathfinderType::PathfindingResult;
  using IndexType = typename NavmeshTriangulationType::IndexType;
public:
  explicit NavmeshRenderArea(QWidget *parent = nullptr);
  QSize minimumSizeHint() const override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  QSize sizeHint() const override;

  void setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation);
  void setPath(const PathfindingResult &pathfindingResult);
  void resetPath() override;

protected:
  // Navmesh data
  const NavmeshTriangulationType *navmeshTriangulation_{nullptr};

  void paintEvent(QPaintEvent *event) final;
  virtual void paintUnderNavmeshTriangulation(QPainter &painter) { /* Do nothing */ }

  pathfinder::Vector transformWidgetCoordinateToNavmeshCoordinate(const pathfinder::Vector &v) const;
  pathfinder::Vector transformNavmeshCoordinateToWidgetCoordinate(const pathfinder::Vector &v) const;

private:
  // View data
  double navmeshRenderAreaMargin_{0.0};
  double navmeshMinX_{0.0}, navmeshMinY_{0.0};
  double navmeshWidth_{0.0}, navmeshHeight_{0.0};

  // Pathfinding data
  const PathfindingResult *pathfindingResult_{nullptr};

  void setSizeBasedOnNavmesh();

  void drawVertices(QPainter &painter);
  void drawEdges(QPainter &painter);

  void drawShortestPath(QPainter &painter);
  void drawPathfindingStartAndGoal(QPainter &painter);
  void drawTriangles(QPainter &painter, const std::vector<IndexType> &triangles, const QColor &color);
  void drawTriangleCorridor(QPainter &painter);
  void drawTrianglesCompletelySearched(QPainter &painter);
  void drawTrianglesVisited(QPainter &painter);

  void drawVertexLabels(QPainter &painter);
  void drawEdgeLabels(QPainter &painter);
  void drawTriangleLabels(QPainter &painter);
};

#include "navmesh_render_area.inl"

#endif // NAVMESH_RENDER_AREA_HPP_
