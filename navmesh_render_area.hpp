#ifndef NAVMESH_RENDER_AREA_HPP_
#define NAVMESH_RENDER_AREA_HPP_

#include "navmesh_render_area_base.hpp"

#include <Pathfinder/pathfinder.h>
#include <Pathfinder/vector.h>

#include <QPainter>
#include <QString>
#include <QWheelEvent>
#include <QWidget>

#include <utility>
#include <vector>

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

  // Polyanya animation
  void stepBackPlaybackAnimation() override;
  void startPlaybackAnimation() override;
  void pausePlaybackAnimation() override;
  void stopPlaybackAnimation() override;
  void stepForwardPlaybackAnimation() override;
  void setFramePlaybackAnimation(const QString &text) override;

  double getNavmeshMinX() const override;
  double getNavmeshMinY() const override;
  double getNavmeshWidth() const override;
  double getNavmeshHeight() const override;

protected:
  // Navmesh data
  const NavmeshTriangulationType *navmeshTriangulation_{nullptr};

  void paintEvent(QPaintEvent *event) final;

  // Override this function to draw something under the navigation mesh triangulation.
  // The painter state is saved by the caller; you are not obligated to do so nor end with the painter in any specific state.
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

  // Data for animating the polyanya pathfinding algorithm
  QTimer *animationTimer_{nullptr};
  int animationIndex_;
  std::vector<std::pair<size_t, size_t>> ends_;
  struct Triangle {
    Triangle(const pathfinder::Vector &va, const pathfinder::Vector &vb, const pathfinder::Vector &vc) : vertexA(va), vertexB(vb), vertexC(vc) {}
    pathfinder::Vector vertexA, vertexB, vertexC;
  };
  std::vector<Triangle> pushedTriangles_;
  std::vector<Triangle> visitedTriangles_;

  void setSizeBasedOnNavmesh();

  void drawVertices(QPainter &painter);
  void drawEdges(QPainter &painter);

  void drawAnimatedPathfinding(QPainter &painter);
  void drawShortestPath(QPainter &painter);
  void drawAllPairsDistances(QPainter &painter);
  void drawPathfindingStartAndGoal(QPainter &painter);
  void drawTriangles(QPainter &painter, const std::vector<IndexType> &triangles, const QColor &color);

  void drawVertexLabels(QPainter &painter);
  void drawEdgeLabels(QPainter &painter);
  void drawTriangleLabels(QPainter &painter);

  // Polyanya animation
  void advanceAnimationFrame();
  void preProcessAnimationData();
};

#include "navmesh_render_area.inl"

#endif // NAVMESH_RENDER_AREA_HPP_
