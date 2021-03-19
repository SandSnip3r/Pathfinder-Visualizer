#ifndef NAVMESHRENDERAREA_HPP
#define NAVMESHRENDERAREA_HPP

#include "Pathfinder/behaviorFactory.h"
#include "Pathfinder/pathfinder.h"
#include "Pathfinder/triangle/triangle_api.h"
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
  QSize sizeHint() const override;
  void openFile(QString fileName);
  void zoomIn(double zoomDiff);
  void zoomOut(double zoomDiff);
protected:
  void paintEvent(QPaintEvent *event) override;
private:
  // View data
  static const int kMargin_{50};
  int renderAreaWidth_{1000}, renderAreaHeight_{static_cast<int>(renderAreaWidth_/1.618033988749895)};
  int widgetWidth_{0}, widgetHeight_{0};
  // int widgetWidth_{1000}, widgetHeight_{static_cast<int>(widgetWidth_/1.618033988749895)};
  double zoomLevel_{0};

  // Navmesh data
  triangleio savedTriangleData_;
  triangleio savedTriangleVoronoiData_;
  BehaviorFactory behaviorFactory_;

  // Pathfinding data
  const Vector startPoint_{480, 330};
  const Vector goalPoint_{730, 770};
  PathfindingResult pathfindingResult_;

  void buildNavmeshFromFile(QString fileName);
  void initializeTriangleData();
  void setSizeBasedOnNavmesh();
  QSize currentSize() const;
  void drawVertices(QPainter &painter);
  void drawEdges(QPainter &painter);
  void drawShortestPath(QPainter &painter);
  void drawPathfindingStartAndGoal(QPainter &painter);
  void resizeForNewZoom();
  double getScaleBasedOnZoomLevel() const;
  Vector transformVectorToRenderArea(const Vector &v) const;
  void rebuildPath();
signals:
};

#endif // NAVMESHRENDERAREA_HPP
