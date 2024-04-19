#ifndef NAVMESH_RENDER_AREA_BASE_HPP_
#define NAVMESH_RENDER_AREA_BASE_HPP_

#include <Pathfinder/vector.h>

#include <QColor>
#include <QString>
#include <QTimer>
#include <QWidget>

#include <map>

class NavmeshRenderAreaBase : public QWidget {
  Q_OBJECT
public:
  using QWidget::QWidget;

  void resetZoom();
  void zoomIn(double zoomDiff);
  void zoomOut(double zoomDiff);

  bool getDisplayVertices() const;
  bool getDisplayNonConstraintEdges() const;
  bool getDisplayTriangleLabels() const;
  bool getDisplayEdgeLabels() const;
  bool getDisplayVertexLabels() const;

  virtual double getNavmeshMinX() const = 0;
  virtual double getNavmeshMinY() const = 0;
  virtual double getNavmeshWidth() const = 0;
  virtual double getNavmeshHeight() const = 0;

  void setAgentRadius(double agentRadius);
  void setPathStartPoint(const pathfinder::Vector &point);
  void setPathGoalPoint(const pathfinder::Vector &point);
  void addPairsDistance(double x, double y, double distance);
  void resetAllPairsDistanceMap();
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

  bool displayTriangleLabels_{false};
  bool displayEdgeLabels_{false};
  bool displayVertexLabels_{false};

  bool displayAllPairsNoPathToGoal_{true};
  bool displayAllPairsException_{true};

  // Pathfinding data
  double agentRadius_{0.0};
  std::optional<pathfinder::Vector> startPoint_;
  std::optional<pathfinder::Vector> goalPoint_;

  // Pathfinding all-pairs
  double allPairsMaxDistance_{0.0};
  std::map<double, std::map<double, double>> allPairsRowToColToDistanceMap_;

  QSize currentSize() const;
  void resizeForNewZoom();
  double getScale() const;

  virtual QColor getColorForEdgeMarker(const int marker);

signals:
  void draggingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  void movingMouseOnNavmesh(const pathfinder::Vector &navmeshPoint);
  // void mouseClickedOnNavmesh(const pathfinder::Vector &navmeshPoint);

  // Polyanya animation
  void animationDataUpdated(int currentIndex, int frameCount);

public slots:
  void setDisplayVertices(bool shouldDisplay);
  void setDisplayNonConstraintEdges(bool shouldDisplay);
  void setDisplayTriangleLabels(bool shouldDisplay);
  void setDisplayEdgeLabels(bool shouldDisplay);
  void setDisplayVertexLabels(bool shouldDisplay);
  void setAllPairsShowNoPathToGoal(bool shouldDisplay);
  void setAllPairsShowException(bool shouldDisplay);

  // Polyanya animation
  virtual void stepBackPlaybackAnimation() = 0;
  virtual void startPlaybackAnimation() = 0;
  virtual void pausePlaybackAnimation() = 0;
  virtual void stopPlaybackAnimation() = 0;
  virtual void stepForwardPlaybackAnimation() = 0;
  virtual void setFramePlaybackAnimation(const QString &text) = 0;
};

#endif // NAVMESH_RENDER_AREA_BASE_HPP_
