#include "navmeshrenderarea.hpp"

#include <QPointF>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <random>

std::mt19937 createRandomEngine() {
  std::random_device rd;
  std::array<int, std::mt19937::state_size> seed_data;
  std::generate_n(seed_data.data(), seed_data.size(), std::ref(rd));
  std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
  return std::mt19937(seq);
}

NavmeshRenderArea::NavmeshRenderArea(QWidget *parent) : QWidget(parent) {
  setMouseTracking(true);
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
}

QSize NavmeshRenderArea::minimumSizeHint() const {
  return currentSize();
}

void NavmeshRenderArea::setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation) {
  navmeshTriangulation_ = &navmeshTriangulation;
  setSizeBasedOnNavmesh();
  resetZoom();
  update();
}

void NavmeshRenderArea::setPathStartPoint(const pathfinder::Vector &point) {
  startPoint_ = point;
  update();
}

void NavmeshRenderArea::setPathGoalPoint(const pathfinder::Vector &point) {
  goalPoint_ = point;
  update();
}

void NavmeshRenderArea::setAgentRadius(double agentRadius) {
  agentRadius_ = agentRadius;
  update();
}

void NavmeshRenderArea::setPath(const PathfindingResult &pathfindingResult) {
  pathfindingResult_ = &pathfindingResult;
  update();
}

void NavmeshRenderArea::resetPathStart() {
  startPoint_.reset();
}

void NavmeshRenderArea::resetPathGoal() {
  goalPoint_.reset();
}

void NavmeshRenderArea::resetPath() {
  pathfindingResult_ = nullptr;
  update();
}

void NavmeshRenderArea::setHandleMouseDrag(bool enabled) {
  handleMouseDrag_ = enabled;
}

void NavmeshRenderArea::setHandleMouseClick(bool enabled) {
  handleMouseClick_ = enabled;
}

void NavmeshRenderArea::mouseMoveEvent(QMouseEvent *event) {
  bool handled = false;
  const auto mouseLocalPos = event->localPos();
  std::optional<pathfinder::Vector> navmeshPoint;
  if (mouseLocalPos.x() >= 0 && mouseLocalPos.x() < width() &&
      mouseLocalPos.y() >= 0 && mouseLocalPos.y() < height()) {
    // Mouse is within the widget
    const auto tmpPoint = transformWidgetCoordinateToNavmeshCoordinate(pathfinder::Vector{mouseLocalPos.x(), mouseLocalPos.y()});
    if (tmpPoint.x() >= navmeshMinX_ && tmpPoint.x() <= navmeshMinX_+navmeshWidth_ &&
        tmpPoint.y() >= navmeshMinY_ && tmpPoint.y() <= navmeshMinY_+navmeshHeight_) {
      // Mouse is on the navmesh
      navmeshPoint = tmpPoint;
    }
  }

  if (navmeshPoint) {
    // Mouse is on the navmesh
    emit movingMouseOnNavmesh(*navmeshPoint);
  }

  if (event->buttons() & Qt::LeftButton) {
    // Dragging while left clicking
    if (handleMouseDrag_) {
      // We are supposed to be handling mouse input
      if (navmeshPoint) {
        // Mouse is on the navmesh
        emit draggingMouseOnNavmesh(*navmeshPoint);
        handled = true;
      }
    }
  }

  if (handled) {
    event->accept();
  } else {
    QWidget::mouseMoveEvent(event);
  }
}

void NavmeshRenderArea::mousePressEvent(QMouseEvent *event) {
  bool handled = false;
  if (handleMouseClick_ && (event->buttons() & Qt::LeftButton)) {
    const auto mouseLocalPos = event->localPos();
    std::optional<pathfinder::Vector> navmeshPoint;
    if (mouseLocalPos.x() >= 0 && mouseLocalPos.x() < width() &&
        mouseLocalPos.y() >= 0 && mouseLocalPos.y() < height()) {
      // Mouse is within the widget
      const auto tmpPoint = transformWidgetCoordinateToNavmeshCoordinate(pathfinder::Vector{mouseLocalPos.x(), mouseLocalPos.y()});
      if (tmpPoint.x() >= navmeshMinX_ && tmpPoint.x() <= navmeshMinX_+navmeshWidth_ &&
          tmpPoint.y() >= navmeshMinY_ && tmpPoint.y() <= navmeshMinY_+navmeshHeight_) {
        // Mouse is on the navmesh
        navmeshPoint = tmpPoint;
      }
    }

    if (navmeshPoint) {
      // Mouse is on the navmesh
      emit mouseClickedOnNavmesh(*navmeshPoint);
      handled = true;
    }
  }

  if (handled) {
    event->accept();
  } else {
    QWidget::mousePressEvent(event);
  }
}

QSize NavmeshRenderArea::sizeHint() const {
  return currentSize();
}

void NavmeshRenderArea::setSizeBasedOnNavmesh() {
  if (navmeshTriangulation_ != nullptr) {
    // Update size of render area to reflect newly parsed navmesh
    navmeshMinX_ = std::numeric_limits<double>::max();
    navmeshMinY_ = std::numeric_limits<double>::max();
    double navmeshMaxX = std::numeric_limits<double>::lowest();
    double navmeshMaxY = std::numeric_limits<double>::lowest();
    for (int vertexIndex=0; vertexIndex<navmeshTriangulation_->getVertexCount(); ++vertexIndex) {
      const auto &vertex = navmeshTriangulation_->getVertex(vertexIndex);
      if (vertex.x() < navmeshMinX_) {
        navmeshMinX_ = vertex.x();
      }
      if (vertex.x() > navmeshMaxX) {
        navmeshMaxX = vertex.x();
      }
      if (vertex.y() < navmeshMinY_) {
        navmeshMinY_ = vertex.y();
      }
      if (vertex.y() > navmeshMaxY) {
        navmeshMaxY = vertex.y();
      }
    }
    navmeshWidth_ = navmeshMaxX - navmeshMinX_;
    navmeshHeight_ = navmeshMaxY - navmeshMinY_;
    // Set the margin based on the size of the navmesh.
    // TODO: Test with tiny navmesh (~1x1)
    navmeshRenderAreaMargin_ = 0.01 * std::max(navmeshWidth_, navmeshHeight_);
    widgetBaseWidth_ = navmeshWidth_ + 2*navmeshRenderAreaMargin_;
    widgetBaseHeight_ = navmeshHeight_ + 2*navmeshRenderAreaMargin_;
    setMinimumSize(widgetBaseWidth_, widgetBaseHeight_);
    resize(widgetBaseWidth_, widgetBaseHeight_);
    updateGeometry();
  }
}

QSize NavmeshRenderArea::currentSize() const {
  double scale = getScale();
  return QSize(widgetBaseWidth_*scale, widgetBaseHeight_*scale);
}

void NavmeshRenderArea::drawVertices(QPainter &painter) {
  const double kPointRadius = 1.5 / getScale();
  for (int vertexIndex=0; vertexIndex<navmeshTriangulation_->getVertexCount(); ++vertexIndex) {
    const auto &vertex = navmeshTriangulation_->getVertex(vertexIndex);
    const auto transformedVertex = transformNavmeshCoordinateToWidgetCoordinate(vertex);
    painter.drawEllipse(QPointF{transformedVertex.x(), transformedVertex.y()}, kPointRadius, kPointRadius);
  }
}

QColor NavmeshRenderArea::getColorForEdgeMarker(const int marker) {
  if (marker == 0) {
    // Non-constraint edge
    return QColor{150,255,150};
  }
  if (marker == 1) {
    // Triangulation-inserted boundary
    return QColor{100,100,100};
  }
  // User defined constraint
  return QColor{255,0,0};
}

void NavmeshRenderArea::drawEdges(QPainter &painter) {
  painter.save();
  QPen pen;
  pen.setWidth(0);
  for (int edgeIndex=0; edgeIndex<navmeshTriangulation_->getEdgeCount(); ++edgeIndex) {
    const int marker = navmeshTriangulation_->getEdgeMarker(edgeIndex);
    if (!displayNonConstraintEdges_ && marker <= 1) {
      // Do not display non-input edges
      continue;
    }
    const auto &[vertexA, vertexB] = navmeshTriangulation_->getEdge(edgeIndex);
    const pathfinder::Vector transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(vertexA);
    const pathfinder::Vector transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(vertexB);
    pen.setColor(getColorForEdgeMarker(marker));
    painter.setPen(pen);
    painter.drawLine(QPointF(transformedVertexA.x(), transformedVertexA.y()), QPointF(transformedVertexB.x(), transformedVertexB.y()));
  }
  painter.restore();
}

void NavmeshRenderArea::drawShortestPath(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    // Make sure we have a path to draw
    const double kPathThickness = 1.5 / getScale();
    const QColor kPathColor(0, 150, 0);
    painter.save();
    QPen pen(kPathColor);
    pen.setWidthF(kPathThickness);
    painter.setPen(pen);
    for (int i=0; i<pathfindingResult_->shortestPath.size(); ++i) {
      const pathfinder::PathSegment *segment = pathfindingResult_->shortestPath.at(i).get();
      const pathfinder::StraightPathSegment *straightSegment = dynamic_cast<const pathfinder::StraightPathSegment*>(segment);
      const pathfinder::ArcPathSegment *arcSegment = dynamic_cast<const pathfinder::ArcPathSegment*>(segment);
      if (straightSegment != nullptr) {
        if (!pathfinder::math::equal(straightSegment->startPoint.x(), straightSegment->endPoint.x()) || !pathfinder::math::equal(straightSegment->startPoint.y(), straightSegment->endPoint.y())) {
          // Don't want to draw a straight line that has length 0. Results in weird rendering
          const auto &point1 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->startPoint);
          const auto &point2 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->endPoint);
          painter.drawLine(QPointF{point1.x(), point1.y()}, QPointF{point2.x(), point2.y()});
        } else {
          throw std::runtime_error("One of the line segments is 0-length. This indicates that something has gone wrong");
        }
      } else if (arcSegment != nullptr) {
        const auto &centerOfCircle = arcSegment->circleCenter;
        const auto transformedCenter = transformNavmeshCoordinateToWidgetCoordinate(centerOfCircle);
        QRectF arcRectangle(transformedCenter.x() - arcSegment->circleRadius, transformedCenter.y() - arcSegment->circleRadius, arcSegment->circleRadius*2, arcSegment->circleRadius*2);
        int startAngle = 360*16 * arcSegment->startAngle / pathfinder::math::k2Pi;
        int spanAngle = 360*16 * pathfinder::math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, arcSegment->angleDirection) / pathfinder::math::k2Pi;
        painter.drawArc(arcRectangle, startAngle, spanAngle);
      }
    }
    painter.restore();
  }
}

void NavmeshRenderArea::drawPathfindingStartAndGoal(QPainter &painter) {
  const double kPointRadius = 4 / getScale();
  painter.save();

  if (startPoint_) {
    painter.setBrush(QBrush(Qt::GlobalColor::green));
    painter.setPen(Qt::GlobalColor::green);
    const auto transformedStartPoint = transformNavmeshCoordinateToWidgetCoordinate(*startPoint_);
    painter.drawEllipse(QPointF{transformedStartPoint.x(), transformedStartPoint.y()}, kPointRadius, kPointRadius);
    if (agentRadius_ > 0.0) {
      QPen pen(Qt::black);
      pen.setWidth(0);
      painter.setBrush(Qt::NoBrush);
      painter.setPen(pen);
      painter.drawEllipse(QPointF{transformedStartPoint.x(), transformedStartPoint.y()}, agentRadius_, agentRadius_);
    }
  }

  if (goalPoint_) {
    painter.setBrush(QBrush(Qt::GlobalColor::red));
    painter.setPen(Qt::GlobalColor::red);
    const auto transformedGoalPoint = transformNavmeshCoordinateToWidgetCoordinate(*goalPoint_);
    painter.drawEllipse(QPointF{transformedGoalPoint.x(), transformedGoalPoint.y()}, kPointRadius, kPointRadius);
    if (agentRadius_ > 0.0) {
      QPen pen(Qt::black);
      pen.setWidth(0);
      painter.setBrush(Qt::NoBrush);
      painter.setPen(pen);
      painter.drawEllipse(QPointF{transformedGoalPoint.x(), transformedGoalPoint.y()}, agentRadius_, agentRadius_);
    }
  }

  painter.restore();
}

void NavmeshRenderArea::drawTriangleCorridor(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    const QColor kTriangleColor(0, 255, 0, 33);
    painter.save();
    drawTriangles(painter, pathfindingResult_->aStarInfo.triangleCorridor, kTriangleColor);
    painter.restore();
  }
}
void NavmeshRenderArea::drawTrianglesCompletelySearched(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    const QColor kTriangleColor(255, 255, 0, 33);
    painter.save();
    std::vector<IndexType> triangles;
    std::copy_if(pathfindingResult_->aStarInfo.trianglesSearched.begin(), pathfindingResult_->aStarInfo.trianglesSearched.end(), std::back_inserter(triangles), [this](const auto triangleIndex){
      // Return true if this isnt in the triangle corridor (and the corridor isnt being displayed)
      if (displayTriangleCorridor_) {
        if (std::find(pathfindingResult_->aStarInfo.triangleCorridor.begin(), pathfindingResult_->aStarInfo.triangleCorridor.end(), triangleIndex) != pathfindingResult_->aStarInfo.triangleCorridor.end()) {
          // This triangle is a part of the corridor
          return false;
        }
      }
      return true;
    });
    drawTriangles(painter, triangles, kTriangleColor);
    painter.restore();
  }
}
void NavmeshRenderArea::drawTrianglesVisited(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    const QColor kTriangleColor(255, 127, 0, 33);
    painter.save();
    std::vector<IndexType> triangles;
    std::copy_if(pathfindingResult_->aStarInfo.trianglesDiscovered.begin(), pathfindingResult_->aStarInfo.trianglesDiscovered.end(), std::back_inserter(triangles), [this](const auto triangleNum){
      // Return true if this isnt in the triangle corridor (and the corridor isnt being displayed)
      if (displayTriangleCorridor_) {
        if (std::find(pathfindingResult_->aStarInfo.triangleCorridor.begin(), pathfindingResult_->aStarInfo.triangleCorridor.end(), triangleNum) != pathfindingResult_->aStarInfo.triangleCorridor.end()) {
          // This triangle is a part of the corridor
          return false;
        }
      }
      if (displayTrianglesCompletelySearched_) {
        if (std::find(pathfindingResult_->aStarInfo.trianglesSearched.begin(), pathfindingResult_->aStarInfo.trianglesSearched.end(), triangleNum) != pathfindingResult_->aStarInfo.trianglesSearched.end()) {
          // This triangle is a part of the completely searched triangles
          return false;
        }
      }
      return true;
    });
    drawTriangles(painter, triangles, kTriangleColor);
    painter.restore();
  }
}

void NavmeshRenderArea::drawTriangles(QPainter &painter, const std::vector<IndexType> &triangles, const QColor &color) {
  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(color));
  for (const auto triangleIndex : triangles) {
    const auto &[vertexA, vertexB, vertexC] = navmeshTriangulation_->getTriangleVertices(triangleIndex);
    const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(vertexA);
    const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(vertexB);
    const auto transformedVertexC = transformNavmeshCoordinateToWidgetCoordinate(vertexC);
    QPolygonF triangle;
    triangle << QPointF{transformedVertexA.x(),transformedVertexA.y()} << QPointF{transformedVertexB.x(),transformedVertexB.y()} << QPointF{transformedVertexC.x(),transformedVertexC.y()};
    painter.drawPolygon(triangle);
  }
}

void NavmeshRenderArea::drawVertexLabels(QPainter &painter) {
  painter.save();

  QFont f;
  f.setPointSizeF(std::clamp(10/getScale(), 1.0, 10.0));
  painter.setFont(f);
  painter.setPen(QPen(QColor(0,100,255)));

  for (int vertexIndex=0; vertexIndex<navmeshTriangulation_->getVertexCount(); ++vertexIndex) {
    const auto &vertex = navmeshTriangulation_->getVertex(vertexIndex);
    const auto transformedVertex = transformNavmeshCoordinateToWidgetCoordinate(vertex);
    painter.drawText(QPointF{transformedVertex.x(), transformedVertex.y()}, QString::number(vertexIndex));
  }
  painter.restore();
}

void NavmeshRenderArea::drawEdgeLabels(QPainter &painter) {
  painter.save();

  QFont f;
  f.setPointSizeF(std::clamp(10/getScale(), 1.0, 10.0));
  painter.setFont(f);
  painter.setPen(Qt::GlobalColor::red);

  for (int edgeIndex=0; edgeIndex<navmeshTriangulation_->getEdgeCount(); ++edgeIndex) {
    const auto &[vertexA, vertexB] = navmeshTriangulation_->getEdge(edgeIndex);
    const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(vertexA);
    const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(vertexB);
    QPointF centerOfEdge{(transformedVertexA.x()+transformedVertexB.x())/2, (transformedVertexA.y()+transformedVertexB.y())/2};
    painter.drawText(centerOfEdge, QString::number(edgeIndex));
  }

  painter.restore();
}

void NavmeshRenderArea::drawTriangleLabels(QPainter &painter) {
  painter.save();

  QFont f;
  f.setPointSizeF(std::clamp(10/getScale(), 1.0, 10.0));
  painter.setFont(f);

  for (int triangleIndex=0; triangleIndex<navmeshTriangulation_->getTriangleCount(); ++triangleIndex) {
    const auto &[vertexA, vertexB, vertexC] = navmeshTriangulation_->getTriangleVertices(triangleIndex);
    const pathfinder::Vector triangleCenter = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{(vertexA.x()+vertexB.x()+vertexC.x())/3, (vertexA.y()+vertexB.y()+vertexC.y())/3});
    painter.drawText(QPointF{triangleCenter.x(), triangleCenter.y()}, QString::number(triangleIndex));
  }

  painter.restore();
}

void NavmeshRenderArea::resetZoom() {
  zoomLevel_ = 0;
  resizeForNewZoom();
}

void NavmeshRenderArea::zoomIn(double zoomDiff) {
  zoomLevel_ += zoomDiff;
  resizeForNewZoom();
}

void NavmeshRenderArea::zoomOut(double zoomDiff) {
  zoomLevel_ -= zoomDiff;
  resizeForNewZoom();
}

void NavmeshRenderArea::resizeForNewZoom() {
  const auto size = currentSize();
  setMinimumSize(size);
  resize(size);
  updateGeometry();
  update();
}

double NavmeshRenderArea::getScale() const {
  return qPow(2.0, zoomLevel_);
}

pathfinder::Vector NavmeshRenderArea::transformWidgetCoordinateToNavmeshCoordinate(const pathfinder::Vector &v) const {
  // Topleft is origin in Qt
  // Bottomleft is origin in navmesh
  // Flip (for y axis only), scale based on zoom, translate for margins, and finally translate for navmesh offset
  double scale = getScale();
  auto x = v.x() / scale - navmeshRenderAreaMargin_ + navmeshMinX_;
  auto y = (height() - v.y()) / scale - navmeshRenderAreaMargin_ + navmeshMinY_;
  return {x,y};
}

pathfinder::Vector NavmeshRenderArea::transformNavmeshCoordinateToWidgetCoordinate(const pathfinder::Vector &v) const {
  // Scale and margin are already handled by translations to the painter
  // Only need to account for different coordinate frame (flip y axis) and a translation for potential negative points in the navmesh
  return pathfinder::Vector{v.x() - navmeshMinX_, navmeshHeight_-(v.y() - navmeshMinY_)};
}

void NavmeshRenderArea::paintEvent(QPaintEvent * /* event */) {
  QPainter painter(this);

  // Scale the painter based on the zoom level of the canvas
  double scale = getScale();
  painter.scale(scale, scale);

  //  QPen pen;
  QBrush brush(Qt::GlobalColor::black);
  //  painter.setPen(pen);
  painter.setBrush(brush);
  painter.setRenderHint(QPainter::Antialiasing, true);

  // Shift the painter a bit for the margin
  painter.translate(navmeshRenderAreaMargin_, navmeshRenderAreaMargin_);

  if (navmeshTriangulation_ != nullptr) {
    // First, draw the silkroad navmesh data so that it's on the bottom
    // drawSilkroadNavmesh(painter);

    // Draw the navmesh data
    if (displayVertices_) {
      drawVertices(painter);
    }
    drawEdges(painter);

    if (displayTriangleCorridor_) {
      drawTriangleCorridor(painter);
    }

    if (displayTrianglesCompletelySearched_) {
      drawTrianglesCompletelySearched(painter);
    }

    if (displayTrianglesVisited_) {
      drawTrianglesVisited(painter);
    }

    // Draw pathfinding data
    drawShortestPath(painter);
    drawPathfindingStartAndGoal(painter);

    // Draw labels last so they're on top
    if (displayVertexLabels_) {
      drawVertexLabels(painter);
    }
    if (displayEdgeLabels_) {
      drawEdgeLabels(painter);
    }
    if (displayTriangleLabels_) {
      drawTriangleLabels(painter);
    }
  }
}

void NavmeshRenderArea::setDisplayVertices(bool shouldDisplay) {
  displayVertices_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayNonConstraintEdges(bool shouldDisplay) {
  displayNonConstraintEdges_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayTriangleCorridor(bool shouldDisplay) {
  displayTriangleCorridor_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayTrianglesCompletelySearched(bool shouldDisplay) {
  displayTrianglesCompletelySearched_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayTrianglesVisited(bool shouldDisplay) {
  displayTrianglesVisited_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayTriangleLabels(bool shouldDisplay) {
  displayTriangleLabels_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayEdgeLabels(bool shouldDisplay) {
  displayEdgeLabels_ = shouldDisplay;
  update();
}

void NavmeshRenderArea::setDisplayVertexLabels(bool shouldDisplay) {
  displayVertexLabels_ = shouldDisplay;
  update();
}
