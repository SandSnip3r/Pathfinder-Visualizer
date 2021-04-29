#include "navmeshrenderarea.hpp"

#include <QPointF>
#include <QPolygonF>
#include <QtMath>

#include <chrono>
#include <iostream>

NavmeshRenderArea::NavmeshRenderArea(QWidget *parent) : QWidget(parent) {
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
}

QSize NavmeshRenderArea::minimumSizeHint() const {
  return currentSize();
}

void NavmeshRenderArea::setNavmesh(const triangle::triangleio &triangleData) {
  triangleData_ = &triangleData;
  resetZoom();
  setSizeBasedOnNavmesh();
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

void NavmeshRenderArea::setPath(const pathfinder::PathfindingResult &pathfindingResult) {
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

void NavmeshRenderArea::mouseMoveEvent(QMouseEvent *event) {
  bool handled = false;
  if (event->buttons() & Qt::LeftButton) {
    // Dragging while left clicking
    if (handleMouseDrag_) {
      // We are supposed to be handling mouse input
      const auto mouseLocalPos = event->localPos();
      if (mouseLocalPos.x() >= 0 && mouseLocalPos.x() < width() &&
          mouseLocalPos.y() >= 0 && mouseLocalPos.y() < height()) {
        // Mouse is within the widget
        auto navmeshPoint = transformWidgetCoordinateToNavmeshCoordinate(pathfinder::Vector{mouseLocalPos.x(), mouseLocalPos.y()});
        if (navmeshPoint.x() >= navmeshMinX_ && navmeshPoint.x() <= navmeshMinX_+navmeshWidth_ &&
            navmeshPoint.y() >= navmeshMinY_ && navmeshPoint.y() <= navmeshMinY_+navmeshHeight_) {
          // Mouse is on the navmesh
          emit draggingMouseOnNavmesh(navmeshPoint);
          handled = true;
        }
      }
    }
  }
  if (handled) {
    event->accept();
  } else {
    QWidget::mouseMoveEvent(event);
  }
}

QSize NavmeshRenderArea::sizeHint() const {
  return currentSize();
}

void NavmeshRenderArea::setSizeBasedOnNavmesh() {
  // Update size of render area to reflect newly parsed navmesh
  navmeshMinX_ = std::numeric_limits<double>::max();
  navmeshMinY_ = std::numeric_limits<double>::max();
  double navmeshMaxX = std::numeric_limits<double>::lowest();
  double navmeshMaxY = std::numeric_limits<double>::lowest();
  for (int i=0; i<triangleData_->numberofpoints; ++i) {
    const auto x = triangleData_->pointlist[i*2];
    const auto y = triangleData_->pointlist[i*2+1];
    if (x < navmeshMinX_) {
      navmeshMinX_ = x;
    }
    if (x > navmeshMaxX) {
      navmeshMaxX = x;
    }
    if (y < navmeshMinY_) {
      navmeshMinY_ = y;
    }
    if (y > navmeshMaxY) {
      navmeshMaxY = y;
    }
  }
  navmeshWidth_ = navmeshMaxX - navmeshMinX_;
  navmeshHeight_ = navmeshMaxY - navmeshMinY_;
  widgetBaseWidth_ = navmeshWidth_ + 2*kMargin_;
  widgetBaseHeight_ = navmeshHeight_ + 2*kMargin_;
  setMinimumSize(widgetBaseWidth_, widgetBaseHeight_);
  resize(widgetBaseWidth_, widgetBaseHeight_);
  updateGeometry();
}

QSize NavmeshRenderArea::currentSize() const {
  double scale = getScale();
  return QSize(widgetBaseWidth_*scale, widgetBaseHeight_*scale);
}

void NavmeshRenderArea::drawVertices(QPainter &painter) {
  if (triangleData_ != nullptr) {
    // Make sure we have a navmesh
    const float kPointRadius = 1.5 / getScale();
    for (int i=0; i<triangleData_->numberofpoints; ++i) {
      const pathfinder::Vector vertex{triangleData_->pointlist[i*2], triangleData_->pointlist[i*2+1]};
      const auto transformedVertex = transformNavmeshCoordinateToWidgetCoordinate(vertex);
      painter.drawEllipse(QPointF{transformedVertex.x(), transformedVertex.y()}, kPointRadius, kPointRadius);
    }
  }
}

void NavmeshRenderArea::drawEdges(QPainter &painter) {
  if (triangleData_ != nullptr) {
    // Make sure we have a navmesh
    painter.save();
    QPen pen;
    pen.setWidth(0);
    for (int i=0; i<triangleData_->numberofedges; ++i) {
      const int vertexAIndex = triangleData_->edgelist[i*2];
      const int vertexBIndex = triangleData_->edgelist[i*2+1];
      if (vertexAIndex == -1 || vertexBIndex == -1) {
        throw std::runtime_error("An edge in the triangle references a nonexistent vertex");
      }
      const int marker = triangleData_->edgemarkerlist[i];
      if (!displayNonConstraintEdges_ && marker <= 1) {
        // Do not display non-input edges
        continue;
      }
      const pathfinder::Vector vertexA = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{triangleData_->pointlist[vertexAIndex*2], triangleData_->pointlist[vertexAIndex*2+1]});
      const pathfinder::Vector vertexB = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{triangleData_->pointlist[vertexBIndex*2], triangleData_->pointlist[vertexBIndex*2+1]});
      if (marker == 1) {
        // Boundary
        pen.setColor(QColor{100,100,100});
        painter.setPen(pen);
      } else if (marker == 0) {
        // Non-constraint edge
        pen.setColor(QColor{150,255,150});
        painter.setPen(pen);
      } else {
        // Constraint edge
        pen.setColor(Qt::GlobalColor::red);
        painter.setPen(pen);
      }
      painter.drawLine(QPointF(vertexA.x(), vertexA.y()), QPointF(vertexB.x(), vertexB.y()));
    }
    painter.restore();
  }
}

void NavmeshRenderArea::drawShortestPath(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    // Make sure we have a path to draw
    const float kPathThickness = 1.5 / getScale();
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
        const auto &point1 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->startPoint);
        const auto &point2 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->endPoint);
        painter.drawLine(QPointF{point1.x(), point1.y()}, QPointF{point2.x(), point2.y()});
      } else if (arcSegment != nullptr) {
        const auto &centerOfCircle = arcSegment->circleCenter;
        const auto transformedCenter = transformNavmeshCoordinateToWidgetCoordinate(centerOfCircle);
        // double scaledRectWidth = arcSegment->circleRadius/1920.0 * kAreaWidth_;
        // double scaledRectHeight = arcSegment->circleRadius/1920.0 * kAreaHeight_;
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
  const float kPointRadius = 4 / getScale();
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
    std::vector<int> triangles;
    std::copy_if(pathfindingResult_->aStarInfo.trianglesSearched.begin(), pathfindingResult_->aStarInfo.trianglesSearched.end(), std::back_inserter(triangles), [this](const int triangleNum){
      // Return true if this isnt in the triangle corridor (and the corridor isnt being displayed)
      if (displayTriangleCorridor_) {
        if (std::find(pathfindingResult_->aStarInfo.triangleCorridor.begin(), pathfindingResult_->aStarInfo.triangleCorridor.end(), triangleNum) != pathfindingResult_->aStarInfo.triangleCorridor.end()) {
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
    std::vector<int> triangles;
    std::copy_if(pathfindingResult_->aStarInfo.trianglesDiscovered.begin(), pathfindingResult_->aStarInfo.trianglesDiscovered.end(), std::back_inserter(triangles), [this](const int triangleNum){
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

void NavmeshRenderArea::drawTriangles(QPainter &painter, const std::vector<int> &triangles, const QColor &color) {
  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(color));
  for (int triangleNum : triangles) {
    const int vertexAIndex = triangleData_->trianglelist[triangleNum*3];
    const int vertexBIndex = triangleData_->trianglelist[triangleNum*3+1];
    const int vertexCIndex = triangleData_->trianglelist[triangleNum*3+2];
    if (vertexAIndex == -1 || vertexBIndex == -1 || vertexCIndex == -1) {
      throw std::runtime_error("A triangle references a nonexistent vertex");
    }
    const auto &vertexA = pathfinder::Vector{triangleData_->pointlist[vertexAIndex*2], triangleData_->pointlist[vertexAIndex*2+1]};
    const auto &vertexB = pathfinder::Vector{triangleData_->pointlist[vertexBIndex*2], triangleData_->pointlist[vertexBIndex*2+1]};
    const auto &vertexC = pathfinder::Vector{triangleData_->pointlist[vertexCIndex*2], triangleData_->pointlist[vertexCIndex*2+1]};
    const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(vertexA);
    const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(vertexB);
    const auto transformedVertexC = transformNavmeshCoordinateToWidgetCoordinate(vertexC);
    QPolygonF triangle;
    triangle << QPointF{transformedVertexA.x(),transformedVertexA.y()} << QPointF{transformedVertexB.x(),transformedVertexB.y()} << QPointF{transformedVertexC.x(),transformedVertexC.y()};
    painter.drawPolygon(triangle);
  }
}

void NavmeshRenderArea::drawVertexLabels(QPainter &painter) {
  if (triangleData_ != nullptr) {
    // Make sure we have a navmesh
    painter.save();

    QFont f;
    // TODO: Scale font based on zoom?
    f.setPointSizeF(10);
    painter.setFont(f);
    painter.setPen(QPen(QColor(0,100,255)));

    for (int i=0; i<triangleData_->numberofpoints; ++i) {
      const pathfinder::Vector vertex{triangleData_->pointlist[i*2], triangleData_->pointlist[i*2+1]};
      const auto transformedVertex = transformNavmeshCoordinateToWidgetCoordinate(vertex);
      painter.drawText(QPointF{transformedVertex.x(), transformedVertex.y()}, QString::number(i));
    }
    painter.restore();
  }
}

void NavmeshRenderArea::drawEdgeLabels(QPainter &painter) {
  if (triangleData_ != nullptr) {
    // Make sure we have a navmesh
    painter.save();

    QFont f;
    // TODO: Scale font based on zoom?
    f.setPointSizeF(10);
    painter.setFont(f);
    painter.setPen(Qt::GlobalColor::red);

    for (int i=0; i<triangleData_->numberofedges; ++i) {
      const int vertexAIndex = triangleData_->edgelist[i*2];
      const int vertexBIndex = triangleData_->edgelist[i*2+1];
      if (vertexAIndex == -1 || vertexBIndex == -1) {
        throw std::runtime_error("An edge in the triangle references a nonexistent vertex");
      }
      const pathfinder::Vector vertexA = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{triangleData_->pointlist[vertexAIndex*2], triangleData_->pointlist[vertexAIndex*2+1]});
      const pathfinder::Vector vertexB = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{triangleData_->pointlist[vertexBIndex*2], triangleData_->pointlist[vertexBIndex*2+1]});
      QPointF centerOfEdge{(vertexA.x()+vertexB.x())/2, (vertexA.y()+vertexB.y())/2};
      painter.drawText(centerOfEdge, QString::number(i));
    }

    painter.restore();
  }
}

void NavmeshRenderArea::drawTriangleLabels(QPainter &painter) {
  if (triangleData_ != nullptr) {
    // Make sure we have a navmesh
    painter.save();

    QFont f;
    // TODO: Scale font based on zoom?
    f.setPointSizeF(10);
    painter.setFont(f);

    for (int i=0; i<triangleData_->numberoftriangles; ++i) {
      const int vertexAIndex = triangleData_->trianglelist[i*3];
      const int vertexBIndex = triangleData_->trianglelist[i*3+1];
      const int vertexCIndex = triangleData_->trianglelist[i*3+2];
      if (vertexAIndex == -1 || vertexBIndex == -1 || vertexCIndex == -1) {
        throw std::runtime_error("A triangle references a nonexistent vertex");
      }
      const auto &vertexA = pathfinder::Vector{triangleData_->pointlist[vertexAIndex*2], triangleData_->pointlist[vertexAIndex*2+1]};
      const auto &vertexB = pathfinder::Vector{triangleData_->pointlist[vertexBIndex*2], triangleData_->pointlist[vertexBIndex*2+1]};
      const auto &vertexC = pathfinder::Vector{triangleData_->pointlist[vertexCIndex*2], triangleData_->pointlist[vertexCIndex*2+1]};
      const pathfinder::Vector triangleCenter = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{(vertexA.x()+vertexB.x()+vertexC.x())/3, (vertexA.y()+vertexB.y()+vertexC.y())/3});
      painter.drawText(QPointF{triangleCenter.x(), triangleCenter.y()}, QString::number(i));
    }

    painter.restore();
  }
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
  auto x = v.x() / scale - kMargin_ + navmeshMinX_;
  auto y = (height() - v.y()) / scale - kMargin_ + navmeshMinY_;
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
  painter.translate(kMargin_, kMargin_);

  // Draw the navmesh data
  drawVertices(painter);
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