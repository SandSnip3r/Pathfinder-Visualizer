template<typename NavmeshTriangulationType>
NavmeshRenderArea<NavmeshTriangulationType>::NavmeshRenderArea(QWidget *parent) : NavmeshRenderAreaBase(parent) {
  setMouseTracking(true);
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
}

template<typename NavmeshTriangulationType>
QSize NavmeshRenderArea<NavmeshTriangulationType>::minimumSizeHint() const {
  return currentSize();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation) {
  navmeshTriangulation_ = &navmeshTriangulation;
  setSizeBasedOnNavmesh();
  resetZoom();
  update();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::setPath(const PathfindingResult &pathfindingResult) {
  pathfindingResult_ = &pathfindingResult;
  preProcessAnimationData();
  update();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::resetPath() {
  pathfindingResult_ = nullptr;
  update();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::stepBackPlaybackAnimation() {
  if (pathfindingResult_ == nullptr) {
    return;
  }
  pausePlaybackAnimation();
  // If index is out of bounds, update it to the last step
  if (animationIndex_ > 0) {
    --animationIndex_;
    update();
  } else {
    animationIndex_ = static_cast<int>(ends_.size())-1;
    update();
  }
  emit animationDataUpdated(animationIndex_, ends_.size());
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::startPlaybackAnimation() {
  if (pathfindingResult_ == nullptr) {
    return;
  }
  if (animationTimer_ != nullptr) {
    // Timer is already running.
    return;
  }
  animationTimer_ = new QTimer(this);
  connect(animationTimer_, &QTimer::timeout, this, &NavmeshRenderArea::advanceAnimationFrame);
  // Set the framerate so that it takes 3s to display the whole algorithm.
  const double frameTime = 3000.0 / ends_.size();
  animationTimer_->setInterval(frameTime);
  animationTimer_->start();
  update();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::pausePlaybackAnimation() {
  // Destroy timer if one exists.
  if (animationTimer_ != nullptr) {
    delete animationTimer_;
    animationTimer_ = nullptr;
  }
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::stopPlaybackAnimation() {
  if (animationTimer_ != nullptr) {
    delete animationTimer_;
    animationTimer_ = nullptr;
  }
  // Reset index.
  animationIndex_ = -1;
  update();
  emit animationDataUpdated(animationIndex_, ends_.size());
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::stepForwardPlaybackAnimation() {
  if (pathfindingResult_ == nullptr) {
    return;
  }
  pausePlaybackAnimation();
  if (animationIndex_+1 < ends_.size()) {
    ++animationIndex_;
    update();
    emit animationDataUpdated(animationIndex_, ends_.size());
  }
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::setFramePlaybackAnimation(const QString &text) {
  if (pathfindingResult_ == nullptr) {
    return;
  }
  bool success;
  int frameNumber = text.toInt(&success);
  if (!success) {
    return;
  }
  pausePlaybackAnimation();
  animationIndex_ = frameNumber;
  update();
}

template<typename NavmeshTriangulationType>
double NavmeshRenderArea<NavmeshTriangulationType>::getNavmeshMinX() const {
  return navmeshMinX_;
}

template<typename NavmeshTriangulationType>
double NavmeshRenderArea<NavmeshTriangulationType>::getNavmeshMinY() const {
  return navmeshMinY_;
}

template<typename NavmeshTriangulationType>
double NavmeshRenderArea<NavmeshTriangulationType>::getNavmeshWidth() const {
  return navmeshWidth_;
}

template<typename NavmeshTriangulationType>
double NavmeshRenderArea<NavmeshTriangulationType>::getNavmeshHeight() const {
  return navmeshHeight_;
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::paintEvent(QPaintEvent * /* event */) {
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

  // Allow a deriving class to draw things under the navmesh triangulation.
  // Save and restore the painter in case the implementer does something weird.
  painter.save();
  paintUnderNavmeshTriangulation(painter);
  painter.restore();

  if (navmeshTriangulation_ != nullptr) {
    // Animating pathfinding.
    drawAnimatedPathfinding(painter);

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
    bool shouldDrawShortestPath{true};
    if constexpr (PathfinderType::hasDebugAnimationData()) {
      // Only draw complete path when we're not animating an in-between step.
      shouldDrawShortestPath = (animationIndex_ < 0 || animationIndex_ >= ends_.size());
    }
    if (shouldDrawShortestPath) {
      drawShortestPath(painter);
    }
    drawAllPairsDistances(painter);
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

template<typename NavmeshTriangulationType>
pathfinder::Vector NavmeshRenderArea<NavmeshTriangulationType>::transformWidgetCoordinateToNavmeshCoordinate(const pathfinder::Vector &v) const {
  // Topleft is origin in Qt
  // Bottomleft is origin in navmesh
  // Flip (for y axis only), scale based on zoom, translate for margins, and finally translate for navmesh offset
  double scale = getScale();
  auto x = v.x() / scale - navmeshRenderAreaMargin_ + navmeshMinX_;
  auto y = (height() - v.y()) / scale - navmeshRenderAreaMargin_ + navmeshMinY_;
  return {x,y};
}

template<typename NavmeshTriangulationType>
pathfinder::Vector NavmeshRenderArea<NavmeshTriangulationType>::transformNavmeshCoordinateToWidgetCoordinate(const pathfinder::Vector &v) const {
  // Scale and margin are already handled by translations to the painter
  // Only need to account for different coordinate frame (flip y axis) and a translation for potential negative points in the navmesh
  return pathfinder::Vector{v.x() - navmeshMinX_, navmeshHeight_-(v.y() - navmeshMinY_)};
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::mouseMoveEvent(QMouseEvent *event) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::mousePressEvent(QMouseEvent *event) {
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
      // emit mouseClickedOnNavmesh(*navmeshPoint);
      handled = true;
    }
  }

  if (handled) {
    event->accept();
  } else {
    QWidget::mousePressEvent(event);
  }
}

template<typename NavmeshTriangulationType>
QSize NavmeshRenderArea<NavmeshTriangulationType>::sizeHint() const {
  return currentSize();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::setSizeBasedOnNavmesh() {
  if (navmeshTriangulation_ == nullptr) {
    throw std::runtime_error("Trying to set size without navmesh triangulation");
  }
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawVertices(QPainter &painter) {
  const double kPointRadius = 1.5 / getScale();
  for (int vertexIndex=0; vertexIndex<navmeshTriangulation_->getVertexCount(); ++vertexIndex) {
    const auto &vertex = navmeshTriangulation_->getVertex(vertexIndex);
    const auto transformedVertex = transformNavmeshCoordinateToWidgetCoordinate(vertex);
    painter.drawEllipse(QPointF{transformedVertex.x(), transformedVertex.y()}, kPointRadius, kPointRadius);
  }
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawEdges(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawAnimatedPathfinding(QPainter &painter) {
  if constexpr (PathfinderType::hasDebugAnimationData()) {
    if (pathfindingResult_ == nullptr) {
      return;
    }
    if (animationIndex_ < 0 || animationIndex_ >= ends_.size()) {
      return;
    }
    
    // Draw the current step of the pathfinding algorithm.
    // This frame will have some number of orange triangles which represent possible intervals to consider next.
    // This frame will have some number of yellow triangles which represent intervals which we have visited.
    // This frame will have an interval which we are currently evaluating.
    //  This interval will have a concrete path up to its root and a green triangle representing the final interval.

    // First, draw all of the pushed triangles.
    const auto [pushedEnd, visitedEnd] = ends_.at(animationIndex_);
    {
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(255,165,0));
      for (int i=0; i<pushedEnd; ++i) {
        const auto &pushedTriangle = pushedTriangles_.at(i);
        const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(pushedTriangle.vertexA);
        const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(pushedTriangle.vertexB);
        const auto transformedVertexC = transformNavmeshCoordinateToWidgetCoordinate(pushedTriangle.vertexC);
        QPolygonF triangle;
        triangle << QPointF{transformedVertexA.x(),transformedVertexA.y()} << QPointF{transformedVertexB.x(),transformedVertexB.y()} << QPointF{transformedVertexC.x(),transformedVertexC.y()};
        painter.drawPolygon(triangle);
      }
    }
    // Second, draw all visited triangles.
    {
      painter.setPen(Qt::NoPen);
      painter.setBrush(Qt::yellow);
      for (int i=0; i<visitedEnd; ++i) {
        const auto &visitedTriangle = visitedTriangles_.at(i);
        const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(visitedTriangle.vertexA);
        const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(visitedTriangle.vertexB);
        const auto transformedVertexC = transformNavmeshCoordinateToWidgetCoordinate(visitedTriangle.vertexC);
        QPolygonF triangle;
        triangle << QPointF{transformedVertexA.x(),transformedVertexA.y()} << QPointF{transformedVertexB.x(),transformedVertexB.y()} << QPointF{transformedVertexC.x(),transformedVertexC.y()};
        painter.drawPolygon(triangle);
      }
      // Third, if there is a current interval, draw the path to its root and then the triangle itself.
      if (visitedEnd > 0) {
        painter.setBrush(Qt::green);
        // The last visited triangle is our current triangle.
        const auto &currentTriangle = visitedTriangles_.at(visitedEnd-1);
        const auto transformedVertexA = transformNavmeshCoordinateToWidgetCoordinate(currentTriangle.vertexA);
        const auto transformedVertexB = transformNavmeshCoordinateToWidgetCoordinate(currentTriangle.vertexB);
        const auto transformedVertexC = transformNavmeshCoordinateToWidgetCoordinate(currentTriangle.vertexC);
        QPolygonF triangle;
        triangle << QPointF{transformedVertexA.x(),transformedVertexA.y()} << QPointF{transformedVertexB.x(),transformedVertexB.y()} << QPointF{transformedVertexC.x(),transformedVertexC.y()};
        painter.drawPolygon(triangle);
        {
          // Draw the path to the root of the current interval.
          const double kPathThickness = 1.5 / getScale();
          const QColor kPathColor(0, 150, 0);
          painter.save();
          QPen pen(kPathColor);
          pen.setWidthF(kPathThickness);
          painter.setPen(pen);
          auto currentInterval = PathfinderType::PathfindingAStarInfo::IntervalType(currentTriangle.vertexA, currentTriangle.vertexB, currentTriangle.vertexC);
          auto it = pathfindingResult_->aStarInfo.previousInterval.find(currentInterval);
          while (it != pathfindingResult_->aStarInfo.previousInterval.end()) {
            // Draw a line from currentInterval to *it.
            if (std::get<0>(currentInterval) != std::get<0>(it->second)) {
              const auto &point1 = transformNavmeshCoordinateToWidgetCoordinate(std::get<0>(currentInterval));
              const auto &point2 = transformNavmeshCoordinateToWidgetCoordinate(std::get<0>(it->second));
              painter.drawLine(QPointF{point1.x(), point1.y()}, QPointF{point2.x(), point2.y()});
            }
            currentInterval = it->second;
            it = pathfindingResult_->aStarInfo.previousInterval.find(currentInterval);
          }
          painter.restore();
        }
      }
    }
  }
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawShortestPath(QPainter &painter) {
  if (pathfindingResult_ == nullptr) {
    // Make sure we have a path to draw
    return;
  }
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
        const auto point1 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->startPoint);
        const auto point2 = transformNavmeshCoordinateToWidgetCoordinate(straightSegment->endPoint);
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawAllPairsDistances(QPainter &painter) {
  // TODO: It might be nice to set the size of the rectangle based on the number of rectangles we have.
  const double rectSize = 5 / getScale();
  painter.save();

  auto getColor = [this](double distance) -> QColor {
    if (distance == std::numeric_limits<double>::max()) {
      if (displayAllPairsNoPathToGoal_) {
        return Qt::GlobalColor::black;
      } else {
        return QColor(0,0,0,0);
      }
    }
    if (distance == std::numeric_limits<double>::lowest()) {
      if (displayAllPairsException_) {
        return QColor(200, 0, 255);
      } else {
        return QColor(0,0,0,0);
      }
    }
    const double distanceRatio = distance/allPairsMaxDistance_;
    // Red goes from 0-100% as distanceRatio goes from 0-50%
    // Green goes from 100-0% as distanceRatio goes from 50-100%
    double redPercent = distanceRatio >= 0.5 ? 1 : distanceRatio * 2;
    double greenPercent = distanceRatio < 0.5 ? 1 : 1 - (distanceRatio - 0.5) * 2;
    return QColor(255 * redPercent, 255 * greenPercent, 0);
  };

  for (const auto &rowMap : allPairsRowToColToDistanceMap_) {
    for (const auto &colDistanceMap : rowMap.second) {
      const double x = rowMap.first;
      const double y = colDistanceMap.first;
      const double distance = colDistanceMap.second;
      // Convert the row,col coordinates
      const auto pos = transformNavmeshCoordinateToWidgetCoordinate({x, y});
      const auto color = getColor(distance);
      painter.setBrush(QBrush(color));
      painter.setPen(Qt::NoPen);
      painter.drawRect(QRectF{pos.x()-rectSize/2, pos.y()-rectSize/2, rectSize, rectSize});
    }
  }
  painter.restore();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawPathfindingStartAndGoal(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawTriangleCorridor(QPainter &painter) {
  if (pathfindingResult_ != nullptr) {
    const QColor kTriangleColor(0, 255, 0, 33);
    painter.save();
    drawTriangles(painter, pathfindingResult_->aStarInfo.triangleCorridor, kTriangleColor);
    painter.restore();
  }
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawTrianglesCompletelySearched(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawTrianglesVisited(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawTriangles(QPainter &painter, const std::vector<IndexType> &triangles, const QColor &color) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawVertexLabels(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawEdgeLabels(QPainter &painter) {
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

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::drawTriangleLabels(QPainter &painter) {
  painter.save();

  QFont f;
  f.setPointSizeF(std::clamp(10/getScale(), 1.0, 10.0));
  painter.setFont(f);
  painter.setPen(Qt::GlobalColor::black);

  for (int triangleIndex=0; triangleIndex<navmeshTriangulation_->getTriangleCount(); ++triangleIndex) {
    const auto &[vertexA, vertexB, vertexC] = navmeshTriangulation_->getTriangleVertices(triangleIndex);
    const pathfinder::Vector triangleCenter = transformNavmeshCoordinateToWidgetCoordinate(pathfinder::Vector{(vertexA.x()+vertexB.x()+vertexC.x())/3, (vertexA.y()+vertexB.y()+vertexC.y())/3});
    painter.drawText(QPointF{triangleCenter.x(), triangleCenter.y()}, QString::number(triangleIndex));
  }

  painter.restore();
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::advanceAnimationFrame() {
  if (pathfindingResult_ == nullptr) {
    std::cout << "Advancing animation frame, but no pathfinding result" << std::endl;
    delete animationTimer_;
    animationTimer_ = nullptr;
    animationIndex_ = -1;
    update();
    emit animationDataUpdated(animationIndex_, ends_.size());
    return;
  }
  if (animationIndex_+1 == ends_.size()) {
    // No frames left; kill timer.
    delete animationTimer_;
    animationTimer_ = nullptr;
    animationIndex_ = -1;
    update();
    emit animationDataUpdated(animationIndex_, ends_.size());
    return;
  }
  // Advance to the next frame.
  ++animationIndex_;
  update();
  emit animationDataUpdated(animationIndex_, ends_.size());
}

template<typename NavmeshTriangulationType>
void NavmeshRenderArea<NavmeshTriangulationType>::preProcessAnimationData() {
  if constexpr (PathfinderType::hasDebugAnimationData()) {
    ends_.clear();
    pushedTriangles_.clear();
    visitedTriangles_.clear();
    for (int i=0; i<pathfindingResult_->aStarInfo.animationData.size(); ++i) {
      const auto &current = pathfindingResult_->aStarInfo.animationData.at(i);
      if (std::holds_alternative<PathfinderType::PathfindingAStarInfo::PushInterval>(current)) {
        const auto &asPush = std::get<PathfinderType::PathfindingAStarInfo::PushInterval>(current);
        pushedTriangles_.emplace_back(std::get<0>(asPush.trianglevertices), std::get<1>(asPush.trianglevertices), std::get<2>(asPush.trianglevertices));
      } else if (std::holds_alternative<PathfinderType::PathfindingAStarInfo::PopInterval>(current)) {
        const auto &asPop = std::get<PathfinderType::PathfindingAStarInfo::PopInterval>(current);
        visitedTriangles_.emplace_back(std::get<0>(asPop.trianglevertices), std::get<1>(asPop.trianglevertices), std::get<2>(asPop.trianglevertices));
      }
      ends_.emplace_back(pushedTriangles_.size(), visitedTriangles_.size());
    }

    // Initialize animation index.
    animationIndex_ = -1;
    animationDataUpdated(animationIndex_, ends_.size());
  }
}