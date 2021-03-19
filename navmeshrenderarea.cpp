#include "navmeshrenderarea.hpp"

#include <QPointF>
#include <QtMath>

#include <chrono>
#include <iostream>

void resetTriangleio(triangleio *io) {
  // TODO: Move into Triangle directory
	io->pointlist = nullptr;
	io->pointattributelist = nullptr;
	io->pointmarkerlist = nullptr;
	io->numberofpoints = 0;
	io->numberofpointattributes = 0;

	io->trianglelist = nullptr;
	io->triangleattributelist = nullptr;
	io->trianglearealist = nullptr;
	io->neighborlist = nullptr;
	io->numberoftriangles = 0;
	io->numberofcorners = 0;
	io->numberoftriangleattributes = 0;

	io->segmentlist = nullptr;
	io->segmentmarkerlist = nullptr;
	io->numberofsegments = 0;

	io->holelist = nullptr;
	io->numberofholes = 0;
	io->regionlist = nullptr;
	io->numberofregions = 0;

	io->edgelist = nullptr;
	io->edgemarkerlist = nullptr;
  io->normlist = nullptr;
	io->numberofedges = 0;
}

void freeTriangleio(triangleio *io) {
  // TODO: Move into Triangle directory
  free(io->pointlist);
  free(io->pointattributelist);
  free(io->pointmarkerlist);

  free(io->trianglelist);
  free(io->triangleattributelist);
  free(io->trianglearealist);
  free(io->neighborlist);

  free(io->segmentlist);
  free(io->segmentmarkerlist);

  free(io->holelist);
  free(io->regionlist);

  free(io->edgelist);
  free(io->edgemarkerlist);
  free(io->normlist);
}

NavmeshRenderArea::NavmeshRenderArea(QWidget *parent) : QWidget(parent) {
  initializeTriangleData();
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
}

QSize NavmeshRenderArea::minimumSizeHint() const {
  return currentSize();
}

QSize NavmeshRenderArea::sizeHint() const {
  return currentSize();
}

void NavmeshRenderArea::initializeTriangleData() {
	resetTriangleio(&savedTriangleData_);
	resetTriangleio(&savedTriangleVoronoiData_);
}

void NavmeshRenderArea::buildNavmeshFromFile(QString fileName) {
  int firstNode;

  // Create a structure for the input data to be read into
  triangleio inputStruct;
  resetTriangleio(&inputStruct);

  // Call Triangle's file reading function
  int res = triangle_read_poly(fileName.toStdString().c_str(), &inputStruct, &firstNode);
  if (res < 0) {
    std::cout << "Reading file error: " << res << std::endl;
    throw std::runtime_error("Unable to open poly file");
  }

  // Create a context
	context *ctx;
	ctx = triangle_context_create();
  // Set context's behavior
  *(ctx->b) = behaviorFactory_.getBehavior();

  // Build the triangle mesh
  int meshCreateResult = triangle_mesh_create(ctx, &inputStruct);
  if (meshCreateResult < 0) {
    // Free memory
    freeTriangleio(&inputStruct);
    std::cout << "Mesh creating error: " << meshCreateResult << std::endl;
    throw std::runtime_error("Error creating mesh");
  }

  // Now, the context holds the mesh, lets extract this data
  // First, prepare data structures
  // Free in case this has already been used
  freeTriangleio(&savedTriangleData_);
  freeTriangleio(&savedTriangleVoronoiData_);
  initializeTriangleData();

  // Copy data
  int copyResult = triangle_mesh_copy(ctx, &savedTriangleData_, 1, 1, &savedTriangleVoronoiData_);
  if (copyResult < 0) {
    // Free memory
    freeTriangleio(&inputStruct);
    std::cout << "Data copying error: " << copyResult << std::endl;
    throw std::runtime_error("Error copying mesh data");
  }

  // Done with input data
  freeTriangleio(&inputStruct);

  // Done with context
  triangle_context_destroy(ctx);
}

void NavmeshRenderArea::setSizeBasedOnNavmesh() {
  // Update size of render area to reflect newly parsed navmesh
  // TODO: Generalize a bit more so we handle negative input data
  renderAreaWidth_ = 0;
  renderAreaHeight_ = 0;
  for (int i=0; i<savedTriangleData_.numberofpoints; ++i) {
    const auto x = savedTriangleData_.pointlist[i*2];
    const auto y = savedTriangleData_.pointlist[i*2+1];
    if (x > renderAreaWidth_) {
      renderAreaWidth_ = x;
    }
    if (y > renderAreaHeight_) {
      renderAreaHeight_ = y;
    }
  }
  widgetWidth_ = renderAreaWidth_ + 2*kMargin_;
  widgetHeight_ = renderAreaHeight_ + 2*kMargin_;
  setMinimumSize(widgetWidth_, widgetHeight_);
  resize(widgetWidth_, widgetHeight_);
  updateGeometry();
  std::cout << "Updated window based on input data. New width: " << widgetWidth_ << ", new height: " << widgetHeight_ << std::endl;
}

QSize NavmeshRenderArea::currentSize() const {
  double scale = getScaleBasedOnZoomLevel();
  return QSize(widgetWidth_*scale, widgetHeight_*scale);
}

void NavmeshRenderArea::openFile(QString fileName) {
  auto beforeTime = std::chrono::high_resolution_clock::now();
  buildNavmeshFromFile(fileName);
  auto afterTime = std::chrono::high_resolution_clock::now();
  std::cout << "All done parsing the poly file, building the mesh, and copying it. Time spent: " << std::chrono::duration_cast<std::chrono::microseconds>(afterTime-beforeTime).count() << " microseconds" << std::endl;

  setSizeBasedOnNavmesh();

  // Pathfind on new navmesh (using old start and goal)
  rebuildPath();
}

void NavmeshRenderArea::drawVertices(QPainter &painter) {
  for (int i=0; i<savedTriangleData_.numberofpoints; ++i) {
    const Vector vertex{savedTriangleData_.pointlist[i*2], savedTriangleData_.pointlist[i*2+1]};
    const auto transformedVertex = transformVectorToRenderArea(vertex);
    painter.drawEllipse(QPointF{transformedVertex.x(), transformedVertex.y()}, 1.5, 1.5);
  }
}

void NavmeshRenderArea::drawEdges(QPainter &painter) {
  painter.save();
  QPen pen;
  pen.setWidth(0);
  for (int i=0; i<savedTriangleData_.numberofedges; ++i) {
    const int vertexA = savedTriangleData_.edgelist[i*2];
    const int vertexB = savedTriangleData_.edgelist[i*2+1];
    if (vertexA == -1 || vertexB == -1) {
      throw std::runtime_error("An edge in the triangle references a nonexistent vertex");
    }
    const int marker = savedTriangleData_.edgemarkerlist[i];
    const auto vertexAX = savedTriangleData_.pointlist[vertexA*2];
    const auto vertexAY = savedTriangleData_.pointlist[vertexA*2+1];
    const auto vertexBX = savedTriangleData_.pointlist[vertexB*2];
    const auto vertexBY = savedTriangleData_.pointlist[vertexB*2+1];
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
    painter.drawLine(QPointF(vertexAX, vertexAY), QPointF(vertexBX, vertexBY));
  }
  painter.restore();
}

void NavmeshRenderArea::drawShortestPath(QPainter &painter) {
  const QColor kPathColor(0, 150, 0);
  painter.save();
  QPen pen(kPathColor);
  pen.setWidthF(1.5);
  painter.setPen(pen);
  for (int i=0; i<pathfindingResult_.shortestPath.size(); ++i) {
    const PathSegment *segment = pathfindingResult_.shortestPath.at(i).get();
    const StraightPathSegment *straightSegment = dynamic_cast<const StraightPathSegment*>(segment);
    const ArcPathSegment *arcSegment = dynamic_cast<const ArcPathSegment*>(segment);
    if (straightSegment != nullptr) {
      const auto &point1 = transformVectorToRenderArea(straightSegment->startPoint);
      const auto &point2 = transformVectorToRenderArea(straightSegment->endPoint);
      painter.drawLine(QPointF{point1.x(), point1.y()}, QPointF{point2.x(), point2.y()});
    } else if (arcSegment != nullptr) {
      const auto &centerOfCircle = arcSegment->circleCenter;
      const auto transformedCenter = transformVectorToRenderArea(centerOfCircle);
      // double scaledRectWidth = arcSegment->circleRadius/1920.0 * kAreaWidth_;
      // double scaledRectHeight = arcSegment->circleRadius/1920.0 * kAreaHeight_;
      QRectF arcRectangle(transformedCenter.x() - arcSegment->circleRadius, transformedCenter.y() - arcSegment->circleRadius, arcSegment->circleRadius*2, arcSegment->circleRadius*2);
      int startAngle = 360*16 * arcSegment->startAngle / math::k2Pi;
      int spanAngle = 360*16 * math::arcAngle(arcSegment->startAngle, arcSegment->endAngle, arcSegment->angleDirection) / math::k2Pi;
      painter.drawArc(arcRectangle, startAngle, spanAngle);
    }
  }
  painter.restore();
}

void NavmeshRenderArea::drawPathfindingStartAndGoal(QPainter &painter) {
  const float kPointRadius = 4;
  painter.save();

  painter.setBrush(QBrush(Qt::GlobalColor::green));
  painter.setPen(Qt::GlobalColor::green);
  const auto transformedStartPoint = transformVectorToRenderArea(startPoint_);
  painter.drawEllipse(QPointF{transformedStartPoint.x(), transformedStartPoint.y()}, kPointRadius, kPointRadius);

  painter.setBrush(QBrush(Qt::GlobalColor::red));
  painter.setPen(Qt::GlobalColor::red);
  const auto transformedGoalPoint = transformVectorToRenderArea(goalPoint_);
  painter.drawEllipse(QPointF{transformedGoalPoint.x(), transformedGoalPoint.y()}, kPointRadius, kPointRadius);

  painter.restore();
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
  std::cout << "Setting new scale (" << getScaleBasedOnZoomLevel() << ") for zoom level " << zoomLevel_ << std::endl;
  const auto size = currentSize();
  setMinimumSize(size);
  resize(size);
  updateGeometry();
  update();
}

double NavmeshRenderArea::getScaleBasedOnZoomLevel() const {
  return qPow(2.0, zoomLevel_);
}

Vector NavmeshRenderArea::transformVectorToRenderArea(const Vector &v) const {
  // Requires flipping the y axis
  return Vector{v.x(), renderAreaHeight_-v.y()};
}

void NavmeshRenderArea::paintEvent(QPaintEvent * /* event */) {
  QPainter painter(this);

  // Scale the painter based on the zoom level of the canvas
  double scale = getScaleBasedOnZoomLevel();
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

  // Draw pathfinding data
  drawShortestPath(painter);
  drawPathfindingStartAndGoal(painter);
}

void NavmeshRenderArea::rebuildPath() {
  try {
    Pathfinder pathfinder(savedTriangleData_, savedTriangleVoronoiData_);
    pathfinder.setCharacterRadius(13.14159);
    pathfindingResult_ = pathfinder.findShortestPath(startPoint_, goalPoint_);
    // emit pathfindingPathChanged();
  } catch (std::runtime_error &ex) {
    std::cout << "Pathfinder threw an exception! \"" << ex.what() << "\"" << std::endl;
  }
}