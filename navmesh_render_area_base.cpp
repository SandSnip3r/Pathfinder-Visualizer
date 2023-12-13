#include "navmesh_render_area_base.hpp"

void NavmeshRenderAreaBase::resetZoom() {
  zoomLevel_ = 0;
  resizeForNewZoom();
}

void NavmeshRenderAreaBase::zoomIn(double zoomDiff) {
  zoomLevel_ += zoomDiff;
  resizeForNewZoom();
}

void NavmeshRenderAreaBase::zoomOut(double zoomDiff) {
  zoomLevel_ -= zoomDiff;
  resizeForNewZoom();
}

bool NavmeshRenderAreaBase::getDisplayVertices() const {
  return displayVertices_;
}

bool NavmeshRenderAreaBase::getDisplayNonConstraintEdges() const {
  return displayNonConstraintEdges_;
}

bool NavmeshRenderAreaBase::getDisplayTriangleCorridor() const {
  return displayTriangleCorridor_;
}

bool NavmeshRenderAreaBase::getDisplayTrianglesCompletelySearched() const {
  return displayTrianglesCompletelySearched_;
}

bool NavmeshRenderAreaBase::getDisplayTrianglesVisited() const {
  return displayTrianglesVisited_;
}

bool NavmeshRenderAreaBase::getDisplayTriangleLabels() const {
  return displayTriangleLabels_;
}

bool NavmeshRenderAreaBase::getDisplayEdgeLabels() const {
  return displayEdgeLabels_;
}

bool NavmeshRenderAreaBase::getDisplayVertexLabels() const {
  return displayVertexLabels_;
}

QSize NavmeshRenderAreaBase::currentSize() const {
  double scale = getScale();
  return QSize(widgetBaseWidth_*scale, widgetBaseHeight_*scale);
}

void NavmeshRenderAreaBase::resizeForNewZoom() {
  const auto size = currentSize();
  setMinimumSize(size);
  resize(size);
  updateGeometry();
  update();
}

double NavmeshRenderAreaBase::getScale() const {
  return qPow(2.0, zoomLevel_);
}

QColor NavmeshRenderAreaBase::getColorForEdgeMarker(const int marker) {
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

void NavmeshRenderAreaBase::setAgentRadius(double agentRadius) {
  agentRadius_ = agentRadius;
  update();
}

void NavmeshRenderAreaBase::setPathStartPoint(const pathfinder::Vector &point) {
  startPoint_ = point;
  update();
}

void NavmeshRenderAreaBase::setPathGoalPoint(const pathfinder::Vector &point) {
  goalPoint_ = point;
  update();
}

void NavmeshRenderAreaBase::resetPathStart() {
  startPoint_.reset();
}

void NavmeshRenderAreaBase::resetPathGoal() {
  goalPoint_.reset();
}

void NavmeshRenderAreaBase::setHandleMouseDrag(bool enabled) {
  handleMouseDrag_ = enabled;
}

void NavmeshRenderAreaBase::setHandleMouseClick(bool enabled) {
  handleMouseClick_ = enabled;
}

void NavmeshRenderAreaBase::setDisplayVertices(bool shouldDisplay) {
  displayVertices_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayNonConstraintEdges(bool shouldDisplay) {
  displayNonConstraintEdges_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayTriangleCorridor(bool shouldDisplay) {
  displayTriangleCorridor_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayTrianglesCompletelySearched(bool shouldDisplay) {
  displayTrianglesCompletelySearched_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayTrianglesVisited(bool shouldDisplay) {
  displayTrianglesVisited_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayTriangleLabels(bool shouldDisplay) {
  displayTriangleLabels_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayEdgeLabels(bool shouldDisplay) {
  displayEdgeLabels_ = shouldDisplay;
  update();
}

void NavmeshRenderAreaBase::setDisplayVertexLabels(bool shouldDisplay) {
  displayVertexLabels_ = shouldDisplay;
  update();
}