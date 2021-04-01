#include "navmeshdisplay.h"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <iostream>

NavmeshDisplay::NavmeshDisplay(QWidget *parent) : QWidget(parent) {
  // Create grid
  // Add Render area at top of grid
  // Add textual display area below grid
  QVBoxLayout *layout = new QVBoxLayout;

  // Create the render area for the navmesh
  navmeshRenderArea_ = new NavmeshRenderArea;

  // Create the zoomable area for the rendering
  navmeshRenderScrollArea_ = new ZoomableScrollArea;
  navmeshRenderScrollArea_->setBackgroundRole(QPalette::ColorRole::Dark);
  navmeshRenderScrollArea_->setWidget(navmeshRenderArea_);

  // Add this scrollable area to the display
  layout->addWidget(navmeshRenderScrollArea_);

  // Create the area to display the text
  auto textDisplayWidget = createTextDisplayArea();
  layout->addWidget(textDisplayWidget);

  setLayout(layout);
}

QWidget* NavmeshDisplay::createTextDisplayArea() {
  // There will be two sections that are horizontally laid out
  QHBoxLayout *layout = new QHBoxLayout;

  // First, create the left section for displaying info about the navmesh
  QGroupBox *navmeshInfoGroupBox = new QGroupBox(tr("Navmesh Info"));
  QVBoxLayout *navmeshInfoLayout = new QVBoxLayout;
  vertexCountLabel_ = new QLabel(vertexCountLabelContents());
  triangleCountLabel_ = new QLabel(triangleCountLabelContents());
  totalEdgeCountLabel_ = new QLabel(totalEdgeCountLabelContents());
  constrainedEdgeCountLabel_ = new QLabel(constrainedEdgeCountLabelContents());
  navmeshInfoLayout->addWidget(vertexCountLabel_);
  navmeshInfoLayout->addWidget(triangleCountLabel_);
  navmeshInfoLayout->addWidget(totalEdgeCountLabel_);
  navmeshInfoLayout->addWidget(constrainedEdgeCountLabel_);
  navmeshInfoLayout->setAlignment(Qt::AlignTop);
  navmeshInfoGroupBox->setLayout(navmeshInfoLayout);
  // Add this groupbox to the overall layout
  layout->addWidget(navmeshInfoGroupBox);

  // Second, create the right section for displaying info about the path found
  QGroupBox *pathInfoGroupBox = new QGroupBox(tr("Path Info"));
  QVBoxLayout *pathInfoLayout = new QVBoxLayout;
  pathStartPositionLabel_ = new QLabel(pathStartPointLabelContents());
  pathGoalPositionLabel_ = new QLabel(pathGoalPointLabelContents());
  pathLengthLabel_ = new QLabel(pathLengthLabelContents());
  pathInfoLayout->addWidget(pathStartPositionLabel_);
  pathInfoLayout->addWidget(pathGoalPositionLabel_);
  pathInfoLayout->addWidget(pathLengthLabel_);
  pathInfoLayout->setAlignment(Qt::AlignTop);
  pathInfoGroupBox->setLayout(pathInfoLayout);
  // Add this groupbox to the overall layout
  layout->addWidget(pathInfoGroupBox);

  QWidget *widget = new QWidget;
  widget->setLayout(layout);

  return widget;
}

NavmeshRenderArea* NavmeshDisplay::getNavmeshRenderArea() {
  return navmeshRenderArea_;
}

void NavmeshDisplay::setNavmesh(const triangleio &triangleData) {
  navmeshRenderArea_->setNavmesh(triangleData);
  vertexCountLabel_->setText(vertexCountLabelContents(triangleData.numberofpoints));
  triangleCountLabel_->setText(triangleCountLabelContents(triangleData.numberoftriangles));
  totalEdgeCountLabel_->setText(totalEdgeCountLabelContents(triangleData.numberofedges));

  // Count the number of contrained edges
  int constrainedEdgeCount{0};
  for (int i=0; i<triangleData.numberofedges; ++i) {
    const int marker = triangleData.edgemarkerlist[i];
    if (marker != 0) {
      // Boundary or constrained edge
      // (1 is boundary)
      ++constrainedEdgeCount;
    }
  }
  constrainedEdgeCountLabel_->setText(constrainedEdgeCountLabelContents(constrainedEdgeCount));
}

void NavmeshDisplay::setPathStartPoint(const Vector &pos) {
  navmeshRenderArea_->setPathStartPoint(pos);
  pathStartPositionLabel_->setText(pathStartPointLabelContents(pos));
}

void NavmeshDisplay::setPathGoalPoint(const Vector &pos) {
  navmeshRenderArea_->setPathGoalPoint(pos);
  pathGoalPositionLabel_->setText(pathGoalPointLabelContents(pos));
}

void NavmeshDisplay::setPath(const PathfindingResult &pathfindingResult) {
  navmeshRenderArea_->setPath(pathfindingResult);
  pathLengthLabel_->setText(pathLengthLabelContents(calculatePathLength(pathfindingResult.shortestPath)));
}

void NavmeshDisplay::resetPathStart() {
  navmeshRenderArea_->resetPathStart();
  pathStartPositionLabel_->setText(pathStartPointLabelContents());
}

void NavmeshDisplay::resetPathGoal() {
  navmeshRenderArea_->resetPathGoal();
  pathGoalPositionLabel_->setText(pathGoalPointLabelContents());
}

void NavmeshDisplay::resetPath() {
  navmeshRenderArea_->resetPath();
  pathLengthLabel_->setText(pathLengthLabelContents());
}

void NavmeshDisplay::setDragModeEnabled(bool enabled) {
  navmeshRenderScrollArea_->setDragModeEnabled(enabled);
}

// ===================================Label contents===================================

QString NavmeshDisplay::pathStartPointLabelContents(const std::optional<Vector> &pos) const {
  if (pos) {
    return QString(tr("Path Start Point: %1,%2").arg(QString::number(pos->x(), 'f', 3), QString::number(pos->y(), 'f', 3)));
  } else {
    return QString(tr("Path Start Point: None"));
  }
}

QString NavmeshDisplay::pathGoalPointLabelContents(const std::optional<Vector> &pos) const {
  if (pos) {
    return QString(tr("Path Goal Point: %1,%2").arg(QString::number(pos->x(), 'f', 3), QString::number(pos->y(), 'f', 3)));
  } else {
    return QString(tr("Path Goal Point: None"));
  }
}

QString NavmeshDisplay::pathLengthLabelContents(const std::optional<double> &length) const {
  if (length) {
    return QString(tr("Path length: %1").arg(QString::number(*length, 'f', 5)));
  } else {
    return QString(tr("Path length: None"));
  }
}

QString NavmeshDisplay::vertexCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of vertices: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of vertices: None"));
  }
}

QString NavmeshDisplay::triangleCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of triangles: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of triangles: None"));
  }
}

QString NavmeshDisplay::totalEdgeCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of edges: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of edges: None"));
  }
}

QString NavmeshDisplay::constrainedEdgeCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of constrained edges: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of constrained edges: None"));
  }
}
