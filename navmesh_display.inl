template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::NavmeshDisplay(QWidget *parent) : NavmeshDisplayBase(parent) {
  // Create grid
  // Add Render area at top of grid
  // Add textual display area below grid
  QVBoxLayout *layout = new QVBoxLayout;

  // Create the render area for the navmesh
  navmeshRenderArea_ = new NavmeshRenderAreaType;

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

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QWidget* NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::createTextDisplayArea() {
  // There will be two sections that are horizontally laid out
  QHBoxLayout *layout = new QHBoxLayout;

  // First, create the left section for displaying info about the navmesh
  QGroupBox *navmeshInfoGroupBox = new QGroupBox(tr("Navmesh Info"));
  QVBoxLayout *navmeshInfoLayout = new QVBoxLayout;
  vertexCountLabel_ = new QLabel(vertexCountLabelContents());
  triangleCountLabel_ = new QLabel(triangleCountLabelContents());
  totalEdgeCountLabel_ = new QLabel(totalEdgeCountLabelContents());
  constrainedEdgeCountLabel_ = new QLabel(constrainedEdgeCountLabelContents());
  mousePositionLabel_ = new QLabel(mousePositionLabelContents());
  navmeshInfoLayout->addWidget(vertexCountLabel_);
  navmeshInfoLayout->addWidget(triangleCountLabel_);
  navmeshInfoLayout->addWidget(totalEdgeCountLabel_);
  navmeshInfoLayout->addWidget(constrainedEdgeCountLabel_);
  navmeshInfoLayout->addWidget(mousePositionLabel_);
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

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
NavmeshRenderAreaBase* NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::getNavmeshRenderArea() {
  return navmeshRenderArea_;
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation) {
  auto &specificNavmeshRenderArea = dynamic_cast<NavmeshRenderAreaType&>(*navmeshRenderArea_);
  specificNavmeshRenderArea.setNavmeshTriangulation(navmeshTriangulation);

  vertexCountLabel_->setText(vertexCountLabelContents(navmeshTriangulation.getVertexCount()));
  triangleCountLabel_->setText(triangleCountLabelContents(navmeshTriangulation.getTriangleCount()));
  totalEdgeCountLabel_->setText(totalEdgeCountLabelContents(navmeshTriangulation.getEdgeCount()));

  // Count the number of contrained edges
  int constrainedEdgeCount{0};
  for (int edgeIndex=0; edgeIndex<navmeshTriangulation.getEdgeCount(); ++edgeIndex) {
    const int marker = navmeshTriangulation.getEdgeMarker(edgeIndex);
    if (marker != 0) {
      // Boundary or constrained edge
      // (1 is boundary)
      ++constrainedEdgeCount;
    }
  }
  constrainedEdgeCountLabel_->setText(constrainedEdgeCountLabelContents(constrainedEdgeCount));
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::setPathStartPoint(const pathfinder::Vector &pos) {
  navmeshRenderArea_->setPathStartPoint(pos);
  pathStartPositionLabel_->setText(pathStartPointLabelContents(pos));
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::setPathGoalPoint(const pathfinder::Vector &pos) {
  navmeshRenderArea_->setPathGoalPoint(pos);
  pathGoalPositionLabel_->setText(pathGoalPointLabelContents(pos));
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::setMousePosition(const pathfinder::Vector &pos) {
  mousePositionLabel_->setText(mousePositionLabelContents(pos));
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::setPath(const PathfindingResult &pathfindingResult) {
  auto &specificNavmeshRenderArea = dynamic_cast<NavmeshRenderAreaType&>(*navmeshRenderArea_);
  specificNavmeshRenderArea.setPath(pathfindingResult);
  pathLengthLabel_->setText(pathLengthLabelContents(calculatePathLength(pathfindingResult.shortestPath)));
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::resetPathStart() {
  navmeshRenderArea_->resetPathStart();
  pathStartPositionLabel_->setText(pathStartPointLabelContents());
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::resetPathGoal() {
  navmeshRenderArea_->resetPathGoal();
  pathGoalPositionLabel_->setText(pathGoalPointLabelContents());
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
void NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::resetPath() {
  navmeshRenderArea_->resetPath();
  pathLengthLabel_->setText(pathLengthLabelContents());
}

// ===================================Label contents===================================

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::pathStartPointLabelContents(const std::optional<pathfinder::Vector> &pos) const {
  if (pos) {
    return QString(tr("Path Start Point: %1,%2").arg(QString::number(pos->x(), 'f', 3), QString::number(pos->y(), 'f', 3)));
  } else {
    return QString(tr("Path Start Point: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::pathGoalPointLabelContents(const std::optional<pathfinder::Vector> &pos) const {
  if (pos) {
    return QString(tr("Path Goal Point: %1,%2").arg(QString::number(pos->x(), 'f', 3), QString::number(pos->y(), 'f', 3)));
  } else {
    return QString(tr("Path Goal Point: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::pathLengthLabelContents(const std::optional<double> &length) const {
  if (length) {
    return QString(tr("Path length: %1").arg(QString::number(*length, 'f', 5)));
  } else {
    return QString(tr("Path length: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::vertexCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of vertices: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of vertices: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::triangleCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of triangles: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of triangles: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::totalEdgeCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of edges: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of edges: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::constrainedEdgeCountLabelContents(const std::optional<int> &count) const {
  if (count) {
    return QString(tr("Number of constrained edges: %1").arg(QString::number(*count)));
  } else {
    return QString(tr("Number of constrained edges: None"));
  }
}

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType>
QString NavmeshDisplay<NavmeshTriangulationType, NavmeshRenderAreaType>::mousePositionLabelContents(const std::optional<pathfinder::Vector> &pos) const {
  if (pos) {
    return QString(tr("Mouse Position: %1,%2").arg(QString::number(pos->x(), 'f', 3), QString::number(pos->y(), 'f', 3)));
  } else {
    return QString(tr("Mouse Position: None"));
  }
}
