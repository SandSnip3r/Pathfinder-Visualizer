#ifndef NAVMESH_DISPLAY_HPP_
#define NAVMESH_DISPLAY_HPP_

#include "navmesh_display_base.hpp"
#include "navmesh_render_area_base.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include <optional>

template<typename NavmeshTriangulationType, typename NavmeshRenderAreaType = NavmeshRenderArea<NavmeshTriangulationType>>
class NavmeshDisplay : public NavmeshDisplayBase {

private:
  using PathfindingResult = typename pathfinder::Pathfinder<NavmeshTriangulationType>::PathfindingResult;

public:
  explicit NavmeshDisplay(QWidget *parent = nullptr);

  // NavmeshDisplayBase functions
  NavmeshRenderAreaBase* getNavmeshRenderArea() override;
  void setPathStartPoint(const pathfinder::Vector &pos) override;
  void setPathGoalPoint(const pathfinder::Vector &pos) override;
  void setMousePosition(const pathfinder::Vector &pos) override;
  void resetPathStart() override;
  void resetPathGoal() override;
  void resetPath() override;

  // Navmesh-type specific functions
  void setNavmeshTriangulation(const NavmeshTriangulationType &navmeshTriangulation);
  void setPath(const PathfindingResult &pathfindingResult);

protected:
  NavmeshRenderAreaBase *navmeshRenderArea_{nullptr};

private:
  QLabel *pathStartPositionLabel_{nullptr};
  QLabel *pathGoalPositionLabel_{nullptr};
  QLabel *pathLengthLabel_{nullptr};
  QLabel *vertexCountLabel_{nullptr};
  QLabel *triangleCountLabel_{nullptr};
  QLabel *totalEdgeCountLabel_{nullptr};
  QLabel *constrainedEdgeCountLabel_{nullptr};
  QLabel *mousePositionLabel_{nullptr};

  QWidget* createTextDisplayArea();
  QString pathStartPointLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;
  QString pathGoalPointLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;
  QString pathLengthLabelContents(const std::optional<double> &length = {}) const;
  QString vertexCountLabelContents(const std::optional<int> &count = {}) const;
  QString triangleCountLabelContents(const std::optional<int> &count = {}) const;
  QString totalEdgeCountLabelContents(const std::optional<int> &count = {}) const;
  QString constrainedEdgeCountLabelContents(const std::optional<int> &count = {}) const;
  QString mousePositionLabelContents(const std::optional<pathfinder::Vector> &pos = {}) const;
};

#include "navmesh_display.inl"

#endif // NAVMESH_DISPLAY_HPP_
