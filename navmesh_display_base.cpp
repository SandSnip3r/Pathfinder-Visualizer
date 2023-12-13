#include "navmesh_display_base.hpp"

void NavmeshDisplayBase::setDragModeEnabled(bool enabled) {
  if (navmeshRenderScrollArea_ == nullptr) {
    throw std::runtime_error("Navmesh ZoomableScrollArea not yet set");
  }
  navmeshRenderScrollArea_->setDragModeEnabled(enabled);
}