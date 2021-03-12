#include "navmeshrenderarea.hpp"

NavmeshRenderArea::NavmeshRenderArea(QWidget *parent) : QWidget(parent) {
  setBackgroundRole(QPalette::Base);
  setAutoFillBackground(true);
}

QSize NavmeshRenderArea::minimumSizeHint() const {
  return QSize(500,500);
}

QSize NavmeshRenderArea::sizeHint() const {
  return minimumSizeHint();
}
