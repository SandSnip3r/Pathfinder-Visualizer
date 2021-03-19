#include "navmeshrenderarea.hpp"
#include "zoomablescrollarea.hpp"

/*
* TODO:
*   This technically shouldn't be coupled to NavmeshRenderArea.
*   Instead, I could have created a "zoomable" QWidget which has zoomIn/zoomOut
*   functions and then inherit NavmeshRenderArea from that
*/

void ZoomableScrollArea::wheelEvent(QWheelEvent *event) {
  NavmeshRenderArea *renderArea = dynamic_cast<NavmeshRenderArea*>(widget());
  if (renderArea != nullptr) {
    if (event->modifiers() & Qt::ControlModifier) {
      if (event->angleDelta().y() > 0) {
        renderArea->zoomIn(0.5);
      } else {
        renderArea->zoomOut(0.5);
      }
      event->accept();
    } else {
      QScrollArea::wheelEvent(event);
    }
  } else {
    QScrollArea::wheelEvent(event);
  }
}
