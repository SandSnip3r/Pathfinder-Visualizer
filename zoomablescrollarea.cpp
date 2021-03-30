#include "navmeshrenderarea.hpp"
#include "zoomablescrollarea.hpp"

#include <QScrollBar>

#include <iostream> //TODO: Remove

void ZoomableScrollArea::mousePressEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    const auto posInChild = widget()->mapFromParent(event->pos());
    std::cout << "Clicked at child pos " << posInChild.x() << ',' << posInChild.y() << std::endl;
    std::cout << "Window width: " << width()-verticalScrollBar()->width() << ", child width: " << widget()->width() << ", horizontal scroll max: " << horizontalScrollBar()->maximum() << ", horizonal scroll val: " << horizontalScrollBar()->value() << std::endl;

  }
  QScrollArea::mousePressEvent(event);
}

void ZoomableScrollArea::mouseReleaseEvent(QMouseEvent *event) {
  if (!(event->buttons() & Qt::LeftButton)) {
    // Left button is no longer down
    lastMouseLocalPos_.reset();
  }
  QScrollArea::mouseReleaseEvent(event);
}

void ZoomableScrollArea::mouseMoveEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    // Dragging while left clicking
    if (dragModeEnabled_) {
      // Moving the render area
      const auto mouseLocalPos = event->localPos();
      if (lastMouseLocalPos_) {
        auto xDiff = mouseLocalPos.x() - lastMouseLocalPos_->x();
        auto yDiff = mouseLocalPos.y() - lastMouseLocalPos_->y();
        verticalScrollBar()->setValue(verticalScrollBar()->value() - yDiff);
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - xDiff);
      }
      lastMouseLocalPos_ = mouseLocalPos;
      event->accept();
      return;
    }
  }
  QScrollArea::mouseMoveEvent(event);
}

void ZoomableScrollArea::wheelEvent(QWheelEvent *event) {
  /*
  * TODO:
  *   This technically shouldn't be coupled to NavmeshRenderArea.
  *   Instead, treat the child widget as a generic widget and call resize.
  *   Within the NavmeshRenderArea, just render based on its overall size.
  */
  bool handled = false;
  NavmeshRenderArea *renderArea = dynamic_cast<NavmeshRenderArea*>(widget());
  if (renderArea != nullptr) {
    if (event->modifiers() & Qt::ControlModifier) {
      // User is scrolling with the CTRL button held, zoom in/out while maintaining the position under the cursor

      // Calculate position of cursor on the widget
      auto mousePosWithinWidget = widget()->mapFromParent(event->pos());

      // Calculate size-independent position of cursor on the widget
      double xPercent = mousePosWithinWidget.x() / static_cast<double>(widget()->width());
      double yPercent = mousePosWithinWidget.y() / static_cast<double>(widget()->height());

      // Zoom the widget
      if (event->angleDelta().y() > 0) {
        // TODO: Use angle delta?
        renderArea->zoomIn(0.2);
      } else {
        renderArea->zoomOut(0.2);
      }

      // Scroll to maintain the cursor position
      // Calculate the new position of the cursor on the widget
      auto newMousePosWithinWidget = widget()->mapFromParent(event->pos());
      // Calculate the desired position of the cursor on the widget
      double desiredXPos = xPercent * widget()->width();
      double desiredYPos = yPercent * widget()->height();
      // Calculate the offset between current and desired
      auto xOffset = desiredXPos - newMousePosWithinWidget.x();
      auto yOffset = desiredYPos - newMousePosWithinWidget.y();

      auto size = widget()->size();
      if (horizontalScrollBar()->maximum() > 0) {
        // The scroll bar exists, scroll to maintain the previous cursor position relative to the child widget
        const auto newVal = horizontalScrollBar()->value() + xOffset;
        // TODO: Dont assume that 1 scroll tick == 1 pixel
        horizontalScrollBar()->setValue(newVal);
      }
      if (verticalScrollBar()->maximum() > 0) {
        // The scroll bar exists, scroll to maintain the previous cursor position relative to the child widget
        const auto newVal = verticalScrollBar()->value() + yOffset;
        // TODO: Dont assume that 1 scroll tick == 1 pixel
        verticalScrollBar()->setValue(newVal);
      }

      handled = true;
    }
  }
  if (handled) {
    event->accept();
  } else {
    QScrollArea::wheelEvent(event);
  }
}

QSize ZoomableScrollArea::sizeHint() const {
  return {1200,1200};
}

void ZoomableScrollArea::setDragModeEnabled(bool enabled) {
  dragModeEnabled_ = enabled;
}
