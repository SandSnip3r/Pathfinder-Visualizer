#ifndef ZOOMABLESCROLLAREA_HPP
#define ZOOMABLESCROLLAREA_HPP

#include <QScrollArea>

#include <optional>

class ZoomableScrollArea : public QScrollArea {
public:
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  QSize sizeHint() const override;

  void setDragModeEnabled(bool enabled);
private:
  bool dragModeEnabled_{true}; // TODO: Sync this initialization value with the UI
  std::optional<QPointF> lastMouseLocalPos_;
};

#endif // ZOOMABLESCROLLAREA_HPP
