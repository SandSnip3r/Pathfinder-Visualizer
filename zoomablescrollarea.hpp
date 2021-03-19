#ifndef ZOOMABLESCROLLAREA_HPP
#define ZOOMABLESCROLLAREA_HPP

#include <QScrollArea>

class ZoomableScrollArea : public QScrollArea {
public:
  void wheelEvent(QWheelEvent *event) override;
private:
};

#endif // ZOOMABLESCROLLAREA_HPP
