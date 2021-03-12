#ifndef NAVMESHRENDERAREA_HPP
#define NAVMESHRENDERAREA_HPP

#include <QString>
#include <QWidget>

class NavmeshRenderArea : public QWidget {
  Q_OBJECT
public:
  explicit NavmeshRenderArea(QWidget *parent = nullptr);
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;
  void openFile(QString fileName);
signals:
};

#endif // NAVMESHRENDERAREA_HPP
