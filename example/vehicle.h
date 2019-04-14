#ifndef VEHICLE_H
#define VEHICLE_H

#include <QObject>

class Simulation;

class Vehicle : public QObject {
  friend class Simulation;

  Q_OBJECT
  Q_PROPERTY(qreal x READ x NOTIFY xChanged)
  Q_PROPERTY(qreal y READ y NOTIFY yChanged)
  Q_PROPERTY(qreal width READ width NOTIFY widthChanged)
  Q_PROPERTY(qreal height READ height NOTIFY heightChanged)
  Q_PROPERTY(qreal rotation READ rotation NOTIFY rotationChanged)

public:
  qreal x();
  qreal y();
  qreal width();
  qreal height();
  qreal rotation();

signals:
  void xChanged();
  void yChanged();
  void widthChanged();
  void heightChanged();
  void rotationChanged();

private:
  void setX(qreal x);
  void setY(qreal y);
  void setWidth(qreal width);
  void setHeight(qreal height);
  void setRotation(qreal rotation);

  qreal m_x = 0.0;
  qreal m_y = 0.0;
  qreal m_width = 10.0;
  qreal m_height = 10.0;
  qreal m_rotation = 0.0;
};

#endif // VEHICLE_H
