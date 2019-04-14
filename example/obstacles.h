#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "simulation.h"
#include <QQuickPaintedItem>

class Obstacles : public QQuickPaintedItem {
  Q_OBJECT
  Q_PROPERTY(qreal meterInPixels READ meterInPixels WRITE setMeterInPixels
                 NOTIFY meterInPixelsChanged)
  Q_PROPERTY(qreal xOffset READ xOffset WRITE setXOffset NOTIFY xOffsetChanged)
  Q_PROPERTY(qreal yOffset READ yOffset WRITE setYOffset NOTIFY yOffsetChanged)

public:
  Obstacles(QQuickItem *parent = 0);
  void paint(QPainter *painter);

  qreal meterInPixels();
  qreal xOffset();
  qreal yOffset();
  Simulation *simulation();

  void setMeterInPixels(qreal meterInPixels);
  void setXOffset(qreal xOffset);
  void setYOffset(qreal yOffset);

signals:
  void meterInPixelsChanged();
  void xOffsetChanged();
  void yOffsetChanged();

private:
  qreal m_meterInPixels;
  qreal m_xOffset = 0.0;
  qreal m_yOffset = 0.0;
};

#endif // OBSTACLES_H
