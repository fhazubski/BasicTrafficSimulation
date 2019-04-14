#include "obstacles.h"
#include <QBrush>
#include <QColor>
#include <QPainter>

Obstacles::Obstacles(QQuickItem *parent) : QQuickPaintedItem(parent) {}

void Obstacles::paint(QPainter *painter) {
  if (m_meterInPixels <= 0) {
    return;
  }

  QBrush brush(QColor("#000000"));
  painter->setBrush(brush);
  /*
  for (size_t i = 0; i < road1.lanesCount; i++) {
    for (size_t j = 0; j < road1.lanes[i]->pointsCount - 1; j++) {
      painter->drawLine(
          (road1.lanes[i]->points[j].x + m_xOffset) * m_meterInPixels,
          (road1.lanes[i]->points[j].y + m_yOffset) * m_meterInPixels,
          (road1.lanes[i]->points[j + 1].x + m_xOffset) * m_meterInPixels,
          (road1.lanes[i]->points[j + 1].y + m_yOffset) * m_meterInPixels);
    }
  }

  QBrush brush1((QColor(m_color1)));
  QBrush brush2((QColor(m_color2)));
  painter->fillRect(contentsBoundingRect(), brush1);

  for (int y = -m_step + (static_cast<int>(height() / 2.0) % m_step);
       y < height(); y += m_step) {
    for (int x = -m_step + (static_cast<int>(width() / 2.0) % m_step);
         x < width(); x += m_step) {
      if (((x + y + m_step * 2) / m_step) % 2 == 0) {
        painter->fillRect(QRectF(x, y, m_step, m_step), brush2);
      }
    }
  }*/
}

qreal Obstacles::meterInPixels() { return m_meterInPixels; }
qreal Obstacles::xOffset() { return m_xOffset; }
qreal Obstacles::yOffset() { return m_yOffset; }

void Obstacles::setMeterInPixels(qreal meterInPixels) {
  if (m_meterInPixels != meterInPixels) {
    m_meterInPixels = meterInPixels;
    emit meterInPixelsChanged();
  }
}

void Obstacles::setXOffset(qreal xOffset) {
  if (m_xOffset != xOffset) {
    m_xOffset = xOffset;
    emit xOffsetChanged();
  }
}

void Obstacles::setYOffset(qreal yOffset) {
  if (m_yOffset != yOffset) {
    m_yOffset = yOffset;
    emit yOffsetChanged();
  }
}
