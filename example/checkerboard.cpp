#include "checkerboard.h"
#include <QBrush>
#include <QPainter>

Checkerboard::Checkerboard(QQuickItem *parent) : QQuickPaintedItem(parent) {}

void Checkerboard::paint(QPainter *painter) {
  if (m_step <= 0) {
    return;
  }
  QBrush brush1((QColor(m_color1)));
  QBrush brush2((QColor(m_color2)));
  painter->fillRect(contentsBoundingRect(), brush1);

  int startY = -m_step + (static_cast<int>(height() / 2.0) % m_step);
  int startX = -m_step + (static_cast<int>(width() / 2.0) % m_step);

  for (int y = startY; y < height(); y += m_step) {
    for (int x = startX; x < width(); x += m_step) {
      if (((x + y + m_step * 2) / m_step) % 2 == 0) {
        painter->fillRect(QRectF(x, y, m_step, m_step), brush2);
      }
    }
  }
}

int Checkerboard::step() { return m_step; }
QString Checkerboard::color1() { return m_color1; }
QString Checkerboard::color2() { return m_color2; }

void Checkerboard::setStep(int step) {
  if (m_step != step) {
    m_step = step;
    emit stepChanged();
  }
}

void Checkerboard::setColor1(QString color1) {
  if (m_color1 != color1) {
    m_color1 = color1;
    emit color1Changed();
  }
}

void Checkerboard::setColor2(QString color2) {
  if (m_color2 != color2) {
    m_color2 = color2;
    emit color2Changed();
  }
}
