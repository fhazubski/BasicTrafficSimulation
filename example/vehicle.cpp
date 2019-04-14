#include "vehicle.h"

qreal Vehicle::x() { return m_x; }
qreal Vehicle::y() { return m_y; }
qreal Vehicle::width() { return m_width; }
qreal Vehicle::height() { return m_height; }
qreal Vehicle::rotation() { return m_rotation; }

void Vehicle::setX(qreal x) {
  if (m_x != x) {
    m_x = x;
    emit xChanged();
  }
}

void Vehicle::setY(qreal y) {
  if (m_y != y) {
    m_y = y;
    emit yChanged();
  }
}

void Vehicle::setWidth(qreal width) {
  if (m_width != width) {
    m_width = width;
    emit widthChanged();
  }
}

void Vehicle::setHeight(qreal height) {
  if (m_height != height) {
    m_height = height;
    emit heightChanged();
  }
}

void Vehicle::setRotation(qreal rotation) {
  if (m_rotation != rotation) {
    m_rotation = rotation;
    emit rotationChanged();
  }
}
