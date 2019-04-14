#ifndef CHECKERBOARD_H
#define CHECKERBOARD_H

#include <QQuickPaintedItem>

class Checkerboard : public QQuickPaintedItem {
  Q_OBJECT
  Q_PROPERTY(int step READ step WRITE setStep NOTIFY stepChanged)
  Q_PROPERTY(QString color1 READ color1 WRITE setColor1 NOTIFY color1Changed)
  Q_PROPERTY(QString color2 READ color2 WRITE setColor2 NOTIFY color2Changed)

public:
  Checkerboard(QQuickItem *parent = 0);
  void paint(QPainter *painter);

  int step();
  QString color1();
  QString color2();

  void setStep(int step);
  void setColor1(QString color1);
  void setColor2(QString color2);

signals:
  void stepChanged();
  void color1Changed();
  void color2Changed();

private:
  int m_step;
  QString m_color1;
  QString m_color2;
};

#endif // CHECKERBOARD_H
