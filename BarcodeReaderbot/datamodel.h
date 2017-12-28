#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>
#include <QStringList>


struct Point{
     int x;
     int y;
     inline Point operator=(Point a) {
         x=a.x;
         y=a.y;
         return a;
     }

};

typedef struct{
 double a;
 double b;
 double c;

}Line;
class DataModel : public QObject
{

    Q_OBJECT
public:
    explicit DataModel(QObject *parent = 0);
    Point GetPointFromString(QString string);

signals:

public slots:
};

#endif // DATAMODEL_H
