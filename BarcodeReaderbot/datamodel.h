#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <QObject>
#include <QStringList>


typedef struct{
 int x;
 int y;

}Point;

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
