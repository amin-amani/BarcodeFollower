#include "datamodel.h"

DataModel::DataModel(QObject *parent) : QObject(parent)
{

}
Point DataModel::GetPointFromString(QString string)
{
    Point result;
    result.x=-1;
    result.y=-1;


    QStringList sp= string.trimmed().split(' ');
        if(sp.count()!=2)return result;
        result.x=sp[0].toInt();
        result.y=sp[1].toInt();
       return result;

}

