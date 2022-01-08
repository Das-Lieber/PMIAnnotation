#ifndef PMIMODEL_H
#define PMIMODEL_H

#include <QHash>

#include <TopoDS_Shape.hxx>

class PMIModel
{
public:
    PMIModel();
    PMIModel(const TopoDS_Shape& origin);

    TopoDS_Shape GetOriginShape() const {
        return myOriginShape;
    }
    void SetOriginShape(const TopoDS_Shape& shape);
    int FindShape(const TopoDS_Shape& shape) const;

private:
    TopoDS_Shape myOriginShape;

    void mappingShape(const TopoDS_Shape& shape);

    QHash<int, TopoDS_Shape> myShapeMap;
    static int shapeNb;

};

#endif // PMIMODEL_H
