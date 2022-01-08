#include "PMIModel.h"

#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>

int PMIModel::shapeNb = 0;

PMIModel::PMIModel()
{
}

PMIModel::PMIModel(const TopoDS_Shape &origin)
{
    SetOriginShape(origin);
}

void PMIModel::SetOriginShape(const TopoDS_Shape &shape)
{
    myOriginShape = shape;
    mappingShape(shape);
}

int PMIModel::FindShape(const TopoDS_Shape &shape) const
{
    QHash<int, TopoDS_Shape>::ConstIterator ite = myShapeMap.constBegin();
    for( ; ite != myShapeMap.constEnd(); ++ite) {
        if(ite.value().IsSame(shape))
            return ite.key();
    }
    return -1;
}

void PMIModel::mappingShape(const TopoDS_Shape &shape)
{
    if(shape.IsNull())
        return;

    myShapeMap.clear();
    shapeNb = 0;

    //face
    TopExp_Explorer aExplorer(shape,TopAbs_FACE);
    for(;aExplorer.More();aExplorer.Next())
    {
        myShapeMap.insert(shapeNb,aExplorer.Current());
        shapeNb++;
    }

    // edge
    for(aExplorer.Init(shape,TopAbs_EDGE);aExplorer.More();aExplorer.Next())
    {
        myShapeMap.insert(shapeNb,aExplorer.Current());
        shapeNb++;
    }
}
