#ifndef _AIS_DraftShape_HeaderFile
#define _AIS_DraftShape_HeaderFile

#include <AIS_InteractiveObject.hxx>

class AIS_DraftShape : public AIS_InteractiveObject
{

public:
    AIS_DraftShape() {}
    virtual ~AIS_DraftShape() {}
    virtual void SetLocation(const gp_Pnt& pnt) = 0;
};

DEFINE_STANDARD_HANDLE(AIS_DraftShape, AIS_InteractiveObject)

#endif // _AIS_DraftShape_HeaderFile
