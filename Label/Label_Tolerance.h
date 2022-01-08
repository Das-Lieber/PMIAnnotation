#ifndef _Label_Tolerance_HeaderFile
#define _Label_Tolerance_HeaderFile

#include "Label_PMI.h"

//! Presentation of the text.
class Label_Tolerance : public Label_PMI
{
public:

    //! Default constructor
    Label_Tolerance();

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override;

    //! Setup text.
    void SetData (const NCollection_Utf8String& tolName,
                  const NCollection_Utf8String& tolVal1,
                  const NCollection_Utf8String& tolVal2 = "",
                  const NCollection_Utf8StringList& baseList = {});

    //! Setup position.
    void SetPosture (const gp_Pnt& touchPnt, const gp_Ax2& oriention);

    //! Setup touch point
    void SetTouchPoint (const gp_Pnt& touchPnt);

protected:

    //! Compute
    virtual void Compute (const Handle(PrsMgr_PresentationManager3d)& thePresentationManager,
                          const Handle(Prs3d_Presentation)& thePresentation,
                          const Standard_Integer theMode) Standard_OVERRIDE;

    //! Compute selection
    virtual void ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                   const Standard_Integer theMode) Standard_OVERRIDE;

    //! Add the arrow between touch point and label
    void appendLeadOfLabel(const Handle(Prs3d_Presentation)& thePrs,
                           const Handle(Prs3d_ShadingAspect)& anAspect);

protected:

    NCollection_Utf8String myToleranceStr;
    NCollection_Utf8String myTolValue1;
    NCollection_Utf8String myTolValue2;
    NCollection_Utf8StringList myBaseStrList;

    //! the location of myOrientation3D controls the source point
    //! of label, also it's the start point of lead line,
    //! myTouchPoint is the end of lead line,
    //! not the origin of local coordinate of label !!!
    gp_Pnt myTouchPoint;
    mutable Standard_Real myLabelWidth;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_Tolerance,Label_PMI)

};

DEFINE_STANDARD_HANDLE(Label_Tolerance, Label_PMI)

#endif // _Label_Tolerance_HeaderFile
