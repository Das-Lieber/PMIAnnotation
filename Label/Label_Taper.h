#ifndef _Label_Taper_HeaderFile
#define _Label_Taper_HeaderFile

#include "Label_PMI.h"

//! Presentation of the text.
class Label_Taper : public Label_PMI
{
public:

    //! Default constructor
    Label_Taper();

    //! Construct the label with data
    Label_Taper(const NCollection_Utf8String& value,
                const gp_Pnt& touch,
                const gp_Ax2& oriention);

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override;

    //! Set the length value by main,sup,sub string
    void SetData(const NCollection_Utf8String& value) {
        myTaperStr = value;
    }

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

    void ComputeLeadLine (const Handle(Prs3d_Presentation)& thePrs,
                          const Handle(Prs3d_ShadingAspect)& anAspect);

protected:

    NCollection_Utf8String myTaperStr;
    gp_Pnt myTouchPoint;
    Standard_Real myLabelWidth;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_Taper,Label_PMI)

};

DEFINE_STANDARD_HANDLE(Label_Taper, Label_PMI)

#endif // _Label_Taper_HeaderFile
