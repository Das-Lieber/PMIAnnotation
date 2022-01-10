#ifndef _Label_Datum_HeaderFile
#define _Label_Datum_HeaderFile

#include "Label_PMI.h"

//! Presentation of the text.
class Label_Datum : public Label_PMI
{
public:

    //! Default constructor
    Label_Datum();

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override;

    //! Setup position.
    void SetPosture (const gp_Pnt& touchPnt, const gp_Ax2& oriention);

    //! Setup text.
    void SetDatumName (const NCollection_Utf8String& name);

    //! Setup touch point
    void SetTouchPoint (const gp_Pnt& touchPnt);

    //! Return the width of the label, include the padding
    Standard_Real StrWidth() const {
        return calculateStringWidth(myDatumName);
    }

    //! Return the width of the string
    Standard_Real LabelWidth() const;

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

    NCollection_Utf8String myDatumName;

    //! the location of myOrientation3D controls the source point
    //! of label, also it's the start point of lead line,
    //! myTouchPoint is the end of lead line,
    //! not the origin of local coordinate of label !!!
    gp_Pnt myTouchPoint;  //点击点

    Standard_Real myLabelWidth;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_Datum,Label_PMI)

};

DEFINE_STANDARD_HANDLE(Label_Datum, Label_PMI)

#endif // _Label_Datum_HeaderFile
