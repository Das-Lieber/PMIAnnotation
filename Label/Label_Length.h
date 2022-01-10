#ifndef _Label_Length_HeaderFile
#define _Label_Length_HeaderFile

#include "Label_PMI.h"

//! Presentation of the text.
class Label_Length : public Label_PMI
{
public:

    //! Default constructor
    Label_Length();

    //! Construct the label with data
    Label_Length(const NCollection_Utf8StringList& values,
                 const gp_Pnt& first, const gp_Pnt& second,
                 const gp_Ax2& oriention);

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override;

    //! Set the length value by main,sup,sub string
    void SetData(const NCollection_Utf8String& main, const NCollection_Utf8String& sup, const NCollection_Utf8String& sub) {
        myMainStr = main;
        mySUPStr = sup;
        mySUBStr = sub;
    }

    //! Set the points which the label is indicated to
    void SetDiamension(const gp_Pnt& p1, const gp_Pnt& p2) {
        myFirstPnt = p1;
        mySecondPnt = p2;
    }

protected:

    //! Compute
    virtual void Compute (const Handle(PrsMgr_PresentationManager3d)& thePresentationManager,
                          const Handle(Prs3d_Presentation)& thePresentation,
                          const Standard_Integer theMode) Standard_OVERRIDE;

    //! Compute selection
    virtual void ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                   const Standard_Integer theMode) Standard_OVERRIDE;

    void JudgeTextOut(bool& left, bool& right);

    void ComputeFlyOut (const Handle(Prs3d_Presentation)& thePrs,
                        const Handle(Prs3d_ShadingAspect)& anAspect);

protected:

    NCollection_Utf8String myMainStr;
    NCollection_Utf8String mySUPStr;
    NCollection_Utf8String mySUBStr;

    Standard_Real myLabelWidth;

    gp_Pnt myFirstPnt;
    gp_Pnt mySecondPnt;

    gp_Pnt myFirstOut;
    gp_Pnt mySecondOut;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_Length,Label_PMI)

};

DEFINE_STANDARD_HANDLE(Label_Length, Label_PMI)

#endif // _Label_Length_HeaderFile
