#ifndef _Label_Angle_HeaderFile
#define _Label_Angle_HeaderFile

#include "Label_PMI.h"

//! Presentation of the text.
class Label_Angle : public Label_PMI
{
public:

    //! Default constructor
    Label_Angle();

    //! Construct the label with data
    Label_Angle(const NCollection_Utf8StringList& values,
                const gp_Pnt p1, const gp_Pnt& p2, const gp_Pnt& p3);

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override;

    //! Set the length value by main,sup,sub string
    void SetData(const NCollection_Utf8String& main, const NCollection_Utf8String& sup, const NCollection_Utf8String& sub) {
        myMainStr = main;
        mySUPStr = sup;
        mySUBStr = sub;
    }

    //! Set the points which the label is indicated to
    void SetDiamension(const gp_Pnt p1, const gp_Pnt& p2, const gp_Pnt& p3) {
        myPntFirst = p1; myPntCorner = p2; myPntSecond = p3;
    }

protected:

    //! Compute
    virtual void Compute (const Handle(PrsMgr_PresentationManager3d)& thePresentationManager,
                          const Handle(Prs3d_Presentation)& thePresentation,
                          const Standard_Integer theMode) Standard_OVERRIDE;

    //! Compute selection
    virtual void ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                   const Standard_Integer theMode) Standard_OVERRIDE;

    void ComputeFlyoutPnts();

    void ComputeLeadLine (const Handle(Prs3d_Presentation)& thePrs,
                          const Handle(Prs3d_ShadingAspect)& anAspect);

    Standard_Boolean JudgePointInRegion(const gp_Pnt& pt);
    Standard_Boolean CloserToFirstEdge(const gp_Pnt& pt);

protected:

    NCollection_Utf8String myMainStr;
    NCollection_Utf8String mySUPStr;
    NCollection_Utf8String mySUBStr;

    Standard_Real myLabelWidth;

    gp_Pnt myPntFirst;
    gp_Pnt myPntCorner;
    gp_Pnt myPntSecond;

    gp_Pnt myFirstFlyOut;
    gp_Pnt mySecondFlyOut;
    gp_Dir myFirstDir;
    gp_Dir mySecondDir;
    gp_Dir myNormal;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_Angle,Label_PMI)

};

DEFINE_STANDARD_HANDLE(Label_Angle, Label_PMI)

#endif // _Label_Angle_HeaderFile
