#ifndef LABEL_PMI_H
#define LABEL_PMI_H

#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include <NCollection_UtfString.hxx>
#include <Font_BRepFont.hxx>

#include <QList>

#include "OCCTool/AIS_DraftShape.hxx"
#include "TolStringInfo.h"

class TopoDS_Shape;

typedef QList<NCollection_Utf8String>  NCollection_Utf8StringList;

struct StringBox
{
    gp_Pnt topLeft;
    gp_Pnt bottomLeft;
    gp_Pnt bottomRight;
    gp_Pnt topRight;

    Standard_Real BoxWidth() const;
    TopoDS_Shape ToShape() const;
};

class Label_PMI : public AIS_DraftShape
{
public:
    Label_PMI();

    //! Return TRUE for supported display mode.
    virtual Standard_Boolean AcceptDisplayMode (const Standard_Integer theMode) const Standard_OVERRIDE { return theMode == 0; }

    //! Setup color of entire text.
    virtual void SetColor (const Quantity_Color& theColor) Standard_OVERRIDE;

    //! Set location of shape, interface for drafting
    virtual void SetLocation(const gp_Pnt& pnt) override = 0;

    //! Setup position.
    void SetOriention (const gp_Ax2& oriention);

    //! Setup zoomable property.
    void SetZoomable (const Standard_Boolean theIsZoomable);

    //! Setup height.
    void SetHeight (const Standard_Real theHeight);

    //! Setup padding between text group
    void SetPadding (const Standard_Real thePadding);

    //! Returns label orientation in the model 3D space.
    const gp_Ax2& Orientation3D() const;

    //! Returns true if the current text placement mode uses text orientation in the model 3D space.
    Standard_Boolean HasOrientation3D() const;

protected:
    //! Calculate label center, width and height
    Standard_Real calculateStringWidth (const NCollection_Utf8String& str) const;

    //! Calculate the transform between default gp_Ax2 and myOrientation3D
    gp_Trsf calculateOrientionTrsf() const;

    //! Calculate the bound box of string
    StringBox calculateStringBox (const NCollection_Utf8String& str) const;

    //! Compute the shape of string list and their box
    TopoDS_Shape ComputeStringList(const NCollection_Utf8StringList& strlist, Standard_Real& width) const;

    TopoDS_Shape ComputeStringWithSupAndSub(const NCollection_Utf8String& main,
                                            const NCollection_Utf8String& sub,
                                            const NCollection_Utf8String& sup,
                                            Standard_Real& width);

protected:
    gp_Ax2 myOrientation3D;

    Standard_Boolean myHasOrientation3D;
    Standard_Boolean myLabelZoomable;

    Standard_Real myFontHeight;
    Standard_Real myFontPadding;
    Quantity_Color myLabelColor;

public:

    //! CASCADE RTTI
    DEFINE_STANDARD_RTTIEXT(Label_PMI,AIS_DraftShape)
};

DEFINE_STANDARD_HANDLE(Label_PMI, AIS_DraftShape)

#endif // LABEL_PMI_H
