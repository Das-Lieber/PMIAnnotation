#include "Label_Datum.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Prs3d_Arrow.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <StdPrs_ShadedShape.hxx>
#include <Prs3d_Presentation.hxx>
#include <Select3D_SensitiveFace.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <AIS_InteractiveContext.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_Line.hxx>
#include <Prs3d_LineAspect.hxx>

IMPLEMENT_STANDARD_RTTIEXT(Label_Datum,Label_PMI)

Label_Datum::Label_Datum()
    : myDatumName(""),
      myLabelWidth(0)
{
}

void Label_Datum::SetLocation(const gp_Pnt &pnt)
{
    myHasOrientation3D = Standard_True;

    gp_Lin lin(myOrientation3D.Location(),myOrientation3D.YDirection());
    Handle(Geom_Line) line = new Geom_Line(lin);
    GeomAPI_ProjectPointOnCurve PPC(pnt,line);
    gp_Pnt pp = PPC.NearestPoint();

    myOrientation3D.SetLocation(pp);
    this->SetToUpdate();
    this->UpdatePresentations();
    this->GetContext()->RecomputeSelectionOnly(this);
}

void Label_Datum::SetPosture(const gp_Pnt &touchPnt, const gp_Ax2 &oriention)
{
    SetOriention(oriention);
    myTouchPoint = touchPnt;
}

void Label_Datum::SetDatumName (const NCollection_Utf8String &name)
{
    myDatumName = name;
}

void Label_Datum::SetTouchPoint(const gp_Pnt &touchPnt)
{
    myTouchPoint = touchPnt;
}

Standard_Real Label_Datum::LabelWidth() const
{
    return myLabelWidth;
}

void Label_Datum::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
                           const Handle(Prs3d_Presentation)& thePrs,
                           const Standard_Integer theMode)
{
    switch (theMode)
    {
    case 0:
    {
        // 0. verify the data
        if(myDatumName.IsEmpty())
            return;

        // 1.set zoomable
        if(!myLabelZoomable) {
            SetTransformPersistence (new Graphic3d_TransformPers (Graphic3d_TMF_ZoomPers, myTouchPoint));
        }

        // 2.set the color and material
        // material
        Graphic3d_MaterialAspect aMaterialAspect;
        aMaterialAspect.SetMaterialName(Graphic3d_NOM_STONE);

        // the shading aspect
        Handle(Prs3d_ShadingAspect) anAspect = new Prs3d_ShadingAspect();
        anAspect->SetMaterial (aMaterialAspect);
        anAspect->SetColor(myLabelColor);

        // the line aspect
        Handle(Prs3d_LineAspect) linAspect = new Prs3d_LineAspect(myLabelColor, Aspect_TOL_SOLID, 1);

        // 3.draw the datum str and it's bound box
        TopoDS_Shape strShape = ComputeStringList({myDatumName}, myLabelWidth);
        StdPrs_ShadedShape::Add(thePrs,strShape,myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());
        StdPrs_ShadedShape::AddWireframeForFreeElements(thePrs,strShape,myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(linAspect->Aspect());

        // 4.draw the lead wire
        appendLeadOfLabel(thePrs,anAspect);

        break;
    }
    }
}

void Label_Datum::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                    const Standard_Integer             theMode)
{
    switch (theMode)
    {
    case 0:
    {
        Handle(SelectMgr_EntityOwner) anEntityOwner   = new SelectMgr_EntityOwner (this, 10);

        // sensitive planar rectangle for text
        gp_Trsf apply = calculateOrientionTrsf();
        Standard_Real aWidth = calculateStringWidth(myDatumName);
        gp_Pnt leftBottom = gp_Pnt(-myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
        gp_Pnt leftTop = gp_Pnt(-myFontPadding,myFontHeight,0).Transformed(apply);
        gp_Pnt rightBottom = gp_Pnt(aWidth+myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
        gp_Pnt rightTop = gp_Pnt(aWidth+myFontPadding,myFontHeight,0).Transformed(apply);

        TColgp_Array1OfPnt aRectanglePoints (1, 5);
        aRectanglePoints.ChangeValue (1) = leftBottom;
        aRectanglePoints.ChangeValue (2) = leftTop;
        aRectanglePoints.ChangeValue (3) = rightTop;
        aRectanglePoints.ChangeValue (4) = rightBottom;
        aRectanglePoints.ChangeValue (5) = aRectanglePoints.Value (1);

        Handle(Select3D_SensitiveFace) aTextSensitive =
                new Select3D_SensitiveFace (anEntityOwner, aRectanglePoints, Select3D_TOS_INTERIOR);
        theSelection->Add (aTextSensitive);
        break;
    }
    }
}

void Label_Datum::appendLeadOfLabel(const Handle(Prs3d_Presentation)& thePrs,
                                    const Handle(Prs3d_ShadingAspect)& anAspect)
{
    gp_Trsf apply = calculateOrientionTrsf();
    Standard_Real aWidth = calculateStringWidth(myDatumName);
    gp_Pnt leftBottom = gp_Pnt(-myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
    gp_Pnt leftTop = gp_Pnt(-myFontPadding,myFontHeight,0).Transformed(apply);
    gp_Pnt rightBottom = gp_Pnt(aWidth+myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
    gp_Pnt rightTop = gp_Pnt(aWidth+myFontPadding,myFontHeight,0).Transformed(apply);
    gp_Pnt midBottom(0.5*leftBottom.XYZ() + 0.5*rightBottom.XYZ());
    gp_Pnt midTop(0.5*leftTop.XYZ() + 0.5*rightTop.XYZ());
    gp_Lin midLin(midBottom,gp_Dir(gp_Vec(midBottom,midTop)));
    Handle(Geom_Line) midCurve = new Geom_Line(midLin);
    GeomAPI_ProjectPointOnCurve PPC(myTouchPoint,midCurve);
    gp_Pnt midBase = PPC.NearestPoint();

    gp_Vec baseVec = myOrientation3D.XDirection();
    baseVec.Normalize();
    gp_Pnt baseEnd = midBase.Translated(baseVec*3);
    gp_Pnt baseStart = midBase.Translated(-baseVec*3);
    gp_Pnt baseTop;
    if(midBase.Distance(midBottom) < midBase.Distance(midTop)) {
        baseTop = midBase.Translated(gp_Vec(midBase,midBottom).Normalized()*3*1.732);

        BRepBuilderAPI_MakeEdge aBuilder(midBase,midBottom);
        StdPrs_ShadedShape::Add(thePrs,aBuilder.Shape(),myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
    }
    else {
        baseTop = midBase.Translated(gp_Vec(midBase,midBottom).Normalized()*3*1.732);

        BRepBuilderAPI_MakeEdge aBuilder(midBase,midTop);
        StdPrs_ShadedShape::Add(thePrs,aBuilder.Shape(),myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
    }

    double distance = qMin(midBase.Distance(midBottom), midBase.Distance(midTop));
    if(distance > 0.7*myFontHeight) {
        Handle(Graphic3d_ArrayOfTriangles) aTriangle = new Graphic3d_ArrayOfTriangles(3);
        aTriangle->AddVertex (baseStart);
        aTriangle->AddVertex (baseEnd);
        aTriangle->AddVertex (baseTop);
        thePrs->CurrentGroup()->AddPrimitiveArray (aTriangle);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
    }
}
