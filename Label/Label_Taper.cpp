#include "Label_Taper.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Prs3d_Arrow.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <StdPrs_ShadedShape.hxx>
#include <Prs3d_Presentation.hxx>
#include <Select3D_SensitiveFace.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

IMPLEMENT_STANDARD_RTTIEXT(Label_Taper,Label_PMI)

Label_Taper::Label_Taper()
    : myLabelWidth(0)
{
}

Label_Taper::Label_Taper(const NCollection_Utf8String &value, const gp_Pnt &touch, const gp_Ax2 &oriention)
    : myLabelWidth(0)
{
    myTaperStr = value;
    myTouchPoint = touch;
    myOrientation3D = oriention;
    myHasOrientation3D = Standard_True;
}

void Label_Taper::SetLocation(const gp_Pnt &pnt)
{
    myHasOrientation3D = Standard_True;
    gp_Pln aPln((gp_Ax3(myOrientation3D)));
    Handle(Geom_Plane) plane = new Geom_Plane(aPln);
    GeomAPI_ProjectPointOnSurf PPOS(pnt,plane);
    gp_Pnt pp = PPOS.NearestPoint();

    myOrientation3D.SetLocation(pp);

    this->SetToUpdate();
    this->UpdatePresentations();
    this->GetContext()->RecomputeSelectionOnly(this);
}

void Label_Taper::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
                           const Handle(Prs3d_Presentation)& thePrs,
                           const Standard_Integer theMode)
{
    switch (theMode)
    {
    case 0:
    {
        // 0. verify the data
        if(myTaperStr.IsEmpty())
            return;

        // 1.set zoomable
        if(!myLabelZoomable) {
            SetTransformPersistence (new Graphic3d_TransformPers (Graphic3d_TMF_ZoomPers, myOrientation3D.Location()));
        }

        // 2.set the color and material
        // material
        Graphic3d_MaterialAspect aMaterialAspect;
        aMaterialAspect.SetMaterialName(Graphic3d_NOM_STONE);

        // the shading aspect
        Handle(Prs3d_ShadingAspect) anAspect = new Prs3d_ShadingAspect();
        anAspect->SetMaterial (aMaterialAspect);
        anAspect->SetColor(myLabelColor);

        // 3.draw the main&sup&sub string
        TopoDS_Shape strShape = ComputeStringWithSupAndSub(myTaperStr, "", "", myLabelWidth);
        StdPrs_ShadedShape::Add(thePrs,strShape,myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());

        // 4.draw the fly out line and arrow
        ComputeLeadLine(thePrs, anAspect);

        break;
    }
    }
}

void Label_Taper::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                    const Standard_Integer             theMode)
{
    switch (theMode)
    {
    case 0:
    {
        Handle(SelectMgr_EntityOwner) anEntityOwner   = new SelectMgr_EntityOwner (this, 10);

        // sensitive planar rectangle for text
        gp_Trsf apply = calculateOrientionTrsf();
        Standard_Real aWidth = myLabelWidth;
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

void Label_Taper::ComputeLeadLine (const Handle(Prs3d_Presentation)& thePrs,
                                   const Handle(Prs3d_ShadingAspect)& anAspect)
{
    gp_Trsf apply = calculateOrientionTrsf();

    const gp_Pnt left = gp_Pnt(0,0,0).Transformed(apply);
    const gp_Pnt right = gp_Pnt(myLabelWidth,0,0).Transformed(apply);// the text's start and end position

    // 1 draw the symbol
    gp_Pnt symLeft = left.Translated(-1.25*myFontHeight*myOrientation3D.XDirection());
    gp_Pnt symBt1 = left.Translated(0.35*myFontHeight*myOrientation3D.YDirection());
    gp_Pnt symBt2 = left.Translated(-0.35*myFontHeight*myOrientation3D.YDirection());
    TopoDS_Shape symbolShape = BRepBuilderAPI_MakePolygon(symBt1, symBt2, symLeft, Standard_True);
    StdPrs_ShadedShape::Add(thePrs,symbolShape,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());

    // 2 draw the horizon segment
    double disl = myTouchPoint.Distance(left);
    double disr = myTouchPoint.Distance(right);

    gp_Pnt segBegin = (disl <= disr) ? right : symLeft;
    gp_Pnt beginPnt = (disl <= disr) ? gp_Pnt(-2.5*myFontHeight,0,0).Transformed(apply) :
                                       right; // where to begin the arrow

    BRepBuilderAPI_MakeEdge aBuilder(segBegin,beginPnt);
    StdPrs_ShadedShape::Add(thePrs,aBuilder.Shape(),myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // 3 draw the arrow
    gp_Dir arrowDir(myTouchPoint.XYZ()-beginPnt.XYZ());
    gp_Pnt arrowMid = myTouchPoint.Translated(4*arrowDir.Reversed());
    gp_Dir arrowBotm = myOrientation3D.YDirection();
    gp_Pnt arrowL = arrowMid.Translated(0.5*arrowBotm);
    gp_Pnt arrowR = arrowMid.Translated(0.5*arrowBotm.Reversed());

    // arrow's lead line
    BRepBuilderAPI_MakeEdge bBuilder(beginPnt,arrowMid);
    StdPrs_ShadedShape::Add(thePrs,bBuilder.Shape(),myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // arrow's triangle
    Handle(Graphic3d_ArrayOfTriangles) aTriangle = new Graphic3d_ArrayOfTriangles(3);
    aTriangle->AddVertex (arrowL);
    aTriangle->AddVertex (myTouchPoint);
    aTriangle->AddVertex (arrowR);
    thePrs->CurrentGroup()->AddPrimitiveArray(aTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
}
