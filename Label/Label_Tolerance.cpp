#include "Label_Tolerance.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Font_BRepFont.hxx>
#include <Font_BRepTextBuilder.hxx>
#include <Prs3d_Arrow.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <StdPrs_ShadedShape.hxx>
#include <Prs3d_Presentation.hxx>
#include <Select3D_SensitiveFace.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Geom_Plane.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Prs3d_LineAspect.hxx>

#include <QDebug>

IMPLEMENT_STANDARD_RTTIEXT(Label_Tolerance,Label_PMI)

Label_Tolerance::Label_Tolerance()
    : myToleranceStr(""),
      myTolValue1(""),
      myTolValue2(""),
      myBaseStrList({}),
      myLabelWidth(0)
{
}

void Label_Tolerance::SetLocation(const gp_Pnt &pnt)
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

void Label_Tolerance::SetData (const NCollection_Utf8String &tolName,
                               const NCollection_Utf8String &tolVal1,
                               const NCollection_Utf8String &tolVal2,
                               const NCollection_Utf8StringList &baseList)
{
    if(tolName.IsEmpty() || tolVal1.IsEmpty())
        return;

    myToleranceStr = tolName;
    myTolValue1 = tolVal1;
    myTolValue2 = tolVal2;
    myBaseStrList = baseList;
}

void Label_Tolerance::SetPosture (const gp_Pnt& touchPnt, const gp_Ax2 &oriention)
{
    SetOriention(oriention);
    myTouchPoint = touchPnt;
}

void Label_Tolerance::SetTouchPoint(const gp_Pnt &touchPnt)
{
    myTouchPoint = touchPnt;
}

void Label_Tolerance::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
                               const Handle(Prs3d_Presentation)& thePrs,
                               const Standard_Integer theMode)
{
    switch (theMode)
    {
    case 0:
    {
        // 0. verify the data
        if(myToleranceStr.IsEmpty() || myTolValue1.IsEmpty())
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

        // 3.draw the tolerance symbol and it's bound box
        NCollection_Utf8StringList strList;
        strList << myToleranceStr << myTolValue1 << myTolValue2 << myBaseStrList;
        TopoDS_Shape strShape = ComputeStringList(strList, myLabelWidth);
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

void Label_Tolerance::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                        const Standard_Integer             theMode)
{
    switch (theMode)
    {
    case 0:
    {
        Handle(SelectMgr_EntityOwner) anEntityOwner   = new SelectMgr_EntityOwner (this, 10);

        // sensitive planar rectangle for text
        gp_Trsf apply = calculateOrientionTrsf();
        gp_Pnt leftBottom = gp_Pnt(-myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
        gp_Pnt leftTop = gp_Pnt(-myFontPadding,myFontHeight,0).Transformed(apply);
        gp_Pnt rightBottom = gp_Pnt(myLabelWidth-myFontPadding,-0.3*myFontHeight,0).Transformed(apply);
        gp_Pnt rightTop = gp_Pnt(myLabelWidth-myFontPadding,myFontHeight,0).Transformed(apply);

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

void Label_Tolerance::appendLeadOfLabel(const Handle(Prs3d_Presentation)& thePrs,
                                        const Handle(Prs3d_ShadingAspect)& anAspect)
{
    gp_Trsf apply = calculateOrientionTrsf();

    gp_Pnt left = gp_Pnt(-myFontPadding,0.35*myFontHeight,0).Transformed(apply);
    gp_Pnt right = gp_Pnt(myLabelWidth-2*myFontPadding,0.35*myFontHeight,0).Transformed(apply);
    double disl = myTouchPoint.Distance(left);
    double disr = myTouchPoint.Distance(right);

    gp_Pnt leadPnt = (disl <= disr) ? left : right;
    gp_Pnt beginPnt = (disl <= disr) ? gp_Pnt(-myFontPadding-2*myFontHeight,0.35*myFontHeight,0).Transformed(apply) :
                                       gp_Pnt(myLabelWidth-2*myFontPadding+2*myFontHeight,0.35*myFontHeight,0).Transformed(apply);

    // horizon segment
    BRepBuilderAPI_MakeEdge aBuilder(leadPnt,beginPnt);
    StdPrs_ShadedShape::Add(thePrs,aBuilder.Shape(),myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // arrow
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
