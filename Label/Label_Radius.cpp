#include "Label_Radius.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
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

IMPLEMENT_STANDARD_RTTIEXT(Label_Radius,Label_PMI)

Label_Radius::Label_Radius()
    : myLabelWidth(0)
{
}

Label_Radius::Label_Radius(const NCollection_Utf8StringList &values,
                           const gp_Circ& circle, const gp_Ax2 &oriention)
    : myLabelWidth(0)
{
    myMainStr = values[0];
    mySUPStr = values[1];
    mySUBStr = values[2];

    myCircle = circle;

    myOrientation3D = oriention;
    myHasOrientation3D = Standard_True;
}

void Label_Radius::SetLocation(const gp_Pnt &pnt)
{
    myHasOrientation3D = Standard_True;
    gp_Pln aPln((gp_Ax3(myOrientation3D)));
    Handle(Geom_Plane) plane = new Geom_Plane(aPln);
    GeomAPI_ProjectPointOnSurf PPOS(pnt,plane);
    gp_Pnt pp = PPOS.NearestPoint();

    myOrientation3D.SetLocation(pp);
    myOrientation3D.SetXDirection(pp.XYZ()-myCircle.Location().XYZ());
    this->SetToUpdate();
    this->UpdatePresentations();
    this->GetContext()->RecomputeSelectionOnly(this);
}

void Label_Radius::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
                            const Handle(Prs3d_Presentation)& thePrs,
                            const Standard_Integer theMode)
{
    switch (theMode)
    {
    case 0:
    {
        // 0. verify the data
        if(myMainStr.IsEmpty())
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
        TopoDS_Shape strShape = ComputeStringWithSupAndSub(myMainStr,mySUBStr,mySUPStr,myLabelWidth);
        StdPrs_ShadedShape::Add(thePrs,strShape,myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());

        // 4.draw the fly out line and arrow
        ComputeLeadLine(thePrs, anAspect);

        break;
    }
    }
}

void Label_Radius::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
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

void Label_Radius::ComputeLeadLine (const Handle(Prs3d_Presentation)& thePrs,
                                    const Handle(Prs3d_ShadingAspect)& anAspect)
{
    const gp_Pnt textFirst = myOrientation3D.Location();
    const gp_Pnt textSecond = gp_Pnt(myLabelWidth, 0, 0).Transformed(calculateOrientionTrsf());
    const gp_Pnt center = myCircle.Location();
    const gp_Dir direc = myOrientation3D.XDirection();
    const gp_Pnt circP = center.Translated(myCircle.Radius() * direc);

    gp_Pnt lead = textSecond;
    double dis1 = center.Distance(textFirst);
    double dis2 = center.Distance(textSecond);
    // text over the circle region
    if(dis2 < myCircle.Radius()) {
        lead = circP;
    }
    // draw the line
    TopoDS_Shape leadLin = BRepBuilderAPI_MakeEdge(center, lead);
    StdPrs_ShadedShape::Add(thePrs,leadLin,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // draw the arrow triangle
    gp_Dir arrowDir = direc.Reversed();
    if(dis1 > myCircle.Radius() + 4) {
        arrowDir = direc;
    }

    gp_Pnt arrowMid = circP.Translated(4*arrowDir);
    gp_Dir arrowBotm = myOrientation3D.YDirection();
    gp_Pnt arrowL = arrowMid.Translated(0.5*arrowBotm);
    gp_Pnt arrowR = arrowMid.Translated(0.5*arrowBotm.Reversed());

    Handle(Graphic3d_ArrayOfTriangles) aTriangle = new Graphic3d_ArrayOfTriangles(3);
    aTriangle->AddVertex (arrowL);
    aTriangle->AddVertex (circP);
    aTriangle->AddVertex (arrowR);
    thePrs->CurrentGroup()->AddPrimitiveArray(aTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
}
