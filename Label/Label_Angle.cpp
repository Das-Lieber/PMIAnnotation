 #include "Label_Angle.h"

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
#include <Geom_Plane.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakePlane.hxx>
#include <BRepBuilderAPI_Transform.hxx>

IMPLEMENT_STANDARD_RTTIEXT(Label_Angle,Label_PMI)

Label_Angle::Label_Angle()
    : myLabelWidth(0)
{
}

Label_Angle::Label_Angle(const NCollection_Utf8StringList &values,
                         const gp_Pnt p1, const gp_Pnt &p2, const gp_Pnt &p3)
    : myLabelWidth(0)
{
    myMainStr = values[0];
    mySUPStr = values[1];
    mySUBStr = values[2];

    myPntFirst = p1; myPntCorner = p2; myPntSecond = p3;

    myHasOrientation3D = Standard_True;
    // 1. normal of label
    myFirstDir = p1.XYZ()-p2.XYZ();
    mySecondDir = p3.XYZ()-p2.XYZ();
    myNormal = myFirstDir ^ mySecondDir;

    // 2.position of label
    gp_Pnt mid = 0.5*(p1.XYZ() + p3.XYZ());
    gp_Dir dvr = mid.XYZ() - p2.XYZ();
    Standard_Real dis = qMax(p2.Distance(p1), p2.Distance(p3));
    myOrientation3D = gp_Ax2(p2.Translated(dis*dvr), myNormal);
    myOrientation3D.SetYDirection(dvr);
}

void Label_Angle::SetLocation(const gp_Pnt &pnt)
{
    myHasOrientation3D = Standard_True;
    Handle(Geom_Plane) plane = GC_MakePlane(myPntFirst, myPntCorner, myPntSecond);
    GeomAPI_ProjectPointOnSurf PPOS(pnt,plane);
    gp_Pnt pp = PPOS.NearestPoint();

    myOrientation3D.SetLocation(pp);
    myOrientation3D.SetYDirection(myOrientation3D.Location().XYZ() - myPntCorner.XYZ());

    this->SetToUpdate();
    this->UpdatePresentations();
    this->GetContext()->RecomputeSelectionOnly(this);
}

void Label_Angle::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
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
        gp_Trsf apply;
        apply.SetTranslation(-0.5*myLabelWidth*myOrientation3D.XDirection());
        BRepBuilderAPI_Transform compTrans(strShape,apply);
        StdPrs_ShadedShape::Add(thePrs,compTrans.Shape(),myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());

        // 4.draw the fly out line and arrow
        ComputeFlyoutPnts();
        ComputeLeadLine(thePrs, anAspect);

        break;
    }
    }
}

void Label_Angle::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
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

void Label_Angle::ComputeFlyoutPnts()
{
    const Standard_Real textDis = myPntCorner.Distance(myOrientation3D.Location());
    const Standard_Real dis1 = myPntCorner.Distance(myPntFirst);
    const Standard_Real dis2 = myPntCorner.Distance(myPntSecond);

    Standard_Real pan1 = qMax(dis1, textDis+4);
    Standard_Real pan2 = qMax(dis2, textDis+4);

    if(pan1 < pan2)
        pan2 = pan1;
    else
        pan1 = pan2;

    myFirstFlyOut = myPntCorner.Translated(pan1*myFirstDir);
    mySecondFlyOut = myPntCorner.Translated(pan2*mySecondDir);
}

void Label_Angle::ComputeLeadLine (const Handle(Prs3d_Presentation)& thePrs,
                                    const Handle(Prs3d_ShadingAspect)& anAspect)
{
    // 1. compute flyout line
    TopoDS_Shape firstLin = BRepBuilderAPI_MakeEdge(myPntCorner, myFirstFlyOut);
    StdPrs_ShadedShape::Add(thePrs,firstLin,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    TopoDS_Shape secondLin = BRepBuilderAPI_MakeEdge(myPntCorner, mySecondFlyOut);
    StdPrs_ShadedShape::Add(thePrs,secondLin,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // 2. compute the arrow
    const Standard_Real textDis = myPntCorner.Distance(myOrientation3D.Location());
    const gp_Pnt arcP1 = myPntCorner.Translated(textDis*myFirstDir);
    const gp_Pnt arcP2 = myPntCorner.Translated(textDis*mySecondDir);// where the arc intersect two edge
    gp_Dir arcDir1 = myNormal ^ myFirstDir; // inside
    gp_Dir arcDir2 = mySecondDir ^ myNormal; // the normal direction at arc point

    // if location is out of region, reverse the arrow's direction
    if(!JudgePointInRegion(myOrientation3D.Location())) {
        arcDir1.Reverse();
        arcDir2.Reverse();
    }

    // the offseted points
    gp_Pnt arrowMid1 = arcP1.Translated(4*arcDir1);
    gp_Pnt arrowMid2 = arcP2.Translated(4*arcDir2);
    // the direction center point to arrow's end
    gp_Dir arrowDir1 = arrowMid1.XYZ() - myPntCorner.XYZ();
    gp_Dir arrowDir2 = arrowMid2.XYZ() - myPntCorner.XYZ();
    // the arrow's end point on circle
    arrowMid1 = myPntCorner.Translated(textDis * arrowDir1);
    arrowMid2 = myPntCorner.Translated(textDis * arrowDir2);

    gp_Pnt arrowL1 = arrowMid1.Translated(0.5*arrowDir1);
    gp_Pnt arrowR1 = arrowMid1.Translated(-0.5*arrowDir1);
    gp_Pnt arrowL2 = arrowMid2.Translated(0.5*arrowDir2);
    gp_Pnt arrowR2 = arrowMid2.Translated(-0.5*arrowDir2);

    Handle(Graphic3d_ArrayOfTriangles) aTriangle = new Graphic3d_ArrayOfTriangles(3);
    aTriangle->AddVertex (arrowL1);
    aTriangle->AddVertex (arcP1);
    aTriangle->AddVertex (arrowR1);
    thePrs->CurrentGroup()->AddPrimitiveArray(aTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    Handle(Graphic3d_ArrayOfTriangles) bTriangle = new Graphic3d_ArrayOfTriangles(3);
    bTriangle->AddVertex (arrowL2);
    bTriangle->AddVertex (arcP2);
    bTriangle->AddVertex (arrowR2);
    thePrs->CurrentGroup()->AddPrimitiveArray(bTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // 2. compute the arc
    gp_Pnt arcStart = arcP1;
    gp_Pnt arcEnd = arcP2;

    if(!JudgePointInRegion(myOrientation3D.Location())) {
        // the text has been translated 0.5*labelWidth by negtive direction of X
        gp_Dir textDir1 = gp_Pnt(-0.5*myLabelWidth, 0, 0).Transformed(calculateOrientionTrsf()).XYZ() - myPntCorner.XYZ();
        gp_Dir textDir2 = gp_Pnt(0.5*myLabelWidth, 0, 0).Transformed(calculateOrientionTrsf()).XYZ() - myPntCorner.XYZ();
        // if label is closer to first edge
        if(CloserToFirstEdge(myOrientation3D.Location())) {
            arcStart = myPntCorner.Translated(textDis*textDir2);
            arcEnd = arcP2.Translated(8*arcDir2);// translate more, it's not on the circle now!
            gp_Dir tmp = arcEnd.XYZ() - myPntCorner.XYZ();
            arcEnd = myPntCorner.Translated(textDis*tmp);// calculate the point on circle
        }
        else {
            arcEnd = myPntCorner.Translated(textDis*textDir1);
            arcStart = arcP1.Translated(8*arcDir1);
            gp_Dir tmp = arcStart.XYZ() - myPntCorner.XYZ();
            arcStart = myPntCorner.Translated(textDis*tmp);
        }
    }

    // draw the arc
    gp_Circ targetCirc(gp_Ax2(myPntCorner, myNormal), textDis);
    Handle(Geom_TrimmedCurve) arcCurve = GC_MakeArcOfCircle(targetCirc, arcStart, arcEnd, Standard_True);
    TopoDS_Shape arc = BRepBuilderAPI_MakeEdge(arcCurve);
    StdPrs_ShadedShape::Add(thePrs,arc,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
}

Standard_Boolean Label_Angle::JudgePointInRegion(const gp_Pnt &pt)
{
    gp_Dir pDir = pt.XYZ() - myPntCorner.XYZ();

    gp_Dir cross1 = pDir ^ myFirstDir;
    gp_Dir cross2 = pDir ^ mySecondDir;

    if(cross1.IsOpposite(cross2, 1e-6) && cross2.IsEqual(myNormal, Precision::Angular())) {
        return Standard_True;
    }
    else {
        return Standard_False;
    }
}

Standard_Boolean Label_Angle::CloserToFirstEdge(const gp_Pnt &pt)
{
    gp_Pnt mid = 0.5*(myPntFirst.XYZ() + myPntSecond.XYZ());
    const gp_Dir dvy = mid.XYZ() - myPntCorner.XYZ();
    gp_Dir textDir = pt.XYZ() - myPntCorner.XYZ();

    Standard_Real ang = textDir.AngleWithRef(dvy, myNormal);
    if(ang > 0) {
        return Standard_True;
    }
    else {
        return Standard_False;
    }
}
