 #include "Label_Length.h"

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

IMPLEMENT_STANDARD_RTTIEXT(Label_Length,Label_PMI)

Label_Length::Label_Length()
    : myLabelWidth(0)
{
}

void Label_Length::SetLocation(const gp_Pnt &pnt)
{
    myHasOrientation3D = Standard_True;

    gp_Lin lin(myOrientation3D.Location(),myOrientation3D.XDirection());
    Handle(Geom_Line) line = new Geom_Line(lin);
    GeomAPI_ProjectPointOnCurve PPC(pnt,line);
    gp_Pnt pp = PPC.NearestPoint();

    myOrientation3D.SetLocation(pp);
    this->SetToUpdate();
    this->UpdatePresentations();
    this->GetContext()->RecomputeSelectionOnly(this);
}

void Label_Length::Compute (const Handle(PrsMgr_PresentationManager3d)& /*thePrsMgr*/,
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

        // 2.compute the points
        gp_Pnt target = myOrientation3D.Location();
        Handle(Geom_Line) dimLine = new Geom_Line(myFirstPnt, mySecondPnt.XYZ()-myFirstPnt.XYZ());
        GeomAPI_ProjectPointOnCurve PPC(target,dimLine);
        gp_Pnt nearest = PPC.NearestPoint();
        gp_Vec flyOutVec = target.XYZ() - nearest.XYZ();

        myFirstOut = myFirstPnt.Translated(flyOutVec);
        mySecondOut = mySecondPnt.Translated(flyOutVec);

        // 3.set the color and material
        // material
        Graphic3d_MaterialAspect aMaterialAspect;
        aMaterialAspect.SetMaterialName(Graphic3d_NOM_STONE);

        // the shading aspect
        Handle(Prs3d_ShadingAspect) anAspect = new Prs3d_ShadingAspect();
        anAspect->SetMaterial (aMaterialAspect);
        anAspect->SetColor(myLabelColor);

        // 4.draw the main&sup&sub string
        TopoDS_Shape strShape = ComputeStringWithSupAndSub(myMainStr,mySUBStr,mySUPStr,myLabelWidth);
        StdPrs_ShadedShape::Add(thePrs,strShape,myDrawer);
        thePrs->CurrentGroup()->SetGroupPrimitivesAspect(anAspect->Aspect());

        // 5.draw the fly out line and arrow
        ComputeFlyOut(thePrs, anAspect);

        break;
    }
    }
}

void Label_Length::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
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

void Label_Length::JudgeTextOut(bool &left, bool &right)
{
    gp_Pnt textFirst = myOrientation3D.Location();
    gp_Pnt textSecond = gp_Pnt(myLabelWidth,0,0).Transformed(calculateOrientionTrsf());

    double dis1=0, dis2=0;
    const double dim = myFirstOut.Distance(mySecondOut);
    // judge left
    dis1 = textFirst.Distance(myFirstOut);
    dis2 = textFirst.Distance(mySecondOut);
    if(dis1 + dis2 - dim > 1e-6)
        left = true;
    else
        left = false;

    // judeg right
    dis1 = textSecond.Distance(myFirstOut);
    dis2 = textSecond.Distance(mySecondOut);
    if(dis1 + dis2 - dim > 1e-6)
        right = true;
    else
        right = false;
}

void Label_Length::ComputeFlyOut (const Handle(Prs3d_Presentation)& thePrs,
                                  const Handle(Prs3d_ShadingAspect)& anAspect)
{
    // 1. compute the fly out line
    TopoDS_Shape firstFlyLin = BRepBuilderAPI_MakeEdge(myFirstPnt, myFirstOut);
    StdPrs_ShadedShape::Add(thePrs,firstFlyLin,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
    TopoDS_Shape secondFlyLin = BRepBuilderAPI_MakeEdge(mySecondPnt, mySecondOut);
    StdPrs_ShadedShape::Add(thePrs,secondFlyLin,myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // 2. compute the arrow's points
    bool leftOver = false; bool rightOver = false;
    JudgeTextOut(leftOver, rightOver);

    gp_Pnt mid = 0.5*(myFirstOut.XYZ() + mySecondOut.XYZ());
    gp_Dir leftDir(myFirstOut.XYZ()-mid.XYZ());
    gp_Dir rightDir(mySecondOut.XYZ()-mid.XYZ());
    gp_Dir arrowBotm = myOrientation3D.YDirection();

    gp_Pnt leftMid = myFirstOut.Translated(4*leftDir.Reversed());
    gp_Pnt rightMid = mySecondOut.Translated(4*rightDir.Reversed());

    gp_Pnt leadLeft = leftMid; gp_Pnt leadRight = rightMid;
    gp_Pnt textFirst = myOrientation3D.Location();
    gp_Pnt textSecond = gp_Pnt(myLabelWidth,0,0).Transformed(calculateOrientionTrsf());
    // only left over the region
    if(leftOver&&!rightOver) {
        leadLeft = textFirst;
    }
    // only right over the region
    else if(!leftOver&&rightOver) {
        leadRight = textSecond;
    }
    // both over the region
    else if(leftOver&&rightOver) {
        leftMid = myFirstOut.Translated(4*leftDir);
        rightMid = mySecondOut.Translated(4*rightDir);

        // in the right hand
        if(textFirst.Distance(myFirstOut) > textFirst.Distance(mySecondOut)) {
            leadRight = textSecond;
            leadLeft = leftMid.Translated(4*leftDir);
        }
        // in the left hand
        else {
            leadLeft = textFirst;
            leadRight = rightMid.Translated(4*rightDir);
        }
    }

    gp_Pnt larrowL = leftMid.Translated(0.5*arrowBotm);
    gp_Pnt larrowR = leftMid.Translated(0.5*arrowBotm.Reversed());
    gp_Pnt rarrowL = rightMid.Translated(0.5*arrowBotm);
    gp_Pnt rarrowR = rightMid.Translated(0.5*arrowBotm.Reversed());

    // 3.1 arrow's lead line
    BRepBuilderAPI_MakeEdge aBuilder(leadLeft,leadRight);
    StdPrs_ShadedShape::Add(thePrs,aBuilder.Shape(),myDrawer);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    // 3.2 arrow's triangle
    Handle(Graphic3d_ArrayOfTriangles) aTriangle = new Graphic3d_ArrayOfTriangles(3);
    aTriangle->AddVertex (larrowL);
    aTriangle->AddVertex (myFirstOut);
    aTriangle->AddVertex (larrowR);
    thePrs->CurrentGroup()->AddPrimitiveArray(aTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());

    Handle(Graphic3d_ArrayOfTriangles) bTriangle = new Graphic3d_ArrayOfTriangles(3);
    bTriangle->AddVertex (rarrowL);
    bTriangle->AddVertex (mySecondOut);
    bTriangle->AddVertex (rarrowR);
    thePrs->CurrentGroup()->AddPrimitiveArray(bTriangle);
    thePrs->CurrentGroup()->SetGroupPrimitivesAspect (anAspect->Aspect());
}
