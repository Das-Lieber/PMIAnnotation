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
