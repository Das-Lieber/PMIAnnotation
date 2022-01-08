#include "AIS_DraftPoint.h"
#include <Graphic3d_ArrayOfPoints.hxx>
#include <StdPrs_Point.hxx>
#include <Select3D_SensitivePoint.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <Geom_CartesianPoint.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Line.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Geom_Plane.hxx>

IMPLEMENT_STANDARD_RTTIEXT(AIS_DraftPoint,AIS_DraftShape)
int AIS_DraftPoint::COUNT = 0;

AIS_DraftPoint::AIS_DraftPoint(const gp_Pnt& p, const gp_Pln& pln)
    :myPnt(p)
    ,myPln(pln)
{
    myDrawer->SetDisplayMode(0);
    myDrawer->SetPointAspect(new Prs3d_PointAspect(Aspect_TOM_O_STAR,Quantity_NOC_ORANGE,3.0));
    myHilightDrawer = new Prs3d_Drawer();
    myHilightDrawer->SetDisplayMode (-99);
    myHilightDrawer->SetPointAspect (new Prs3d_PointAspect (Aspect_TOM_PLUS, Quantity_NOC_GRAY80, 3.0));
    myHilightDrawer->SetColor (Quantity_NOC_GRAY80);
    myHilightDrawer->SetZLayer (Graphic3d_ZLayerId_UNKNOWN);
    myDynHilightDrawer = new Prs3d_Drawer();
    myDynHilightDrawer->SetDisplayMode (-99);
    myDynHilightDrawer->SetPointAspect (new Prs3d_PointAspect (Aspect_TOM_PLUS, Quantity_NOC_CYAN1, 3.0));
    myDynHilightDrawer->SetColor (Quantity_NOC_CYAN1);
    myDynHilightDrawer->SetZLayer (Graphic3d_ZLayerId_Top);

    COUNT++;
}

void AIS_DraftPoint::SetLocation(const gp_Pnt &pnt)
{
    if(myEditable){
        Handle(Geom_Plane) plane = new Geom_Plane(myPln);
        GeomAPI_ProjectPointOnSurf PPOS(pnt,plane);
        gp_Pnt pp = PPOS.NearestPoint();

        myPnt = pp;
        this->SetToUpdate();
        this->UpdatePresentations();
        this->GetContext()->RecomputeSelectionOnly(this);

        emit PosChanged();
    }
}

bool AIS_DraftPoint::getMyEditable() const
{
    return myEditable;
}

void AIS_DraftPoint::setMyEditable(bool newMyEditable)
{
    myEditable = newMyEditable;
}

void AIS_DraftPoint::Compute(const Handle(PrsMgr_PresentationManager3d)& /*aPresentationManager*/,
                        const Handle(Prs3d_Presentation)& aPresentation,
                        const Standard_Integer aMode)
{
  aPresentation->SetInfiniteState(myInfiniteState);

  if (aMode==0){
      Handle(Geom_CartesianPoint) pt = new Geom_CartesianPoint(myPnt);
    StdPrs_Point::Add(aPresentation,pt,myDrawer);
  }
  else if (aMode== -99)
    {
      Handle(Graphic3d_Group) TheGroup = aPresentation->CurrentGroup();
      TheGroup->SetPrimitivesAspect (myHilightDrawer->PointAspect()->Aspect());
      Handle(Graphic3d_ArrayOfPoints) aPoint = new Graphic3d_ArrayOfPoints (1);
      aPoint->AddVertex (myPnt.X(),myPnt.Y(),myPnt.Z());
      TheGroup->AddPrimitiveArray (aPoint);
    }

}

//=======================================================================
//function : ComputeSelection
//purpose  :
//=======================================================================
void AIS_DraftPoint::ComputeSelection(const Handle(SelectMgr_Selection)& aSelection,
                                 const Standard_Integer /*aMode*/)
{
  Handle(SelectMgr_EntityOwner) eown = new SelectMgr_EntityOwner(this,10);
  Handle(Select3D_SensitivePoint) sp = new Select3D_SensitivePoint(eown,
                                   myPnt);
  aSelection->Add(sp);
}
