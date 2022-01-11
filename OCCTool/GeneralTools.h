#pragma once
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopoDS_Shape.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt.hxx>
#include <V3d_View.hxx>
#include <gp_Circ.hxx>
#include <Geom_Line.hxx>
#include <AIS_Trihedron.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cone.hxx>

#include <list>
#include <map>
#include <vector>
using namespace std;
class GeneralTools
{
public:
    GeneralTools(void);
    static TopoDS_Shape RemoveDegenerateEdge(TopoDS_Shape oldshape);
    static Standard_Real CalcBoundingBoxDiam(TopoDS_Shape shape);
    static void CalcBoundingBoxPts(TopoDS_Shape targetShape,gp_Pnt& minpt,gp_Pnt& maxpt);
    static  TopoDS_Shape MakeCompoundFromShapes(list<TopoDS_Shape> origalShapes);
    static list<gp_Pnt> getPointsFromEdges(TopoDS_Shape aedge,int Num);
    static list<gp_Pnt> EdgeConvertToPnts(TopoDS_Edge aEdge,bool isF);
    static gp_Vec getNormalByPointOnFace(gp_Pnt P,TopoDS_Face face);
    static bool JudgeShapePlane(TopoDS_Shape aface,Standard_Real Tot,gp_Pln &pln);
    static bool JudgeShapeLine(TopoDS_Shape aface,Standard_Real Tot,gp_Lin &lin);
    static gp_Pnt GetMiddlePointOnFace(TopoDS_Face aFace);
    static bool JudgePointOnFace(gp_Pnt P, TopoDS_Shape aFace);
    static bool CompareCylinderForGroove(gp_Cylinder cylind1,gp_Cylinder cylind2,double & L);

    //static list<int> GetNoAdjacentByAdjacentFaceForNoEntity(Workpiece* wk,int face);
    static gp_Pnt getViewPointForm3DPoint(gp_Pnt P,Handle(V3d_View) myView);
    static void JudgeVectorByView(gp_Pnt P,Handle(V3d_View) myView,gp_Vec &V);
    static gp_Pnt GetMiddlePnt(gp_Pnt P1, gp_Pnt P2);
    static double GetHeightForPlaneFace(TopoDS_Shape Shape,gp_Vec V,gp_Pnt &endPoint1,gp_Pnt &endPoint2);
    static double GetHeightForCylinderFace(TopoDS_Shape Shape,gp_Pnt &endPoint1,gp_Pnt &endPoint2);
    static bool FitEllips(list<gp_Pnt> pts,gp_Elips& aElips);
    static bool FitCone(Handle(Geom_Surface) aSurface,gp_Ax1& axR);
    static bool FitCicle(list<gp_Pnt2d> pts2d,gp_Pnt2d &Center, Standard_Real& Radius);
    static bool FitCicle(list<gp_Pnt> pts,gp_Circ& cir);
    static bool FitSphere(list<gp_Pnt> pts,gp_Sphere& sphere);


    static bool CompareCylinder(gp_Cylinder cylind1,gp_Cylinder cylind2);
    static list<gp_Pnt> DiscreteShapeToPoints(TopoDS_Shape shape,bool isEdge);
    static void FaceConvertToPnts(TopoDS_Face aface,list<gp_Pnt> &Pts);
    static list<int> SortPtsByLine(vector<gp_Pnt> pts);
    static list<gp_Pnt> getMaxDistencePtsByLine(vector<gp_Pnt> pts,gp_Pnt P);
    static list<gp_Pnt> SortMeasurePointForLine(list<gp_Pnt> pts,Handle(Geom_Line) aline);
    static void TrsfPointByCoordinate(gp_Ax2 axA,gp_Ax2 axB,gp_Pnt &P);
    //static list<TopoDS_Shape> HandleProbeStyliShape(TopoDS_Shape shape,StyliParameter *MyStyliParameter);
    static list<gp_Pnt2d> TranslatePntToPnt2d(list<gp_Pnt> points,gp_Ax2 coordAx);
    static list<gp_Pnt> TranslatePnt2dToPnt(list<gp_Pnt2d> points,gp_Ax2 coordAx);
    static gp_Pnt TranslatePnt2dToPnt(gp_Pnt2d point,gp_Ax2 coordAx);

    static bool GetLine(Handle (Geom_Curve) anEdgeCurve,gp_Lin &aline);
    static bool GetPlane(TopoDS_Shape shape,gp_Pln &aPlane);
    static bool GetCylinder(Handle(Geom_Surface) aSurface,gp_Cylinder& cylind);
    static bool GetCicle(Handle(Geom_Curve) aCurve,gp_Circ& cir);
    static bool GetCone(Handle(Geom_Surface) aSurface,gp_Cone& aCone);
    static bool GetEllips(Handle(Geom_Curve) anEdgeCurve,gp_Elips& aElips);
    static bool GetSphere(TopoDS_Shape shape,list<gp_Pnt> Pts,gp_Sphere& sphere);
    static bool GetAxis(const Handle(Geom_Surface)& aSurface, gp_Ax1& ax);
    static bool GetCenter(const Handle(Geom_Curve)& aCurve, gp_Ax2& ax2);
    static bool GetShapeNormal(const TopoDS_Shape& shape, const gp_Pnt& p, gp_Dir& normal);
};
