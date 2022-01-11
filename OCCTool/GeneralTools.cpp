#include "GeneralTools.h"
#include <ShapeBuild_ReShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Face.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <GProp_PEquation.hxx>
#include <BRepGProp_Face.hxx>

#include <BRepFeat_SplitShape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <Precision.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepGProp_EdgeTool.hxx>
#include <math.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLib_Tool.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cone.hxx>
#include <gp_Circ.hxx>
#include <gp_Torus.hxx>
#include <gp_Elips.hxx>
#include <V3d_View.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Ax2.hxx>
#include <Geom_Line.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_Plane.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Geom_Circle.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <math_Matrix.hxx>
#include <Geom_Axis2Placement.hxx>
#include <gp_Lin.hxx>
#include <gp_Ax2.hxx>
#include <Geom_Line.hxx>

#include "math.h"
#include "pca.h"

#include <QObject>
#include <GeomLProp_SLProps.hxx>
#include <GeomLProp_CLProps.hxx>

#define  GLOG_NO_ABBREVIATED_SEVERITIES
//#include "glog/logging.h"
GeneralTools::GeneralTools(void)
{
}
TopoDS_Shape GeneralTools::RemoveDegenerateEdge(TopoDS_Shape oldshape)
{
    Handle_ShapeBuild_ReShape rebuild = new ShapeBuild_ReShape;
    rebuild->Apply(oldshape);
    TopExp_Explorer exp1;
    for (exp1.Init (oldshape, TopAbs_EDGE); exp1.More(); exp1.Next())
    {
        TopoDS_Edge edge = TopoDS::Edge(exp1.Current());
        if ( BRep_Tool::Degenerated(edge) )
            rebuild->Remove(edge);
    }
    TopoDS_Shape newshape = rebuild->Apply(oldshape);
    return newshape;
}
Standard_Real GeneralTools::CalcBoundingBoxDiam(TopoDS_Shape shape)
{
    gp_Pnt P1,P2;
    CalcBoundingBoxPts(shape,P1,P2);
    return P1.Distance(P2);
}
void GeneralTools::CalcBoundingBoxPts(TopoDS_Shape targetShape,gp_Pnt& minpt,gp_Pnt& maxpt)
{
    Bnd_Box bb;
    BRepBndLib::Add (targetShape, bb);
    double x1,y1,z1,x2,y2,z2;
    bb.Get (x1,y1,z1,x2,y2,z2);
    minpt.SetCoord(min(x1,x2),min(y1,y2),min(z1,z2));
    maxpt.SetCoord(max(x1,x2),max(y1,y2),max(z1,z2));
}
TopoDS_Shape GeneralTools::MakeCompoundFromShapes(list<TopoDS_Shape> origalShapes)
{
    TopoDS_Compound aRes;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound (aRes);
    for (list<TopoDS_Shape>::iterator it = origalShapes.begin();it != origalShapes.end();it++)
    {
        aBuilder.Add (aRes, *it);
    }
    return aRes;
}
list<gp_Pnt> GeneralTools::getPointsFromEdges(TopoDS_Shape aedge,int Num)
{
    list<gp_Pnt> repts;
    if (aedge.ShapeType() != TopAbs_EDGE && aedge.ShapeType() != TopAbs_WIRE)
        return repts;
    list<gp_Pnt> allpt;
    if (aedge.ShapeType() == TopAbs_WIRE)
    {
        TopExp_Explorer Ex;
        for (Ex.Init(aedge,TopAbs_EDGE);Ex.More();Ex.Next())
        {
            TopoDS_Shape Aedge = Ex.Current();
            list<gp_Pnt> pps = EdgeConvertToPnts(TopoDS::Edge(Aedge),false);
            for(list<gp_Pnt>::iterator it =pps.begin();it != pps.end();it++)
            {
                allpt.push_back(*it);
            }
        }
    }
    else
    {
        allpt = EdgeConvertToPnts(TopoDS::Edge(aedge),true);
    }

    int allnum = allpt.size();
    if(allnum <Num)
    {
        return allpt;
    }
    else
    {
        int step = allnum / Num;
        int i=1;
        int nowNum=1;
        for(list<gp_Pnt>::iterator it =allpt.begin();it != allpt.end();it++)
        {
            if(i==nowNum)
            {
                if(repts.empty())
                {
                    repts.push_back(*it);
                }
                else
                {
                    if(!repts.back().IsEqual(*it,Precision::Confusion()))
                    {
                        repts.push_back(*it);
                    }
                }
                nowNum += step;
            }
            i++;
        }
    }
    return repts;
}
list<gp_Pnt> GeneralTools::EdgeConvertToPnts(TopoDS_Edge aEdge,bool isF)
{
    list<gp_Pnt> rePnts;
    BRepAdaptor_Curve   C;
    C.Initialize(aEdge);
    Standard_Real Lower    = BRepGProp_EdgeTool::FirstParameter  (C);
    Standard_Real Upper    = BRepGProp_EdgeTool::LastParameter   (C);
    Standard_Integer Order = 0;//Min(BRepGProp_EdgeTool::IntegrationOrder (C),math::GaussPointsMax());
    Standard_Integer mm = BRepGProp_EdgeTool::IntegrationOrder (C);
    Standard_Integer mm1 = math::GaussPointsMax();
    if(isF)
        Order = mm>mm1?mm:mm1;
    else
        Order = mm>mm1?mm1:mm;
    gp_Pnt P;
    gp_Vec V1;
    Standard_Real ur, um, u;
    math_Vector GaussP (1, Order);
    math::GaussPoints  (Order,GaussP);
    Standard_Integer nbIntervals = BRepGProp_EdgeTool::NbIntervals(C, GeomAbs_CN);
    Standard_Boolean bHasIntervals = (nbIntervals > 1);
    TColStd_Array1OfReal TI(1, nbIntervals + 1);
    if(bHasIntervals)
    {
        BRepGProp_EdgeTool::Intervals(C, TI, GeomAbs_CN);
    }
    else
    {
        nbIntervals = 1;
    }
    Standard_Integer nIndex = 0;
    Standard_Real UU1 = Min(Lower, Upper);
    Standard_Real UU2 = Max(Lower, Upper);

    for(nIndex = 1; nIndex <= nbIntervals; nIndex++)
    {
        if(bHasIntervals)
        {
            Lower = Max(TI(nIndex), UU1);
            Upper = Min(TI(nIndex+1), UU2);
        }
        else
        {
            Lower = UU1;
            Upper = UU2;
        }
        Standard_Integer i;
        um = 0.5 * (Upper + Lower);
        ur = 0.5 * (Upper - Lower);
        int hafOrder = Order/2;
        if(Order%2 != 0)
            hafOrder +=1;
        list<gp_Pnt> rePntsNew;
        for (i = 1; i <= Order; i++)
        {
            u   = um + ur * GaussP (i);
            BRepGProp_EdgeTool::D1 (C,u, P, V1);
            if (i<=hafOrder)
            {
                rePnts.push_back(P);
            }
            else
            {
                rePntsNew.push_front(P);
            }
        }
        for (list<gp_Pnt>::iterator it = rePntsNew.begin();it != rePntsNew.end();it++)
        {
            rePnts.push_back(*it);
        }
    }
    return rePnts;
}
gp_Vec GeneralTools::getNormalByPointOnFace(gp_Pnt P,TopoDS_Face face)
{
    gp_Vec normal;
    Handle(Geom_Surface) aSurface = BRep_Tool::Surface(face);
    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if (st == GeomAbs_Plane )
    {

        gp_Pln aPlane = sSurface.Plane();
        gp_Ax1 ax1 = aPlane.Axis();
        gp_Dir dir = ax1.Direction ();
        normal.SetCoord(dir.X(),dir.Y(),dir.Z());
        TopAbs_Orientation orient = face.Orientation();
        if (orient!=TopAbs_FORWARD)
            normal.Reverse();
    }
    else
    {
        Standard_Real U,V;
        if (GeomLib_Tool::Parameters(aSurface,P,Precision::PConfusion(),U,V))
        {
            BRepGProp_Face theFace(face);
            theFace.Normal(U, V, P, normal);
            //            gp_Vec D1U,D1V;
            //            aSurface->D1(U,V,P,D1U,D1V);
            //            normal = D1U.Crossed(D1V);
        }
        else
        {
            GeomAPI_ProjectPointOnSurf PPS(P,aSurface);
            gp_Pnt np = PPS.NearestPoint();
            normal.SetCoord(P.X()-np.X(),P.Y()-np.Y(),P.Z()-np.Z());
        }
    }
    normal.Normalize();
    return normal;
}
void GeneralTools::FaceConvertToPnts(TopoDS_Face aface,list<gp_Pnt> &Pts)
{
    BRepGProp_Face theSurface(aface);
    Standard_Real LowerU, UpperU, LowerV, UpperV;
    theSurface.Bounds(LowerU, UpperU, LowerV, UpperV);
    const Standard_Integer UOrder = Min(theSurface.UIntegrationOrder(), math::GaussPointsMax());
    const Standard_Integer VOrder = Min(theSurface.VIntegrationOrder(), math::GaussPointsMax());
    math_Vector GaussPU(1, UOrder);
    math_Vector GaussPV(1, VOrder);
    math::GaussPoints (UOrder, GaussPU);
    math::GaussPoints (VOrder, GaussPV);
    const Standard_Real um = 0.5 * (UpperU+  LowerU);
    const Standard_Real vm = 0.5 * (UpperV+  LowerV);
    Standard_Real ur = 0.5 * (UpperU-LowerU);
    Standard_Real vr = 0.5 * (UpperV-LowerV);
    gp_Vec aNormal;
    gp_Pnt aPoint;
    for (Standard_Integer j = 1; j <= VOrder; ++j)
    {
        const Standard_Real v = vm + vr*GaussPV(j);
        for (Standard_Integer i = 1; i <= UOrder; ++i)
        {
            const Standard_Real u = um + ur*GaussPU (i);
            theSurface.Normal(u, v, aPoint, aNormal);
            Pts.push_back(aPoint);
        }
    }
}
list<gp_Pnt> GeneralTools::DiscreteShapeToPoints(TopoDS_Shape shape,bool isEdge)
{
    list<gp_Pnt> pts;
    if(isEdge)
    {
        TopExp_Explorer Ex;
        for(Ex.Init(shape,TopAbs_EDGE);Ex.More();Ex.Next())
        {
            list<gp_Pnt> ptsedges = EdgeConvertToPnts(TopoDS::Edge(Ex.Current()),false);
            if(!pts.empty() && !ptsedges.empty())
            {
                if(pts.back().Distance(ptsedges.front()) > pts.back().Distance(ptsedges.back()))
                {
                    ptsedges.reverse();
                }
            }
            list<gp_Pnt>::iterator it = pts.begin();
            advance(it,pts.size());
            pts.splice(it,ptsedges);
        }
    }
    else
    {
        TopExp_Explorer ExFace;
        for(ExFace.Init(shape,TopAbs_FACE);ExFace.More();ExFace.Next())
        {
            FaceConvertToPnts(TopoDS::Face(ExFace.Current()),pts);
        }
    }
    return pts;
}
void GeneralTools::JudgeVectorByView(gp_Pnt P,Handle(V3d_View) myView,gp_Vec &V)
{
    Standard_Integer viewX,viewY;
    myView->Convert(P.X(),P.Y(),P.Z(),viewX,viewY);
    double viewx,viewy,viewz,viewVX,viewVY,viewVZ;
    myView->ConvertWithProj(viewX,viewY,viewx,viewy,viewz,viewVX,viewVY,viewVZ);
    gp_Vec viewV(viewVX,viewVY,viewVZ);
    if(V.Angle(viewV)<M_PI/2)
        V.Reverse();
}

bool GeneralTools::FitCone(Handle(Geom_Surface) aSurface,gp_Ax1& axR)
{
    Standard_Real LowerU=0;
    Standard_Real UpperU=0;
    Standard_Real LowerV=0;
    Standard_Real UpperV=0;
    aSurface->Bounds(LowerU, UpperU, LowerV, UpperV);
    Standard_Real nowU = 0;
    Standard_Real nowV = 0;
    if(LowerU != Precision::Infinite())
        nowU = LowerU;
    else if(UpperU != Precision::Infinite())
        nowU = UpperU;
    if(LowerV != Precision::Infinite())
        nowV = LowerV;
    else if(UpperV != Precision::Infinite())
        nowV = UpperV;
    Handle(Geom_Curve) Ucurve = aSurface->UIso(nowU);
    gp_Circ cir;
    if(!GetCicle(Ucurve, cir))
    {
        Handle(Geom_Curve) Vcurve = aSurface->UIso(nowV);
        if(!GetCicle(Vcurve, cir))
        {
            return false;
        }
    }
    axR.SetLocation(cir.Location());
    axR.SetDirection(cir.Axis().Direction());
    return true;
}

bool GeneralTools::GetCicle(Handle(Geom_Curve) aCurve,gp_Circ& cir)
{
    GeomAdaptor_Curve  sanEdgeCurve(aCurve);
    if (sanEdgeCurve.GetType() == GeomAbs_Circle)
    {
        cir = sanEdgeCurve.Circle();
        return true;
    }
    else if(sanEdgeCurve.GetType() == GeomAbs_BSplineCurve )
    {
        Standard_Real firstParameter = aCurve->FirstParameter();
        Standard_Real lastParameter = aCurve->LastParameter();
        int num=100;
        Standard_Real step = (lastParameter - firstParameter)/num;
        list<gp_Pnt> pts;
        for(int i=0;i<=num;i++)
        {
            gp_Pnt P(0,0,0);
            aCurve->D0(firstParameter+step*i,P);
            pts.push_back(P);
        }
        return FitCicle(pts,cir);
    }
    else
        return false;
}
bool GeneralTools::FitCicle(list<gp_Pnt> pts,gp_Circ& cir)
{
    if (pts.size() < 3)
        return false;
    double tot = 0.001;
    PCA pca(pts);
    pca.Run();
    vector<gp_Vec> aVectors = pca.GetEigenVector();
    gp_Pnt acenter = pca.GetCenterPoint();
    if(aVectors[0].Magnitude ()<=tot || aVectors[1].Magnitude ()<=tot)
        return false;
    gp_Vec normalV = aVectors[0].Crossed(aVectors[1]);
    normalV.Normalize();
    gp_Ax2 coordAx(acenter,normalV,aVectors[0]);
    list<gp_Pnt2d> pts2d = TranslatePntToPnt2d(pts,coordAx);
    gp_Pnt2d Center2d(0,0);
    Standard_Real Radius = 0;
    if(!FitCicle(pts2d,Center2d,Radius))
    {
        return false;
    }
    gp_Pnt centerP = TranslatePnt2dToPnt(Center2d,coordAx);
    gp_Circ cir1(coordAx.Translated(coordAx.Location(),centerP),Radius);
    cir = cir1;
    return true;
}
bool GeneralTools::FitCicle(list<gp_Pnt2d> pts2d,gp_Pnt2d &Center, Standard_Real& Radius)
{
    if (pts2d.size() < 3)
        return false;
    double X1,X2, X3,Y1,Y2,Y3,X1Y1,X1Y2,X2Y1;
    X1=X2=X3=Y1=Y2=Y3=X1Y1=X1Y2=X2Y1=0;
    int i = 0;
    bool Equalx = false, Equaly = false;
    for (list<gp_Pnt2d>::iterator it=pts2d.begin();it!=pts2d.end();it++)
    {
        X1 = X1 + it->X();
        Y1 = Y1 + it->Y();
        if (it->Y() != Y1 / (i + 1))
        {
            Equaly = true;
        }
        if (it->X() != X1 / (i + 1))
        {
            Equalx = true;
        }
        X2 = X2 + it->X() * it->X();
        Y2 = Y2 + it->Y() * it->Y();
        X3 = X3 + it->X() * it->X() * it->X();
        Y3 = Y3 + it->Y() * it->Y() *it->Y();
        X1Y1 = X1Y1 + it->X() * it->Y();
        X1Y2 = X1Y2 + it->X() * it->Y() * it->Y();
        X2Y1 = X2Y1 + it->X() * it->X() * it->Y();
        i++;
    }
    if(!Equalx||!Equaly)
    {
        return false;
    }
    double C, D, E, G, H, N;
    double a, b, c;
    N = pts2d.size();
    C = N*X2 - X1*X1;
    D = N*X1Y1 - X1*Y1;
    E = N*X3 + N*X1Y2 - (X2 + Y2)*X1;
    G = N*Y2 - Y1*Y1;
    H = N*X2Y1 + N*Y3 - (X2 + Y2)*Y1;
    a = (H*D - E*G) / (C*G - D*D + 1e-10);
    b = (H*C - E*D) / (D*D - G*C + 1e-10);
    c = -(a*X1 + b*Y1 + X2 + Y2) / (N + 1e-10);

    Center.SetCoord(a / (-2),b / (-2));
    double dTemp = a*a + b*b - 4 * c;
    if (dTemp < 0)
    {
        return false;
    }
    Radius = sqrt(dTemp) / 2;
    return true;
}
bool GeneralTools::FitSphere(list<gp_Pnt> pts,gp_Sphere& sphere)
{
    if (pts.size() < 3)
    {
        return false;
    }
    double sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f, sum_z3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    double sum_xz = 0.0f, sum_x1z2 = 0.0f, sum_x2z1 = 0.0f;
    double sum_yz = 0.0f, sum_y1z2 = 0.0f, sum_y2z1 = 0.0f;
    int N = pts.size();
    for (list<gp_Pnt>::iterator it = pts.begin();it != pts.end();it++)
    {
        double x = it->X();
        double y = it->Y();
        double z = it->Z();
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        sum_x += x;
        sum_y += y;
        sum_z += z;

        sum_x2 += x2;
        sum_y2 += y2;
        sum_z2 += z2;

        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_z3 += z2 * z;

        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;

        sum_xz += x * z;
        sum_x1z2 += x * z2;
        sum_x2z1 += x2 * z;

        sum_yz += y * z;
        sum_y1z2 += y * z2;
        sum_y2z1 += y2 * z;
    }
    double C, D, E, P, G, F, H, J, K;
    double a, b, c, d;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    P = N * sum_xz - sum_x * sum_z;
    E = N * sum_x3 + N * sum_x1y2 + N * sum_x1z2 - (sum_x2 + sum_y2 + sum_z2) * sum_x;

    G = N * sum_y2 - sum_y * sum_y;
    F = N * sum_yz - sum_y * sum_z;
    H = N * sum_x2y1 + N * sum_y3 + N * sum_y1z2 - (sum_x2 + sum_y2 + sum_z2) * sum_y;

    J = N * sum_z2 - sum_z * sum_z;
    K = N * sum_x2z1 + N * sum_y2z1 + N * sum_z3 - (sum_x2 + sum_y2 + sum_z2) * sum_z;

    double CC, DD, EE, GG, HH;
    CC = C*J - P*P;
    DD = D*J - F*P;
    EE = E*J - P*K;
    GG = G*J - F*F;
    HH = H*J - F*K;

    a = (HH * DD - EE * GG) / (CC * GG - DD * DD);
    b = (HH * CC - EE * DD) / (DD * DD - GG * CC);
    c = 0.0f;
    if (P != 0)
    {
        c = (-C*a - D*b - E) / P;
    }
    else if (F != 0)
    {
        c = (-D*a - G*b - H) / F;
    }
    else if (J != 0)
    {
        c = (-P*a - F*b - K) / J;
    }
    d = -(a * sum_x + b * sum_y + c * sum_z + sum_x2 + sum_y2 + sum_z2) / N;

    gp_Pnt centerP(a / (-2),b / (-2),c / (-2));
    gp_Ax2 ax2(centerP,gp::OZ().Direction());
    Standard_Real R = sqrt(a * a + b * b + c * c - 4 * d) / 2;
    gp_Sphere sphere1(ax2,R);
    sphere = sphere1;
    return true;
}

gp_Pnt GeneralTools::getViewPointForm3DPoint(gp_Pnt P,Handle(V3d_View) myView)
{
    Standard_Real x,y,z;
    myView->Project(P.X(),P.Y(),P.Z(),x,y,z);
    return gp_Pnt(x,y,z);
}
gp_Pnt GeneralTools::GetMiddlePnt(gp_Pnt P1, gp_Pnt P2)
{
    return gp_Pnt((P1.X()+P2.X())/2,(P1.Y()+P2.Y())/2,(P1.Z()+P2.Z())/2);
}
double GeneralTools::GetHeightForPlaneFace(TopoDS_Shape Shape,gp_Vec V,gp_Pnt &endPoint1,gp_Pnt &endPoint2)
{
    double tol = 0.0001;
    double reL = 0;
    TopTools_IndexedMapOfShape aEdges;
    TopExp::MapShapes(Shape, TopAbs_EDGE, aEdges);
    TopoDS_Shape aEdge;
    for (int i=1;i<=aEdges.Extent();i++)
    {
        Standard_Real FirstDummy,LastDummy;
        Handle (Geom_Curve) anEdgeCurve = BRep_Tool::Curve(TopoDS::Edge(aEdges(i)),FirstDummy,LastDummy);
        GeomAdaptor_Curve  sanEdgeCurve(anEdgeCurve);
        gp_Lin lin;
        if (sanEdgeCurve.GetType() == GeomAbs_Line || JudgeShapeLine(Shape,tol,lin))
        {
            if(sanEdgeCurve.GetType() == GeomAbs_Line)
                lin = sanEdgeCurve.Line();
            gp_Dir dirV = lin.Direction();
            if(dirV.IsParallel(V,tol))
            {
                GProp_GProps System;
                BRepGProp::LinearProperties(aEdges(i),System);
                double L = System.Mass();
                if(L>reL || reL == 0 )
                {
                    reL = L;
                    aEdge = aEdges(i);
                }
            }
        }
    }
    TopTools_IndexedMapOfShape aVertexs;
    TopExp::MapShapes(aEdge, TopAbs_VERTEX, aVertexs);
    endPoint1 = BRep_Tool::Pnt(TopoDS::Vertex(aVertexs(1)));
    endPoint2 = BRep_Tool::Pnt(TopoDS::Vertex(aVertexs(2)));
    return reL;
}
double GeneralTools::GetHeightForCylinderFace(TopoDS_Shape Shape,gp_Pnt &endPoint1,gp_Pnt &endPoint2)
{
    double reL = 0;
    TopExp_Explorer Ex;
    TopoDS_Shape aEdge;
    for (Ex.Init(Shape,TopAbs_EDGE); Ex.More(); Ex.Next())
    {
        Standard_Real FirstDummy,LastDummy;
        Handle (Geom_Curve) anEdgeCurve = BRep_Tool::Curve(TopoDS::Edge(Ex.Current()),FirstDummy,LastDummy);
        GeomAdaptor_Curve  sanEdgeCurve(anEdgeCurve);
        gp_Lin lin;
        if (sanEdgeCurve.GetType() == GeomAbs_Line ||JudgeShapeLine(Ex.Current(),Precision::Confusion(),lin))
        {
            GProp_GProps System;
            BRepGProp::LinearProperties(Ex.Current(),System);
            reL = System.Mass();
            aEdge = Ex.Current();
        }
    }
    TopTools_IndexedMapOfShape aVertexs;
    TopExp::MapShapes(aEdge, TopAbs_VERTEX, aVertexs);
    endPoint1 = BRep_Tool::Pnt(TopoDS::Vertex(aVertexs(1)));
    endPoint2 = BRep_Tool::Pnt(TopoDS::Vertex(aVertexs(2)));
    return reL;
}
bool GeneralTools::CompareCylinder(gp_Cylinder cylind1,gp_Cylinder cylind2)
{
    double tol = 0.0001;
    if(abs(cylind1.Radius() - cylind2.Radius())<=tol)
    {
        gp_Lin lin(cylind1.Location(),cylind1.Axis().Direction());
        Handle(Geom_Line) gline = new Geom_Line(lin);
        GeomAPI_ProjectPointOnCurve PPC (cylind2.Location(),gline);
        if(PPC.LowerDistance()<=tol)
            return true;
    }
    return false;
}
gp_Pnt GeneralTools::GetMiddlePointOnFace(TopoDS_Face aFace)
{
    gp_Pnt Pt;
    BRepGProp_Face theSurface(aFace);
    Standard_Real LowerU, UpperU, LowerV, UpperV;
    theSurface.Bounds(LowerU, UpperU, LowerV, UpperV);
    const Standard_Integer UOrder = Min(theSurface.UIntegrationOrder(), math::GaussPointsMax());
    const Standard_Integer VOrder = Min(theSurface.VIntegrationOrder(), math::GaussPointsMax());
    math_Vector GaussPU(1, UOrder);
    math_Vector GaussPV(1, VOrder);
    math::GaussPoints (UOrder, GaussPU);
    math::GaussPoints (VOrder, GaussPV);
    const Standard_Real um = 0.5 * (UpperU+  LowerU);
    const Standard_Real vm = 0.5 * (UpperV+  LowerV);
    Standard_Real ur = 0.5 * (UpperU-LowerU);
    Standard_Real vr = 0.5 * (UpperV-LowerV);
    const Standard_Real v = vm+vr*GaussPV(VOrder/2);
    const Standard_Real u = um+ur*GaussPU (UOrder/2);
    gp_Vec aNormal;
    theSurface.Normal(u, v, Pt, aNormal);
    return Pt;
}


bool GeneralTools::JudgePointOnFace(gp_Pnt P, TopoDS_Shape aFace)
{
    BRepClass3d_SolidClassifier aSC(aFace);
    aSC.Perform(P,Precision::Confusion());  //设置容差
    TopAbs_State aState = aSC.State();
    if(aState == TopAbs_ON)
        return true;
    else
        return false;
}
bool GeneralTools::CompareCylinderForGroove(gp_Cylinder cylind1,gp_Cylinder cylind2,double & L)
{
    double tol = 0.0001;
    if(abs(cylind1.Radius() - cylind2.Radius())<=tol)
    {
        gp_Lin lin(cylind1.Location(),cylind1.Axis().Direction());
        Handle(Geom_Line) gline = new Geom_Line(lin);
        GeomAPI_ProjectPointOnCurve PPC (cylind2.Location(),gline);
        L = PPC.LowerDistance();
        if(L<=tol)
            return false;
        if (cylind1.Axis().Direction().IsParallel(cylind2.Axis().Direction(),tol))
            return true;
        else
            return false;
    }
    return false;
}
bool GeneralTools::JudgeShapePlane(TopoDS_Shape aface,Standard_Real Tot,gp_Pln &pln)
{
    list<gp_Pnt> Pts = DiscreteShapeToPoints( aface,false);
    if(!Pts.empty())
    {
        TColgp_Array1OfPnt Pnts(1,Pts.size());
        int i=1;
        for (list<gp_Pnt>::iterator it = Pts.begin();it != Pts.end();it++)
        {
            Pnts(i++) = *it;
        }
        GProp_PEquation gpe(Pnts, Tot);
        if(gpe.IsPlanar())
        {
            pln = gpe.Plane();
            return true;
        }
    }
    return false;
}
bool GeneralTools::JudgeShapeLine(TopoDS_Shape aface,Standard_Real Tot,gp_Lin &lin)
{
    list<gp_Pnt> Pts = DiscreteShapeToPoints( aface,true);
    if(!Pts.empty())
    {
        TColgp_Array1OfPnt Pnts(1,Pts.size());
        int i=1;
        for (list<gp_Pnt>::iterator it = Pts.begin();it != Pts.end();it++)
        {
            Pnts(i++) = *it;
        }
        GProp_PEquation gpe(Pnts, Tot);
        if(gpe.IsLinear())
        {
            lin = gpe.Line();
            return true;
        }
    }
    return false;
}
list<int> GeneralTools::SortPtsByLine(vector<gp_Pnt> pts)
{
    list<int> registeredP;
    if(pts.size()<=1)
    {
        for(unsigned int i=0;i<pts.size();i++)
        {
            registeredP.push_back(i);
        }
        return registeredP;
    }
    gp_Pnt P = pts[0];
    Standard_Real maxDis = 0;
    int maxI = -1;
    int maxJ = -1;
    for(unsigned int i=0;i<pts.size();i++)
    {
        for(unsigned int j=i+1;j<pts.size();j++)
        {
            Standard_Real d = pts[i].Distance(pts[j]);
            if(maxDis<d)
            {
                maxDis = d;
                maxI = i;
                maxJ = j;
            }
        }
    }
    if(pts[maxI].Distance(P)>pts[maxI].Distance(P))
    {
        int temp = maxI;
        maxI = maxJ;
        maxJ = temp;
    }
    registeredP.push_back(maxI);
    //registeredP.push_back(maxJ);
    list<gp_Pnt> resultPts;
    resultPts.push_back(pts[maxI]);
    while(registeredP.size()<pts.size())
    {
        Standard_Real  minDis = -1;
        gp_Pnt nowP = resultPts.back();
        int minI = -1;
        for(unsigned int i=0;i<pts.size();i++)
        {
            if(find(registeredP.begin(),registeredP.end(),i)==registeredP.end())
            {
                Standard_Real d = pts[i].Distance(nowP);
                if(minDis>d || minDis==-1)
                {
                    minI = i;
                    minDis = d;
                }
            }
        }
        registeredP.push_back(minI);
        resultPts.push_back(pts[minI]);
    }
    return registeredP;
}
list<gp_Pnt> GeneralTools::getMaxDistencePtsByLine(vector<gp_Pnt> pts,gp_Pnt P)
{
    list<gp_Pnt> resultPts;
    if(pts.size()<=1)
    {
        for(unsigned int i=0;i<pts.size();i++)
        {
            resultPts.push_back(pts[i]);
        }
        return resultPts;
    }
    Standard_Real maxDis = 0;
    int maxI = -1;
    int maxJ = -1;
    for(unsigned int i=0;i<pts.size();i++)
    {
        for(unsigned int j=i+1;j<pts.size();j++)
        {
            Standard_Real d = pts[i].Distance(pts[j]);
            if(maxDis<d)
            {
                maxDis = d;
                maxI = i;
                maxJ = j;
            }
        }
    }
    if(pts[maxI].Distance(P)>pts[maxJ].Distance(P))
    {
        int temp = maxI;
        maxI = maxJ;
        maxJ = temp;
    }
    resultPts.push_back(pts[maxI]);
    resultPts.push_back(pts[maxJ]);
    return resultPts;
}
list<gp_Pnt> GeneralTools::SortMeasurePointForLine(list<gp_Pnt> pts,Handle(Geom_Line) aline)
{
    vector<gp_Pnt> points;
    points.reserve(pts.size());
    for(list<gp_Pnt>::iterator it=pts.begin();it != pts.end();it++)
    {
        GeomAPI_ProjectPointOnCurve PPC (*it,aline);
        gp_Pnt TP = PPC.NearestPoint();
        points.push_back(TP);
    }
    list<gp_Pnt> resultPts;
    list<int> repts = SortPtsByLine(points);
    for(list<int>::iterator it =repts.begin();it !=repts.end();it++ )
    {
        resultPts.push_back(points[*it]);
    }
    return resultPts;
}
void GeneralTools::TrsfPointByCoordinate(gp_Ax2 axA,gp_Ax2 axB,gp_Pnt &P)
{//点P为坐标系B下的坐标系，变换到坐标系A，坐标系B为A的下级坐标系
    gp_Ax2 tempB(axA.Location(),axB.Direction(),axB.XDirection());
    gp_Trsf aTransformation;
    aTransformation.SetTransformation(tempB,axA);
    P.Transform(aTransformation);
    P.Translate(gp_Pnt(0,0,0),axB.Location());
}

bool GeneralTools::FitEllips(list<gp_Pnt> pts,gp_Elips& aElips)
{
    double tot = 0.001;
    PCA pca(pts);
    pca.Run();
    vector<gp_Vec> aVectors = pca.GetEigenVector();
    gp_Pnt acenter = pca.GetCenterPoint();
    if(aVectors.size() < 2)
        return false;

    if(aVectors[0].Magnitude ()<=tot || aVectors[1].Magnitude ()<=tot)
        return false;
    gp_Vec normalV = aVectors[0].Crossed(aVectors[1]);
    normalV.Normalize();
    gp_Ax2 coordAx(acenter,normalV,aVectors[0]);
    list<gp_Pnt2d> pts2d = TranslatePntToPnt2d(pts,coordAx);

    long double A = 0.00,B = 0.00,C = 0.00,D = 0.00,E = 0.00;
    long double x2y2=0.0,x1y3=0.0,x2y1=0.0,x1y2=0.0,x1y1=0.0,yyy4=0.0,yyy3=0.0,yyy2=0.0,xxx2=0.0,xxx1=0.0,yyy1=0.0,x3y1=0.0,xxx3=0.0;
    long double N = pts2d.size();
    for(list<gp_Pnt2d>::iterator it=pts2d.begin();it!=pts2d.end();it++)
    {
        long double xi = it->X(), yi = it->Y();

        x2y2 += xi*xi*yi*yi;
        x1y3 += xi*yi*yi*yi;
        x2y1 += xi*xi*yi;
        x1y2 += xi*yi*yi;
        x1y1 += xi*yi;
        yyy4 += yi*yi*yi*yi;
        yyy3 += yi*yi*yi;
        yyy2 += yi*yi;
        xxx2 += xi*xi;
        xxx1 += xi;
        yyy1 += yi;
        x3y1 += xi*xi*xi*yi;
        xxx3 += xi*xi*xi;
    }
    long double matrix[5][5]={{x2y2,x1y3,x2y1,x1y2,x1y1},
                              {x1y3,yyy4,x1y2,yyy3,yyy2},
                              {x2y1,x1y2,xxx2,x1y1,xxx1},
                              {x1y2,yyy3,x1y1,yyy2,yyy1},
                              {x1y1,yyy2,xxx1,yyy1,N}
                             };

    long double matrix2[5][1]={x3y1,x2y2,xxx3,x2y1,xxx2};
    long double matrix3[5][1] = {A,B,C,D,E};

    ////求矩阵matrix的逆，结果为InverseMatrix
    ///单位矩阵
    int n = 5;
    long double E_Matrix[5][5];
    long double mik;
    long double m = 2*n;
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            if(i == j)
                E_Matrix[i][j] = 1.00;
            else
                E_Matrix[i][j] = 0.00;
        }
    }
    long double CalcuMatrix[5][2*5];
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            CalcuMatrix[i][j] = matrix[i][j];
        }
        for(int k = n; k < m; k++)
        {
            CalcuMatrix[i][k] = E_Matrix[i][k-n];
        }
    }

    for(int i = 1; i <= n-1; i++)
    {
        for(int j = i+1; j <= n; j++)
        {
            mik = CalcuMatrix[j-1][i-1]/CalcuMatrix[i-1][i-1];
            for(int k = i+1;k <= m; k++)
            {
                CalcuMatrix[j-1][k-1] -= mik*CalcuMatrix[i-1][k-1];
            }
        }
    }
    for(int i=1;i<=n;i++)
    {
        long double temp = CalcuMatrix[i-1][i-1];
        for(int j=1;j<=m;j++)
        {
            CalcuMatrix[i-1][j-1] = CalcuMatrix[i-1][j-1]/temp;
        }
    }
    for(int k=n-1;k>=1;k--)
    {
        for(int i=k;i>=1;i--)
        {
            mik = CalcuMatrix[i-1][k];
            for(int j=k+1;j<=m;j++)
            {
                CalcuMatrix[i-1][j-1] -= mik*CalcuMatrix[k][j-1];
            }
        }
    }
    long double InverseMatrix[5][5];
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            InverseMatrix[i][j] = CalcuMatrix[i][j+n];
        }
    }
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<5;j++)
        {
            if(fabs(InverseMatrix[i][j]) < 0.0000001)
                InverseMatrix[i][j] = 0.00;
        }
    }
    ///求参数A,B,C,D,E
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<5;j++)
        {
            matrix3[i][0] += InverseMatrix[i][j]*(-matrix2[j][0]);
        }
    }
    A = matrix3[0][0];
    B = matrix3[1][0];
    C = matrix3[2][0];
    D = matrix3[3][0];
    E = matrix3[4][0];
    ///求拟合结果重要参数
    double Xc = (2*B*C-A*D)/(A*A-4*B);
    double Yc = (2*D-A*D)/(A*A-4*B);
    double a = sqrt(fabs(2*(A*C*D-B*C*C-D*D+4*B*E-A*A*E)/((A*A-4*B)*(B-sqrt(A*A+(1-B)*(1-B))+1))));
    double b = sqrt(fabs(2*(A*C*D-B*C*C-D*D+4*B*E-A*A*E)/((A*A-4*B)*(B+sqrt(A*A+(1-B)*(1-B))+1))));
    double theta = atan2(a*a-b*b*B,a*a*B-b*b);
    gp_Pnt centerP = TranslatePnt2dToPnt(gp_Pnt2d(Xc,Yc),coordAx);
    coordAx.Translate(acenter,centerP);
    gp_Ax2 otherAx = coordAx.Rotated(coordAx.Axis(),theta);
    gp_Elips aElips1(otherAx,a>b?a:b,a>b?b:a);
    aElips = aElips1;
    return true;
}
/// <summary>
/// 将3维点转化为2维点
/// </summary>
list<gp_Pnt2d> GeneralTools::TranslatePntToPnt2d(list<gp_Pnt> points,gp_Ax2 coordAx)
{
    list<gp_Pnt2d> points2d;
    //int count = points.size();
    //points2d.reserve(count);
    math_Matrix matrix(0,3,0,3);  //变换矩阵
    matrix(0,0) = coordAx.XDirection().X();
    matrix(0,1) = coordAx.YDirection().X();
    matrix(0,2) = coordAx.Direction().X();
    matrix(0,3) = coordAx.Location().X();
    matrix(1,0) = coordAx.XDirection().Y();
    matrix(1,1) = coordAx.YDirection().Y();
    matrix(1,2) = coordAx.Direction().Y();
    matrix(1,3) = coordAx.Location().Y();
    matrix(2,0) = coordAx.XDirection().Z();
    matrix(2,1) = coordAx.YDirection().Z();
    matrix(2,2) = coordAx.Direction().Z();
    matrix(2,3) = coordAx.Location().Z();
    matrix(3,0) = 0;
    matrix(3,1) = 0;
    matrix(3,2) = 0;
    matrix(3,3) = 1;

    math_Matrix matrixInv(0,3,0,3);
    matrixInv = matrix.Inverse();
    for (list<gp_Pnt>::iterator it=points.begin();it != points.end();it++)
    {
        math_Matrix matrixP(0,3,0,0);
        matrixP(0,0) = it->X();
        matrixP(1,0) = it->Y();
        matrixP(2,0) = it->Z();
        matrixP(3,0) = 1;
        math_Matrix matrixP2(0,3,0,0);
        matrixP2 = matrixInv * matrixP;
        gp_Pnt2d P2d(matrixP2(0,0),matrixP2(1,0));
        points2d.push_back(P2d);
    }
    return points2d;
}
/// <summary>
/// 将2维点转化为3维点
/// </summary>
list<gp_Pnt> GeneralTools::TranslatePnt2dToPnt(list<gp_Pnt2d> points,gp_Ax2 coordAx)
{
    list<gp_Pnt> points3d;
    //int count = points.size();
    math_Matrix matrix(0,3,0,3);  //变换矩阵
    matrix(0,0) = coordAx.XDirection().X();
    matrix(0,1) = coordAx.YDirection().X();
    matrix(0,2) = coordAx.Direction().X();
    matrix(0,3) = coordAx.Location().X();
    matrix(1,0) = coordAx.XDirection().Y();
    matrix(1,1) = coordAx.YDirection().Y();
    matrix(1,2) = coordAx.Direction().Y();
    matrix(1,3) = coordAx.Location().Y();
    matrix(2,0) = coordAx.XDirection().Z();
    matrix(2,1) = coordAx.YDirection().Z();
    matrix(2,2) = coordAx.Direction().Z();
    matrix(2,3) = coordAx.Location().Z();
    matrix(3,0) = 0;
    matrix(3,1) = 0;
    matrix(3,2) = 0;
    matrix(3,3) = 1;
    for (list<gp_Pnt2d>::iterator it=points.begin();it != points.end();it++)
    {
        math_Matrix matrixP(0,3,0,0);
        matrixP(0,0) = it->X();
        matrixP(1,0) = it->Y();
        matrixP(2,0) = 0;
        matrixP(3,0) = 1;
        math_Matrix matrixP2(0,3,0,0);
        matrixP2 = matrix * matrixP;
        gp_Pnt P3d(matrixP2(0,0),matrixP2(1,0),matrixP2(2,0));
        points3d.push_back(P3d);
    }
    return points3d;
}

gp_Pnt GeneralTools::TranslatePnt2dToPnt(gp_Pnt2d point,gp_Ax2 coordAx)
{
    /*if(!hasCoordAx)
        return false;*/
    gp_Pnt Tpoint;
    math_Matrix matrix(0,3,0,3);  //变换矩阵
    matrix(0,0) = coordAx.XDirection().X();
    matrix(0,1) = coordAx.YDirection().X();
    matrix(0,2) = coordAx.Direction().X();
    matrix(0,3) = coordAx.Location().X();
    matrix(1,0) = coordAx.XDirection().Y();
    matrix(1,1) = coordAx.YDirection().Y();
    matrix(1,2) = coordAx.Direction().Y();
    matrix(1,3) = coordAx.Location().Y();
    matrix(2,0) = coordAx.XDirection().Z();
    matrix(2,1) = coordAx.YDirection().Z();
    matrix(2,2) = coordAx.Direction().Z();
    matrix(2,3) = coordAx.Location().Z();
    matrix(3,0) = 0;
    matrix(3,1) = 0;
    matrix(3,2) = 0;
    matrix(3,3) = 1;
    math_Matrix matrixP(0,3,0,0);
    matrixP(0,0) = point.X();
    matrixP(1,0) = point.Y();
    matrixP(2,0) = 0;
    matrixP(3,0) = 1;
    math_Matrix matrixP2(0,3,0,0);
    matrixP2 = matrix * matrixP;
    Tpoint.SetCoord(matrixP2(0,0),matrixP2(1,0),matrixP2(2,0));
    return Tpoint;
}

bool GeneralTools::GetCylinder(Handle(Geom_Surface) aSurface,gp_Cylinder& cylind)
{
    double tol = 0.0001;
    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if (st == GeomAbs_Cylinder )
    {
        cylind =  sSurface.Cylinder();
        return true;
    }
    else if(st == GeomAbs_SurfaceOfRevolution)
    {
        gp_Ax1 ax1R = sSurface.AxeOfRevolution();
        Standard_Real LowerU=0;
        Standard_Real UpperU=0;
        Standard_Real LowerV=0;
        Standard_Real UpperV=0;
        aSurface->Bounds(LowerU, UpperU, LowerV, UpperV);
        Standard_Real nowU = 0;
        Standard_Real nowV = 0;
        if(LowerU != Precision::Infinite())
            nowU = LowerU;
        else if(UpperU != Precision::Infinite())
            nowU = UpperU;
        if(LowerV != Precision::Infinite())
            nowV = LowerV;
        else if(UpperV != Precision::Infinite())
            nowV = UpperV;
        Handle(Geom_Curve) Ucurve = aSurface->UIso(nowU);
        Handle(Geom_Curve) Vcurve = aSurface->VIso(nowV);
        gp_Circ cir;
        gp_Lin aLine;
        if(GetCicle(Ucurve, cir))
        {
            if(GetLine(Vcurve,aLine))
            {
                if(aLine.Direction().IsParallel(ax1R.Direction(),tol))
                {
                    gp_Ax2 ax2(ax1R.Location(),ax1R.Direction());
                    Handle(Geom_Line) lineC = new Geom_Line(aLine);
                    GeomAPI_ProjectPointOnCurve PPC (ax1R.Location(),lineC);
                    gp_Cylinder cylind1(ax2,PPC.LowerDistance());
                    cylind = cylind1;
                    return true;
                }
            }
        }
        else if(GetCicle(Vcurve, cir))
        {
            if(GetLine(Ucurve,aLine))
            {
                if(aLine.Direction().IsParallel(ax1R.Direction(),tol))
                {
                    gp_Ax2 ax2(ax1R.Location(),ax1R.Direction());
                    Handle(Geom_Line) lineC = new Geom_Line(aLine);
                    GeomAPI_ProjectPointOnCurve PPC (ax1R.Location(),lineC);
                    gp_Cylinder cylind1(ax2,PPC.LowerDistance());
                    cylind = cylind1;
                    return true;
                }
            }
        }
    }
    else
    {
        Standard_Real LowerU=0;
        Standard_Real UpperU=0;
        Standard_Real LowerV=0;
        Standard_Real UpperV=0;
        aSurface->Bounds(LowerU, UpperU, LowerV, UpperV);
        Standard_Real nowU = 0;
        Standard_Real nowV = 0;
        if(LowerU != Precision::Infinite())
            nowU = LowerU;
        else if(UpperU != Precision::Infinite())
            nowU = UpperU;
        if(LowerV != Precision::Infinite())
            nowV = LowerV;
        else if(UpperV != Precision::Infinite())
            nowV = UpperV;
        Handle(Geom_Curve) Ucurve = aSurface->UIso(nowU);
        Handle(Geom_Curve) Vcurve = aSurface->VIso(nowV);
        gp_Circ cir;
        gp_Lin aLine;
        if((GetCicle(Ucurve, cir)&&GetLine(Vcurve,aLine))||(GetCicle(Vcurve, cir)&&GetLine(Ucurve,aLine)))
        {
            gp_Cylinder cylind1(cir.Position(),cir.Radius());
            cylind = cylind1;
            return true;
        }
    }
    return false;
}
bool GeneralTools::GetLine(Handle (Geom_Curve) anEdgeCurve,gp_Lin &aline)
{  
    GeomAdaptor_Curve  sanEdgeCurve(anEdgeCurve);
    if (sanEdgeCurve.GetType() == GeomAbs_Line )
    {
        aline = sanEdgeCurve.Line();
        return true;
    }
    else
    {
        Standard_Real FirstDummy = anEdgeCurve->FirstParameter();
        Standard_Real LastDummy = anEdgeCurve->LastParameter();
        TColgp_Array1OfPnt Pnts(1,100);
        for (int i=0;i<100;i++)
        {
            Standard_Real Upara = FirstDummy + (LastDummy - FirstDummy) / 99 * i;
            Pnts(i+1) = anEdgeCurve->Value(Upara);
        }
        GProp_PEquation gpe(Pnts, Precision::Confusion());
        if(gpe.IsLinear())
        {
            aline = gpe.Line();
            return true;
        }
    }
    return false;
}
bool GeneralTools::GetCone(Handle(Geom_Surface) aSurface,gp_Cone& aCone)
{
    double tol = 0.0001;
    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if (st == GeomAbs_Cone)
    {
        aCone =  sSurface.Cone();
        return true;
    }
    else
    {

        Standard_Real LowerU=0;
        Standard_Real UpperU=0;
        Standard_Real LowerV=0;
        Standard_Real UpperV=0;
        aSurface->Bounds(LowerU, UpperU, LowerV, UpperV);
        Standard_Real nowU = 0;
        Standard_Real nowV = 0;
        if(LowerU != Precision::Infinite())
            nowU = LowerU;
        else if(UpperU != Precision::Infinite())
            nowU = UpperU;
        if(LowerV != Precision::Infinite())
            nowV = LowerV;
        else if(UpperV != Precision::Infinite())
            nowV = UpperV;
        Handle(Geom_Curve) Ucurve = aSurface->UIso(nowU);
        Handle(Geom_Curve) Vcurve = aSurface->VIso(nowV);
        gp_Circ cir;
        gp_Lin aLine;
        gp_Ax1 ax1R;
        if(GetCicle(Ucurve, cir))
        {
            if(GetLine(Vcurve,aLine))
            {
                if(st == GeomAbs_SurfaceOfRevolution)
                    ax1R = sSurface.AxeOfRevolution();
                else
                    ax1R = cir.Axis();
                Standard_Real an = aLine.Direction().Angle(ax1R.Direction());
                if(an>tol)
                {
                    Standard_Real FirstDummy = Vcurve->FirstParameter();
                    Standard_Real LastDummy = Vcurve->LastParameter();
                    gp_Pnt P1,P2,P;
                    gp_Vec V;
                    Standard_Real R;
                    Vcurve->D0(FirstDummy,P1);
                    Vcurve->D0(LastDummy,P2);
                    Handle(Geom_Line) lineC = new Geom_Line(ax1R.Location(),ax1R.Direction());
                    GeomAPI_ProjectPointOnCurve PPC1 (P1,lineC);
                    GeomAPI_ProjectPointOnCurve PPC2 (P2,lineC);
                    if(PPC1.LowerDistance()>PPC2.LowerDistance())
                    {
                        P1 = PPC1.NearestPoint();
                        P2 = PPC2.NearestPoint();
                        P = P1;
                        V.SetCoord(P2.X()-P1.X(),P2.Y()-P1.Y(),P2.Z()-P1.Z());
                        R = PPC1.LowerDistance();
                    }
                    else
                    {
                        P1 = PPC1.NearestPoint();
                        P2 = PPC2.NearestPoint();
                        P = P2;
                        V.SetCoord(P1.X()-P2.X(),P1.Y()-P2.Y(),P1.Z()-P2.Z());
                        R = PPC2.LowerDistance();
                    }
                    gp_Ax2 ax2(P,V);
                    gp_Cone cone1(ax2,an,R);
                    aCone = cone1;
                    return true;
                }
            }
        }
        else if(GetCicle(Vcurve, cir))
        {
            if(GetLine(Ucurve,aLine))
            {
                if(st == GeomAbs_SurfaceOfRevolution)
                    ax1R = sSurface.AxeOfRevolution();
                else
                    ax1R = cir.Axis();
                Standard_Real an = aLine.Direction().Angle(ax1R.Direction());
                if(an>tol)
                {
                    Standard_Real FirstDummy = Ucurve->FirstParameter();
                    Standard_Real LastDummy = Ucurve->LastParameter();
                    gp_Pnt P1,P2,P;
                    gp_Vec V;
                    Standard_Real R;
                    Vcurve->D0(FirstDummy,P1);
                    Vcurve->D0(LastDummy,P2);
                    Handle(Geom_Line) lineC = new Geom_Line(ax1R.Location(),ax1R.Direction());
                    GeomAPI_ProjectPointOnCurve PPC1 (P1,lineC);
                    GeomAPI_ProjectPointOnCurve PPC2 (P2,lineC);
                    if(PPC1.LowerDistance()>PPC2.LowerDistance())
                    {
                        P1 = PPC1.NearestPoint();
                        P2 = PPC2.NearestPoint();
                        P = P1;
                        V.SetCoord(P2.X()-P1.X(),P2.Y()-P1.Y(),P2.Z()-P1.Z());
                        R = PPC1.LowerDistance();
                    }
                    else
                    {
                        P1 = PPC1.NearestPoint();
                        P2 = PPC2.NearestPoint();
                        P = P2;
                        V.SetCoord(P1.X()-P2.X(),P1.Y()-P2.Y(),P1.Z()-P2.Z());
                        R = PPC2.LowerDistance();
                    }
                    gp_Ax2 ax2(P,V);
                    gp_Cone cone1(ax2,an,R);
                    aCone = cone1;
                    return true;
                }
            }
        }
    }
    return false;
}
bool GeneralTools::GetPlane(TopoDS_Shape shape,gp_Pln &aPlane)
{
    TopoDS_Face aface;
    if(shape.ShapeType() == TopAbs_ShapeEnum::TopAbs_FACE)
        aface= TopoDS::Face(shape);
    else if(shape.ShapeType() == TopAbs_ShapeEnum::TopAbs_SHELL)
    {
        TopExp_Explorer Ex;
        for (Ex.Init(shape,TopAbs_FACE);Ex.More();Ex.Next())
        {
            aface = TopoDS::Face(Ex.Current());
            break;
        }
    }
    else
    {
        return false;
    }
    Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aface);
    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if (st == GeomAbs_Plane )
    {
        aPlane = sSurface.Plane();
        return true;
    }
    else if(JudgeShapePlane(shape,Precision::Confusion(),aPlane))
    {
        return true;
    }
    return false;
}
bool GeneralTools::GetEllips(Handle(Geom_Curve) anEdgeCurve,gp_Elips& aElips)
{
    GeomAdaptor_Curve  sanEdgeCurve(anEdgeCurve);
    if (sanEdgeCurve.GetType() == GeomAbs_Ellipse)
    {
        aElips = sanEdgeCurve.Ellipse();
        return true;
    }
    else
    {
        Standard_Real FirstDummy = anEdgeCurve->FirstParameter();
        Standard_Real LastDummy = anEdgeCurve->LastParameter();
        list<gp_Pnt> pts;
        for (int i=0;i<100;i++)
        {
            Standard_Real Upara = FirstDummy + (LastDummy - FirstDummy) / 99 * i;
            pts.push_back(anEdgeCurve->Value(Upara));
        }
        if(FitEllips(pts,aElips))
        {
            return true;
        }
    }
    return false;
}
bool GeneralTools::GetSphere(TopoDS_Shape shape,list<gp_Pnt> Pts,gp_Sphere& sphere)
{
    TopoDS_Face aface;
    if(shape.ShapeType() == TopAbs_ShapeEnum::TopAbs_FACE)
        aface= TopoDS::Face(shape);
    else if(shape.ShapeType() == TopAbs_ShapeEnum::TopAbs_SHELL)
    {
        TopExp_Explorer Ex;
        for (Ex.Init(shape,TopAbs_FACE);Ex.More();Ex.Next())
        {
            aface = TopoDS::Face(Ex.Current());
            break;
        }
    }
    else
    {
        return false;
    }
    Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aface);
    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if (st == GeomAbs_Sphere )
    {
        sphere = sSurface.Sphere();
        return true;
    }
    else if(st == GeomAbs_BSplineSurface || st == GeomAbs_SurfaceOfRevolution)
    {
        if(FitSphere(Pts,sphere))
            return true;
    }
    return false;
}

bool GeneralTools::GetAxis(const Handle(Geom_Surface)& aSurface, gp_Ax1& ax)
{
    gp_Cone cone;
    gp_Cylinder cylinder;
    if(GeneralTools::GetCone(aSurface, cone)) {
        ax = cone.Axis();
        return true;
    }else if(GeneralTools::GetCylinder(aSurface, cylinder)) {
        ax = cylinder.Axis();
        return true;
    }

    GeomAdaptor_Surface  sSurface(aSurface);
    GeomAbs_SurfaceType st = sSurface.GetType();
    if(st == GeomAbs_SurfaceOfRevolution) {
        ax = sSurface.AxeOfRevolution();
        return true;
    }

    return false;
}

bool GeneralTools::GetCenter(const Handle(Geom_Curve)& aCurve, gp_Ax2& ax2)
{
    gp_Circ circ;
    gp_Elips elips;
    if(GeneralTools::GetCicle(aCurve, circ)) {
        ax2 = circ.Position();
        return true;
    }
    else if(GeneralTools::GetEllips(aCurve, elips)) {
        ax2 = elips.Position();
        return true;
    }

    return false;
}

bool GeneralTools::GetShapeNormal(const TopoDS_Shape &shape, const gp_Pnt &p, gp_Dir &normal)
{
    if(shape.ShapeType()==TopAbs_EDGE)
    {
        Standard_Real first, last;
        Handle(Geom_Curve) aCurve  = BRep_Tool::Curve(TopoDS::Edge(shape),first,last);
        gp_Lin lin; gp_Ax2 axis;
        bool ret1 = GetLine(aCurve, lin);
        bool ret2 = GetCenter(aCurve, axis);

        if(ret1) {
            normal = lin.Direction();
            return true;
        }
        else if(ret2) {
            gp_Dir axd = axis.Direction();
            GeomAPI_ProjectPointOnCurve  projectpt(p, aCurve);
            Standard_Real U = projectpt.LowerDistanceParameter();
            GeomLProp_CLProps  props(aCurve, U, 1, Precision::Confusion());
            gp_Dir tang;
            props.Tangent(tang);
            normal = tang.Crossed(axd);
            return true;
        }
        else
            return false;

        return true;
    }
    else if(shape.ShapeType()==TopAbs_FACE)
    {
        TopoDS_Face tmpFace = TopoDS::Face(shape);
        Handle_Geom_Surface  hSurface = BRep_Tool::Surface(tmpFace);
        GeomAPI_ProjectPointOnSurf  projectpt(p, hSurface);
        Standard_Real U = 0.0,  V = 0.0;
        projectpt.LowerDistanceParameters(U, V);
        GeomLProp_SLProps  props(hSurface, U, V, 1, Precision::Confusion());
        normal = props.Normal();
        if(tmpFace.Orientation() == TopAbs_REVERSED)
            normal = normal. Reversed();
        return true;
    }
    return false;
}
