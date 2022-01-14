#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QDockWidget>
#include <QToolBar>

#include <STEPCAFControl_Reader.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <AIS_Shape.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>

#include <IntCurvesFace_ShapeIntersector.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TopExp.hxx>
#include <GC_MakePlane.hxx>
#include <ProjLib.hxx>
#include <IntAna2d_AnaIntersection.hxx>
#include <ElCLib.hxx>
#include <GeomAPI_IntSS.hxx>

#include "Dialogs/ToleranceInput.h"
#include "Dialogs/DiamensionInput.h"
#include "Dialogs/DatumInput.h"
#include "Label/Label_Tolerance.h"
#include "Label/Label_Datum.h"
#include "Label/Label_Length.h"
#include "Label/Label_Radius.h"
#include "Label/Label_Diameter.h"
#include "Label/Label_Angle.h"
#include "Label/Label_Taper.h"
#include "OCCTool/PMIModel.h"
#include "OCCTool/GeneralTools.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initFunction();
    initToolBar();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initFunction()
{
    occWidget = new OccWidget(this);
    setCentralWidget(occWidget);

    connect(occWidget,&OccWidget::pickPixel,this,[=](int Xp ,int Yp) {
        Handle(AIS_InteractiveContext) context = occWidget->GetContext();
        TopoDS_Shape selected;
        for(context->InitSelected();context->MoreSelected();context->NextSelected()) {
            selected = occWidget->GetContext()->SelectedShape();
            break;
        }

        if(selected.IsNull())
            return;

        gp_Pnt ResultPoint;
        double X,Y,Z,VX,VY,VZ;
        occWidget->GetView()->ConvertWithProj(Xp,Yp,X,Y,Z,VX,VY,VZ);
        ResultPoint.SetCoord(X,Y,Z);
        gp_Lin eyeLin(ResultPoint,gp_Dir(VX,VY,VZ));

        //面
        if(selected.ShapeType() == TopAbs_COMPOUND ||
                selected.ShapeType() == TopAbs_FACE ||
                selected.ShapeType() == TopAbs_SHELL)
        {
            double minD = -1;
            gp_Pnt minP;
            IntCurvesFace_ShapeIntersector ICFSI;
            if(occWidget->GetContext()->SelectedShape().IsNull())
                return;
            ICFSI.Load(selected,Precision::Confusion());//Precision::Confusion()精度
            ICFSI.Perform(eyeLin,0,100000);
            if (ICFSI.IsDone())
            {
                for (int i=1;i<=ICFSI.NbPnt();i++)
                {
                    gp_Pnt Pi = ICFSI.Pnt(i);
                    double Dis = ResultPoint.Distance(Pi);
                    if (minD == -1 || minD>Dis)
                    {
                        minD = Dis;
                        minP = Pi;
                    }
                }
            }
            if (minD !=-1)
            {
                ResultPoint.SetCoord(minP.X(),minP.Y(),minP.Z());
            }
            else return;
        }
        //线
        else if(selected.ShapeType() == TopAbs_EDGE ||
                selected.ShapeType() == TopAbs_WIRE )
        {
            TopoDS_Edge targetEdge = TopoDS::Edge(occWidget->GetContext()->SelectedShape());
            Standard_Real first, last;
            Handle(Geom_Curve) aCurve  = BRep_Tool::Curve(targetEdge,first,last);
            TopoDS_Edge eyeEdge = BRepBuilderAPI_MakeEdge(eyeLin);//通过射线构造边
            Handle(Geom_Curve) eCurve  = BRep_Tool::Curve(eyeEdge,first,last);

            GeomAPI_ExtremaCurveCurve extCC(eCurve,aCurve);
            if(extCC.NbExtrema() == 0)
                return ;
            gp_Pnt pt1,pt2;
            extCC.NearestPoints(pt1,pt2);//求解曲线间最近点
            ResultPoint = pt2;
        }
        else return;

        if(requestPointOnPlane)
        {
            requestPointOnPlane = false;
            emit PointOnPlaneSelected(ResultPoint, context);
        }

        if(requestShape)
        {
            requestShape = false;

            if(pmiModel)
            {
                int index = pmiModel->FindShape(selected);
                if(!selected.IsNull())
                {
                    emit ShapeSelected(index, selected, ResultPoint);
                }
            }
        }
    });
}

void MainWindow::initToolBar()
{
    QToolBar *toolBar_view = new QToolBar(tr("View"),this);
    toolBar_view->setAllowedAreas(Qt::TopToolBarArea);

    QAction *act = new QAction(tr("Vertex"),this);
    connect(act,&QAction::triggered,this,[=](){
        occWidget->GetContext()->Deactivate();
        occWidget->GetContext()->Activate(TopAbs_VERTEX ,Standard_True);
    });
    toolBar_view->addAction(act);
    act = new QAction(tr("Edge"),this);
    connect(act,&QAction::triggered,this,[=](){
        occWidget->GetContext()->Deactivate();
        occWidget->GetContext()->Activate(TopAbs_SOLID ,Standard_True);
    });
    toolBar_view->addAction(act);
    act = new QAction(tr("Face"),this);
    connect(act,&QAction::triggered,this,[=](){
        occWidget->GetContext()->Deactivate();
        occWidget->GetContext()->Activate(TopAbs_FACE ,Standard_True);
    });
    toolBar_view->addAction(act);
    act = new QAction(tr("Shape"),this);
    connect(act,&QAction::triggered,this,[=](){
        occWidget->GetContext()->Deactivate();
        occWidget->GetContext()->Activate(TopAbs_SHAPE ,Standard_True);
    });
    toolBar_view->addAction(act);

    this->addToolBar(Qt::TopToolBarArea,toolBar_view);
}

void MainWindow::on_actionImport_triggered()
{
    QString modelFileName = QFileDialog::getOpenFileName(this,tr("Select Model"),"",tr("STP Files(*.step *.STEP *.stp *.STP));;"
                                                                                       "IGES Files(*.IGES *.IGS *.iges *.igs);;"
                                                                                       "BREP Files(*.brep *.brp)"));
    if(modelFileName.isEmpty())
        return;

    TCollection_AsciiString theAscii(modelFileName.toUtf8().data());

    QFileInfo info(modelFileName);
    std::shared_ptr<XSControl_Reader> aReader;
    if(info.suffix()=="step"||info.suffix()=="stp"||info.suffix()=="STEP"||info.suffix()=="STP")
    {
        aReader = std::make_shared<STEPControl_Reader>();
    }
    else if(info.suffix()=="iges"||info.suffix()=="igs"||info.suffix()=="IGES"||info.suffix()=="IGS")
    {
        aReader = std::make_shared<IGESControl_Reader>();
    }
    else if(info.suffix()=="brep"||info.suffix()=="brp")
    {
        TopoDS_Shape aShape;
        BRep_Builder aBuilder;
        BRepTools::Read(aShape,theAscii.ToCString(),aBuilder);
        pmiModel = new PMIModel(aShape);

        Handle(AIS_Shape) anAIS = new AIS_Shape(aShape);
        occWidget->GetContext()->Display(anAIS,false);
        return;
    }

    if(!aReader->ReadFile(theAscii.ToCString()))
    {
        QMessageBox::critical(this,tr("Error"),tr("Import failed!"));
        return;
    }

    if(aReader->TransferRoots() == 0)
    {
        QMessageBox::critical(this,tr("Error"),tr("Empty file!"));
        return;
    }

    pmiModel = new PMIModel(aReader->OneShape());
    Handle(AIS_Shape) anAIS = new AIS_Shape(aReader->OneShape());
    anAIS->Attributes()->SetFaceBoundaryDraw(true);
    anAIS->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, 1.));
    anAIS->Attributes()->SetIsoOnTriangulation(true);
    occWidget->GetContext()->SetColor(anAIS,Quantity_NOC_GRAY80,Standard_False);
    occWidget->GetContext()->Display(anAIS,false);
    occWidget->GetView()->FitAll();
}

void MainWindow::on_actionAdd_Tolerence_triggered()
{
    if(existPMIDock)
        return;
    QDockWidget* tolDock = new QDockWidget(tr("Add tollerance"),this);
    tolDock->setObjectName("Add tollerance");
    tolDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    this->addDockWidget(Qt::RightDockWidgetArea,tolDock);
    existPMIDock = true;

    ToleranceInput* anInput = new ToleranceInput(tolDock);
    connect(anInput,&ToleranceInput::labelEditFinish,this,&MainWindow::on_addTolLabel);
    connect(anInput,&ToleranceInput::requestSelectShape,this,[=]() {
        requestShape = true;
    });
    connect(anInput,&ToleranceInput::requestPointOnPlane,this,[=]() {
        requestPointOnPlane = true;
    });
    connect(anInput,&ToleranceInput::readyToClose,this,[=]() {
        this->removeDockWidget(tolDock);
        tolDock->deleteLater();
    });
    connect(tolDock,&QDockWidget::visibilityChanged,this,[=](bool visual){
        existPMIDock = visual;
    });
    connect(this,&MainWindow::ShapeSelected,anInput,&ToleranceInput::SetBindShape,Qt::UniqueConnection);
    connect(this,&MainWindow::PointOnPlaneSelected,anInput,&ToleranceInput::SetPointOnPlane,Qt::UniqueConnection);

    tolDock->setWidget(anInput);
}

void MainWindow::on_actionAdd_Dimension_triggered()
{
    if(existPMIDock)
        return;

    QDockWidget* diamensionDock = new QDockWidget(tr("Add Diamension"),this);
    diamensionDock->setObjectName("Add Diamension");
    diamensionDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    this->addDockWidget(Qt::RightDockWidgetArea,diamensionDock);
    existPMIDock = true;

    DiamensionInput* aDlg = new DiamensionInput(diamensionDock);
    connect(aDlg,&DiamensionInput::labelEditFinish,this,&MainWindow::on_addDiamensionLabel);
    connect(aDlg,&DiamensionInput::requestSelectShape,this,[=]() {
        requestShape = true;
    });
    connect(aDlg,&DiamensionInput::readyToClose,this,[=]() {
        this->removeDockWidget(diamensionDock);
        diamensionDock->deleteLater();
    });
    connect(diamensionDock,&QDockWidget::visibilityChanged,this,[=](bool visual){
        existPMIDock = visual;
    });
    connect(this,&MainWindow::ShapeSelected,aDlg,&DiamensionInput::SetBindShape,Qt::UniqueConnection);

    diamensionDock->setWidget(aDlg);
}

void MainWindow::on_actionAdd_Datum_triggered()
{
    if(existPMIDock)
        return;

    QDockWidget* datumDock = new QDockWidget(tr("Add Datum"),this);
    datumDock->setObjectName("Add Datum");
    datumDock->setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);
    this->addDockWidget(Qt::RightDockWidgetArea,datumDock);
    existPMIDock = true;

    DatumInput* aDlg = new DatumInput(datumDock);
    connect(aDlg,&DatumInput::labelEditFinish,this,&MainWindow::on_addDatumLabel);
    connect(aDlg,&DatumInput::requestSelectShape,this,[=]() {
        requestShape = true;
    });
    connect(aDlg,&DatumInput::readyToClose,this,[=]() {
        this->removeDockWidget(datumDock);
        datumDock->deleteLater();
    });
    connect(datumDock,&QDockWidget::visibilityChanged,this,[=](bool visual){
        existPMIDock = visual;
    });
    connect(this,&MainWindow::ShapeSelected,aDlg,&DatumInput::SetBindShape,Qt::UniqueConnection);

    datumDock->setWidget(aDlg);
}

void MainWindow::on_addTolLabel(const NCollection_Utf8String &tolName,
                                const NCollection_Utf8String &tolVal,
                                const NCollection_Utf8String &tolVal2,
                                const QList<NCollection_Utf8String> &baseName,
                                const TopoDS_Shape& shape,
                                const gp_Pln& place,
                                const gp_Pnt& touch)
{
    Handle(Label_Tolerance) aLabel = new Label_Tolerance();
    aLabel->SetData(tolName,tolVal,tolVal2,baseName);
    Handle(AIS_Shape) pMd = new AIS_Shape(pmiModel->GetOriginShape());
    Bnd_Box box = pMd->BoundingBox();

    gp_Dir direc;
    GeneralTools::GetShapeNormal(shape,touch,direc);

    gp_Pnt target = targetWithBox(touch,direc,box);

    gp_Ax2 oriention;
    oriention.SetLocation(target);
    oriention.SetDirection(direc);

    aLabel->SetPosture(touch,oriention);
    occWidget->GetContext()->Display(aLabel, Standard_True);
}

void MainWindow::on_addDiamensionLabel(const QList<NCollection_Utf8String> &valList,
                                       const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
                                       const gp_Pnt &touch1, const gp_Pnt &touch2,
                                       const gp_Pln& place, int type)
{
    Handle(AIS_Shape) pMd = new AIS_Shape(pmiModel->GetOriginShape());
    Bnd_Box box = pMd->BoundingBox();
    switch(type)
    {
    //尺寸
    case 0:{
        if(shape1.ShapeType() == TopAbs_EDGE) {
            TopoDS_Vertex vertex1, vertex2;
            TopExp::Vertices (TopoDS::Edge (shape1), vertex1, vertex2);
            gp_Pnt p1 = BRep_Tool::Pnt (vertex1);
            gp_Pnt p2 = BRep_Tool::Pnt (vertex2);

            gp_Dir normal = place.Axis().Direction().Reversed();
            gp_Dir lin(p2.XYZ()-p1.XYZ());
            if(!lin.IsNormal(normal, 1e-6)) {
                QMessageBox::critical(this,"错误","所选放置面法向不与直线垂直!");
                return;
            }

            gp_Pnt mid = 0.5*(p1.XYZ() + p2.XYZ());
            gp_Pnt target = targetWithBox(mid,normal,box);
            gp_Ax2 oriention(target, lin^normal);
            oriention.SetYDirection(normal);

            Handle(Label_Length) aLabel = new Label_Length(valList, p1,p2,oriention);
            occWidget->GetContext()->Display(aLabel, Standard_True);
            return;
        }
        break;
    }
        //距离
    case 1:{
        measureLength(box, valList, shape1, shape2);
        return;
    }
        //角度
    case 2:{
        measureAngle(valList, shape1, shape2, touch1, touch2);
        return;
    }
        //直径
    case 3:{
        if(shape1.ShapeType() == TopAbs_EDGE) {
            BRep_Tool bpt;
            double a,b;
            Handle(Geom_Curve) gc =bpt.Curve(TopoDS::Edge(shape1),a,b);
            gp_Circ circle;
            if(GeneralTools::GetCicle(gc,circle)) {
                gp_Dir direc = touch1.XYZ() - circle.Location().XYZ();
                gp_Pnt target = targetWithBox(touch1, direc, box);
                gp_Ax2 oriention(target, circle.Axis().Direction(), direc);
                // 偏移避免遮挡
                gp_Dir pan = circle.Axis().Direction();
                circle.Translate(0.01*pan);
                oriention.Translate(0.01*pan);

                Handle(Label_Diameter) aLabel = new Label_Diameter(valList, circle, oriention);
                occWidget->GetContext()->Display(aLabel, true);
                return;
            }
        }
        break;
    }
        //半径
    case 4:{
        if(shape1.ShapeType() == TopAbs_EDGE) {
            BRep_Tool bpt;
            double a,b;
            Handle(Geom_Curve) gc =bpt.Curve(TopoDS::Edge(shape1),a,b);
            gp_Circ circle;
            if(GeneralTools::GetCicle(gc,circle)) {
                gp_Dir direc = touch1.XYZ() - circle.Location().XYZ();
                gp_Pnt target = targetWithBox(touch1, direc, box);
                gp_Ax2 oriention(target, circle.Axis().Direction(), direc);
                // 偏移避免遮挡
                gp_Dir pan = circle.Axis().Direction();
                circle.Translate(0.01*pan);
                oriention.Translate(0.01*pan);

                Handle(Label_Radius) aLabel = new Label_Radius(valList, circle, oriention);
                occWidget->GetContext()->Display(aLabel, true);
                return;
            }
        }
        break;
    }
        //锥度
    case 5:{
        if(shape1.ShapeType() == TopAbs_FACE) {
            Handle(Geom_Surface) face = BRep_Tool::Surface(TopoDS::Face(shape1));
            gp_Cone cone;
            if(GeneralTools::GetCone(face, cone)) {
                gp_Lin center = cone.Axis();
                GeomAPI_ProjectPointOnCurve PPOC(touch1, new Geom_Line(center));
                gp_Pnt pc = PPOC.NearestPoint();
                gp_Dir direc = touch1.XYZ() - pc.XYZ();
                gp_Pnt target = targetWithBox(touch1, direc, box);

                gp_Dir dvx = center.Direction();
                gp_Dir normal = dvx ^ direc;
                gp_Ax2 oriention(target, normal, dvx);

                Handle(Label_Taper) aLabel = new Label_Taper(valList[0], touch1, oriention);
                occWidget->GetContext()->Display(aLabel, true);
                return;
            }
            else {
                QMessageBox::critical(this,"错误","仅支持锥面的锥度标注!");
                return;
            }
        }
    }
    }
    QMessageBox::critical(this,"错误","所选类型暂不支持!");
}

void MainWindow::on_addDatumLabel(const QString &str, const TopoDS_Shape& shape, const gp_Pln &place, const gp_Pnt& touch)
{
    Handle(Label_Datum) aLabel = new Label_Datum();
    aLabel->SetDatumName(str.toStdString().data());

    Handle(AIS_Shape) pMd = new AIS_Shape(pmiModel->GetOriginShape());
    Bnd_Box box = pMd->BoundingBox();
    gp_Pnt origin = touch;

    // 1.normal at touch point, set as label's Y axis
    gp_Dir direc;
    if(!GeneralTools::GetShapeNormal(shape,origin,direc)) {
        QMessageBox::critical(this,"错误","所选形状不能作为基准!");
        return;
    }

    // 2. reset the touch point if shape is circle/cylinder...
    // and get the place location out of bounding box
    if(shape.ShapeType() == TopAbs_EDGE) {
        double a,b;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(shape),a,b);
        gp_Ax2 ax2;
        if(GeneralTools::GetCenter(curve, ax2)) {
            origin = ax2.Location();
            direc = ax2.Direction();
        }
        else {
            origin = curve->Value(b);//直线从一个端点引出
        }
    }
    else if(shape.ShapeType() == TopAbs_FACE) {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(shape));
        gp_Ax1 ax1;
        if(GeneralTools::GetAxis(surface,ax1)) {
            gp_Lin lin(ax1);
            Handle(Geom_Line) line  = new Geom_Line(lin);
            GeomAPI_ProjectPointOnCurve  ppc(touch, line);
            origin = ppc.NearestPoint();
            direc = ax1.Direction();
        }
    }
    gp_Pnt target = targetWithBox(origin,direc,box);

    // 3.normal of label plane, set as label's Z axis
    gp_Dir normal = place.Axis().Direction();
    if(direc.IsParallel(normal, 1e-6)) {
        QMessageBox::critical(this,"错误","放置面法向与基准元素法向一致!");
        return;
    }

    if(!direc.IsNormal(normal, 1e-6)) {
        QMessageBox::critical(this,"错误","放置面法向与基准元素法向不垂直!");
        return;
    }

    // 4.label's X axis
    gp_Dir VX = direc.Crossed(normal);

    // 5.offset the target position by X axis
    target.Translate(-0.5*aLabel->StrWidth()*VX);

    // 6. the placement of label
    gp_Ax2 oriention(target,normal,VX);

    aLabel->SetTouchPoint(origin);
    aLabel->SetOriention(oriention);
    occWidget->GetContext()->Display(aLabel, Standard_True);
}

gp_Pnt MainWindow::targetWithBox(const gp_Pnt &input, const gp_Dir &dir, const Bnd_Box &box)
{
    gp_Pnt target;
    double lx,ly,lz,ux,uy,uz;
    box.Get(lx,ly,lz,ux,uy,uz);
    gp_Pnt lp(lx,ly,lz);
    gp_Pnt up(ux,uy,uz);

    gp_Pnt border;
    IntCurvesFace_ShapeIntersector ICFSI;
    ICFSI.Load(BRepPrimAPI_MakeBox(lp,up),Precision::Confusion());
    ICFSI.Perform(gp_Lin(input,dir),0,100000);
    if (ICFSI.IsDone()) {
        border = ICFSI.Pnt(1);
    }

    double dif = input.Distance(border);
    if(dif < 15)
        dif = 15;

    target = input.XYZ() + 1.2*dif*dir.XYZ();
    return target;
}

void MainWindow::measureLength(const Bnd_Box &box, const QList<NCollection_Utf8String> &valList,
                               const TopoDS_Shape &shape1, const TopoDS_Shape &shape2)
{
    Handle(Label_Length) aLabel;
    if(shape1.ShapeType() == TopAbs_FACE && shape2.ShapeType() == TopAbs_FACE) {
        Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(shape1));
        Handle(Geom_Surface) face2 = BRep_Tool::Surface(TopoDS::Face(shape2));

        gp_Ax1 axis1, axis2;
        gp_Pln pln1, pln2;
        bool reta1 = GeneralTools::GetAxis(face1,axis1);
        bool reta2 = GeneralTools::GetAxis(face2,axis2);
        bool retb1 = GeneralTools::GetPlane(shape1,pln1);
        bool retb2 = GeneralTools::GetPlane(shape2,pln2);

        //两个旋转面
        if(reta1 && reta2) {
            if(!axis1.IsParallel(axis2, 1e-6)) {
                QMessageBox::critical(this,"错误","两面不平行!");
                return;
            }

            if(gp_Lin(axis1).Distance(gp_Lin(axis2)) < 1e-6) {
                QMessageBox::critical(this,"错误","两转轴重合!");
                return;
            }

            if(!axis1.IsParallel(axis2, 1e-6)) {
                QMessageBox::critical(this,"错误","两转不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, axis2, p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //两个平面
        else if(retb1 && retb2){
            if(pln1.Distance(pln2) < 1e-6) {
                QMessageBox::critical(this,"错误","两面不平行!");
                return;
            }

            TopExp_Explorer exp;
            exp.Init(shape1, TopAbs_EDGE);
            Standard_Real low, up;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(exp.Value()), low, up);
            gp_Pnt plnPt = curve->Value(low);
            gp_Ax1 plnAxis1(plnPt, pln1.YAxis().Direction());
            GeomAPI_ProjectPointOnSurf PPOS(plnPt,face2);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis2(np,plnAxis1.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis1, plnAxis2, p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //1是旋转面，2是平面
        else if(reta1 && retb2) {
            if(pln2.Distance(gp_Lin(axis1)) < 1e-6) {
                QMessageBox::critical(this,"错误","平面与转轴重合!");
                return;
            }

            if(!pln2.Axis().IsNormal(axis1, 1e-6)) {
                QMessageBox::critical(this,"错误","平面与转轴不平行!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(axis1.Location(),face2);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,axis1.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, plnAxis, p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //1是平面2是旋转面
        else if(reta2 && retb1){
            if(pln1.Distance(gp_Lin(axis2)) < 1e-6) {
                QMessageBox::critical(this,"错误","平面与转轴重合!");
                return;
            }

            if(!pln1.Axis().IsNormal(axis2, 1e-6)) {
                QMessageBox::critical(this,"错误","平面与转轴不平行!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(axis2.Location(),face1);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,axis2.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis, axis2, p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
    }
    else if(shape1.ShapeType() == TopAbs_FACE && shape2.ShapeType() == TopAbs_EDGE) {
        Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(shape1));
        double a,b;
        Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(shape2),a,b);

        gp_Ax1 axis1;gp_Pln pln1;
        gp_Lin lin2;gp_Ax2 ax2;
        bool reta1 = GeneralTools::GetAxis(face1,axis1);
        bool reta2 = GeneralTools::GetLine(curve2,lin2);
        bool retb1 = GeneralTools::GetPlane(shape1,pln1);
        bool retb2 = GeneralTools::GetCenter(curve2,ax2);
        //平面和直线
        if(retb1 && reta2) {
            if(pln1.Distance(lin2) < 1e-6) {
                QMessageBox::critical(this,"错误","直线在平面上!");
                return;
            }

            if(!pln1.Axis().IsNormal(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线不与平面垂直!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(lin2.Location(),face1);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,lin2.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis, lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //平面和圆弧
        else if(retb1 && retb2) {
            if(!pln1.Axis().IsNormal(ax2.Axis(), 1e-6) && !pln1.Axis().IsParallel(ax2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","平面与转轴不平行!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(ax2.Location(),face1);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,ax2.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis, ax2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //旋转面和直线
        else if(reta1 && reta2) {
            if(gp_Lin(axis1).Distance(lin2) < 1e-6) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            if(!axis1.IsParallel(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //旋转面和圆弧
        else if(reta1 && retb2) {
            if(gp_Lin(axis1).Distance(gp_Lin(ax2.Axis())) < 1e-6) {
                QMessageBox::critical(this,"错误","面转轴与弧转轴重合!");
                return;
            }

            if(!axis1.IsParallel(ax2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, ax2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
    }
    else if(shape2.ShapeType() == TopAbs_FACE && shape1.ShapeType() == TopAbs_EDGE) {
        Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(shape2));
        double a,b;
        Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(shape1),a,b);

        gp_Ax1 axis1;gp_Pln pln1;
        gp_Lin lin2;gp_Ax2 ax2;
        bool reta1 = GeneralTools::GetAxis(face1,axis1);
        bool reta2 = GeneralTools::GetLine(curve2,lin2);
        bool retb1 = GeneralTools::GetPlane(shape2,pln1);
        bool retb2 = GeneralTools::GetCenter(curve2,ax2);
        //平面和直线
        if(retb1 && reta2) {
            if(pln1.Distance(lin2) < 1e-6) {
                QMessageBox::critical(this,"错误","直线在平面上!");
                return;
            }

            if(!pln1.Axis().IsNormal(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线不与平面垂直!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(lin2.Location(),face1);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,lin2.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis, lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //平面和圆弧
        else if(retb1 && retb2) {
            if(!pln1.Axis().IsNormal(ax2.Axis(), 1e-6) && !pln1.Axis().IsParallel(ax2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","平面与圆弧不平行!");
                return;
            }

            GeomAPI_ProjectPointOnSurf PPOS(ax2.Location(),face1);
            gp_Pnt np = PPOS.NearestPoint();
            gp_Ax1 plnAxis(np,ax2.Direction());
            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, plnAxis, ax2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //旋转面和直线
        else if(reta1 && reta2) {
            if(gp_Lin(axis1).Distance(lin2) < 1e-6) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            if(!axis1.IsParallel(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //旋转面和圆弧
        else if(reta1 && retb2) {
            if(gp_Lin(axis1).Distance(gp_Lin(ax2.Location(),ax2.Direction())) < 1e-6) {
                QMessageBox::critical(this,"错误","面转轴与圆弧转轴不平行!");
                return;
            }

            if(!axis1.IsParallel(ax2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1, ax2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
    }
    else if(shape1.ShapeType() == TopAbs_EDGE && shape2.ShapeType() == TopAbs_EDGE) {
        double a,b,c,d;
        Handle(Geom_Curve) curve1 = BRep_Tool::Curve(TopoDS::Edge(shape1),a,b);
        Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(shape2),c,d);
        gp_Pnt p1,p2,p3,p4;
        curve1->D0(a,p1);curve1->D0(b,p2);curve2->D0(c,p3);curve2->D0(d,p4);

        gp_Ax2 axis1, axis2;
        gp_Lin lin1, lin2;
        bool ret1 = GeneralTools::GetLine(curve1,lin1);
        bool ret2 = GeneralTools::GetLine(curve2,lin2);
        bool ret3 = GeneralTools::GetCenter(curve1,axis1);
        bool ret4 = GeneralTools::GetCenter(curve2,axis2);

        // 两条线段
        if(ret1 && ret2) {
            if(lin1.Distance(lin2) < 1e-6) {
                QMessageBox::critical(this,"错误","两直线相交!");
                return;
            }

            if(!lin1.Position().IsParallel(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","两直线不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, lin1.Position(), lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //1线段2圆弧
        else if(ret1 && ret4) {
            if(lin1.Distance(gp_Lin(axis2.Axis())) < 1e-6) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            if(!lin1.Position().IsParallel(axis2.Axis(), 1e-6) && !lin1.Position().IsNormal(axis2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, lin1.Position(), axis2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //1圆弧2线段
        else if(ret3 && ret2) {
            if(lin2.Distance(gp_Lin(axis1.Axis())) < 1e-6) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            if(!lin2.Position().IsParallel(axis1.Axis(), 1e-6) && !lin2.Position().IsNormal(axis1.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1.Axis(), lin2.Position(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
        //两条圆弧
        else if(ret3 && ret4) {
            gp_Lin lct1(axis1.Axis());
            gp_Lin lct2(axis2.Axis());
            if(lct1.Distance(lct2) < 1e-6) {
                QMessageBox::critical(this,"错误","两个转轴不平行!");
                return;
            }

            if(!axis1.Axis().IsParallel(axis2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","两个转轴不平行!");
                return;
            }

            gp_Pnt p1 ,p2;gp_Ax2 oriention;
            lengthOfTwoAxis(box, axis1.Axis(), axis2.Axis(), p1, p2, oriention);
            aLabel = new Label_Length(valList, p1,p2,oriention);
        }
    }

    if(aLabel.IsNull()) {
        QMessageBox::critical(this,"错误","不支持的距离类型!");
        return;
    }

    occWidget->GetContext()->Display(aLabel, Standard_True);
}

void MainWindow::measureAngle(const QList<NCollection_Utf8String> &valList,
                              const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
                              const gp_Pnt &touch1, const gp_Pnt &touch2)
{
    Handle(Label_Angle) aLabel;
    if(shape1.ShapeType() == TopAbs_EDGE && shape2.ShapeType() == TopAbs_EDGE) {
        gp_Lin lin1,lin2;
        BRep_Tool bpt;
        double a,b,c,d;
        Handle(Geom_Curve) cva =bpt.Curve(TopoDS::Edge(shape1),a,b);
        Handle(Geom_Curve) cvb =bpt.Curve(TopoDS::Edge(shape2),c,d);
        bool ret1 = GeneralTools::GetLine(cva,lin1);
        bool ret2 = GeneralTools::GetLine(cvb,lin2);
        if(ret1 && ret2) {
            if(lin1.Position().IsParallel(lin2.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","两直线平行!");
                return;
            }

            if(lin1.Distance(lin2) > 1e-6) {
                QMessageBox::critical(this,"错误","两直线异面!");
                return;
            }

            gp_Pnt center = intersectionOfLines(lin1, lin2);

            aLabel = new Label_Angle(valList,
                                     touch1, center, touch2);
            gp_Trsf pan;
            gp_Dir normal = (touch1.XYZ()-center.XYZ()) ^ (touch2.XYZ()-center.XYZ());
            pan.SetTranslation(0.01*normal);
            aLabel->SetLocalTransformation(pan);
        }
        else {
            QMessageBox::critical(this,"错误","所选类型不能计算角度!");
            return;
        }
    }
    else if(shape1.ShapeType() == TopAbs_FACE && shape2.ShapeType() == TopAbs_FACE) {
        Handle(Geom_Surface) surface1 = BRep_Tool::Surface(TopoDS::Face(shape1));
        Handle(Geom_Surface) surface2 = BRep_Tool::Surface(TopoDS::Face(shape2));
        gp_Pln pln1, pln2;gp_Ax1 axis1, axis2;
        bool ret1 = GeneralTools::GetPlane(shape1,pln1);
        bool ret2 = GeneralTools::GetPlane(shape2,pln2);

        //两个平面
        if(ret1 && ret2) {
            if(pln1.Axis().IsParallel(pln2.Axis(), 1e-6)) {
                QMessageBox::critical(this,"错误","两平面平行!");
                return;
            }

            GeomAPI_IntSS ISS(surface1, surface2, 1e-6);
            Handle(Geom_Curve) curve = ISS.Line(1);

            GeomAPI_ProjectPointOnCurve PPOC(touch1, curve);
            gp_Pnt center = PPOC.NearestPoint();

            gp_Lin inter;
            GeneralTools::GetLine(curve, inter);
            Handle(Geom_Plane) tmp = GC_MakePlane(inter.Position());
            tmp->SetLocation(center);
            GeomAPI_ProjectPointOnSurf PPOS(touch2, tmp);
            gp_Pnt pcs = PPOS.NearestPoint();

            aLabel = new Label_Angle(valList, touch1, center, pcs);
        }
        else {
            QMessageBox::critical(this,"错误","所选类型不能计算角度!");
            return;
        }
    }
    else if(shape1.ShapeType() == TopAbs_FACE && shape2.ShapeType() == TopAbs_EDGE) {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(shape1));
        double a,b;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(shape2),a,b);

        gp_Ax1 axis; gp_Lin lin;
        bool ret1 = GeneralTools::GetAxis(surface,axis);
        bool ret2 = GeneralTools::GetLine(curve,lin);
        if(ret1 && ret2) {
            if(axis.IsParallel(lin.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与轴线平行!");
                return;
            }

            if(lin.Distance(gp_Lin(axis)) > 1e-6) {
                QMessageBox::critical(this,"错误","直线与轴线异面!");
                return;
            }

            gp_Lin linf(axis);
            gp_Pnt center = intersectionOfLines(linf, lin);
            GeomAPI_ProjectPointOnCurve PPOC(touch1, new Geom_Line(linf));
            gp_Pnt pax = PPOC.NearestPoint();
            aLabel = new Label_Angle(valList, touch2, center, pax);
        }
        else {
            QMessageBox::critical(this,"错误","所选类型不能计算角度!");
            return;
        }
    }
    else if(shape2.ShapeType() == TopAbs_FACE && shape1.ShapeType() == TopAbs_EDGE) {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(shape2));
        double a,b;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(shape1),a,b);

        gp_Ax1 axis; gp_Lin lin;
        bool ret1 = GeneralTools::GetAxis(surface,axis);
        bool ret2 = GeneralTools::GetLine(curve,lin);
        if(ret1 && ret2) {
            if(axis.IsParallel(lin.Position(), 1e-6)) {
                QMessageBox::critical(this,"错误","直线与轴线平行!");
                return;
            }

            if(lin.Distance(gp_Lin(axis)) > 1e-6) {
                QMessageBox::critical(this,"错误","直线与轴线异面!");
                return;
            }

            gp_Lin linf(axis);
            gp_Pnt center = intersectionOfLines(linf, lin);
            GeomAPI_ProjectPointOnCurve PPOC(touch2, new Geom_Line(linf));
            gp_Pnt pax = PPOC.NearestPoint();
            aLabel = new Label_Angle(valList, touch1, center, pax);
        }
        else {
            QMessageBox::critical(this,"错误","所选类型不能计算角度!");
            return;
        }
    }

    if(aLabel.IsNull()) {
        QMessageBox::critical(this,"错误","不支持的距离类型!");
        return;
    }

    occWidget->GetContext()->Display(aLabel, Standard_True);
}

void MainWindow::lengthOfTwoAxis(const Bnd_Box &box, const gp_Ax1 &ax1, const gp_Ax1 &ax2, gp_Pnt &first, gp_Pnt &second, gp_Ax2 &oriention)
{
    gp_Pnt p1 = ax1.Location();
    gp_Pnt p2 = ax2.Location();
    gp_Dir pp(p1.XYZ()-p2.XYZ());
    gp_Dir c2 = ax2.Direction();
    gp_Dir normal = c2.Crossed(pp);

    Handle(Geom_Line) line = new Geom_Line(gp_Lin(ax2));
    GeomAPI_ProjectPointOnCurve PPC(p1,line);

    gp_Pnt p3 = PPC.NearestPoint();

    gp_Pnt mid = 0.5*(p1.XYZ() + p3.XYZ());
    gp_Pnt target = targetWithBox(mid,c2,box);
    first = p1;
    second = p3;
    oriention = gp_Ax2(target, normal, p3.XYZ()-p1.XYZ());
}

gp_Pnt MainWindow::intersectionOfLines(const gp_Lin &lin1, const gp_Lin &lin2)
{
    gp_Pln plane = gp_Pln (lin2.Location(), gp_Vec (lin1.Direction()) ^ gp_Vec (lin2.Direction()));
    // Find intersection
    gp_Lin2d aFirstLin2d  = ProjLib::Project (plane, lin1);
    gp_Lin2d aSecondLin2d = ProjLib::Project (plane, lin2);

    IntAna2d_AnaIntersection anInt2d (aFirstLin2d, aSecondLin2d);
    gp_Pnt2d anIntersectPoint = gp_Pnt2d (anInt2d.Point(1).Value());
    gp_Pnt center = ElCLib::To3d (plane.Position().Ax2(), anIntersectPoint);
    return center;
}
