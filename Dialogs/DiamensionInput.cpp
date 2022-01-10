#include "DiamensionInput.h"
#include "ui_DiamensionInput.h"

#include <QDoubleValidator>
#include <QDebug>
#include <QMessageBox>

#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <GC_MakePlane.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

#include "TolStringInfo.h"
#include "OCCTool/GeneralTools.h"

DiamensionInput::DiamensionInput(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DiamensionInput)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    QDoubleValidator *validator = new QDoubleValidator(this);
    validator->setRange(-999,999,4);
    ui->lineEdit_mainVal->setValidator( validator );
    ui->lineEdit_upVal->setValidator( validator );
    ui->lineEdit_lowVal->setValidator( validator );
}

DiamensionInput::~DiamensionInput()
{
    delete ui;
}

void DiamensionInput::on_pushButton_selectEle1_clicked()
{
    selectPlace = false;
    emit requestSelectShape();
    shapeIndex = 1;
}

void DiamensionInput::on_pushButton_selectEle2_clicked()
{
    selectPlace = false;
    emit requestSelectShape();
    shapeIndex = 2;
}

void DiamensionInput::on_pushButton_sure_clicked()
{
    if(myBindShape1.IsNull() && myBindShape2.IsNull()) {
        QMessageBox::critical(this,"错误","未选择标注元素!");
        return;
    }

    if(ui->stackedWidget_reference->currentIndex() == 1 && ui->lineEdit_place->text().isEmpty()) {
        QMessageBox::critical(this,"错误","未选择放置平面!");
        return;
    }

    
	
	QString subStr = ui->lineEdit_lowVal->text();
	QString supStr = ui->lineEdit_upVal->text();
	
    if(subStr.toDouble() > 0 && !subStr.contains('+')) {
        subStr.prepend('+');
	}
    if(supStr.toDouble() > 0 && !supStr.contains('+')) {
        supStr.prepend('+');
	}

	mainVal = ui->lineEdit_mainVal->text().toStdString().data();
	upVal = supStr.toStdString().data();
    lowVal = subStr.toStdString().data();
	
    if(mainVal.IsEmpty()) {
        QMessageBox::critical(this,"错误","未输入主尺寸!");
        return;
    }

    emit labelEditFinish({mainVal,upVal,lowVal},
                         myBindShape1, myBindShape2,
                         myTouch1, myTouch2,
                         myPlace, diamensionType);
    this->close();
    emit readyToClose();
}

void DiamensionInput::on_pushButton_cancle_clicked()
{
    this->close();
    emit readyToClose();
}

void DiamensionInput::SetBindShape(int index, const TopoDS_Shape &shape, const gp_Pnt &touch)
{
    QString content;
    if(shape.ShapeType() == TopAbs_FACE)
    {
        content += "F ";
        content += QString::number(index);
    }
    else if(shape.ShapeType() == TopAbs_EDGE)//wire
    {
        content += "E ";
        content += QString::number(index);
    }
    else if(shape.ShapeType() == TopAbs_VERTEX)
    {
        gp_Pnt P = BRep_Tool::Pnt(TopoDS::Vertex(shape));
        content += QString::number(P.X(),'f',2);
        content += ",";
        content += QString::number(P.Y(),'f',2);
        content += ",";
        content += QString::number(P.Z(),'f',2);
    }

    if(!selectPlace) {
        //长度
        if(ui->comboBox_measureType->currentIndex() == 0) {
            if(shape.ShapeType() != TopAbs_EDGE) {
                QMessageBox::critical(this,"错误","仅支持直线段长度!");
                return;
            }
            BRep_Tool bpt;
            double a,b;
            Handle(Geom_Curve) gc =bpt.Curve(TopoDS::Edge(shape),a,b);
            gp_Lin alin;
            if(!GeneralTools::GetLine(gc,alin)) {
                QMessageBox::critical(this,"错误","仅支持直线段长度!");
                return;
            }
            TopoDS_Vertex vertex1, vertex2;
            TopExp::Vertices (TopoDS::Edge (shape), vertex1, vertex2);
            gp_Pnt p1 = BRep_Tool::Pnt (vertex1);
            gp_Pnt p2 = BRep_Tool::Pnt (vertex2);
            ui->lineEdit_mainVal->setText(QString::number(p1.Distance(p2)));
        }
        //距离
        else if(ui->comboBox_measureType->currentIndex() == 1) {
            TopoDS_Shape last;
            if(shapeIndex == 1) {
                last = myBindShape2;
            }
            else if(shapeIndex == 2) {
                last = myBindShape1;
            }
            if(!last.IsNull()) {
                if(last.ShapeType() == TopAbs_FACE && shape.ShapeType() == TopAbs_FACE) {
                    Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(last));
                    Handle(Geom_Surface) face2 = BRep_Tool::Surface(TopoDS::Face(shape));

                    gp_Ax1 axis1, axis2;
                    gp_Pln pln1, pln2;
                    bool reta1 = GeneralTools::GetAxis(face1,axis1);
                    bool reta2 = GeneralTools::GetAxis(face2,axis2);
                    bool retb1 = GeneralTools::GetPlane(last,pln1);
                    bool retb2 = GeneralTools::GetPlane(shape,pln2);

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

                        double dis = gp_Lin(axis1).Distance(axis2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //两个平面
                    else if(retb1 && retb2){
                        double dis = pln1.Distance(pln2);
                        if(dis < 1e-6) {
                            QMessageBox::critical(this,"错误","两面不平行!");
                            return;
                        }

                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = pln2.Distance(gp_Lin(axis1));
                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = pln1.Distance(gp_Lin(axis2));
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                }
                else if(last.ShapeType() == TopAbs_FACE && shape.ShapeType() == TopAbs_EDGE) {
                    Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(last));
                    double a,b;
                    Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(shape),a,b);

                    gp_Ax1 axis1;gp_Pln pln1;
                    gp_Lin lin2;gp_Ax2 ax2;
                    bool reta1 = GeneralTools::GetAxis(face1,axis1);
                    bool reta2 = GeneralTools::GetLine(curve2,lin2);
                    bool retb1 = GeneralTools::GetPlane(last,pln1);
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

                        double dis = pln1.Distance(lin2);
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //平面和圆弧
                    else if(retb1 && retb2) {
                        if(!pln1.Axis().IsNormal(ax2.Axis(), 1e-6) && !pln1.Axis().IsParallel(ax2.Axis(), 1e-6)) {
                            QMessageBox::critical(this,"错误","平面与转轴不平行!");
                            return;
                        }

                        double dis = pln1.Distance(ax2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = lin2.Distance(gp_Lin(axis1));
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //旋转面和圆弧
                    else if(reta1 && retb2) {
                        if(gp_Lin(axis1).Distance(gp_Lin(ax2.Location(),ax2.Direction())) < 1e-6) {
                            QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                            return;
                        }

                        if(!axis1.IsParallel(ax2.Axis(), 1e-6)) {
                            QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                            return;
                        }

                        double dis = gp_Lin(axis1).Distance(ax2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                }
                else if(shape.ShapeType() == TopAbs_FACE && last.ShapeType() == TopAbs_EDGE) {
                    Handle(Geom_Surface) face1 = BRep_Tool::Surface(TopoDS::Face(shape));
                    double a,b;
                    Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(last),a,b);

                    gp_Ax1 axis1;gp_Pln pln1;
                    gp_Lin lin2;gp_Ax2 ax2;
                    bool reta1 = GeneralTools::GetAxis(face1,axis1);
                    bool reta2 = GeneralTools::GetLine(curve2,lin2);
                    bool retb1 = GeneralTools::GetPlane(shape,pln1);
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

                        double dis = pln1.Distance(lin2);
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //平面和圆弧
                    else if(retb1 && retb2) {
                        if(!pln1.Axis().IsNormal(ax2.Axis(), 1e-6) && !pln1.Axis().IsParallel(ax2.Axis(), 1e-6)) {
                            QMessageBox::critical(this,"错误","平面与圆弧不平行!");
                            return;
                        }

                        double dis = pln1.Distance(ax2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = lin2.Distance(gp_Lin(axis1));
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //旋转面和圆弧
                    else if(reta1 && retb2) {
                        if(gp_Lin(axis1).Distance(gp_Lin(ax2.Location(),ax2.Direction())) < 1e-6) {
                            QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                            return;
                        }

                        if(!axis1.IsParallel(ax2.Axis(), 1e-6)) {
                            QMessageBox::critical(this,"错误","面转轴与弧转轴不平行!");
                            return;
                        }

                        double dis = gp_Lin(axis1).Distance(ax2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                }
                else if(last.ShapeType() == TopAbs_EDGE && shape.ShapeType() == TopAbs_EDGE) {
                    double a,b,c,d;
                    Handle(Geom_Curve) curve1 = BRep_Tool::Curve(TopoDS::Edge(last),a,b);
                    Handle(Geom_Curve) curve2 = BRep_Tool::Curve(TopoDS::Edge(shape),c,d);
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
                            QMessageBox::critical(this,"错误","两直线不平行!");
                            return;
                        }

                        if(!lin1.Position().IsParallel(lin2.Position(), 1e-6)) {
                            QMessageBox::critical(this,"错误","两直线不平行!");
                            return;
                        }

                        double dis = lin1.Distance(lin2);
                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = lin1.Distance(axis2.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
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

                        double dis = lin2.Distance(axis1.Location());
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                    //两条圆弧
                    else if(ret3 && ret4) {
                        gp_Lin lct1(axis1.Axis());
                        gp_Lin lct2(axis2.Axis());
                        if(lct1.Distance(lct2) < 1e-6) {
                            QMessageBox::critical(this,"错误","两个转轴重合!");
                            return;
                        }

                        if(!axis1.Axis().IsParallel(axis2.Axis(), 1e-6)) {
                            QMessageBox::critical(this,"错误","两个转轴不平行!");
                            return;
                        }

                        double dis = lct1.Distance(lct2);
                        ui->lineEdit_mainVal->setText(QString::number(dis));
                    }
                }
            }
        }
        //角度
        else if(ui->comboBox_measureType->currentIndex() == 2) {
            TopoDS_Shape last;
            if(shapeIndex == 1) {
                last = myBindShape2;
            }
            else if(shapeIndex == 2) {
                last = myBindShape1;
            }
            if(!last.IsNull()) {
                if(last.ShapeType() == TopAbs_EDGE && shape.ShapeType() == TopAbs_EDGE) {
                    gp_Lin lin1,lin2;
                    BRep_Tool bpt;
                    double a,b;
                    Handle(Geom_Curve) cva =bpt.Curve(TopoDS::Edge(last),a,b);
                    Handle(Geom_Curve) cvb =bpt.Curve(TopoDS::Edge(shape),a,b);
                    if(GeneralTools::GetLine(cva,lin1) && GeneralTools::GetLine(cvb,lin2)) {
                        if(!lin1.Position().IsParallel(lin2.Position(), 1e-6)) {
                            ui->lineEdit_mainVal->setText(QString::number(lin1.Angle(lin2)*180/M_PI));
                        }
                        else {
                            QMessageBox::critical(this,"错误","所选直线平行!");
                            return;
                        }
                    }
                }
                else if(last.ShapeType() == TopAbs_FACE && shape.ShapeType() == TopAbs_FACE) {
                    gp_Pln pln1;gp_Pln pln2;
                    if(GeneralTools::GetPlane(last,pln1) &&GeneralTools::GetPlane(shape,pln2)) {
                        if(!pln1.Axis().IsParallel(pln2.Axis(),1e-6)) {
                            ui->lineEdit_mainVal->setText(QString::number(pln1.Axis().Angle(pln2.Axis())*180/M_PI));
                        }
                        else {
                            QMessageBox::critical(this,"错误","所选平面平行!");
                            return;
                        }
                    }
                }
                else if(last.ShapeType() == TopAbs_FACE && shape.ShapeType() == TopAbs_EDGE) {
                    Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(last));
                    double a,b;
                    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(shape),a,b);

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

                        double ang = axis.Angle(lin.Position())*180/M_PI;
                        ui->lineEdit_mainVal->setText(QString::number(ang));
                    }
                    else {
                        QMessageBox::critical(this,"错误","所选类型不能计算角度!");
                        return;
                    }
                }
                else if(last.ShapeType() == TopAbs_EDGE && shape.ShapeType() == TopAbs_FACE) {
                    Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(shape));
                    double a,b;
                    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(last),a,b);

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

                        double ang = axis.Angle(lin.Position())*180/M_PI;
                        ui->lineEdit_mainVal->setText(QString::number(ang));
                    }
                    else {
                        QMessageBox::critical(this,"错误","所选类型不能计算角度!");
                        return;
                    }
                }
            }
        }
        //直径
        else if(ui->comboBox_measureType->currentIndex() == 3) {
            if(shape.ShapeType() != TopAbs_EDGE) {
                QMessageBox::critical(this,"错误","仅支持圆弧!");
                return;
            }
            BRep_Tool bpt;
            double a,b;
            Handle(Geom_Curve) gc =bpt.Curve(TopoDS::Edge(shape),a,b);
            gp_Circ circ;
            if(!GeneralTools::GetCicle(gc,circ)) {
                QMessageBox::critical(this,"错误","仅支持圆弧!");
                return;
            }
            ui->lineEdit_mainVal->setText(QString::number(2*circ.Radius()));
        }
        //半径
        else if(ui->comboBox_measureType->currentIndex() == 4) {
            if(shape.ShapeType() != TopAbs_EDGE) {
                QMessageBox::critical(this,"错误","仅支持圆弧!");
                return;
            }
            BRep_Tool bpt;
            double a,b;
            Handle(Geom_Curve) gc =bpt.Curve(TopoDS::Edge(shape),a,b);
            gp_Circ circ;
            if(!GeneralTools::GetCicle(gc,circ)) {
                QMessageBox::critical(this,"错误","仅支持圆弧!");
                return;
            }
            ui->lineEdit_mainVal->setText(QString::number(circ.Radius()));
        }
        //锥度
        else if(ui->comboBox_measureType->currentIndex() == 5) {
        }

        if(shapeIndex == 1) {
            ui->lineEdit_eleName1->clear();
            myBindShape1 = shape;
            myTouch1 = touch;
            ui->lineEdit_eleName1->setText(content);
        }
        else if(shapeIndex == 2) {
            ui->lineEdit_eleName2->clear();
            myBindShape2 = shape;
            myTouch2 = touch;
            ui->lineEdit_eleName2->setText(content);
        }
    }
    else {
        if(!GeneralTools::GetPlane(shape,myPlace)) {
            QMessageBox::critical(this,"错误","请选择平面!");
            return;
        }
        ui->lineEdit_place->setText(content);
    }
}

void DiamensionInput::on_pushButton_selectPlace_clicked()
{
    selectPlace = true;
    emit requestSelectShape();
}

void DiamensionInput::on_comboBox_measureType_currentIndexChanged(int index)
{
    diamensionType = index;
    switch(index)
        {
        case 0:{
            ui->stackedWidget_element->setCurrentIndex(0);
            ui->stackedWidget_reference->setCurrentIndex(1);
        }break;
        case 1:
        case 2:{
            ui->stackedWidget_element->setCurrentIndex(1);
            ui->stackedWidget_reference->setCurrentIndex(0);
        }break;
        case 3:
        case 4:{
            ui->stackedWidget_element->setCurrentIndex(0);
            ui->stackedWidget_reference->setCurrentIndex(0);
        }break;
        case 5:{
            ui->stackedWidget_element->setCurrentIndex(0);
            ui->stackedWidget_reference->setCurrentIndex(0);
        }break;
        }
}

