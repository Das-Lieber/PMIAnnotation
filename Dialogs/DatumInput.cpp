#include "DatumInput.h"
#include "ui_DatumInput.h"

#include <QRegExpValidator>
#include <QMessageBox>

#include <BRep_Tool.hxx>
#include <TopoDS.hxx>

#include "OCCTool/GeneralTools.h"

DatumInput::DatumInput(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DatumInput)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);

    QRegExp regx("[a-zA-Z]$");
    QValidator *validator = new QRegExpValidator(regx,this);
    ui->lineEdit->setValidator( validator );
}

DatumInput::~DatumInput()
{
    delete ui;
}

void DatumInput::SetBindShape(int index, const TopoDS_Shape &shape, const gp_Pnt &touch)
{
    QString content;
    if(shape.ShapeType() == TopAbs_FACE)
    {
        content += "F ";
        content += QString::number(index);
    }
    else if(shape.ShapeType() == TopAbs_EDGE)
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
        ui->lineEdit_elementName->clear();
        myBindShape = shape;
        myTouch = touch;
        ui->lineEdit_elementName->setText(content);
    }
    else {
        if(!GeneralTools::GetPlane(shape,myPlace)) {
            QMessageBox::critical(this,"错误","请选择平面!");
            return;
        }

        if(!myBindShape.IsNull()) {
            gp_Dir normal = myPlace.Axis().Direction();
            gp_Dir direc;
            if(GeneralTools::GetShapeNormal(myBindShape,myTouch,direc)) {
                if(myBindShape.ShapeType() == TopAbs_EDGE) {
                    double a,b;
                    Handle(Geom_Curve) curve = BRep_Tool::Curve(TopoDS::Edge(myBindShape),a,b);
                    gp_Ax2 ax2;
                    if(GeneralTools::GetCenter(curve, ax2)) {
                        direc = ax2.Direction();
                    }
                }
                else if(myBindShape.ShapeType() == TopAbs_FACE) {
                    Handle(Geom_Surface) surface = BRep_Tool::Surface(TopoDS::Face(myBindShape));
                    gp_Ax1 ax1;
                    if(GeneralTools::GetAxis(surface,ax1)) {
                        direc = ax1.Direction();
                    }
                }

                if(direc.IsParallel(normal, 1e-6)) {
                    QMessageBox::critical(this,"错误","放置面法向与基准元素法向一致!");
                    return;
                }

                if(!direc.IsNormal(normal, 1e-6)) {
                    QMessageBox::critical(this,"错误","放置面法向与基准元素法向不垂直!");
                    return;
                }
            }
            else {
                QMessageBox::critical(this,"错误","所选形状不能作为基准!");
                return;
            }
        }

        ui->lineEdit_place->setText(content);
    }
}

void DatumInput::on_pushButton_select_clicked()
{
    selectPlace = false;
    requestSelectShape();
}

void DatumInput::on_pushButton_ok_clicked()
{
    if(myBindShape.IsNull()) {
        QMessageBox::critical(this,"错误","未选择标注元素!");
        return;
    }

    QString content;
    content = ui->lineEdit->text();
    if(content.isEmpty()) {
        QMessageBox::critical(this,"错误","未输入基准名称!");
        return;
    }

    if(ui->lineEdit_place->text().isEmpty()) {
        QMessageBox::critical(this,"错误","未选择放置平面!");
        return;
    }

    emit labelEditFinish(content,myBindShape, myPlace, myTouch);
    this->close();
    emit readyToClose();
}

void DatumInput::on_pushButton_cancle_clicked()
{
    this->close();
    emit readyToClose();
}

void DatumInput::on_lineEdit_textEdited(const QString &arg1)
{
    ui->lineEdit->setText(arg1.toUpper());
}

void DatumInput::on_pushButton_selectPlace_clicked()
{
    selectPlace = true;
    requestSelectShape();
}
