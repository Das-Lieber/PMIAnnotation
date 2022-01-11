#include "ToleranceInput.h"
#include "ui_ToleranceInput.h"

#include <QFontMetrics>
#include <QDebug>
#include <QMessageBox>


#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Geom_CartesianPoint.hxx>

#include "TolStringInfo.h"
#include "TolBaseInput.h"
#include "OCCTool/GeneralTools.h"

ToleranceInput::ToleranceInput(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ToleranceInput),
    tolState1({{false,false,false,
              false,false,false,
              false,false,false,
              false},"",""}),
    tolState2({{false,false,false,
              false,false,false,
              false,false,false,
              false},"",""}),
    lastIsTol1(false),
    baseState1({{{false,false,false,false},""}
               ,{{false,false,false,false},""}
               ,{{false,false,false,false},""}}),
    baseState2({{{false,false,false,false},""}
               ,{{false,false,false,false},""}
               ,{{false,false,false,false},""}}),
    baseState3({{{false,false,false,false},""}
               ,{{false,false,false,false},""}
               ,{{false,false,false,false},""}})
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    ui->horizontalLayout_label->setContentsMargins(0,0,0,0);

    ui->lineEdit_Tol1->installEventFilter(this);
    ui->lineEdit_Tol2->installEventFilter(this);

    initEnableState();
    limitInput();
}

ToleranceInput::~ToleranceInput()
{
    delete ui;
}

void ToleranceInput::SetBindShape(int index, const TopoDS_Shape &shape, const gp_Pnt &touch)
{
    QString content;
    if(shape.ShapeType() == TopAbs_FACE||shape.ShapeType() == TopAbs_SHELL)
    {
        content += "F ";
        content += QString::number(index);
    }
    else if(shape.ShapeType() == TopAbs_EDGE)
    {
        if(ui->comboBox_symbol->currentIndex() == 1){
            QMessageBox::critical(this,"错误","只能在平面上测量直线度!");
            return;
        }

        content += "E ";
        content += QString::number(index);
    }
    else if(shape.ShapeType() == TopAbs_VERTEX)
    {
        if(ui->comboBox_symbol->currentIndex() == 1){
            QMessageBox::critical(this,"错误","只能在平面上测量直线度!");
            return;
        }

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
        ui->lineEdit_place->setText(content);

    }
}

void ToleranceInput::SetPointOnPlane(const gp_Pnt &pnt, const Handle(AIS_InteractiveContext)& context)
{
    gp_Pln pln;
    if(!GeneralTools::GetPlane(myBindShape,pln)) {
        QMessageBox::critical(this,"错误","只能在平面上测量直线度!");
    }

    if(pln.Distance(pnt) > 1e-6){
        QMessageBox::critical(this,"错误","所选的点不在直线度测量平面上!");
        return;
    }

    Handle(AIS_DraftPoint) dfp;
    if(pointCnt < 2 ) {
        dfp = new AIS_DraftPoint(pnt,pln);
        pointCnt++;
        context->Display(dfp,Standard_True);
        if(selectIndex == 1){
            connect(dfp.get(),&AIS_DraftPoint::PosChanged,this,[=]() {
                ui->lineEdit_firstVertex->setText(dfp->Info());
                if(myPlanePnt1&&!myPlanePnt1.IsNull()) {
                    context->Erase(myPlaneLine,Standard_False);
                    myPlaneLine = new AIS_Line(new Geom_CartesianPoint(myPlanePnt1->Pnt()),
                                               new Geom_CartesianPoint(myPlanePnt2->Pnt()));
                    context->Display(myPlaneLine,Standard_True);
                }
            },Qt::UniqueConnection);
            myPlanePnt1 = dfp;
            ui->lineEdit_firstVertex->setText(dfp->Info());
        }
        else if(selectIndex == 2) {
            connect(dfp.get(),&AIS_DraftPoint::PosChanged,this,[=]() {
                ui->lineEdit_secondVertex->setText(dfp->Info());
                if(myPlanePnt2&&!myPlanePnt2.IsNull()) {
                    context->Erase(myPlaneLine,Standard_False);
                    myPlaneLine = new AIS_Line(new Geom_CartesianPoint(myPlanePnt1->Pnt()),
                                               new Geom_CartesianPoint(myPlanePnt2->Pnt()));
                    context->Display(myPlaneLine,Standard_True);
                }
            },Qt::UniqueConnection);
            myPlanePnt2 = dfp;
            ui->lineEdit_secondVertex->setText(dfp->Info());
        }

        if(pointCnt == 2) {
            myPlaneLine = new AIS_Line(new Geom_CartesianPoint(myPlanePnt1->Pnt()),
                                       new Geom_CartesianPoint(myPlanePnt2->Pnt()));
            context->Display(myPlaneLine,Standard_True);
        }
    }
    else {
        if(selectIndex == 1) {
            myPlanePnt1->SetLocation(pnt);
        }
        else if(selectIndex == 2)
            myPlanePnt2->SetLocation(pnt);

        context->Erase(myPlaneLine,Standard_False);
        myPlaneLine = new AIS_Line(new Geom_CartesianPoint(myPlanePnt1->Pnt()),
                                   new Geom_CartesianPoint(myPlanePnt2->Pnt()));
        context->Display(myPlaneLine,Standard_True);
    }
}

bool ToleranceInput::eventFilter(QObject *watched, QEvent *event)
{
    if(watched == ui->lineEdit_Tol1) {
        if(event->type() == QEvent::FocusIn) {
            loadTolState(tolState1);
            lastIsTol1 = true;
        }
    }
    if(watched == ui->lineEdit_Tol2) {
        if(event->type() == QEvent::FocusIn) {
            loadTolState(tolState2);
            lastIsTol1 = false;
        }
    }
    return QWidget::eventFilter(watched,event);
}

void ToleranceInput::on_pushButton_select_clicked()
{
    selectPlace = false;
    requestSelectShape();
    selectIndex = 0;
}

//different type of tolerance corresponding to different symbol
void ToleranceInput::on_comboBox_symbol_currentIndexChanged(int index)
{
    if(index == 1) {
        if(!ui->lineEdit_elementName->text().contains('F')){
            QMessageBox::critical(this,"错误","只能在平面上测量直线度!");
            ui->comboBox_symbol->setCurrentIndex(0);
            return;
        }

        ui->stackedWidget->setCurrentIndex(1);
    }
    else {
        ui->stackedWidget->setCurrentIndex(0);
    }

    switch(index)
    {
    case 0:initEnableState();break;
    case 1:
    {
        setCharEnable({true,true,true,
                       true,true,true,
                       true,false,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(false);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFLinearity;
        tolWChar = FONT_Linearity;

        break;
    }
    case 2:
    {
        setCharEnable({true,true,false,
                       false,false,true,
                       true,true,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(false);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFPlanarity;
        tolWChar = FONT_Planarity;

        break;
    }
    case 3:
    {
        setCharEnable({false,true,false,
                       false,false,true,
                       true,false,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(false);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFCircularity;
        tolWChar = FONT_Circularity;

        break;
    }
    case 4:
    {
        setCharEnable({false,false,false,
                       false,false,false,
                       true,false,false,
                       false});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(false);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFCylindricity;
        tolWChar = FONT_Cylindricity;

        break;
    }
    case 5:
    {
        setCharEnable({false,true,true,
                       true,true,true,
                       true,false,true,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFLineProfile;
        tolWChar = FONT_LineProfile;

        break;
    }
    case 6:
    {
        setCharEnable({false,true,true,
                       true,true,true,
                       true,true,true,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFPlaneProfile;
        tolWChar = FONT_PlaneProfile;

        break;
    }
    case 7:
    {
        setCharEnable({true,true,true,
                       true,true,true,
                       true,true,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFParallelism;
        tolWChar = FONT_Parallelism;

        break;
    }
    case 8:
    {
        setCharEnable({true,true,true,
                       true,true,true,
                       true,true,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFVerticality;
        tolWChar = FONT_Verticality;

        break;
    }
    case 9:
    {
        setCharEnable({true,true,true,
                       true,true,true,
                       true,true,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFGradient;
        tolWChar = FONT_Gradient;

        break;
    }
    case 10:
    {
        setCharEnable({false,false,false,
                       false,false,false,
                       true,false,false,
                       false});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFRunout;
        tolWChar = FONT_Runout;

        break;
    }
    case 11:
    {
        setCharEnable({false,false,false,
                       false,false,false,
                       true,false,false,
                       false});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFTotalRunout;
        tolWChar = FONT_TotalRunout;

        break;
    }
    case 12:
    {
        setCharEnable({true,true,true,
                       true,true,true,
                       true,false,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFPosition;
        tolWChar = FONT_Position;

        break;
    }
    case 13:
    {
        setCharEnable({true,true,true,
                       true,false,false,
                       true,false,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFAxiality;
        tolWChar = FONT_Axialit;

        break;
    }
    case 14:
    {
        setCharEnable({false,true,true,
                       true,false,false,
                       true,false,false,
                       true});

        ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
        ui->checkBox_Tol2->setEnabled(true);

        ui->lineEdit_Tol1->setEnabled(true);
        ui->lineEdit_Tol2->setEnabled(false);

        setBaseInputEnable(true);
        ui->lineEdit_panVal->setEnabled(false);

        tolName = UTFSymmetry;
        tolWChar = FONT_Symmetry;

        break;
    }
    }

    handleLabelContent();
}

void ToleranceInput::on_toolButton_main_clicked()
{
    QPoint p = ui->toolButton_main->pos();
    p.ry() += ui->toolButton_main->height();
    showBaseInput(mapToGlobal(p),1);
}

void ToleranceInput::on_toolButton_second_clicked()
{
    QPoint p = ui->toolButton_second->pos();
    p.ry() += ui->toolButton_second->height();
    showBaseInput(mapToGlobal(p),2);
}

void ToleranceInput::on_toolButton_third_clicked()
{
    QPoint p = ui->toolButton_third->pos();
    p.ry() += ui->toolButton_third->height();
    showBaseInput(mapToGlobal(p),3);
}

void ToleranceInput::on_pushButton_ok_clicked()
{
    if(myBindShape.IsNull()) {
        QMessageBox::critical(this,"错误","未选择标注元素!");
        return;
    }
    if(ui->lineEdit_Tol1->text().isEmpty()) {
        QMessageBox::critical(this,"错误","未输入公差值!");
        return;
    }

    if(ui->comboBox_symbol->currentIndex() == 1) {
        if(ui->lineEdit_firstVertex->text().isEmpty()
                ||ui->lineEdit_secondVertex->text().isEmpty()) {
            QMessageBox::critical(this,"错误","直线度需要在平面上选择两点!");
            return;
        }
    }

    if(ui->lineEdit_place->text().isEmpty()) {
        QMessageBox::critical(this,"错误","未选择放置平面!");
        return;
    }

    NCollection_Utf8String tol = nstrFromTolState(tolState1);
    NCollection_Utf8String tol2 = "";
    if(ui->checkBox_Tol2->isChecked())
        tol2 = nstrFromTolState(tolState2);
    QList<NCollection_Utf8String> base;
    base.append(nstrFromBaseStates(baseState1));
    base.append(nstrFromBaseStates(baseState2));
    base.append(nstrFromBaseStates(baseState3));
    labelEditFinish(tolWChar,tol,tol2,base,myBindShape,myPlace,myTouch);

    if(!myPlanePnt1.IsNull()) {
        myPlanePnt1->setMyEditable(false);
    }
    if(!myPlanePnt2.IsNull()) {
        myPlanePnt2->setMyEditable(false);
    }
    this->close();
    emit readyToClose();
}

void ToleranceInput::on_pushButton_cancle_clicked()
{
    if(!myPlanePnt1.IsNull()) {
        myPlanePnt1->setMyEditable(false);
    }
    if(!myPlanePnt2.IsNull()) {
        myPlanePnt2->setMyEditable(false);
    }
    this->close();
    emit readyToClose();
}

void ToleranceInput::on_toolButton_R_clicked(bool checked)
{
    if(this->focusWidget() == ui->lineEdit_Tol1)
        tolState1.checked[0] = checked;
    else if(this->focusWidget() == ui->lineEdit_Tol2)
        tolState2.checked[0] = checked;
    else
        ui->toolButton_R->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_M_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_S->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[1] = checked;
        if(checked) {
            tolState1.checked[2] = false;
            tolState1.checked[3] = false;
            tolState1.checked[9] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[1] = checked;
        if(checked) {
            tolState2.checked[2] = false;
            tolState2.checked[3] = false;
            tolState2.checked[9] = false;
        }
    }
    else
        ui->toolButton_M->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_L_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_M->setChecked(false);
        ui->toolButton_S->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[2] = checked;
        if(checked) {
            tolState1.checked[1] = false;
            tolState1.checked[3] = false;
            tolState1.checked[9] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[2] = checked;
        if(checked) {
            tolState2.checked[1] = false;
            tolState2.checked[3] = false;
            tolState2.checked[9] = false;
        }
    }
    else
        ui->toolButton_L->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_S_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_M->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[3] = checked;
        if(checked) {
            tolState1.checked[1] = false;
            tolState1.checked[2] = false;
            tolState1.checked[9] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[3] = checked;
        if(checked) {
            tolState2.checked[1] = false;
            tolState2.checked[2] = false;
            tolState2.checked[9] = false;
        }
    }
    else
        ui->toolButton_S->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_T_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_F->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[4] = checked;
        if(checked) {
            tolState1.checked[5] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[4] = checked;
        if(checked) {
            tolState2.checked[5] = false;
        }
    }
    else
        ui->toolButton_T->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_F_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_T->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[5] = checked;
        if(checked) {
            tolState1.checked[4] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[5] = checked;
        if(checked) {
            tolState2.checked[4] = false;
        }
    }
    else
        ui->toolButton_F->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_P_clicked(bool checked)
{
    if(this->focusWidget() == ui->lineEdit_Tol1) {
        ui->lineEdit_panVal->setEnabled(checked);
        tolState1.checked[6] = checked;
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        ui->lineEdit_panVal->setEnabled(checked);
        tolState2.checked[6] = checked;
    }
    else
        ui->toolButton_P->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_Square_clicked(bool checked)
{
    if(this->focusWidget() == ui->lineEdit_Tol1)
        tolState1.checked[7] = checked;
    else if(this->focusWidget() == ui->lineEdit_Tol2)
        tolState2.checked[7] = checked;
    else
        ui->toolButton_Square->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_U_clicked(bool checked)
{
    if(this->focusWidget() == ui->lineEdit_Tol1)
        tolState1.checked[8] = checked;
    else if(this->focusWidget() == ui->lineEdit_Tol2)
        tolState2.checked[8] = checked;
    else
        ui->toolButton_U->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_toolButton_Pan_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_M->setChecked(false);
        ui->toolButton_S->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_Tol1) {
        tolState1.checked[9] = checked;
        if(checked) {
            tolState1.checked[2] = false;
            tolState1.checked[3] = false;
            tolState1.checked[1] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_Tol2) {
        tolState2.checked[9] = checked;
        if(checked) {
            tolState2.checked[2] = false;
            tolState2.checked[3] = false;
            tolState2.checked[1] = false;
        }
    }
    else
        ui->toolButton_Pan->setChecked(!checked);

    handleLabelContent();
}

void ToleranceInput::on_lineEdit_panVal_textEdited(const QString &arg1)
{
    if(lastIsTol1) {
        tolState1.panVal = arg1;
    }
    else
        tolState2.panVal = arg1;

    handleLabelContent();
}

void ToleranceInput::on_lineEdit_Tol1_textChanged(const QString &arg1)
{
    tolState1.tolVal = arg1;

    handleLabelContent();
}

void ToleranceInput::on_lineEdit_Tol2_textChanged(const QString &arg1)
{
    tolState2.tolVal = arg1;

    handleLabelContent();
}

void ToleranceInput::on_checkBox_Tol2_stateChanged(int arg1)
{
    Q_UNUSED(arg1)
    ui->lineEdit_Tol2->setEnabled(ui->checkBox_Tol2->isChecked());

    handleLabelContent();
}

void ToleranceInput::on_lineEdit_main_textEdited(const QString &arg1)
{
    baseState1[0].baseVal = arg1;
    handleLabelContent();
}

void ToleranceInput::on_lineEdit_second_textEdited(const QString &arg1)
{
    baseState2[0].baseVal = arg1;
    handleLabelContent();
}

void ToleranceInput::on_lineEdit_third_textEdited(const QString &arg1)
{
    baseState3[0].baseVal = arg1;
    handleLabelContent();
}

void ToleranceInput::initEnableState()
{
    // 1.公差用到的10个符号
    setCharEnable({false,false,false,
                   false,false,false,
                   false,false,false,
                   false});

    // 2.公差2的勾选框
    ui->checkBox_Tol2->setCheckState(Qt::Unchecked);
    ui->checkBox_Tol2->setEnabled(false);

    // 3.两个公差输入框
    ui->lineEdit_Tol1->setEnabled(false);
    ui->lineEdit_Tol2->setEnabled(false);

    // 3.3个基准的弹出菜单按钮
    setBaseInputEnable(false);

    // 4.投影公差的高度值
    ui->lineEdit_panVal->setEnabled(false);
}

void ToleranceInput::limitInput()
{
    QDoubleValidator *validator = new QDoubleValidator(this);
    validator->setRange(-999,999,4);
    ui->lineEdit_panVal->setValidator( validator );
    ui->lineEdit_Tol1->setValidator( validator );
    ui->lineEdit_Tol2->setValidator( validator );

    QRegExp regx("[a-zA-Z]$");
    QValidator *validator2 = new QRegExpValidator(regx,this);
    ui->lineEdit_main->setValidator( validator2 );
    ui->lineEdit_second->setValidator( validator2 );
    ui->lineEdit_third->setValidator( validator2 );
}

void ToleranceInput::setCharEnable(const QList<bool> &list)
{
    if(list.size() != 10)
        return;

    ui->toolButton_R->setEnabled(list[0]);
    ui->toolButton_M->setEnabled(list[1]);
    ui->toolButton_L->setEnabled(list[2]);
    ui->toolButton_S->setEnabled(list[3]);
    ui->toolButton_T->setEnabled(list[4]);
    ui->toolButton_F->setEnabled(list[5]);
    ui->toolButton_P->setEnabled(list[6]);
    ui->toolButton_Square->setEnabled(list[7]);
    ui->toolButton_U->setEnabled(list[8]);
    ui->toolButton_Pan->setEnabled(list[9]);
}

void ToleranceInput::setBaseInputEnable(bool enable)
{
    ui->lineEdit_main->setEnabled(enable);
    ui->toolButton_main->setEnabled(enable);
    ui->lineEdit_second->setEnabled(enable);
    ui->toolButton_second->setEnabled(enable);
    ui->lineEdit_third->setEnabled(enable);
    ui->toolButton_third->setEnabled(enable);
}

void ToleranceInput::showBaseInput(const QPoint &pnt, int index)
{
    QList<BaseEditState> state;
    if(index == 1)
        state = baseState1;
    else if(index == 2)
        state = baseState2;
    else if(index == 3)
        state = baseState3;
    TolBaseInput* aInput = new TolBaseInput(ui->comboBox_symbol->currentIndex(),state);
    aInput->show();
    aInput->move(pnt);

    connect(aInput,&TolBaseInput::baseStrChanged,this,[=](const QList<BaseEditState>& states) {
        if(index == 1) {
            ui->lineEdit_main->setText(states[0].baseVal[0]);
            baseState1 = states;
            handleLabelContent();
        }
        else if(index == 2) {
            ui->lineEdit_second->setText(states[0].baseVal[0]);
            baseState2 = states;
            handleLabelContent();
        }
        else if(index == 3) {
            ui->lineEdit_third->setText(states[0].baseVal[0]);
            baseState3 = states;
            handleLabelContent();
        }
        else return;
    });
}

void ToleranceInput::loadTolState(const TolEditState &state)
{
    ui->toolButton_R->setChecked(state.checked[0]);
    ui->toolButton_M->setChecked(state.checked[1]);
    ui->toolButton_L->setChecked(state.checked[2]);
    ui->toolButton_S->setChecked(state.checked[3]);
    ui->toolButton_T->setChecked(state.checked[4]);
    ui->toolButton_F->setChecked(state.checked[5]);
    ui->toolButton_P->setChecked(state.checked[6]);
    ui->toolButton_Square->setChecked(state.checked[7]);
    ui->toolButton_U->setChecked(state.checked[8]);
    ui->toolButton_Pan->setChecked(state.checked[9]);

    ui->lineEdit_panVal->setText(state.panVal);
    ui->lineEdit_panVal->setEnabled(state.checked[6]);
}

QString ToleranceInput::strFromTolState(const TolEditState &state)
{
    QString result;

    result.append(state.tolVal);

    QStringList list;
    list<< UTFRadius << UTFMMC << UTFLMC << UTFRSize << UTFTanBase
        << UTFFree << UTFPTZ+state.panVal << UTFSquare << UTFUAC << UTFPan;

    if(state.checked[0])
        result.prepend(list[0]);
    if(state.checked[9])
        result.append(list[9]);

    for(int i=1;i<7;++i) {
        if(state.checked[i]) {
            result.append(list[i]);
        }
    }

    if(state.checked[8])
        result.append(list[8]);
    if(state.checked[7])
        result.append(list[7]);
    return result;
}

NCollection_Utf8String ToleranceInput::nstrFromTolState(const TolEditState &state)
{
    NCollection_Utf8String result;

    // 1.prepare the symbols
    QList<NCollection_Utf8String> list;
    NCollection_Utf8String panV = state.panVal.toStdString().data();
    list<< FONT_Radius << FONT_MMC << FONT_LMC << FONT_RSize << FONT_TanBase
        << FONT_Free << FONT_PTZ+panV << FONT_Square << FONT_UAC << FONT_Pan;
    //the radius first
    if(state.checked[0])
        result += list[0];

    // 2.tolerance value
    result += state.tolVal.toStdString().data();

    // 3.other symbols
    // pan first
    if(state.checked[9])
        result += list[9];

    for(int i=1;i<7;++i) {
        if(state.checked[i]) {
            result += list[i];
        }
    }

    //change the position of U and square
    if(state.checked[8])
        result += list[8];
    if(state.checked[7])
        result += list[7];
    return result;
}

QString ToleranceInput::strFromBaseStates(const QList<BaseEditState> &states)
{
    if(states.size()!=3)
        return "";

    QString content;
    QString base1 = strFromBaseState(states[0]);
    QString base2 = strFromBaseState(states[1]);
    QString base3 = strFromBaseState(states[2]);
    content.append(base1);
    if(!base2.isEmpty()) {
        content.append('-');
        content.append(base2);
    }
    if(!base3.isEmpty()) {
        content.append('-');
        content.append(base3);
    }
    return content;
}

QString ToleranceInput::strFromBaseState(const BaseEditState &state)
{
    QString result;
    result.append(state.baseVal);
    QStringList list;
    list<< UTFMMC << UTFLMC << UTFRSize << UTFPan;
    for(int i=0;i<state.checked.size();++i) {
        if(state.checked[i])
            result.append(list[i]);
    }
    return result;
}

NCollection_Utf8String ToleranceInput::nstrFromBaseStates(const QList<BaseEditState> &states)
{
    if(states.size()!=3)
        return "";

    NCollection_Utf8String content;
    NCollection_Utf8String base1 = nstrFromBaseState(states[0]);
    NCollection_Utf8String base2 = nstrFromBaseState(states[1]);
    NCollection_Utf8String base3 = nstrFromBaseState(states[2]);
    content += base1;
    if(!base2.IsEmpty()) {
        content += "-";
        content += base2;
    }
    if(!base3.IsEmpty()) {
        content += "-";
        content += base3;
    }
    return content;
}

NCollection_Utf8String ToleranceInput::nstrFromBaseState(const BaseEditState &state)
{
    NCollection_Utf8String result;
    result += state.baseVal.toStdString().data();
    QList<NCollection_Utf8String> list;
    list<< FONT_MMC << FONT_LMC << FONT_RSize << FONT_Pan;
    for(int i=0;i<state.checked.size();++i) {
        if(state.checked[i])
            result += list[i];
    }
    return result;
}

void ToleranceInput::handleLabelContent()
{
    if(tolName.isEmpty())
        return;

    // 1. remove all the widgets
    QLayoutItem* child;
    while(ui->horizontalLayout_label->count()!=0)
    {
        child = ui->horizontalLayout_label->takeAt(0);
        if(child->layout() != 0)
        {
            ui->horizontalLayout_label->removeItem(child->layout());
        }
        else if(child->widget() != 0)
        {
            delete child->widget();
        }

        delete child;
    }

    // 2.append the string by group
    // 2.1tolerance name
    appendString(tolName);
    // 2.2tolLineEdit and 10 character
    appendString(strFromTolState(tolState1));
    if(ui->checkBox_Tol2->isChecked())
        appendString(strFromTolState(tolState2));
    // 2.3base system
    appendString(strFromBaseStates(baseState1));
    appendString(strFromBaseStates(baseState2));
    appendString(strFromBaseStates(baseState3));

    // 3.add a stretch to make all label align left
    ui->horizontalLayout_label->addStretch();
}

void ToleranceInput::appendString(const QString &str)
{
    if(str.isEmpty())
        return;

    QFont ft("Arial",15);
    QFontMetrics ftMetrics(ft);

    QRect symbolRec = ftMetrics.boundingRect(str);
    QLabel* symbol = new QLabel(str,this);
    symbol->setAlignment(Qt::AlignCenter);
    symbol->setFont(ft);
    symbol->setFrameShape(QFrame::Box);
    symbol->setFixedSize(symbolRec.width()+10,50);
    ui->horizontalLayout_label->addWidget(symbol);
}

void ToleranceInput::on_pushButton_selectFirstVertex_clicked()
{
    requestPointOnPlane();
    selectIndex = 1;
}

void ToleranceInput::on_pushButton_selectSecondVertex_clicked()
{
    requestPointOnPlane();
    selectIndex = 2;
}

void ToleranceInput::on_pushButton_selectPlace_clicked()
{
    selectPlace = true;
    requestSelectShape();
}

