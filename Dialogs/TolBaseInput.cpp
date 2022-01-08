#include "TolBaseInput.h"
#include "ui_TolBaseInput.h"

#include <QDebug>
#include <QRegExpValidator>

#include "TolStringInfo.h"

TolBaseInput::TolBaseInput(const int &symbolIndex, const QList<BaseEditState> &states, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TolBaseInput),
    baseState1({{false,false,false,false},""}),
    baseState2({{false,false,false,false},""}),
    baseState3({{false,false,false,false},""})
{
    ui->setupUi(this);
    this->setWindowFlags(windowFlags() | Qt::FramelessWindowHint | Qt::Popup | Qt::WindowStaysOnTopHint);
    this->setAttribute(Qt::WA_DeleteOnClose);

    ui->lineEdit_second->setEnabled(false);
    ui->lineEdit_thrid->setEnabled(false);
    switch(symbolIndex)
    {
    case 5:
    case 6:
    case 12:setCharEnable({true,true,true,true});break;
    case 7:
    case 8:
    case 9:setCharEnable({true,true,false,true});break;
    case 10:
    case 11:
    case 13:
    case 14:setCharEnable({false,false,false,true});break;
    default: break;
    }

    loadState(states);

    ui->lineEdit_first->installEventFilter(this);
    ui->lineEdit_second->installEventFilter(this);
    ui->lineEdit_thrid->installEventFilter(this);

    QRegExp regx("[a-zA-Z]$");
    QValidator *validator = new QRegExpValidator(regx,this);
    ui->lineEdit_first->setValidator( validator );
    ui->lineEdit_second->setValidator( validator );
    ui->lineEdit_thrid->setValidator( validator );
}

TolBaseInput::~TolBaseInput()
{
    delete ui;
}

bool TolBaseInput::eventFilter(QObject *watched, QEvent *event)
{
    if(watched == ui->lineEdit_first) {
        if(event->type() == QEvent::FocusIn) {
            loadBaseState(baseState1);
        }
    }
    if(watched == ui->lineEdit_second) {
        if(event->type() == QEvent::FocusIn) {
            loadBaseState(baseState2);
        }
    }
    if(watched == ui->lineEdit_thrid) {
        if(event->type() == QEvent::FocusIn) {
            loadBaseState(baseState3);
        }
    }
    return QWidget::eventFilter(watched,event);
}

void TolBaseInput::on_toolButton_M_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_S->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_first) {
        baseState1.checked[0] = checked;
        if(checked) {
            baseState1.checked[1] = false;
            baseState1.checked[2] = false;
            baseState1.checked[3] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_second) {
        baseState2.checked[0] = checked;
        if(checked) {
            baseState2.checked[1] = false;
            baseState2.checked[2] = false;
            baseState2.checked[2] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_thrid) {
        baseState3.checked[0] = checked;
        if(checked) {
            baseState3.checked[1] = false;
            baseState3.checked[2] = false;
            baseState3.checked[3] = false;
        }
    }
    else
        ui->toolButton_M->setChecked(!checked);

    handleBaseContent();
}

void TolBaseInput::on_toolButton_L_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_M->setChecked(false);
        ui->toolButton_S->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_first) {
        baseState1.checked[1] = checked;
        if(checked) {
            baseState1.checked[0] = false;
            baseState1.checked[2] = false;
            baseState1.checked[3] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_second) {
        baseState2.checked[1] = checked;
        if(checked) {
            baseState2.checked[0] = false;
            baseState2.checked[2] = false;
            baseState2.checked[3] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_thrid) {
        baseState3.checked[1] = checked;
        if(checked) {
            baseState3.checked[0] = false;
            baseState3.checked[2] = false;
            baseState3.checked[3] = false;
        }
    }
    else
        ui->toolButton_L->setChecked(!checked);

    handleBaseContent();
}

void TolBaseInput::on_toolButton_S_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_M->setChecked(false);
        ui->toolButton_Pan->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_first) {
        baseState1.checked[2] = checked;
        if(checked) {
            baseState1.checked[0] = false;
            baseState1.checked[1] = false;
            baseState1.checked[3] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_second) {
        baseState2.checked[2] = checked;
        if(checked) {
            baseState2.checked[0] = false;
            baseState2.checked[1] = false;
            baseState2.checked[3] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_thrid) {
        baseState3.checked[2] = checked;
        if(checked) {
            baseState3.checked[0] = false;
            baseState3.checked[1] = false;
            baseState3.checked[3] = false;
        }
    }
    else
        ui->toolButton_S->setChecked(!checked);

    handleBaseContent();
}

void TolBaseInput::on_toolButton_Pan_clicked(bool checked)
{
    if(checked) {
        ui->toolButton_L->setChecked(false);
        ui->toolButton_M->setChecked(false);
        ui->toolButton_S->setChecked(false);
    }

    if(this->focusWidget() == ui->lineEdit_first) {
        baseState1.checked[3] = checked;
        if(checked) {
            baseState1.checked[0] = false;
            baseState1.checked[1] = false;
            baseState1.checked[2] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_second) {
        baseState2.checked[3] = checked;
        if(checked) {
            baseState2.checked[0] = false;
            baseState2.checked[1] = false;
            baseState2.checked[2] = false;
        }
    }
    else if(this->focusWidget() == ui->lineEdit_thrid) {
        baseState3.checked[3] = checked;
        if(checked) {
            baseState3.checked[0] = false;
            baseState3.checked[1] = false;
            baseState3.checked[2] = false;
        }
    }
    else
        ui->toolButton_S->setChecked(!checked);

    handleBaseContent();
}

void TolBaseInput::on_pushButton_cancle_clicked()
{
    this->close();
}

void TolBaseInput::on_pushButton_ok_clicked()
{
    this->close();
}

void TolBaseInput::on_lineEdit_first_textEdited(const QString &arg1)
{
    ui->lineEdit_second->setEnabled(!arg1.isEmpty());
    if(!ui->lineEdit_second->isEnabled()) {
        ui->lineEdit_second->clear();
        ui->lineEdit_thrid->clear();
    }

    baseState1.baseVal = arg1;
    handleBaseContent();
}

void TolBaseInput::on_lineEdit_second_textEdited(const QString &arg1)
{
    ui->lineEdit_thrid->setEnabled(!arg1.isEmpty());
    if(!ui->lineEdit_thrid->isEnabled())
        ui->lineEdit_thrid->clear();

    baseState2.baseVal = arg1;
    handleBaseContent();
}

void TolBaseInput::on_lineEdit_thrid_textEdited(const QString &arg1)
{
    baseState3.baseVal = arg1;
    handleBaseContent();
}

void TolBaseInput::setCharEnable(const QList<bool> &list)
{
    if(list.size() != 4)
        return;
    ui->toolButton_M->setEnabled(list[0]);
    ui->toolButton_L->setEnabled(list[1]);
    ui->toolButton_S->setEnabled(list[2]);
    ui->toolButton_Pan->setEnabled(list[3]);
}

void TolBaseInput::loadState(const QList<BaseEditState> &states)
{
    if(states.size() != 3)
        return;

    baseState1 = states[0];
    baseState2 = states[1];
    baseState3 = states[2];
    ui->lineEdit_first->setText(states[0].baseVal);
    if(!states[0].baseVal.isEmpty()) {
        ui->lineEdit_second->setEnabled(true);
    }
    if(!states[1].baseVal.isEmpty()) {
        ui->lineEdit_second->setEnabled(true);
        ui->lineEdit_thrid->setEnabled(true);
        ui->lineEdit_second->setText(states[1].baseVal);
    }
    if(!states[2].baseVal.isEmpty()) {
        ui->lineEdit_thrid->setEnabled(true);
        ui->lineEdit_thrid->setText(states[2].baseVal);
    }
}

void TolBaseInput::loadBaseState(const BaseEditState &state)
{
    ui->toolButton_M->setChecked(state.checked[0]);
    ui->toolButton_L->setChecked(state.checked[1]);
    ui->toolButton_S->setChecked(state.checked[2]);
    ui->toolButton_Pan->setChecked(state.checked[3]);
}

void TolBaseInput::handleBaseContent()
{
    emit baseStrChanged({baseState1,baseState2,baseState3});
}
