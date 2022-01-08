#ifndef TOLERANCEINPUT_H
#define TOLERANCEINPUT_H

#include <QWidget>

#include <NCollection_UtfString.hxx>
#include <TopoDS_Shape.hxx>
#include <AIS_Line.hxx>

#include "TolBaseInput.h"
#include "OCCTool/AIS_DraftPoint.h"

class QLabel;

namespace Ui {
class ToleranceInput;
}

class ToleranceInput : public QWidget
{
    Q_OBJECT

public:
    explicit ToleranceInput(QWidget *parent = nullptr);
    ~ToleranceInput();

    TopoDS_Shape GetBindShape() const {
        return myBindShape;
    }

public slots:
    void SetBindShape(int index, const TopoDS_Shape& shape, const gp_Pnt& touch);
    void SetPointOnPlane(const gp_Pnt& pnt, const Handle(AIS_InteractiveContext)& context);

protected:
    bool eventFilter(QObject* watched, QEvent*event);

private slots:
    // select the element
    void on_pushButton_select_clicked();

    // select the type of tolerance
    void on_comboBox_symbol_currentIndexChanged(int index);

    // three button to call the base edit widget
    void on_toolButton_main_clicked();
    void on_toolButton_second_clicked();
    void on_toolButton_third_clicked();

    void on_pushButton_ok_clicked();
    void on_pushButton_cancle_clicked();

    // special of tolerance
    // M&L&S&Pan mutex
    // T&F mutex
    // P control the enable of linedit height
    void on_toolButton_R_clicked(bool checked);
    void on_toolButton_M_clicked(bool checked);
    void on_toolButton_L_clicked(bool checked);
    void on_toolButton_S_clicked(bool checked);
    void on_toolButton_T_clicked(bool checked);
    void on_toolButton_F_clicked(bool checked);
    void on_toolButton_P_clicked(bool checked);
    void on_toolButton_Square_clicked(bool checked);
    void on_toolButton_U_clicked(bool checked);
    void on_toolButton_Pan_clicked(bool checked);

    void on_lineEdit_panVal_textEdited(const QString &arg1);

    // two tolerence value editer
    void on_lineEdit_Tol1_textChanged(const QString &arg1);
    void on_lineEdit_Tol2_textChanged(const QString &arg1);
    void on_checkBox_Tol2_stateChanged(int arg1);

    // three base editer
    void on_lineEdit_main_textEdited(const QString &arg1);
    void on_lineEdit_second_textEdited(const QString &arg1);
    void on_lineEdit_third_textEdited(const QString &arg1);

    // two point of Linearity
    void on_pushButton_selectFirstVertex_clicked();
    void on_pushButton_selectSecondVertex_clicked();

    void on_pushButton_selectPlace_clicked();

private:
    Ui::ToleranceInput *ui;

    TopoDS_Shape myBindShape;
    Handle(AIS_DraftPoint) myPlanePnt1;
    Handle(AIS_DraftPoint) myPlanePnt2;
    Handle(AIS_Line) myPlaneLine;
    bool selectPlace = false;
    gp_Pln myPlace;
    gp_Pnt myTouch;
    int selectIndex = -1;
    int pointCnt = 0;

    QString tolName;
    NCollection_Utf8String tolWChar;
    TolEditState tolState1;
    TolEditState tolState2;
    bool lastIsTol1;

    QList<BaseEditState> baseState1;
    QList<BaseEditState> baseState2;
    QList<BaseEditState> baseState3;

    void initEnableState();
    void limitInput();
    void setCharEnable(const QList<bool> &list);
    void setBaseInputEnable(bool enable);

    void showBaseInput(const QPoint& pnt, int index);

    void loadTolState(const TolEditState& state);
    QString strFromTolState(const TolEditState& state);
    NCollection_Utf8String nstrFromTolState(const TolEditState& state);

    QString strFromBaseStates(const QList<BaseEditState>& states);
    QString strFromBaseState(const BaseEditState& state);

    NCollection_Utf8String nstrFromBaseStates(const QList<BaseEditState>& states);
    NCollection_Utf8String nstrFromBaseState(const BaseEditState& state);

    void handleLabelContent();
    void appendString(const QString& str);

signals:
    void labelEditFinish(const NCollection_Utf8String& tolName,
                         const NCollection_Utf8String& tolVal,
                         const NCollection_Utf8String& tolVal2,
                         const QList<NCollection_Utf8String>& baseName,
                         const TopoDS_Shape& myShape,
                         const gp_Pln& place,
                         const gp_Pnt& touch);
    void requestSelectShape();
    void requestPointOnPlane();
    void readyToClose();
};

#endif // TOLERANCEINPUT_H
