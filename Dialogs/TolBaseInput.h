#ifndef TOLBASEINPUT_H
#define TOLBASEINPUT_H

#include <QWidget>

struct TolEditState
{
    QList<bool> checked; // 10个特殊符号的选中情况
    QString tolVal; // 公差值
    QString panVal; // value of PTZ
};

struct BaseEditState
{
    QList<bool> checked;//4个准则的选中情况
    QString baseVal;//基准符号
};

namespace Ui {
class TolBaseInput;
}

class TolBaseInput : public QWidget
{
    Q_OBJECT

public:
    TolBaseInput(const int &symbolIndex, const QList<BaseEditState>& states, QWidget *parent = nullptr);
    ~TolBaseInput();    

protected:
    bool eventFilter(QObject* watched, QEvent*event);

private slots:
    void on_toolButton_M_clicked(bool checked);
    void on_toolButton_L_clicked(bool checked);
    void on_toolButton_S_clicked(bool checked);
    void on_toolButton_Pan_clicked(bool checked);

    void on_pushButton_cancle_clicked();
    void on_pushButton_ok_clicked();

    void on_lineEdit_first_textEdited(const QString &arg1);
    void on_lineEdit_second_textEdited(const QString &arg1);
    void on_lineEdit_thrid_textEdited(const QString &arg1);

private:
    Ui::TolBaseInput *ui;

    BaseEditState baseState1;
    BaseEditState baseState2;
    BaseEditState baseState3;

    void setCharEnable(const QList<bool>& list);
    void loadState(const QList<BaseEditState>& states);
    void loadBaseState(const BaseEditState& state);
    void handleBaseContent();

signals:
    void baseStrChanged(const QList<BaseEditState>& states);
};

#endif // TOLBASEINPUT_H
