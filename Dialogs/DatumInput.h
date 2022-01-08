#ifndef STRINGINPUT_H
#define STRINGINPUT_H

#include <QWidget>

#include <TopoDS_Shape.hxx>
#include <gp_Pln.hxx>

namespace Ui {
class DatumInput;
}

class DatumInput : public QWidget
{
    Q_OBJECT

public:
    explicit DatumInput(QWidget *parent = nullptr);
    ~DatumInput();

    TopoDS_Shape GetBindShape() const {
        return myBindShape;
    }

private:
    Ui::DatumInput *ui;
    bool selectPlace = false;
    gp_Pln myPlace;
    gp_Pnt myTouch;

    TopoDS_Shape myBindShape;

public slots:
    void SetBindShape(int index, const TopoDS_Shape& shape, const gp_Pnt& touch);

signals:
    void requestSelectShape();
    void readyToClose();
    void labelEditFinish(const QString& str, const TopoDS_Shape& myShape, const gp_Pln& place, const gp_Pnt& touch);

private slots:
    void on_pushButton_select_clicked();
    void on_pushButton_ok_clicked();
    void on_pushButton_cancle_clicked();
    void on_lineEdit_textEdited(const QString &arg1);
    void on_pushButton_selectPlace_clicked();
};

#endif // STRINGINPUT_H
