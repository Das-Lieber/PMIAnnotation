#ifndef DIAMENSIONINPUT_H
#define DIAMENSIONINPUT_H

#include <QWidget>

#include <TopoDS_Shape.hxx>
#include <gp_Pln.hxx>
#include <NCollection_UtfString.hxx>

namespace Ui {
class DiamensionInput;
}

class DiamensionInput : public QWidget
{
    Q_OBJECT

public:
    explicit DiamensionInput(QWidget *parent = nullptr);
    ~DiamensionInput();

private slots:
    void on_pushButton_selectEle1_clicked();
    void on_pushButton_selectEle2_clicked();
    void on_pushButton_sure_clicked();
    void on_pushButton_cancle_clicked();
    void on_pushButton_selectPlace_clicked();
    void on_comboBox_measureType_currentIndexChanged(int index);

private:
    Ui::DiamensionInput *ui;
    int shapeIndex = 0;
    TopoDS_Shape myBindShape1;
    TopoDS_Shape myBindShape2;
    gp_Pnt myTouch1;
    gp_Pnt myTouch2;

    bool selectPlace = false;
    gp_Pln myPlace;

    int diamensionType = 0;

    NCollection_Utf8String mainVal;
    NCollection_Utf8String upVal;
    NCollection_Utf8String lowVal;

public slots:
    void SetBindShape(int index, const TopoDS_Shape& shape, const gp_Pnt& touch);

signals:
    void requestSelectShape();
    void readyToClose();
    void labelEditFinish(const QList<NCollection_Utf8String>& valList,
                         const TopoDS_Shape& myShape1, const TopoDS_Shape& myShape2,
                         const gp_Pnt& touch1, const gp_Pnt& touch2,
                         const gp_Pln& place, int type);
};

#endif // DIAMENSIONINPUT_H
