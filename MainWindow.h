#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <NCollection_UtfString.hxx>

#include "OCCTool/OccWidget.h"

class PMIModel;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void initFunction();
    void initToolBar();

private slots:
    void on_actionImport_triggered();
    void on_actionAdd_Tolerence_triggered();
    void on_actionAdd_Dimension_triggered();
    void on_actionAdd_Datum_triggered();

    void on_addTolLabel(const NCollection_Utf8String& tolName,
                        const NCollection_Utf8String& tolVal,
                        const NCollection_Utf8String& tolVal2,
                        const QList<NCollection_Utf8String>& baseName,
                        const TopoDS_Shape& shape,
                        const gp_Pln& place,
                        const gp_Pnt& touch);
    void on_addDiamensionLabel(const QList<NCollection_Utf8String>& valList,
                               const TopoDS_Shape& shape1, const TopoDS_Shape& shape2,
                               const gp_Pnt& touch1, const gp_Pnt& touch2,
                               const gp_Pln& place, int type);
    void on_addDatumLabel(const QString& str, const TopoDS_Shape& shape, const gp_Pln& place, const gp_Pnt &touch);

private:
    Ui::MainWindow *ui;

    OccWidget *occWidget;
    PMIModel *pmiModel;

    bool existPMIDock = false;
    bool existOtherDock = false;

    bool requestShape;
    bool requestPointOnPlane = false;

    gp_Pnt targetWithBox(const gp_Pnt& input, const gp_Dir& dir, const Bnd_Box& box);

    void measureLength(const Bnd_Box& box, const QList<NCollection_Utf8String> &valList,
                       const TopoDS_Shape &shape1, const TopoDS_Shape &shape2);
    void measureAngle(const QList<NCollection_Utf8String> &valList,
                      const TopoDS_Shape &shape1, const TopoDS_Shape &shape2,
                      const gp_Pnt& touch1, const gp_Pnt& touch2);

    void lengthOfTwoAxis(const Bnd_Box& box, const gp_Ax1& ax1, const gp_Ax1& ax2,
                            gp_Pnt& first, gp_Pnt& second, gp_Ax2& oriention);
    gp_Pnt intersectionOfLines(const gp_Lin& lin1, const gp_Lin& lin2);

signals:
    void ShapeSelected(int index, const TopoDS_Shape &shape, const gp_Pnt& touch);
    void PointOnPlaneSelected(const gp_Pnt& pnt, const Handle(AIS_InteractiveContext)& context);
};

#endif // MAINWINDOW_H
