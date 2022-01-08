#ifndef AIS_DRAFTPOINT_H
#define AIS_DRAFTPOINT_H

#include "AIS_DraftShape.hxx"
#include <QString>
#include <QObject>

class AIS_DraftPoint : public QObject, public AIS_DraftShape
{
 Q_OBJECT

public:
    AIS_DraftPoint(const gp_Pnt& p, const gp_Pln& pln);
    gp_Pnt Pnt() const {
        return myPnt;
    }

    double X() const {
        return myPnt.X();
    }
    double Y() const {
        return myPnt.Y();
    }
    double Z() const {
        return myPnt.Z();
    }

    QString Info() const {
        QString content;
        content += QString::number(X(),'f',2);
        content += ",";
        content += QString::number(Y(),'f',2);
        content += ",";
        content += QString::number(Z(),'f',2);
        return content;
    }

    virtual void SetLocation(const gp_Pnt& pnt) override;

    static int COUNT;

private:
    gp_Pnt myPnt;
    gp_Pln myPln;
    bool myEditable = true;

signals:
    void PosChanged();

protected:

  Standard_EXPORT virtual void Compute (const Handle(PrsMgr_PresentationManager3d)& aPresentationManager, const Handle(Prs3d_Presentation)& aPresentation, const Standard_Integer aMode = 0) Standard_OVERRIDE;

private:

  Standard_EXPORT void ComputeSelection (const Handle(SelectMgr_Selection)& aSelection, const Standard_Integer aMode) Standard_OVERRIDE;

public:

    //! CASCADE RTTI
  DEFINE_STANDARD_RTTIEXT(AIS_DraftPoint,AIS_DraftShape)
  bool getMyEditable() const;
  void setMyEditable(bool newMyEditable);
};

DEFINE_STANDARD_HANDLE(AIS_DraftPoint, AIS_DraftShape)

#endif // AIS_DRAFTPOINT_H
