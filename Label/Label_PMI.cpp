#include "Label_PMI.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Font_BRepTextBuilder.hxx>

IMPLEMENT_STANDARD_RTTIEXT(Label_PMI,AIS_DraftShape)

Label_PMI::Label_PMI()
    : myHasOrientation3D(Standard_False),
      myLabelZoomable(Standard_True),
      myFontHeight(4),
      myFontPadding(2),
      myLabelColor(Quantity_NOC_BLACK)
{
    myDrawer->SetDisplayMode (0);
}

void Label_PMI::SetColor(const Quantity_Color &theColor)
{
    myLabelColor = theColor;
}

void Label_PMI::SetOriention(const gp_Ax2 &oriention)
{
    myHasOrientation3D = Standard_True;
    myOrientation3D = oriention;
}

void Label_PMI::SetZoomable(const Standard_Boolean theIsZoomable)
{
    myLabelZoomable = theIsZoomable;
}

void Label_PMI::SetHeight(const Standard_Real theHeight)
{
    myFontHeight = theHeight;
}

void Label_PMI::SetPadding(const Standard_Real thePadding)
{
    myFontPadding = thePadding;
}

const gp_Ax2 &Label_PMI::Orientation3D() const
{
    return myOrientation3D;
}

Standard_Boolean Label_PMI::HasOrientation3D() const
{
    return myHasOrientation3D;
}

Standard_Real Label_PMI::calculateStringWidth(const NCollection_Utf8String &str) const
{
    Font_BRepFont aBrepFont(FONT_FILE_PATH, myFontHeight);
    Standard_Real tWidth=0;
    for(NCollection_Utf8Iter anIter = str.Iterator(); *anIter != 0; ) {
        Standard_Utf32Char aCurrChar = *anIter;
        Standard_Utf32Char aNextChar = *(++anIter);
        tWidth += aBrepFont.AdvanceX(aCurrChar,aNextChar);
    }

    return tWidth;
}

gp_Trsf Label_PMI::calculateOrientionTrsf() const
{
    gp_Trsf result;

    if(!myHasOrientation3D)
        return result;

    result.SetTransformation(gp_Ax3(myOrientation3D), gp_Ax3(gp::XOY()));

    return result;
}

StringBox Label_PMI::calculateStringBox(const NCollection_Utf8String &str) const
{
    Standard_Real aWidth = calculateStringWidth(str);
    Standard_Real aHeight = myFontHeight;

    StringBox box;
    // setup the 4 corner points at the plane gp_Ax2({0,0,0},{0,0,1})
    box.bottomLeft = gp_Pnt (-myFontPadding, -0.3*aHeight, 0.0);
    box.topLeft = gp_Pnt (-myFontPadding, aHeight, 0.0);
    box.topRight = gp_Pnt (aWidth+myFontPadding, aHeight, 0.0);
    box.bottomRight = gp_Pnt (aWidth+myFontPadding, -0.3*aHeight, 0.0);
    return box;
}

TopoDS_Shape Label_PMI::ComputeStringList(const NCollection_Utf8StringList &strlist, Standard_Real &width) const
{
    if(strlist.isEmpty())
        return TopoDS_Shape();

    TopoDS_Compound result;
    BRep_Builder compBuilder;
    compBuilder.MakeCompound(result);

    gp_Vec offset;
    offset.SetXYZ({0,0,0});
    width = myFontPadding;
    for(int i=0;i<strlist.size();++i) {
        if(strlist[i].IsEmpty())
            continue;

        // 1.draw the shape of str
        Font_BRepFont aBrepFont(FONT_FILE_PATH, myFontHeight);
        Font_BRepTextBuilder aTextBuilder;
        TopoDS_Shape txtShape = aTextBuilder.Perform(aBrepFont, strlist[i]);

        // 2.draw the str box
        StringBox box = calculateStringBox(strlist[i]);
        gp_Pnt end = box.bottomRight;
        TopoDS_Shape boxShape = box.ToShape();

        // 3.offset the txtShape and boxShape
        gp_Trsf translate;
        translate.SetTranslation(offset);
        BRepBuilderAPI_Transform txtTrans(txtShape,translate);
        BRepBuilderAPI_Transform boxTrans(boxShape,translate);
        compBuilder.Add(result,txtTrans.Shape());
        compBuilder.Add(result,boxTrans.Shape());

        // 4.set the value of offset
        gp_Pnt next = offset.XYZ() + end.XYZ() + gp_Pnt(myFontPadding,0.3*myFontHeight,0).XYZ();
        width += box.BoxWidth();
        offset.SetXYZ(next.XYZ());
    }

    // transform the compound to the work plane
    gp_Trsf apply = calculateOrientionTrsf();
    BRepBuilderAPI_Transform aTransform(result,apply);

    return aTransform.Shape();
}

TopoDS_Shape Label_PMI::ComputeStringWithSupAndSub(const NCollection_Utf8String &main,
                                                   const NCollection_Utf8String &sub,
                                                   const NCollection_Utf8String &sup,
                                                   Standard_Real &width)
{
    if(main.IsEmpty())
        return TopoDS_Shape();

    TopoDS_Compound result;
    BRep_Builder compBuilder;
    compBuilder.MakeCompound(result);

    // 1.draw the main string with full font height
    Font_BRepFont aBrepFont(FONT_FILE_PATH, myFontHeight);
    Font_BRepTextBuilder aTextBuilder;
    TopoDS_Shape mainShape = aTextBuilder.Perform(aBrepFont, main);
    compBuilder.Add(result,mainShape);

    // 2.draw the sub&sup string with half height
    Font_BRepFont bBrepFont(FONT_FILE_PATH, 0.5*myFontHeight);
    TopoDS_Shape subShape = aTextBuilder.Perform(bBrepFont, sub);
    TopoDS_Shape supShape = aTextBuilder.Perform(bBrepFont, sup);

    // 3.offset the sub&sup shape
    width = calculateStringWidth(main);
    gp_Pnt rightBottom = gp_Pnt (width, 0, 0.0);
    gp_Pnt rightMid = gp_Pnt (width, 0.5*myFontHeight, 0.0);
    gp_Trsf subTrsf;
    subTrsf.SetTranslation(rightBottom.XYZ());
    gp_Trsf supTrsf;
    supTrsf.SetTranslation(rightMid.XYZ());

    Standard_Real widSub = calculateStringWidth(sub);
    Standard_Real widSup = calculateStringWidth(sup);
    width += 0.5*qMax(widSub,widSup);

    BRepBuilderAPI_Transform aTransform(subShape,subTrsf);
    BRepBuilderAPI_Transform bTransform(supShape,supTrsf);
    compBuilder.Add(result,aTransform.Shape());
    compBuilder.Add(result,bTransform.Shape());

    // 4.transform the compound to the work plane
    gp_Trsf apply = calculateOrientionTrsf();
    BRepBuilderAPI_Transform compTrans(result,apply);

    return compTrans.Shape();
}

Standard_Real StringBox::BoxWidth() const
{
    return topLeft.Distance(topRight);
}

TopoDS_Shape StringBox::ToShape() const
{
    BRepBuilderAPI_MakePolygon aMaker;
    aMaker.Add(bottomLeft);
    aMaker.Add(topLeft);
    aMaker.Add(topRight);
    aMaker.Add(bottomRight);
    aMaker.Add(bottomLeft);
    aMaker.Build();

    if(aMaker.IsDone()) {
        return aMaker.Shape();
    }
    else return TopoDS_Shape();
}
