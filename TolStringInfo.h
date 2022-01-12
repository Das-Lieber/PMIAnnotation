#ifndef TOLSTRINGINFO_H
#define TOLSTRINGINFO_H

#include <QString>
#include <NCollection_String.hxx>

//! string used in Qt ui
const QString UTFLinearity = QString::fromUtf8("\342\217\244");       //⏤
const QString UTFPlanarity = QString::fromUtf8("\342\217\245");       //⏥
const QString UTFCircularity = QString::fromUtf8("\342\227\213");     //○
const QString UTFCylindricity = QString::fromUtf8("\342\214\255");    //⌭
const QString UTFLineProfile = QString::fromUtf8("\342\214\222");     //⌒
const QString UTFPlaneProfile = QString::fromUtf8("\342\214\223");    //⌓
const QString UTFParallelism = QString::fromUtf8("\342\210\245");     //∥
const QString UTFVerticality = QString::fromUtf8("\342\237\202");     //⟂
const QString UTFGradient = QString::fromUtf8("\342\210\240");        //∠
const QString UTFRunout = QString::fromUtf8("\342\255\247");          //⭧
const QString UTFTotalRunout = QString::fromUtf8("\342\214\260");     //⌰
const QString UTFPosition = QString::fromUtf8("\342\214\226");        //⌖
const QString UTFAxiality = QString::fromUtf8("\342\214\276");        //⦾
const QString UTFSymmetry = QString::fromUtf8("\342\214\257");        //⌯

const QString UTFRadius = QString::fromUtf8("\342\214\200");          //⌀
const QString UTFMMC = QString::fromUtf8("\342\223\202");             //Ⓜ 最大实体要求
const QString UTFLMC = QString::fromUtf8("\342\223\201");             //Ⓛ 最小实体要求
const QString UTFRSize = QString::fromUtf8("\342\223\210");           //Ⓢ 无论特征大小如何
const QString UTFTanBase = QString::fromUtf8("\342\223\211");         //Ⓣ 相切基准面
const QString UTFFree = QString::fromUtf8("\342\222\273");            //Ⓕ 自由状态
const QString UTFPTZ = QString::fromUtf8("\342\223\205");             //Ⓟ 投影公差
const QString UTFSquare = QString::fromUtf8("\342\226\241");          //□ 方形
const QString UTFUAC = QString::fromUtf8("\342\223\212");             //Ⓤ 不相等排列的轮廓
const QString UTFPan = QString::fromUtf8("\342\226\267");             //▷ 平移

//! string used in occ 3d label
const NCollection_String FONT_FILE_PATH = "./Font/Label.ttf";

const Standard_WideChar FONT_Linearity[2] = {0x0130,};
const Standard_WideChar FONT_Planarity[2] = {0x0131,};
const Standard_WideChar FONT_Circularity[2] = {0x0132,};
const Standard_WideChar FONT_Cylindricity[2] = {0x0133,};
const Standard_WideChar FONT_LineProfile[2] = {0x0134,};
const Standard_WideChar FONT_PlaneProfile[2] = {0x0135,};
const Standard_WideChar FONT_Parallelism[2] = {0x0137,};
const Standard_WideChar FONT_Verticality[2] = {0x0138,};
const Standard_WideChar FONT_Gradient[2] = {0x0136,};
const Standard_WideChar FONT_Runout[2] = {0x013c,};
const Standard_WideChar FONT_TotalRunout[2] = {0x013d,};
const Standard_WideChar FONT_Position[2] = {0x0139,};
const Standard_WideChar FONT_Axialit[2] = {0x013a,};
const Standard_WideChar FONT_Symmetry[2] = {0x013b,};

const Standard_WideChar FONT_DEGREE[2] = {0x0060,};
const Standard_WideChar FONT_Radius[2] = {0x0040,};
const Standard_WideChar FONT_MMC[2] = {0x0158,};
const Standard_WideChar FONT_LMC[2] = {0x0160,};
const Standard_WideChar FONT_RSize[2] = {0x015e,};
const Standard_WideChar FONT_TanBase[2] = {0x015f,};
const Standard_WideChar FONT_Free[2] = {0x0164,};
const Standard_WideChar FONT_PTZ[2] = {0x015b,};
const Standard_WideChar FONT_Square[2] = {0x019b,};
const Standard_WideChar FONT_UAC[2] = {0x0161,};
const Standard_WideChar FONT_Pan[2] = {0x018c,};

#endif // TOLSTRINGINFO_H
