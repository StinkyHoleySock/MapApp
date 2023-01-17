/****** MAPTYPE.H ************** Belenkov O.V.   ****** 08/08/16 ***
******* MAPTYPE.H ************** Kruzhkov A.E.   ****** 23/10/15 ***
******* MAPTYPE.H ************** Panteleeva N.A  ****** 14/11/14 ***
******* MAPTYPE.H ************** Dishlenko S.G.  ****** 07/06/16 ***
******* MAPTYPE.H ************** Alexeev    S.A. ****** 26/06/12 ***
******* MAPTYPE.H ************** Shabakov D.A.   ****** 21/07/16 ***
******* MAPTYPE.H ************** Gorbunov A.V.   ****** 17/04/13 ***
******* MAPTYPE.H ************** Tarakanova I.R. ****** 19/05/11 ***
******* MAPTYPE.H ************** Borzov A.U.     ****** 22/12/14 ***
******* MAPTYPE.H ************** Nefedieva O.A.  ****** 13/01/12 ***
******* MAPTYPE.H ************** Железняков В.A. ****** 06/05/13 ***
******* MAPTYPE.H ************** Derevyagina Zh.A.***** 05/07/16 ***
******* MAPTYPE.H ************** Korjilov A.      ***** 04/02/16 ***
******* MAPTYPE.H ************** Savelov A.S.     ***** 04/09/15 ***
******* MAPTYPE.H ************** Vitsko D.A.      ***** 27/07/16 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*               ОПИСАНИЕ БАЗОВЫХ ТИПОВ ПЕРЕМЕННЫХ                  *
*                                                                  *
*******************************************************************/

#ifndef MAPTYPE_H
#define MAPTYPE_H

#ifndef MAPMACRO_H
 #include "mapmacro.h"
#endif

#ifndef MAPERR_RH
  #include "maperr.rh"
#endif

#pragma pack(push,1)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ОБ'ЯВЛЕНИЕ ИДЕНТИФИКАТОРОВ
// (ЯВЛЯЮТСЯ УКАЗАТЕЛЯМИ НА СООТВЕТСТВУЮЩИЕ КЛАССЫ С++ )
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(_M_X64) || defined(BUILD_DLL64)

typedef __int64   HMAP;    // ИДЕНТИФИКАТОР ОТКРЫТОЙ ВЕКТОРНОЙ КАРТЫ
                           // (УКАЗАТЕЛЬ НА TMapAccess)
typedef __int64   HSITE;   // ИДЕНТИФИКАТОР ОТКРЫТОЙ ПОЛЬЗОВАТЕЛЬСКОЙ
                           // КАРТЫ (УКАЗАТЕЛЬ НА ЭЛЕМЕНТ ЦЕПОЧКИ)
                           // ЕСЛИ РАВЕН HMAP (ИЛИ 0) - РАБОТАЕМ С
                           // КАРТОЙ МЕСТНОСТИ (ФОНОВОЙ КАРТОЙ)
typedef __int64   HOBJ;    // ИДЕНТИФИКАТОР ОБ'ЕКТА КАРТЫ
                           // (УКАЗАТЕЛЬ НА TObjectInfo)
typedef __int64   HSELECT; // ИДЕНТИФИКАТОР КОНТЕКСТА ПОИСКА
                           // (УКАЗАТЕЛЬ НА TMapSelect)
typedef __int64   HRSC;    // ИДЕНТИФИКАТОР КЛАССИФИКАТОРА КАРТЫ
                           // (УКАЗАТЕЛЬ НА TMapRsc)
typedef __int64   HMTR3D;  // ИДЕНТИФИКАТОР ОТКРЫТОГО СЕАНСА
                           // ТРЕХМЕРНОЙ ВИЗУАЛИЗАЦИИ МЕСТНОСТИ
                           // (УКАЗАТЕЛЬ НА TMtr3D)
typedef __int64   HCROSS;  // (УКАЗАТЕЛЬ НА TObjectCut)
typedef __int64   HPOINT;  // (УКАЗАТЕЛЬ НА структуру CROSSPOINT)
typedef __int64   HDRAW;   // (УКАЗАТЕЛЬ НА структуру TDrawEdit)
typedef __int64   HPRINTER;// (УКАЗАТЕЛЬ НА TPrinter)
typedef __int64   HVECT;   // (УКАЗАТЕЛЬ НА TVectorImageEdit)    

// ИНТЕРФЕЙС PANTASK/PANACTION
typedef __int64  HPANACTION;   // УКАЗАТЕЛЬ НА TPanAction
typedef __int64  HMAPACTION;   // УКАЗАТЕЛЬ НА TUserAction

typedef __int64  HPANTASK;     // УКАЗАТЕЛЬ НА TPanTask
typedef __int64  HMAPTASK;     // УКАЗАТЕЛЬ НА TUserTask

typedef __int64  HMAPDOC;      // УКАЗАТЕЛЬ НА TMapWindow

typedef __int64  HOBJSET;      // УКАЗАТЕЛЬ НА TObjectSet

typedef __int64  HFORMULA;     // УКАЗАТЕЛЬ НА TStrFormula  

typedef __int64  HPARAM;       // ПАРАМЕТР ФУНКЦИИ ОБРАБОТКИ СОБЫТИЯ

#else

typedef long int  HMAP;    // ИДЕНТИФИКАТОР ОТКРЫТОЙ ВЕКТОРНОЙ КАРТЫ
                           // (УКАЗАТЕЛЬ НА TMapAccess)
typedef long int  HSITE;   // ИДЕНТИФИКАТОР ОТКРЫТОЙ ПОЛЬЗОВАТЕЛЬСКОЙ
                           // КАРТЫ (УКАЗАТЕЛЬ НА ЭЛЕМЕНТ ЦЕПОЧКИ)
                           // ЕСЛИ РАВЕН HMAP (ИЛИ 0) - РАБОТАЕМ С
                           // КАРТОЙ МЕСТНОСТИ (ФОНОВОЙ КАРТОЙ)
typedef long int  HOBJ;    // ИДЕНТИФИКАТОР ОБ'ЕКТА КАРТЫ
                           // (УКАЗАТЕЛЬ НА TObjectInfo)
typedef long int  HSELECT; // ИДЕНТИФИКАТОР КОНТЕКСТА ПОИСКА
                           // (УКАЗАТЕЛЬ НА TMapSelect)
typedef long int  HRSC;    // ИДЕНТИФИКАТОР КЛАССИФИКАТОРА КАРТЫ
                           // (УКАЗАТЕЛЬ НА TMapRsc)
typedef long int  HMTR3D;  // ИДЕНТИФИКАТОР ОТКРЫТОГО СЕАНСА
                           // ТРЕХМЕРНОЙ ВИЗУАЛИЗАЦИИ МЕСТНОСТИ
                           // (УКАЗАТЕЛЬ НА TMtr3D)
typedef long int  HCROSS;  // (УКАЗАТЕЛЬ НА TObjectCut)
typedef long int  HPOINT;  // (УКАЗАТЕЛЬ НА структуру CROSSPOINT)
typedef long int  HDRAW;   // (УКАЗАТЕЛЬ НА структуру TDrawEdit)
typedef long int  HPRINTER;// (УКАЗАТЕЛЬ НА TPrinter)
typedef long int  HVECT;   // (УКАЗАТЕЛЬ НА TVectorImageEdit)    

// ИНТЕРФЕЙС PANTASK/PANACTION
typedef long int HPANACTION;   // УКАЗАТЕЛЬ НА TPanAction
typedef long int HMAPACTION;   // УКАЗАТЕЛЬ НА TUserAction

typedef long int HPANTASK;     // УКАЗАТЕЛЬ НА TPanTask
typedef long int HMAPTASK;     // УКАЗАТЕЛЬ НА TUserTask

typedef long int HMAPDOC;      // УКАЗАТЕЛЬ НА TMapWindow

typedef long int HOBJSET;      // УКАЗАТЕЛЬ НА TObjectSet

typedef long int HFORMULA;     // УКАЗАТЕЛЬ НА TStrFormula       

typedef long int HPARAM;       // ПАРАМЕТР ФУНКЦИИ ОБРАБОТКИ СОБЫТИЯ
#endif

typedef long int HIMAGE;       // ИДЕНТИФИКАТОР TCopyImage (ИНДЕКС В СПИСКЕ)
typedef void *   HMAPREG;      // ИДЕНТИФИКАТОР СПИСКА ПАРАМЕТРОВ СИСТЕМ ОТСЧЕТА
typedef void *   HPAINT;       // ИДЕНТИФИКАТОР TPaintControl


enum KIND                 // ВИДЫ ФОРМАТОВ МЕТРИКИ
{
  IDSHORT2  = 0x7FFF7FFF, // двухбайтовая целочисленная
  IDLONG2   = 0x7FFE7FFE, // четырехбайтовая целочисленная
  IDFLOAT2  = 0x7FFD7FFD, // с плавающей запятой
  IDDOUBLE2 = 0x7FFC7FFC, // с плавающей запятой двойной точностью
  IDSHORT3  = 0x7FFB7FFB, // двухбайтовая целочисленная трехмерная
  IDLONG3   = 0x7FFA7FFA, // четырехбайтовая целочисленная трехмерная
  IDFLOAT3  = 0x7FF97FF9, // с плавающей запятой трехмерная
  IDDOUBLE3 = 0x7FF87FF8, // с плавающей запятой двойной точностью трехмерная
  IDBAD     = 0x7FF87FF7  // неизвестный вид
};


#if defined(__BORLANDC__)              
#ifndef max
 #define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
 #define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
#endif

#define LT left
#define RT right
#define UP top
#define DN bottom

// Точность обработки координат при визуализации
#define DELTANULL  ((double)(1e-3))   

// Точность обработки координат при выполнении расчетов
#define DOUBLENULL  ((double)(1e-6))  

// Функция округления 
#define ROUND(x)    (((x) < 0) ? (int)((x) - 0.5) : (int)((x) + 0.5))

// Число градусов в 1 радиане
#define DEGREEINRAD  57.29577951308232l

// Число секунд в 1 радиане
#define SECONDINRAD  206264.806247096l

typedef struct SHORTPOINT    // КООРДИНАТЫ ТОЧКИ
{
  unsigned short int X;
  unsigned short int Y;
}
  SHORTPOINT;

typedef struct LONGPOINT     // КООРДИНАТЫ ТОЧКИ (ДЛИННАЯ МЕТРИКА)
{
  union { int X; int x; };               // 26/03/12
  union { int Y; int y; };
}
  LONGPOINT;

typedef struct FLOATPOINT    // КООРДИНАТЫ ТОЧКИ (ПЛАВАЮЩАЯ МЕТРИКА)
{
  float X;
  float Y;
}
  FLOATPOINT;

typedef struct DOUBLEPOINT   // КООРДИНАТЫ ТОЧКИ (ПЛАВАЮЩАЯ МЕТРИКА)
{
  union { double X; double x; };
  union { double Y; double y; };

  void operator  = (double value)  {         x  = value;   y  = value;  }
   int operator != (double value)  { return (x != value || y != value); }
   int operator == (double value)  { return (x == value && y == value); }
}
  DOUBLEPOINT;


typedef struct DRAWPOINT     // Аналог TPoint
{
#ifndef WINCE
  union { XYTYPE x; XYTYPE X; };        
  union { XYTYPE y; XYTYPE Y; };
#else
  union { long int x; long int X; };
  union { long int y; long int Y; };
#endif

  void operator  = (int value)  {         x  = value;   y  = value;  }
   int operator != (int value)  { return (x != value || y != value); }
   int operator == (int value)  { return (x == value && y == value); }

  DRAWPOINT& operator - (DRAWPOINT& draw)
    { x -= draw.x;  y -= draw.y;  return *this;}

  DRAWPOINT& operator + (DRAWPOINT& draw)
    { x += draw.x;  y += draw.y;  return *this;}
}
  DRAWPOINT;


typedef struct DRAWSIZE      // Аналог TSize
{
  union { int cx; int CX; };
  union { int cy; int CY; };

  void operator  = (int value)  {         cx  = value;   cy  = value;  }
  int operator != (int value)  { return (cx != value || cy != value); }
  int operator == (int value)  { return (cx == value && cy == value); }

  DRAWSIZE& operator - (DRAWSIZE& draw)
    { cx -= draw.cx;  cy -= draw.cy;  return *this;}

  DRAWSIZE& operator + (DRAWSIZE& draw)
    { cx += draw.cx;  cy += draw.cy;  return *this;}
}
  DRAWSIZE;


typedef struct DRAWLINE      // Координаты линии
{
  DRAWPOINT First;
  DRAWPOINT Second;
}
  DRAWLINE;

typedef struct DRAWBOX       // Координаты наклонного прямоугольника
{
  DRAWPOINT Point1;
  DRAWPOINT Point2;
  DRAWPOINT Point3;
  DRAWPOINT Point4;
}
  DRAWBOX;


//-----------------------------------------------------------------
// КООРДИНАТЫ ПРЯМОУГОЛЬНИКА В РАЙОНЕ РАБОТ
// (X вверх, Y вправо, границы включаются)
//-----------------------------------------------------------------

typedef struct FRAME         // ЦЕЛЫЕ
{
  int X1;                                                   // 26/03/12
  int Y1;
  int X2;
  int Y2;

  void operator = (int value)
  {
    X1 = value;  Y1 = value;
    X2 = value;  Y2 = value;
  }

  FRAME operator = (FRAME& frame)
  {
    X1 = frame.X1;  Y1 = frame.Y1;
    X2 = frame.X2;  Y2 = frame.Y2;  return *this;
  }

  int operator != (int value)
  {
    return ( X1 != value || Y1 != value ||
             X2 != value || Y2 != value );
  }

  int operator == (int value)
  {
    return ( X1 == value && Y1 == value &&
             X2 == value && Y2 == value);
  }
}
  FRAME;


typedef struct DFRAME        // ПЛАВАЮЩИЕ
{
  double X1;
  double Y1;
  double X2;
  double Y2;

  void operator = (double value)
  {
    X1 = value;  Y1 = value;
    X2 = value;  Y2 = value;
  }
}
  DFRAME;


//-----------------------------------------------------------------
// КООРДИНАТЫ ПРЯМОУГОЛЬНИКА В ОКНЕ
// (X вправо, Y вниз, границы включаются)
//-----------------------------------------------------------------

typedef struct BOX           // ЦЕЛЫЕ
{
  union { int LT;  int lt; };                 // 26/03/12
  union { int UP;  int up; };
  union { int RT;  int rt; };
  union { int DN;  int dn; };

  void operator = (int value)
  {
    LT = value;  UP = value;
    RT = value;  DN = value;
  }
}
  BOX;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++  СТРУКТУРА ЗАДАНИЯ МЕТРИКИ ОБ'ЕКТА  +++++++++
//++++++++++++++  В СИСТЕМЕ КООРДИНАТ ОКНА  +++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Значения поля Flags (для IMLAPI)
#define PF_SKIPNUMBER    2  // Флаг пропуска текста с #    
#define PF_UNICODETEXT   4  // Флаг текста в UNICODE       
#define PF_SKIPSEMCOLOR  8  // Флаг пропуска цвета по семантике 
#define PF_SCALETOLEVEL 16  // Флаг масштабирования знака по заданным уровням   // 23/10/14
#define PF_FACTOR10     32  // Флаг записи координат со 10 кратным увеличением  // 11/03/16
#define PF_MULTIPOLYGON 64  // Флаг рисования мультиполигона

#if !defined(_M_X64) && !defined(BUILD_DLL64)       
typedef struct /*_DLLCLASS*/ POLYDATA  // 27/03/13
{
  POLYDATA() { memset(this, 0, sizeof(*this)); }

  DRAWPOINT * Points;  // Адрес массива объектов типа POINT
                       // с координатами точек
  int * PolyCounts;    // Адрес массива количества точек                  // 26/03/12
  int        Count;    // Число ломаных (многоугольников, текстов)
  char **      Text;   // Указатель на массив адресов текстов
                       // подписей (сами тексты лежат
                       // в первоначальной метрике объекта)
  int    AlignCode;    // Код выравнивания для отображения текста
  char *   Semantic;   // Указатель на семантику объекта (SEMANTIC *)
                       // Если семантика не нужна в изображении - равно 0
  BOX        Border;   // Прямоугольник габаритов объекта в пикселах
                       // относительно области рисования
  double  ShowScale;   // Текущий масштаб отображения электронной карты
}
  POLYDATA;

typedef struct /*_DLLCLASS */ POLYDATAEX
{
  POLYDATAEX() { memset(this, 0, sizeof(POLYDATAEX)); }

  DRAWPOINT * Points;  // Адрес массива объектов типа POINT
                       // с координатами точек
  int * PolyCounts;    // Адрес массива количества точек                 // 26/03/12
  int        Count;    // Число ломаных (многоугольников, текстов)
  char **      Text;   // Указатель на массив адресов текстов
                       // подписей (сами тексты лежат
                       // в первоначальной метрике объекта)
  short   AlignCode;   // Код выравнивания для отображения текста
  short       Flags;   // Флаги обработки метрики - равно 0          
  char *   Semantic;   // Указатель на семантику объекта (SEMANTIC *)
                       // Если семантика не нужна в изображении - равно 0
  BOX        Border;   // Прямоугольник габаритов объекта в пикселах
                       // относительно области рисования
  double  ShowScale;   // Текущий масштаб отображения электронной карты
  HRSC       MapRsc;   // Классификатор карты, содержащей отображаемый объект
  float      Height;   // Высота первой точки объекта (если 3D-координаты) или 0 
  float      DX;       // Приращение от первой точки до второй на карте          
  float      DY;       // в единицах карты - для ориентирования векторных объектов
}
  POLYDATAEX;
#else
typedef struct POLYDATAEX
{
  POLYDATAEX() { memset(this, 0, sizeof(POLYDATAEX)); }

  DRAWPOINT * Points;  // Адрес массива объектов типа POINT
                       // с координатами точек
  int * PolyCounts;    // Адрес массива количества точек                 // 26/03/12
  char **      Text;   // Указатель на массив адресов текстов
                       // подписей (сами тексты лежат
                       // в первоначальной метрике объекта)
  char *   Semantic;   // Указатель на семантику объекта (SEMANTIC *)
                       // Если семантика не нужна в изображении - равно 0
  int        Count;    // Число ломаных (многоугольников, текстов)
  short   AlignCode;   // Код выравнивания для отображения текста
  short       Flags;   // Флаги обработки метрики - равно 0         
  BOX        Border;   // Прямоугольник габаритов объекта в пикселах
                       // относительно области рисования
  double  ShowScale;   // Текущий масштаб отображения электронной карты
  HRSC       MapRsc;   // Классификатор карты, содержащей отображаемый объект
  float      Height;   // Высота первой точки объекта (если 3D-координаты) или 0 
  float      DX;       // Приращение от первой точки до второй на карте          
  float      DY;       // в единицах карты - для ориентирования векторных объектов
  float      Zero;     // Выравнивание
}
  POLYDATAEX;

typedef POLYDATAEX POLYDATA;
#endif

// Резервное число строк в DIB
#define SHIFTUP32 0

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Максимальные значения характеристик метрики
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(COMPACTGIS) || defined(MINIDATA)
#define GD_POINTCOUNT  64000L  // Наибольшее возможное количество точек
                               // метрики объекта
#define GD_POLYCOUNT   16000L  // Наибольшее возможное количество
                               // подобъектов метрики
#define GD_TEXTCOUNT   16000L  // Наибольшее возможное количество
                               // текстов в метрике подписи

#define MAXDATALENGTH (1024*1024L) // Максимальная длина метрики

#else

#define GD_POINTCOUNT (1024*1024L) // Наибольшее возможное количество точек
                                   // метрики объекта
#define GD_POLYCOUNT   64000L      // Наибольшее возможное количество
                                   // подобъектов метрики
#define GD_TEXTCOUNT   32000L      // Наибольшее возможное количество
                                   // текстов в метрике подписи

#define MAXDATALENGTH (32*1024*1024L) // Максимальная длина метрики

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++ Структуры форматов представления метрики объектов     +++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct TEXTDATA      // Текст подписи в метрике
{                            // Общая длина = Length + 2
  unsigned char Length;      // Число символов
           char Text[256];   // Текст переменной длины
  unsigned char Zero;        // Замыкающий ноль
}
  TEXTDATA;

typedef struct TEXTDATAUN    // Текст подписи в кодировке UNICODE в метрике
{                            // Общая длина = Length + 2
  unsigned char  Pass;       // Байт для выравнивания начала поля Text на 2 байта
  unsigned char  Length;     // Длина текста в байтах (кратно 2)
          WCHAR  Text[127];  // Текст переменной длины
  unsigned short Zero;       // Замыкающий ноль
}
  TEXTDATAUN;

// ВИДЫ ФОРМАТОВ ОТДЕЛЬНОЙ ТОЧКИ МЕТРИКИ
typedef struct
{
  unsigned short int X;
  unsigned short int Y;
}
  XYSHORT;

typedef struct
{
  unsigned short int X;
  unsigned short int Y;
           float     H;
}
  XYHSHORT;

typedef struct
{
  int X;                             // 26/03/12
  int Y;
}
  XYLONG;

typedef struct
{
  int X;
  int Y;
  float    H;
}
  XYHLONG;

typedef struct
{
  float X;
  float Y;
}
  XYFLOAT;

typedef DOUBLEPOINT XYDOUBLE;          // 12/02/10

typedef struct
{
  float X;
  float Y;
  float H;
}
  XYHFLOAT;

typedef struct
{
  double X;
  double Y;
  double H;
}
  XYHDOUBLE;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++ Структуры интерфейса MAPAPI +++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Идентификаторы, возвращаемые функцией mapCheckFile.

enum  MAPFILES                // ИДЕНТИФИКАТОРЫ ФАЙЛОВ (Intel)
{
    FILE_SXF  = 0x00465853,   // ДВОИЧНЫЙ SXF ФАЙЛ
    FILE_TXT  = 0x4658532E,   // ТЕКСТОВЫЙ SXF ФАЙЛ (TXF с меткой .SXF - MAP)
    FILE_TXFMAP = 0x4658532E, // ТЕКСТОВЫЙ SXF ФАЙЛ (TXF с меткой .SXF - MAP)
    FILE_DIR  = 0x524944,     // ТЕКСТОВЫЙ DIR ФАЙЛ
    FILE_PCX  = 0x10,         // ФАЙЛ PCX (ОПРЕДЕЛЯЕТСЯ НЕ ЧЕТКО)
    FILE_BMP  = 0x4D42,       // ФАЙЛ BMP
    FILE_TIFF = 0x4949,       // ФАЙЛ TIFF
    FILE_BIGTIFF = 0x49492B,  // ФАЙЛ bigTIFF                    // 18/09/13
    FILE_JPEG = 0xD8FF,       // ФАЙЛ JPEG
    FILE_MRSID= 0x6469736D,   // ФАЙЛ MrSid (SID,JPEG2000,NITF)  // 26/09/08
    FILE_MAP  = 0x00505350,   // ПАСПОРТ РАЙОНА РАБОТ MAP
    FILE_MAPSIT = 0x00544953, // ПАСПОРТ РАЙОНА РАБОТ SIT        // 27/10/08
    FILE_SITX = 0x58544953,   // ФАЙЛ SITX                       // 12/03/13
    FILE_RSW  = 0x00575352,   // ФАЙЛ RSW
    FILE_RSWTIFF = 0x54575352,//(RSWT)RSW+TIFF                   // 24/05/11
    FILE_MTW  = 0x0057544D,   // ФАЙЛ MTW
    FILE_MTL  = 0x004C544D,   // ФАЙЛ MTL
    FILE_MTD  = 0x0044544D,   // ФАЙЛ MTD  28/11/07
    FILE_SIT  = 0x5449532E,   // ТЕКСТОВЫЙ SXF ФАЙЛ (TXF с меткой .SIT - SIT)
    FILE_TXFSIT = 0x5449532E, // ТЕКСТОВЫЙ SXF ФАЙЛ (TXF с меткой .SIT - SIT)
    FILE_DXF  = 0x00465844,   // ФАЙЛ DXF
    FILE_MIF  = 0x0046494D,   // ФАЙЛ MAPINFO
    FILE_S57  = 0x00003000,   // ФАЙЛ S57(КАТАЛОГ)
    FILE_DGN  = 0x004E4744,   // ФАЙЛ Macrostation(DGN)    
    FILE_MPT  = 0x0054504D,   // ФАЙЛ MPT(ПРОЕКТ)
    FILE_RSC  = 0x00435352,   // ФАЙЛ RSC
    FILE_MTQ  = 0x0051544D,   // ФАЙЛ MTQ
    FILE_PLN  = 0x004E4C50,   // ФАЙЛ PLN (Talka)                
    FILE_SHP  = 0x00504853,   // ФАЙЛ SHP (ArcView)              
    FILE_PLS  = 0x00534C50,   // ФАЙЛ PLS (Список растров)       
    FILE_TEXT = 0x00545854,   // Файл TXT(геодезические приборы) 
    FILE_GPS  = 0x47504724,   // Файл GPS/NMEA ($GPG)            
    FILE_GRD  = 0x00445247,   // ФАЙЛ GRD (матрицы формата FOTOMOD) 
    FILE_DBF  = 0x00464244,   // ФАЙЛ DBF (База данных)             
    FILE_TIN  = 0x004E4954,   // ФАЙЛ TIN                           
    FILE_XLS  = 0x0000CFD0,   // ФАЙЛ EXCEL                         
    FILE_WMS  = 0x00534D57,   // URL для подключения к WMS-сервису  
    FILE_IMG  = 0x00494D47,   // ФАЙЛ IMG                           // 15/09/12
    FILE_PNG  = 0x00504E47,   // ФАЙЛ PNG                           // 15/09/12
    FILE_GIF  = 0x00474946,   // ФАЙЛ GIF                           // 15/09/12
    FILE_WFS  = 0x00221497,   // URL для подключения к WFS-сервису  // 29/03/13
    FILE_WCS  = 0x00331924,   // URL для подключения к WCS-сервису  // 17/04/13
    FILE_KML  = 0x25FF5621,   // ФАЙЛ KML                           // 06/05/13
    FILE_GML  = 0x25FF5622,   // ФАЙЛ GML                           // 06/05/13
    FILE_MP   = 0x25FF5623,   // ФАЙЛ MP                            // 06/05/13
    FILE_OZF2 = 0x7778,       // ФАЙЛ OZF2 (графика для OziExplorer)// 20/11/13
    FILE_OZFX3= 0x7780,       // ФАЙЛ OZFX3(графика для OziExplorer)// 20/11/13
    FILE_EMF  = 0x00464D45,   // ФАЙЛ EMF
    FILE_DBM  = 0x3C3F786D,   // ФАЙЛ SQL-карты                     // 27/07/16
    FILE_XML  = 0x25FF5624,   // ФАЙЛ XML                           // 08/08/16 


                              // ИДЕНТИФИКАТОРЫ ФАЙЛОВ (Sparc, Mips)

    FILE_MAP_TURN = 0x50535000, // ПАСПОРТ РАЙОНА РАБОТ
    FILE_MTW_TURN = 0x4D545700, // ФАЙЛ MTW
    FILE_SXF_TURN = 0x53584600, // ДВОИЧНЫЙ SXF ФАЙЛ   
    FILE_DIR_TURN = 0x44495200, // ТЕКСТОВЫЙ DIR ФАЙЛ  
    FILE_RSW_TURN = 0x52535700, // ФАЙЛ RSW  
    FILE_RSC_TURN = 0x52534300, // ФАЙЛ RSC
    FILE_SIT_TURN = 0x2E534954, // ТЕКСТОВЫЙ SXF(.SIT) ФАЙЛ // 30/06/08
    FILE_MTR_TURN = 0x4D545200, // ФАЙЛ MTR
    FILE_MTQ_TURN = 0x4D545100, // ФАЙЛ MTQ
    FILE_RST_TURN = 0x52535400, // ФАЙЛ RST
};

enum                            // ИДЕНТИФИКАТОРЫ ФАЙЛОВ ЛИСТА КАРТЫ
{
    LABEL_HDR      = 0x00524448L,  // HDR0
    LABEL_DAT      = 0x00544144L,  // DAT0
    LABEL_SEM      = 0x004D4553L,  // SEM0
    LABEL_DRW      = 0x00575244L,  // DRW0
    LABEL_VEC      = 0x00434556L,  // VEC0

    LABEL_HDR_TURN = 0x48445200L,  // 0RDH
    LABEL_DAT_TURN = 0x44415400L,  // 0TAD
    LABEL_SEM_TURN = 0x53454D00L,  // 0MES
    LABEL_DRW_TURN = 0x44525700L,  // 0WRD
    LABEL_VEC_TURN = 0x56454300L,  // 0CEV
};

// Префикс к имени карты для WMS
#define WMAPWMS L"WMS#"
#define MAPWMS "WMS#"
#define WMSSIZE 4

// Префикс к имени карты для WMTS         // 07/02/12
#define WMAPWMTS L"WMTS#"
#define MAPWMTS "WMTS#"
#define WMTSSIZE 5

// Префикс к имени карты для WFS // 29/03/13
#define WMAPWFS L"WFS#"
#define MAPWFS "WFS#"
#define WFSSIZE 4

// Префикс к имени карты для WCS // 17/04/13
#define WMAPWCS L"WCS#"
#define MAPWCS "WCS#"
#define WCSSIZE 4

// Признак открытия файла на асинхронную запись
#define GENERIC_OVERLAP  0x0AA

typedef struct FRAME MAPFRAME;

typedef struct DFRAME MAPDFRAME;

typedef struct MAPADJACENTSECTION  // ОПИСАНИЕ СМЕЖНОГО УЧАСТКА
{
  int number;                 // номер участка                          // 26/03/12
  int first;                  // первая точка участка
  int last;                   // последняя точка участка
}
  MAPADJACENTSECTION;

typedef struct MAPADJACENTLISTEX
{
  char ListName[32];           // имя листа
  int  List;                   // номер листа в районе                 // 26/03/12
  int  Key;                    // номер объекта
  int  Object;                 // последовательный номер объекта на листе
  int  Excode;                 // классификационный код объекта
  int  First;                  // первая точка участка
  int  Last;                   // последняя точка участка
}
  MAPADJACENTLISTEX;

// Описание смежного участка // 07/02/14
typedef struct CONNECTPATH
{
  int IsForward;      // направление участка на главном подобъекте
                      // (направление на соседнем подобъекте всегда совпадает
                      // с направлением цифрования)
  int FirstPoint;     // первая точка участка на главном подобъекте
  int LastPoint;      // последняя точка участка на главном подобъекте
  int NearSubject;    // номер соседнего подобъекта
  int NearFirstPoint; // первая точка участка на соседнем подобъекте
  int NearLastPoint;  // последняя точка участка на соседнем подобъекте
}
  CONNECTPATH;

typedef enum how       // КОДЫ УСЛОВИЙ ПОИСКА ПО СЕМАНТИКЕ
{
    CMBAD      = 0,    // ОШИБКА В ДАННЫХ
    CMPASS     = 0,    // ОТСУТСТВИЕ КОДА СЕМАНТИКИ 
      LESS     = 1,    // МЕНЬШЕ
    CMLESS     = 1,    // МЕНЬШЕ
    CMLESSEQ   = 3,    // МЕНЬШЕ ИЛИ РАВНО
      EQUAL    = 2,    // РАВНО
    CMEQUAL    = 2,    // РАВНО
    CMNOTEQ    = 5,    // НЕ РАВНО
      MORE     = 4,    // БОЛЬШЕ
    CMMORE     = 4,    // БОЛЬШЕ
    CMMOREEQ   = 6,    // БОЛЬШЕ ИЛИ РАВНО
    CMANY      = 7,    // ЛЮБОЕ ЗНАЧЕНИЕ

    // КОДЫ ОБОБЩАЮЩИХ УСЛОВИЙ
    CMOR       = 16,   // "ИЛИ" - ХОТЯ БЫ ОДНО ВЫПОЛНЯЕТСЯ
    CMAND      = 32,   // "И"   - ВСЕ ДОЛЖНЫ ВЫПОЛНИТЬСЯ
}
    HOW;

// Флажки, определяющие порядок поиска объектов
// Если указано WO_INMAP - номер карты определяется
// из HSELECT !

enum SEEKTYPE        // ПОРЯДОК ПОИСКА ОБ'ЕКТОВ
{
   WO_FIRST  = 0,         // Первый в цепочке
   WO_LAST   = 2,         // Последний в цепочке
   WO_NEXT   = 4,         // Следующий за найденным ранее
   WO_BACK   = 8,         // Предыдущий от ранее найденного
   WO_CANCEL = 16,        // Включая удаленные объекты
   WO_INMAP  = 32,        // Только по одной карте (соответствующей HSELECT)
   WO_VISUAL = 64,        // Поиск только среди видимых объектов
   WO_VISUALIGNORE = 128, // Поиск среди всех объектов без учета видимости
   WO_SKIPTEXT = 256,     // Текст при поиске выбирать последним 
};

enum SEEKINAREA                // 26/04/11
{
   SO_SEEKBYDISTANCE     = 0,  // Поиск по расстоянию
   SO_SEEKINSIDEANDCROSS = 1,  // Поиск внутри области, включая пересечение границы области
   SO_SEEKINSIDEONLY     = 2,  // Поиск внутри области, исключая пересечение границы
   SO_SEEKOUTSIDEONLY    = 4,  // Поиск вне области, исключая пересечение границы
};

enum SELECTTYPEFLAG  // СПОСОБ ВЫДЕЛЕНИЯ ОБЪЕКТОВ
{
   STF_CONTOUR = 0,  // Выделение контуров
   STF_OBJECT  = 1,  // Выделение всего изображения объекта
};

enum SELECTLISTCODE           // ПРАВИЛО ФОРМИРОВАНИЯ СПИСКА УСЛОВИЙ ПОИСКА
{
   SELECTLIST_EXCLUSIVE = 0,  // Добавляемое условие единственное в списке
   SELECTLIST_UNION     = 1,  // Условие логически складывается с остальными
   SELECTLIST_CROSS     = 2,  // Условие логически умножается с остальными
};

enum SELECTMEASURE            // ТИП ИЗМЕРЕНИЙ ОБ"ЕКТОВ ДЛЯ ПОИСКА 
{
   MEASURE_LENGTH      = 1,   // Длина (м)
   MEASURE_SQUARE      = 2,   // Площадь (кв.м)
   MEASURE_PERIMETER   = 3,   // Периметр (м)
   MEASURE_HEIGHT      = 10,  // Высота (м)
};

enum PPLACE            // ПРИМЕНЯЕМАЯ СИСТЕМА КООРДИНАТ
{
   PP_MAP      = 1,    // КООРДИНАТЫ ТОЧЕК В СИСТЕМЕ КАРТЫ В ДИСКРЕТАХ
   PP_PICTURE  = 2,    // КООРДИНАТЫ ТОЧЕК В СИСТЕМЕ ИЗОБРАЖЕНИЯ В ПИКСЕЛАХ
   PP_PLANE    = 3,    // КООРДИНАТЫ ТОЧЕК В ПЛОСКОЙ ПРЯМОУГОЛЬНОЙ СИСТЕМЕ
                       // НА МЕСТНОСТИ В МЕТРАХ
   PP_GEO      = 4,    // КООРДИНАТЫ ТОЧЕК В ГЕОДЕЗИЧЕСКИХ КООРДИНАТАХ
                       // В РАДИАНАХ
   PP_GEOWGS84 = 8,    // КООРДИНАТЫ ТОЧЕК В ГЕОДЕЗИЧЕСКИХ КООРДИНАТАХ
                       // В РАДИАНАХ В СИСТЕМЕ WGS-84
};

enum CURRENTPOINTFORMAT  // ФОРМАТ ОТОБРАЖЕНИЯ ТОЧКИ  // 09/06/10
{
   PLANEPOINT      = 1,  // В метрах на местности в текущих параметрах проекции ("DocProjection")
                         // эллипсоид Красовского 1942г. СК-42 или СК-95,
                         // если текущая карта в СК-95 
   MAPPOINT        = 2,  // В условных единицах карты (дискретах)
   PICTUREPOINT    = 3,  // В пикселах текущего полного изображения

   GEORAD          = 4,  // В радианах
   GEOGRAD         = 5,  // В градусах
   GEOGRADMIN      = 6,  // В градусах, минутах, секундах

                         // общеземной эллипсоид WGS84
   GEORADWGS84     = 7,  // В радианах
   GEOGRADWGS84    = 8,  // В градусах
   GEOGRADMINWGS84 = 9,  // В градусах, минутах, секундах

   PLANE42POINT    = 10, // В метрах на местности по ближайшей зоне в СК-42

                         // общеземной эллипсоид ПЗ-90.02 (SGS-85)
   GEORADPZ90      = 11, // В радианах
   GEOGRADPZ90     = 12, // В градусах
   GEOGRADMINPZ90  = 13, // В градусах, минутах, секундах

   PLANEWORKSYS      = 14, // В метрах на местности в местной системе в соответствии с параметрами ("WorkSystem")
   GEORADWORKSYS     = 15, // В радианах в местной системе
   GEOGRADWORKSYS    = 16, // В градусах в местной системе
   GEOGRADMINWORKSYS = 17, // В градусах минутах, секундах в местной системе

   PLANE95POINT      = 18, // В метрах на местности по ближайшей зоне в СК-95          // 15/11/10

   LASTPOINTFORMAT   = 18, // ПРЕДЕЛЬНОЕ ЗНАЧЕНИЕ
};

enum CURRENTUNITSFORMAT  // ЕДИНИЦЫ ИЗМЕРЕНИЙ  // 05/07/16
{
   UNITSPLANE        = 0,  // МЕТРЫ
   UNITSFT           = 1,  // ФУТЫ
   LASTUNITSFORMAT   = 1, // ПРЕДЕЛЬНОЕ ЗНАЧЕНИЕ
};

enum VTYPE               // ТИП ОТОБРАЖЕНИЯ КАРТЫ
{
   // ЭКРАННЫЙ ВЫВОД
   VT_SCREEN          = 1, // ЭКРАННЫЙ (ЧЕРЕЗ DIB)
   VT_SCREENCONTOUR   = 2, // ЭКРАННЫЙ КОНТУРНЫЙ

   // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ ВЫВОД
   VT_PRINT           = 3, // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ (ЧЕРЕЗ WIN API)
   VT_PRINTGLASS      = 4, // ПРИНТЕРНЫЙ БЕЗ ЗАЛИВКИ ПОЛИГОНОВ
   VT_PRINTCONTOUR    = 5, // ПРИНТЕРНЫЙ КОНТУРНЫЙ, БЕЗ УСЛОВНЫХ ЗНАКОВ

   // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ ВЫВОД
   VT_PRINTRST        = 6, // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ (ЧЕРЕЗ WIN API)
   VT_PRINTGLASSRST   = 7, // ПРИНТЕРНЫЙ БЕЗ ЗАЛИВКИ ПОЛИГОНОВ
   VT_PRINTCONTOURRST = 8, // ПРИНТЕРНЫЙ КОНТУРНЫЙ, БЕЗ УСЛОВНЫХ ЗНАКОВ

   // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ (СПЕЦИАЛЬНЫЙ) ВЫВОД       
   VT_PRINTRSTSQUARE  = 9, // ПРИНТЕРНЫЙ (ЗАЛИВКИ ПЛОЩАДНЫХ, ПОДПИСИ С ФОНОМ,
                           //             РАСТРЫ, МАТРИЦЫ)
   VT_PRINTRSTLINE    =10, // ПРИНТЕРНЫЙ (ЛИНИИ, ТОЧЕЧНЫЕ, ВЕКТОРНЫЕ,
                           //             ПОДПИСИ С ФОНОМ, ПОЛЬЗОВАТЕЛЬСКИЕ)
   VT_PRINTRSTTEXT    =11, // ПРИНТЕРНЫЙ (ПОДПИСИ, ШАБЛОНЫ)

   // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ (СПЕЦИАЛЬНЫЙ) ВЫВОД. ИСПОЛЬЗУЕТСЯ ДЛЯ ЭКСПОРТА
   // ГРАФИКИ (POSTSCRIPT, WMF, EMF)                    
   VT_PRINTEX         =15, // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ (ЧЕРЕЗ WIN API)
};

enum OUTPUTAREA            // РЕЖИМЫ ВЫБОРА ОБЛАСТИ ВЫВОДА
{
  AREA_ALL            = 0, // Открытие : область вывода - весь район
                           // Закрытие : область вывода не сохранять

  AREA_FRAME          = 1, // Открытие : область вывода - из RectMetr
                           // Закрытие : область вывода -> в RectMetr

  AREA_SELECT_FRAME   = 2, // Открытие : область вывода - из RectMetr
                           // Закрытие : для выбора прямоугольного фрагмента

  AREA_CHANGE_FRAME   = 3, // Открытие : область вывода - из RectMetr
                           // Закрытие : для изменения фрагмента

  AREA_MOVE_FRAME     = 4, // Открытие : область вывода - из RectMetr
                           // Закрытие : для перемещения фрагмента

  AREA_SELECT_OBJECT  = 5, // Открытие : область вывода - из RectMetr
                           // Закрытие : для выбора области вывода по объекту

  AREA_SELECT_CONTOUR = 6, // Открытие : область вывода - из RectMetr
                           // Закрытие :  для выбора области вывода по контуру

  AREA_OBJECT         = 7, // Обработчик ничего не делает  // 13/02/13
                           // Область установлена ранее по объекту

  AREA_CONTOUR        = 8, // Обработчик ничего не делает
                           // Область установлена ранее по контуру

  AREA_OBJECTS        = 9, // Обработка выполняется по выделенным объектам

  AREA_SELECT_TURN_FRAME = 10, // Флаг выбора повернутого фрагмента
};

enum CHARCODE         // ОПИСАНИЕ КОДИРОВКИ ТЕКСТОВ
{
  CC_ASCIIZ  = 0  ,   // CТРОКА СИМВОЛОВ (ASCIIZ), ОГРАНИЧЕННАЯ НУЛЕМ
  CC_KOI8    = 125,   // CТРОКА (KOI8), ОГРАНИЧЕННАЯ НУЛЕМ
  CC_ANSI    = 126,   // CТРОКА (ANSI, WINDOWS), ОГРАНИЧЕННАЯ НУЛЕМ
  CC_UNICODE = 127,   // CТРОКА СИМВОЛОВ (UNICODE), ОГРАНИЧЕННАЯ НУЛЕМ
};

enum SPLINETYPE           // ТИПЫ СПЛАЙНОВ
{
  SPLINETYPE_NONE   = 0,  // Ломаная линия
  SPLINETYPE_SMOOTH = 2,  // Сглаживающий сплайн (срезает углы)
  SPLINETYPE_POINTS = 4,  // Огибающий сплайн (проходит через точки)
};

enum POINTPOSITION        // РАЗМЕЩЕНИЕ ТОЧКИ ОТНОСИТЕЛЬНО ОТРЕЗКА
{
  PS_FIRST   = 1,         // Совпадает с первой точкой отрезка
  PS_SECOND  = 2,         // Совпадает со второй точкой отрезка
  PS_BEHIND  = 3,         // Лежит позади первой точки отрезка
  PS_BEYOND  = 4,         // Лежит впереди второй точки отрезка
  PS_BETWEEN = 5,         // Лежит на отрезке (между точками)
  PS_LEFT    = 6,         // Слева
  PS_RIGHT   = 7          // Справа
};


typedef struct GEODEGREE   // КООРДИНАТЫ ТОЧКИ В ГРАДУСАХ
{
  int   Degree;         // Градусы
  int   Minute;         // Минуты
  float Second;         // Секунды
}
  GEODEGREE;

typedef struct SIGNDEGREE  // КООРДИНАТЫ ТОЧКИ В ГРАДУСАХ
{
  int   Degree;       // Градусы
  int   Minute;       // Минуты
  float Second;       // Секунды
  int   Sign;         // Знак числа (+1 или -1)
}
  SIGNDEGREE;

typedef struct MTRDESCRIBE // ОПИСАНИЕ ОТКРЫТОЙ МАТРИЦЫ
{
  char   Name[256];      // Имя файла
  double MinValue;       // Минимальное значение элемента в матрице
  double MaxValue;       // Максимальное значение элемента в матрице
  double ElementInPlane; // Размер элемента в метрах на местности
  DFRAME FrameMeters;    // Габариты матрицы (метры)
  int    ReliefType;     // Тип матрицы (0 - абсолютные высоты,
                         //  1 - абсолютные + относительные)
  int    UserType;       // Тип пользовательских данных
  int    View;           // Признак отображаемости
}
  MTRDESCRIBE;

#define MTRTILEIDENT      0x7F7F7F7F

typedef struct MTRTILE    // ОПИСАНИЕ ФРАГМЕНТА МАТРИЦЫ НА УЧАСТОК МЕСТНОСТИ
{
  int  Ident;             // Идентификатор записи 0x7F7F7F7F
  int  Length;            // Длина записи в байтах с заголовком (262208 байт)
  int  Width;             // Число столбцов (256)
  int  Height;            // Число строк    (256)
  int  Epsg;              // Код EPSG плановой системы координат (например, 3857)
  int  Level;             // Уровень матрицы в пирамиде матриц (аналогично пирамиде тайлов)
  int  WidthNumber;       // Номер столбца матрицы в заданной системе координат (в пространстве от -180 до +180 градусов)
  int  HeightNumber;      // Номер строки матрицы в заданной системе координат (в пространстве от -90 до +90 градусов)
  int  Unit;              // Единица измерения высоты (0 - метры, 1 - дециметры, 2 - сантиметры, 3 - миллиметры)
  int  UnitPlane;         // Единица измерения в плане (0 - метры, 1 - дециметры, 2 - сантиметры, 3 - миллиметры)
  int  ItemWidth;         // Ширина элемента в заданных единицах измерения в плане (справочно)
  int  ItemHeight;        // Высота элемента в заданных единицах измерения в плане (справочно)
  int  MinValue;          // Минимальное значение в матрице
  int  MaxValue;          // Максимальное значение в матрице
  int  Reserve[2];        // Резерв, содержит нули

  int  Value[256][256];    // Значения высот в заданных единицах измерения высоты
}
  MTRTILE;

typedef struct MTRTILE2   // ОПИСАНИЕ ФРАГМЕНТА МАТРИЦЫ НА УЧАСТОК МЕСТНОСТИ
{
  int  Ident;             // Идентификатор записи 0x7F7F7F7F
  int  Length;            // Длина записи в байтах с заголовком (131136 байт)
  int  Width;             // Число столбцов (256)
  int  Height;            // Число строк    (256)
  int  Epsg;              // Код EPSG плановой системы координат (например, 3857)
  int  Level;             // Уровень матрицы в пирамиде матриц (аналогично пирамиде тайлов)
  int  WidthNumber;       // Номер столбца матрицы в заданной системе координат (в пространстве от -180 до +180 градусов)
  int  HeightNumber;      // Номер строки матрицы в заданной системе координат (в пространстве от -90 до +90 градусов)
  int  Unit;              // Единица измерения высоты (0 - метры, 1 - дециметры, 2 - сантиметры, 3 - миллиметры)
  int  UnitPlane;         // Единица измерения в плане (0 - метры, 1 - дециметры, 2 - сантиметры, 3 - миллиметры)
  int  ItemWidth;         // Ширина элемента в заданных единицах измерения в плане (справочно)
  int  ItemHeight;        // Высота элемента в заданных единицах измерения в плане (справочно)
  int  MinValue;          // Минимальное значение в матрице
  int  MaxValue;          // Максимальное значение в матрице
  int  Reserve[2];        // Резерв, содержит нули

  unsigned short int Value[256][256];  // Значения высот в заданных единицах измерения высоты, относительно MinValue
}
  MTRTILE2;

typedef struct MTRDESCRIBEUN // ОПИСАНИЕ ОТКРЫТОЙ МАТРИЦЫ
{
  WCHAR  Name[1024];     // Имя файла
  double MinValue;       // Минимальное значение элемента в матрице
  double MaxValue;       // Максимальное значение элемента в матрице
  double ElementInPlane; // Размер элемента в метрах на местности
  DFRAME FrameMeters;    // Габариты матрицы (метры)
  int    ReliefType;     // Тип матрицы (0 - абсолютные высоты, 1 - абсолютные + относительные)
  int    UserType;       // Тип пользовательских данных
  int    View;           // Признак отображаемости
  int    Zero;           // Резерв
}
  MTRDESCRIBEUN;


typedef struct MTRCOLORDESCEX  // ОПИСАНИЕ ДИАПАЗОНА ВЫСОТ
                               // МАТРИЦЫ
{
  double MinHeight;            // Минимальная высота дипазона
  double MaxHeight;            // Максимальная высота дипазона
  COLORREF Color;              // Цвет,соответствующий диапазону высот
}
  MTRCOLORDESCEX;

typedef struct MTR3DVIEW     // ПАРАМЕТРЫ 3D-ОТОБРАЖЕНИЯ МАТРИЦЫ ВЫСОТ
{
  double AreaCenterX;        // Центр отображаемой области X,Y
  double AreaCenterY;        //   (в метрах на местности)
  double ViewPointX;         // Точка наблюдения X,Y,H
  double ViewPointY;         //   (в метрах на местности)
  double ViewPointH;         //
  unsigned int  ShowScale;   // Знаменатель масштаба
  unsigned char Style;       // Cтиль (0 - серый, 1 - цветной)
  unsigned char ShowGrid;    // Наличие сетки (0 - нет, 1 - есть)
  unsigned char GridStep;    // Шаг сетки в пикселах (2 - 255)
  unsigned char ModelHeight; // Высота модели в пикселах (2 - 127)
}
  MTR3DVIEW;

enum CUTSHAPE                // Форма разреза
{
  CS_NONE          =   0,    // Отсутствует
  CS_VERTICALANGLE =   1,    // Вертикальный угловой разрез
                             //   с вершиной в точке CutX,CutY
  CS_VOLUMETRICAL  =  16,    // Объемный вырез
                             //   с вершиной в точке CutX,CutY,CutZ
  CS_HORIZONTAL    = 128,    // Горизонтальный срез по высоте CutH
};

enum SCALETYPE               // Тип шкалы
{
  ST_NONE          =   0,    // Отсутствует
  ST_ABSOLUTE      =   1,    // Абсолютная
  ST_FROMRELIEF    =   2,    // Относительная от рельефа
  ST_WITHOUTRELIEF =   3,    // Относительная без учета рельефа
};

#define MTL3D_MAXMODEL 0x01FF  // Максимальная высота модели матрицы слоев
                               // (является маской выделения высоты)

typedef struct MTR3DVIEWEX   // ПАРАМЕТРЫ 3D-ОТОБРАЖЕНИЯ МАТРИЦ (MTW и MTL)
{
  double AreaCenterX;        // Центр отображаемой области X,Y
  double AreaCenterY;        //   (в метрах на местности)
  double CutX;               // Координаты вершины разреза/выреза X,Y,Z
  double CutY;               //   (в метрах на местности)      - для MTL
  double CutZ;
  double CutH;               // Высота горизонтального среза (в метрах)

    long ShowScale;          // Знаменатель масштаба отображения местности
    long ModelHeight;        // Высота модели в пикселах (2 - MTL3D_MAXMODEL)
    long GridStep;           // Шаг сетки в пикселах (2 - 100)
    long CutShape;           // Форма разреза (см. CUTSHAPE)   - для MTL

   short ViewAngle;          // Угол наблюдения/наклона (в градусах)
   short RotationAngle;      // Угол поворота относительно центра AreaCenter
                             //   (в градусах)
    char Style;              // Cтиль (0 - серый, 1 - цветной)
    char ShowGrid;           // Наличие сетки (0 - нет, 1 - есть)
    char Shadow;             // Наличие тени (0 - нет, 1 - есть)
    char ScaleType;          // Тип шкалы (см. SCALETYPE)      - для MTL

    char CoverMatrix;        // Наличие матрицы высот   (0 - нет, 1 - есть)
    char CoverMap;           // Наличие карты           (0 - нет, 1 - есть)
    char CoverRaster;        // Наличие растра          (0 - нет, 1 - есть)
    char AccordScale;        // Согласование масштабов  (0 - нет, 1 - есть)
    char CoverMtq;           // Наличие матрицы качества(0 - нет, 1 - есть)  // 06/10/04
    char IsUpdate ;          // Перерисовка изображения (0 - нет, 1 - да)    // 21/10/05

    char Reserve[26];

  double CursorX;            // Координаты указателя X,Y
  double CursorY;            //   (в метрах на местности)

    long Width;              // Ширина изображения
    long Height;             // Высота изображения

    char Name[256];          // Имя файла отображаемой матрицы
                             //   (0 - все матрицы слоев)    - для MTL
}
  MTR3DVIEWEX;

                             // ПАРАМЕТРЫ 3D-ОТОБРАЖЕНИЯ МАТРИЦ СЛОЕВ (MTL)
typedef struct MTR3DVIEWEX MTL3DVIEW;


typedef struct METAFILEBUILDPARM  // ПАРАМЕТРЫ СОЗДАНИЯ МЕТАФАЙЛА
{
  long CenterX;     // Координаты центра выводимой области
  long CenterY;     // (в метрах на местности)
  long Width;       // Ширина фрагмента (в миллиметрах)
  long Height;      // Высота фрагмента (в миллиметрах)
  long Scale;       // Масштаб изображения
  long VisualType;  // Тип визуализации (от VT_SCREEN до VT_PRINTEX)
   // ЭКРАННЫЙ ВЫВОД
   //   VT_SCREEN          = 1, // ЭКРАННЫЙ (ЧЕРЕЗ DIB)
   //   VT_SCREENCONTOUR   = 2, // ЭКРАННЫЙ КОНТУРНЫЙ

   // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ ВЫВОД
   //   VT_PRINT           = 3, // ПРИНТЕРНЫЙ ВЕКТОРНЫЙ (ЧЕРЕЗ WIN API)
   //   VT_PRINTGLASS      = 4, // ПРИНТЕРНЫЙ БЕЗ ЗАЛИВКИ ПОЛИГОНОВ
   //   VT_PRINTCONTOUR    = 5, // ПРИНТЕРНЫЙ КОНТУРНЫЙ, БЕЗ УСЛОВНЫХ ЗНАКОВ

   // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ ВЫВОД
   //   VT_PRINTRST        = 6, // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ (ЧЕРЕЗ WIN API)
   //   VT_PRINTGLASSRST   = 7, // ПРИНТЕРНЫЙ БЕЗ ЗАЛИВКИ ПОЛИГОНОВ
   //   VT_PRINTCONTOURRST = 8, // ПРИНТЕРНЫЙ КОНТУРНЫЙ, БЕЗ УСЛОВНЫХ ЗНАКОВ
}
  METAFILEBUILDPARM;


typedef struct METAFILEBUILDPARMEX  // ПАРАМЕТРЫ СОЗДАНИЯ EMF-ФАЙЛА
{
  DFRAME Frame;     // Габариты (метры)
  long Scale;       // Масштаб изображения
  char VisualType;  // Тип визуализации (от VT_SCREEN до VT_PRINTEX)
  char Border;      // Флаг рисования рамки
  char Intensity;   // Интенсивность заливки % (0-100) 
  char Black;       // Флаг черно-белого изображения(1 - есть, 0 - нет)

  char DontClip;    // Не обрезать выбранный фрагмент по рамке (!= 0)
                    // (например, Corel9 не понимает ::SelectClipRgn() )
  char Reserve[7];  // Резерв (должен быть обнулен)
}
  METAFILEBUILDPARMEX;

// Виды сжатия блоков данных растров и матриц
#define RMF_COMPR_LZW    1             // Алгоритм сжатия LZW для растров
#define RMF_COMPR_JPEG   2             // Алгоритм сжатия JPEG для 24 битных растров
#define RMF_COMPR_32     32            // Алгоритм сжатия для матриц


enum RST_STATE                   // состояние файла растра                      // 18/04/16
{
  RSTST_NONE              =   0,    // нет данных; или создание УК и сжатие не выполнялись
  RSTST_DUPL_JPEG         =   1,    // создание всех уровней УК, сжатие JPEG растра
  RSTST_DUPL_LZW          =   2,    // создание всех уровней УК, сжатие LZW растра

  RSTST_DUPL              =   4,    // создание всех уровней УК
};


enum MTW_STATE                   // состояние файла матрицы                     // 18/04/16
{
  MTWST_NONE              =   0,    // нет данных; создание УК и сжатие не выполнялись
  MTWST_DUPL_COMPR32      =   3,    // создание всех уровней УК, сжатие RMF_COMPR_32 матрицы

  MTWST_DUPL              =   4,    // создание всех уровней УК
};

typedef struct XIMAGEDESC   // Описатель битовой области XImage
{                           // (для применения с XWindow)   
  char *    Point;          // Адрес начала области пикселов
  long int  Width;          // Ширина строки в пикселах
  long int  Height;         // Число строк
  long int  Depth;          // Размер элемента в битах (8,15,16,24,32)
  long int  CellSize;       // Размер элемента(пиксела) в байтах
  long int  RowSize;        // Ширина строки в байтах
}
  XIMAGEDESC;


enum GRIDTYPE                // ТИП СЕТКИ
{
  GRT_NONE  =   0,           // Отсутствует
  GRT_LINE  =   1,           // Сетка линий
  GRT_NODE  =   2,           // Сетка точек
  GRT_CROSS =   3,           // Сетка крестов
};

typedef struct _DLLCLASS GRIDPARM     // СТРУКТУРА ПАРАМЕТРОВ ПРЯМОУГОЛЬНОЙ СЕТКИ
{
  GRIDPARM()  { Step = 1000; Type = GRT_LINE;
                Image.Color = 0x040404; Image.Thick = 250;
                Size = 10*250; Shadow = 0; Under = 0; }

  double    Step;           // Шаг сетки в метрах на местности
  long int  Type;           // Тип элемента (сетка, точки, кресты)
  struct
  {
   long int Color;
   long int Thick;
  }
            Image;          // Цвет и толщина линий (RGB, мкм)
  long int  Size;           // Размер элемента типа "крест" в мкм (1:250)
  long int  Shadow;         // Признак полупрозрачного отображения
  long int  Under;          // Признак отображения под картой
}
  GRIDPARM;

typedef struct /*_DLLCLASS*/ GRIDPARMEX     // СТРУКТУРА ПАРАМЕТРОВ ПРЯМОУГОЛЬНОЙ СЕТКИ
{
  GRIDPARMEX()  { Step = 1000; Type = GRT_LINE;
                  Image.Color = 0x040404; Image.Thick = 250;
                  Size = 10*250; Shadow = 0; Under = 0; BeginX = 0; BeginY = 0; }

  double    Step;           // Шаг сетки в метрах на местности
  long int  Type;           // Тип элемента (сетка, точки, кресты)
  struct
  {
   long int Color;
   long int Thick;
  }
            Image;          // Цвет и толщина линий (RGB, мкм)
  long int  Size;           // Размер элемента типа "крест" в мкм (1:250)
  long int  Shadow;         // Признак полупрозрачного отображения
  long int  Under;          // Признак отображения под картой

  double    BeginX;         // Начало координат сетки
  double    BeginY;
}
  GRIDPARMEX;

// ПАРАМЕТРЫ ПОСТРОЕНИЯ ЗОНЫ ВИДИМОСТИ
typedef struct TBUILDZONEVISIBILITY
{
  DOUBLEPOINT PointCenter;      // Исходная точка
  double      RadiusMeter;      // Расстояние (в метрах на местности)
  double      Azimuth;          // Направление (в радианах)
  double      Angle;            // Угол обзора (в радианах)
                                // При Azimuth = 0, Angle = 2*M_PI (360 градусов)
                                // строится круговая зона
  double      DeltaHight;       // Высота наблюдения (в метрах)
  double      DeltaObservation; // Приращение наблюдаемой точки (в метрах)
  long int    VisionRst;        // Видимость зон пересечения
                                // 0-со всех точек, 1-хотя бы с одной
  long int    StyleRst;         // Степень видимости растра
                                // 0-прозрачный, 1-полупрозрачный
                                // 2-заполненный
  long int    ColorRst;         // Цвет растра
  long int    Inversion;        // Инверсия видимости

}
  TBUILDZONEVISIBILITY;

enum PRECISIONFLAG              // ТОЧНОСТЬ ХРАНЕНИЯ КООРДИНАТ НА КАРТЕ
{
  PF_FLOAT  = 0,                // Нормальная точность в дискретах
  PF_DOUBLE = 1,                // Повышенная точность в дискретах
  PF_METRE  = 2,                // Повышенная точность в метрах
  PF_GEO    = 4                 // Повышенная точность в радианах
};


enum TMCMODE               // РЕЖИМЫ ДОСТУПА К ДАННЫМ
{
  TMM_READ         = 1,    // Разрешено чтение данных
  TMM_WRITE        = 2,    // Разрешено редактирование данных
  TMM_COPY         = 4,    // Разрешено копирование данных на клиенте
  TMM_NOTPRINT     = 8,    // Запрещен вывод на печать на клиенте     // 28/08/12
  TMM_CACHECODING  = 16,   // Шифровать кэш ключом сервера            // 21/09/12
  TMM_STREAMCODING = 32,   // Шифровать передачу                      // 21/09/12
  TMM_TEMP         = 64,   // Признак временной карты (удалить при закрытии)
  TMM_HIDEPASP     = 128,  // Запрещен показ паспорта карты           // 16/11/12
  TMM_RSCEDIT      = 256,  // Разрешено редактирование классификатора // 27/12/12
  TMM_DOCSTORAGE   = 512,  // Разрешено хранение документов с картой  // 09/02/13
  TMM_ADMIN        = 1024, // Разрешено администрирование данных
  TMM_MONITOR      = 2048, // Разрешен мониторинг состояния сервера
  TMM_FOLDEREDIT   = 4096, // Разрешено редактирование состава файлов в папке  // 18/07/14
  TMM_GEODB        = 8192, // Разрешено подключение к Банку Данных ЦК и ДЗЗ    // 14/08/15
};

#define TMM_STREAMCODING4K 33

enum OBM_TYPE              // ФОРМА МЕТРИКИ ОБЪЕКТОВ
{
  OBM_RECTANGLE    = 1,    // Метрика имеет форму горизонтального (вертикального) прямоугольника
  OBM_CIRCLE       = 2,    // Метрика имеет форму окружности
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++ СТРУКТУРЫ ДЛЯ СОЗДАНИЯ ПЛАНОВ, КАРТ И ++++++++++++++++++
//++++++++ ПОЛЬЗОВАТЕЛЬСКИХ КАРТ                 ++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct APPENDPLANELIST
{
int  Length;              // Длина записи структуры APPENDLIST        // 26/03/12

char Nomenclature[32];    // Номенклатура листа
char ListName[32];        // Условное название листа
char FileName[256];       // Имя файла по которому в районе будет
                          // создан лист с данным именем и расширениями
                          // HDR,DAT,SEM ...

int  Reserv;              // Резерв для выравнивания, должен быть 0
                          // 17/10/01

// Прямоугольные координаты листа в метрах
// (для пользовательской карты (SITE) должны быть 0)

double XSouthWest   ;  // X ю-з
double YSouthWest   ;  // Y ю-з
double XNorthWest   ;  // X с-з
double YNorthWest   ;  // Y с-з
double XNorthEast   ;  // X с-в
double YNorthEast   ;  // Y с-в
double XSouthEast   ;  // X ю-в
double YSouthEast   ;  // Y ю-в
}
  APPENDPLANELIST;


typedef struct CREATEPLANE
{
int Length;              // Длина записи структуры CREATEPLANE   // 26/03/12

char MapName[32];         // Имя района
long Scale;               // Знаменатель масштаба карты

APPENDPLANELIST List;     // Описание листа плана
}
  CREATEPLANE;


typedef struct CREATESITE
{
 long     Length;             // Длина записи структуры CREATESITE (80)
 char     MapName[32];        // Имя района
 int      MapType;            // Обобщенный тип карты
 int      MaterialProjection; // Проекция исх. материала
 int      Scale;              // Знаменатель масштаба карты

                              // В радианах
 double FirstMainParallel;    // Первая главная параллель
 double SecondMainParallel;   // Вторая главная параллель
 double AxisMeridian;         // Осевой меридиан
 double MainPointParallel;    // Параллель главной точки
}
  CREATESITE;

typedef struct CREATESITEEX   // 11/05/07
{
 long     Length;             // Длина записи структуры CREATESITE (328)
 char     MapName[256];       // Имя района в кодировке ANSI (UNICODE - для функций "Un")
 long int MapType;            // Обобщенный тип карты
 long int MaterialProjection; // Проекция исх. материала
 long int EllipsoideKind;     // Тип эллипсоида (1 - Красовского, 9 - WGS84,...)
 long     Scale;              // Знаменатель масштаба карты
 long     Reserve;            // Резерв (должен быть 0)

                              // В радианах
 double FirstMainParallel;    // Первая главная параллель
 double SecondMainParallel;   // Вторая главная параллель
 double AxisMeridian;         // Осевой меридиан
 double MainPointParallel;    // Параллель главной точки
 double PoleLatitude;         // Широта полюса проекции
 double PoleLongitude;        // Долгота полюса проекции
}
  CREATESITEEX;

typedef struct CREATESITEUN   // 11/05/07
{
 long     Length;             // Длина записи структуры CREATESITE
 WCHAR    MapName[128];       // Имя района в кодировке UNICODE (для функций "Un")
 long int MapType;            // Обобщенный тип карты
 long int MaterialProjection; // Проекция исх. материала
 long int EllipsoideKind;     // Тип эллипсоида (1 - Красовского, 9 - WGS84,...)
 long     Scale;              // Знаменатель масштаба карты
 long     Reserve;            // Резерв (должен быть 0)

                              // В радианах
 double FirstMainParallel;    // Первая главная параллель
 double SecondMainParallel;   // Вторая главная параллель
 double AxisMeridian;         // Осевой меридиан
 double MainPointParallel;    // Параллель главной точки
 double PoleLatitude;         // Широта полюса проекции
 double PoleLongitude;        // Долгота полюса проекции
}
  CREATESITEUN;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++ СТРУКТУРА РАЗДЕЛЕНИЯ 4 БАЙТОВ НА СОСТАВЛЯЮЩИЕ ++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef union
{
  struct
  {
    unsigned char b1;
    unsigned char b2;
    unsigned char b3;
    unsigned char b4;
  }
    Byte;              // Представление в виде 8-битовых полей

  struct
  {
    unsigned short w1;
    unsigned short w2;
  }
    Word;              // Представление в виде 16-битовых полей

  unsigned int Long;  // Представление в виде 32-битового поля   // 26/03/12
  float        Float; // Представление в виде 32-битового поля   // 16/08/12

  int Value() { return (int) Long; }

  // Запросить значение (long)
  int GetLong(int & value)
  {
    return (int)(Long = (unsigned int)value);
  }

  // Запросить значение (long)
  int GetLong(unsigned int & value)
  {
    return (int)(Long = value);
  }

  // Запросить значение (long) по адресу address (4 байта)
  long GetLong(const char* address)
  {
#ifdef RISCCPU
    Byte.b1 = address[0];
    Byte.b2 = address[1];
    Byte.b3 = address[2];
    Byte.b4 = address[3];
#else
    Long = *((unsigned int *)address);
#endif
    return (int)Long;
  }

  // Запросить значение (long) по адресу address (4 байта)
  // в обратной последовательности байт
  int GetTurnLong(int & value)
  {
    return GetTurnLong((char *)&value);
  }

  // Запросить значение (long) по адресу address (4 байта)
  // в обратной последовательности байт
  int GetTurnLong(unsigned int & value)
  {
    return GetTurnLong((char *)&value);
  }

  // Запросить значение (long) по адресу address (4 байта)
  // в обратной последовательности байт
  int GetTurnLong(char* address)
  {
    Byte.b4 = address[0];
    Byte.b3 = address[1];
    Byte.b2 = address[2];
    Byte.b1 = address[3];
    return (int)Long;
  }

  // Установить значение (long) по адресу address (4 байта)
  int PutLong(int & value)
  {
    return (value = (int)Long);
  }

  // Установить значение (long) по адресу address (4 байта)
  int PutLong(unsigned int & value)
  {
    return (int)(value = (unsigned int)Long);
  }

  // Установить значение (long) по адресу address (4 байта)
  int PutLong(char* address)
  {
#ifdef RISCCPU
    address[0] = Byte.b1;
    address[1] = Byte.b2;
    address[2] = Byte.b3;
    address[3] = Byte.b4;
#else
    *((unsigned int *)address) = Long;
#endif
    return (int)Long;
  }

  // Запросить значение (float)
  float GetFloat(float & value)
  {
    return (Float = value);
  }

  // Запросить значение (float) по адресу address (4 байта)
  float GetFloat(const char* address)
  {
#ifdef RISCCPU
    Byte.b1 = address[0];
    Byte.b2 = address[1];
    Byte.b3 = address[2];
    Byte.b4 = address[3];
#else
    Float = *((float *)address);
#endif
    return Float;
  }

  // Установить значение (float) по адресу address (4 байта)   // 16/08/12
  float PutFloat(float & value)
  {
    return (value = Float);
  }

  // Установить значение (float) по адресу address (4 байта)   // 16/08/12
  float PutFloat(char* address)
  {
#ifdef RISCCPU
    address[0] = Byte.b1;
    address[1] = Byte.b2;
    address[2] = Byte.b3;
    address[3] = Byte.b4;
#else
    *((float *)address) = Float;
#endif

    return Float;
  }

  // Запросить значение (short int)
  int GetShort(short int & value)
  {
    return GetShort((char *)&value);
  }

  // Запросить значение (short int)
  int GetShort(unsigned short int & value)
  {
    return GetShort((char *)&value);
  }

  // Запросить значение (int) по адресу address (2 байта)
  int GetShort(char* address)
  {
    Long = 0;
#ifdef RISCCPU
#ifdef NUMBERTURN              // 04/03/00
    Byte.b3 = address[0];
    Byte.b4 = address[1];
#else
    Byte.b1 = address[0];
    Byte.b2 = address[1];
#endif
#else
    Word.w1 = *((unsigned short *)address);
#endif
    return (int)Long;
  }

  // Запросить значение (int) по адресу address (2 байта)
  // в обратной последовательности байт
  int GetTurnShort(short int & value)
  {
    return GetTurnShort((char *)&value);
  }

  // Запросить значение (int) по адресу address (2 байта)
  // в обратной последовательности байт
  int GetTurnShort(unsigned short int & value)
  {
    return GetTurnShort((char *)&value);
  }

  // Запросить значение (int) по адресу address (2 байта)
  // в обратной последовательности байт
  int GetTurnShort(char* address)
  {
    Long = 0;
#ifdef NUMBERTURN
    Byte.b4 = address[0];
    Byte.b3 = address[1];
#else
    Byte.b2 = address[0];
    Byte.b1 = address[1];
#endif
    return (int)Long;
  }

  // Установить значение (short int)
  int PutShort(short int & value)
  {
    return PutShort((char *)&value);
  }

  // Установить значение (short int)
  int PutShort(unsigned short int & value)
  {
    return PutShort((char *)&value);
  }

  // Установить значение (int) по адресу address (2 байта)
  int PutShort(char* address)
  {
#ifdef RISCCPU
#ifdef NUMBERTURN              // 04/03/00
    address[0] = Byte.b3;
    address[1] = Byte.b4;
#else
    address[0] = Byte.b1;
    address[1] = Byte.b2;
#endif
#else
    *((unsigned short *)address) = Word.w1;
#endif
    return (int)Long;
  }
}
  REGISTER;

// Определения для С (REGISTER , char*) :

#if defined(__BORLANDC__) // Для Borland C++

#define LongToRegister(register, point) register.Long = ((long *)point)[0];

#else

#define LongToRegister(register, point) { register.Byte.b1 = ((char *)point)[0]; \
                                          register.Byte.b2 = ((char *)point)[1]; \
                                          register.Byte.b3 = ((char *)point)[2]; \
                                          register.Byte.b4 = ((char *)point)[3]; }
#endif

#define TurnLongToRegister(register, point) { register.Byte.b4 = ((char *)point)[0]; \
                                              register.Byte.b3 = ((char *)point)[1]; \
                                              register.Byte.b2 = ((char *)point)[2]; \
                                              register.Byte.b1 = ((char *)point)[3]; }

// 04/02/16 Korjilov
#define TurnLong24ToRegister(register, point) { register.Byte.b4 = 0; \
                                                register.Byte.b3 = ((char *)point)[0]; \
                                                register.Byte.b2 = ((char *)point)[1]; \
                                                register.Byte.b1 = ((char *)point)[2]; }

#define RegisterToLong(register, point) { ((char *)point)[0] = register.Byte.b1; \
                                          ((char *)point)[1] = register.Byte.b2; \
                                          ((char *)point)[2] = register.Byte.b3; \
                                          ((char *)point)[3] = register.Byte.b4; }

#define TurnRegisterToLong(register, point) { ((char *)point)[0] = register.Byte.b4; \
                                              ((char *)point)[1] = register.Byte.b3; \
                                              ((char *)point)[2] = register.Byte.b2; \
                                              ((char *)point)[3] = register.Byte.b1; }

#ifdef NUMBERTURN
#define ShortToRegister(register, point) {register.Long    = 0;        \
                                          register.Byte.b3 = ((char *)point)[0]; \
                                          register.Byte.b4 = ((char *)point)[1]; }

#define RegisterToShort(register, point) {((char *)point)[0] = register.Byte.b3; \
                                          ((char *)point)[1] = register.Byte.b4; }

#else
#define ShortToRegister(register, point) {register.Long    = 0;        \
                                          register.Byte.b1 = ((char *)point)[0]; \
                                          register.Byte.b2 = ((char *)point)[1]; }

#define RegisterToShort(register, point) {((char *)point)[0] = register.Byte.b1; \
                                          ((char *)point)[1] = register.Byte.b2; }
#endif

#ifdef NUMBERTURN
#define TurnShortToRegister(register, point) {register.Long    = 0;    \
                                          register.Byte.b4 = ((char *)point)[0]; \
                                          register.Byte.b3 = ((char *)point)[1]; }
#else
#define TurnShortToRegister(register, point) {register.Long    = 0;    \
                                          register.Byte.b2 = ((char *)point)[0]; \
                                          register.Byte.b1 = ((char *)point)[1]; }
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++ СТРУКТУРА РАЗДЕЛЕНИЯ 8 БАЙТОВ НА СОСТАВЛЯЮЩИЕ ++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef union
{
  struct
  {
    unsigned char b1;
    unsigned char b2;
    unsigned char b3;
    unsigned char b4;
    unsigned char b5;
    unsigned char b6;
    unsigned char b7;
    unsigned char b8;
  }
    Byte;              // Представление в виде 8-битовых полей

  double Double;  //  Представление в виде 64-битового поля

  double Value() { return Double; }

  // Запросить значение (double) по адресу address (8 байт)
  double GetDouble(double & value)
  {
    return GetDouble((char *)&value);
  }

  // Запросить значение (double) по адресу address (8 байт)
  double GetDouble(const char* address)
  {
#ifdef RISCCPU
    Byte.b1 = address[0];
    Byte.b2 = address[1];
    Byte.b3 = address[2];
    Byte.b4 = address[3];
    Byte.b5 = address[4];
    Byte.b6 = address[5];
    Byte.b7 = address[6];
    Byte.b8 = address[7];
#else
    Double = *((double *)address);
#endif
    return (double)Double;
  }

  // Запросить значение (double) по адресу address (8 байт)
  // в обратной последовательности байт
  double GetTurnDouble(double & value)
  {
    return GetTurnDouble((char *)&value);
  }

  // Запросить значение (double) по адресу address (8 байт)
  // в обратной последовательности байт
  double GetTurnDouble(const char* address)
  {
    Byte.b8 = address[0];
    Byte.b7 = address[1];
    Byte.b6 = address[2];
    Byte.b5 = address[3];
    Byte.b4 = address[4];
    Byte.b3 = address[5];
    Byte.b2 = address[6];
    Byte.b1 = address[7];
    return (double)Double;
  }

  // Установить значение (double) по адресу address (8 байт)
  double PutDouble(double & value)
  {
    return PutDouble((char *)&value);
  }

  // Установить значение (double) по адресу address (8 байт)
  double PutDouble(char* address)
  {
#ifdef RISCCPU
    address[0] = Byte.b1;
    address[1] = Byte.b2;
    address[2] = Byte.b3;
    address[3] = Byte.b4;
    address[4] = Byte.b5;
    address[5] = Byte.b6;
    address[6] = Byte.b7;
    address[7] = Byte.b8;
#else
    *((double *)address) = Double;
#endif
    return (double)Double;
  }
}
  DOUBLEREGISTER;

// Определения для С (DOUBLEREGISTER , char*) :

#define DoubleToRegister(register, point) { register.Byte.b1 = ((char *)point)[0]; \
                                          register.Byte.b2 = ((char *)point)[1]; \
                                          register.Byte.b3 = ((char *)point)[2]; \
                                          register.Byte.b4 = ((char *)point)[3]; \
                                          register.Byte.b5 = ((char *)point)[4]; \
                                          register.Byte.b6 = ((char *)point)[5]; \
                                          register.Byte.b7 = ((char *)point)[6]; \
                                          register.Byte.b8 = ((char *)point)[7]; }

#define TurnDoubleToRegister(register, point) { register.Byte.b8 = ((char *)point)[0]; \
                                              register.Byte.b7 = ((char *)point)[1]; \
                                              register.Byte.b6 = ((char *)point)[2]; \
                                              register.Byte.b5 = ((char *)point)[3]; \
                                              register.Byte.b4 = ((char *)point)[4]; \
                                              register.Byte.b3 = ((char *)point)[5]; \
                                              register.Byte.b2 = ((char *)point)[6]; \
                                              register.Byte.b1 = ((char *)point)[7]; }

#define RegisterToDouble(register, point) { ((char *)point)[0] = register.Byte.b1; \
                                          ((char *)point)[1] = register.Byte.b2; \
                                          ((char *)point)[2] = register.Byte.b3; \
                                          ((char *)point)[3] = register.Byte.b4; \
                                          ((char *)point)[4] = register.Byte.b5; \
                                          ((char *)point)[5] = register.Byte.b6; \
                                          ((char *)point)[6] = register.Byte.b7; \
                                          ((char *)point)[7] = register.Byte.b8; }

// Запросить значение (short int)

#if !defined(WINCE) && !defined(NUMBERTURN)   
  #define GetWordValue(value) ((int)((unsigned short) value))
#else
  inline int GetWordValue(long int value) 
   {
     REGISTER Register;
     return Register.GetShort((char *)&value);
   }
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++   CТРУКТУРА ОПИСАНИЯ СТАНДАРТНОЙ ПАЛИТРЫ   ++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct PALETTE256
{
  WORD         palVersion;
  WORD         palNumEntries;
  PALETTEENTRY palPalEntry[256];
}
  PALETTE256;

// ФОРМАТ ЗАПОЛНЯЕМОГО ЭЛЕМЕНТА DIB

typedef unsigned int  CELL;
#define CELLSIZE      4

// ----------------------------------------------------------------
// Cтруктура размеров и свойств изображения объектов
// все размеры даны в мкм. (см. MAPGDI.H)
// Для шрифтов и шаблонов - размеры одной буквы
// Если объект "векторный" - TwoPoint = 1, иначе - 0;
// Для "неограниченного" объекта - размеры 0;
// ----------------------------------------------------------------

typedef struct IMAGESIZE
{
   unsigned int Number;          // номер функции отображения               // 26/03/12
   unsigned int Base  ;          // размер базы объекта
            int DeltaH;          // расстояние от левого верхнего
            int DeltaV;          // угла габаритов до базы(точки
                                  // отсчета) объекта
   unsigned int HorizontalSize;  // размеры по горизонтали
   unsigned int VerticalSize  ;  // размеры по вертикали

   unsigned int Horizontal : 1;  // расположен горизонтально
   unsigned int Vertical   : 1;  // расположен вертикально
   unsigned int TwoPoint   : 1;  // метрика из 2 точек
   unsigned int AlignV     : 2;  // выравнивание по вертикали
   unsigned int AlignH     : 2;  // выравнивание по горизонтали
   unsigned int Wide       : 2;  // ширина шрифта
   unsigned int Type       : 8;  // тип шрифта (0-4)
   unsigned int Italic     : 1;  // наклон символов 
   unsigned int Rezerv     :14;  // резерв
}
  IMAGESIZE;

// ----------------------------------------------------------------
// Cтруктура описывающая габаритную рамку изображения объекта
// в виде развернутого прямоугольника. Все размеры даны в мкм
// на изображении карты в базовом масштабе относительно первой
// точки метрики объекта. Направление осей картографическое (X вверх,
// Y вправо).
// ----------------------------------------------------------------

typedef struct IMAGEFRAME
{
   DOUBLEPOINT LeftTop;          // Координаты первой точки в мкм
   DOUBLEPOINT RightTop;         // Координаты второй точки в мкм
   DOUBLEPOINT RightBottom;      // Координаты третьей точки в мкм
   DOUBLEPOINT LeftBottom;       // Координаты четвертой точки в мкм
}
  IMAGEFRAME;

// ----------------------------------------------------------------
// Функции обратного вызова
// ----------------------------------------------------------------

typedef long int (WINAPI * BREAKCALLEX)(void * parm, long int isrepaint); 
typedef long int (WINAPI * EVENTSTATE)(void * parm, long int percent); // 09/03/16
typedef long int (WINAPI * BREAKCALL)(void * parm);                    // 07/06/16
typedef long int (WINAPI * BEFOREPAINT)
                 (void * parm, HDC hdc, RECT * rect);
typedef long int (WINAPI * MASKCALL)(void *  parm);
typedef long int (WINAPI * EVENTCALL)(void * parm, HPARAM value1,
                                      HPARAM value2, HPARAM value3);
typedef long int (WINAPI * MESSAGEBOXCALL)(void * parm, HWND hwnd,
                                           const WCHAR * message, const WCHAR * title,
                                           long int flag);

#ifdef LINUXAPI
// ------------------------------------------------------------------
//  Функция MessageBox для LINUX
// ------------------------------------------------------------------
typedef int (WINAPI * MESSAGEBOX)(HWND hwnd, const TCHAR * message,
                           const TCHAR * title, int flag);
#endif

// ----------------------------------------------------------------
//  КОДЫ ЯЗЫКОВ СООБЩЕНИЙ И ПУНКТОВ МЕНЮ
// ----------------------------------------------------------------

enum MAPAPILANGUAGE
{
  ML_ENGLISH   = 1,  // АНГЛИЙСКИЙ
  ML_RUSSIAN   = 2,  // РУССКИЙ
  ML_FRENCH    = 3,  // ФРАНЦУЗСКИЙ
  ML_SPANISH   = 4,  // ИСПАНСКИЙ
  ML_UKRAINIAN = 5,  // УКРАИНСКИЙ
  ML_LAST      = 5,  // Последний код  // 23/10/15
};

// ----------------------------------------------------------------
//  ИДЕНТИФИКАТОРЫ КОМАНД (WM_COMMAND)
// ----------------------------------------------------------------

#define WM_ERRORCOORD   0x583    // Информация о ошибках паспорта
#define WM_INFOLIST     0x584    // Информация о листе
#define WM_OBJECT       0x585    // Смена объекта
#define WM_LIST         0x586    // Смена листа
#define WM_ERROR        0x587    // Информация об ошибках для записи в ЛОГ-файл
#define WM_MAP          0x588    // Смена текущей карты
#define WM_ERRORSXF     0x589    // Информация об ошибках обработки SXF

// ----------------------------------------------------------------
//  ИДЕНТИФИКАТОРЫ СООБЩЕНИЙ
// ----------------------------------------------------------------
#define WM_PROGRESSBAR  0x590    // Сообщение о соcтоянии процесса 
                                 // wparam - (-1 - старт), (-2 - стоп), % выполнения 
                                 // lparam - комментарий (текст в кодировке приложения ANSI \ UNICODE)
                                 // Если комментарий в % выполнения не задан, то нажатие ESC не приводит к вопросу о прерывании процесса
                                 // При прерывании возвращает WM_PROGRESSBAR

#define WM_MAPEVENT     0x591    // Сообщение о событиях карты
#define WM_PROGRESSICON 0x592    // Сообщение о соcтоянии процесса
                                 // для отображения в иконке программы
                                 // при свернутом главном окне
#define WM_PROGRESSBARUN 0x593   // Сообщение о соcтоянии процесса 
                                 // wparam - (-1 - старт), (-2 - стоп), % выполнения 
                                 // lparam - комментарий (текст в кодировке UNICODE)
                                 // Если комментарий в % выполнения не задан, то нажатие ESC не приводит к вопросу о прерывании процесса
                                 // При прерывании возвращает WM_PROGRESSBAR


// ----------------------------------------------------------------
//  ИДЕНТИФИКАТОРЫ СООБЩЕНИЙ (MESSAGE)
// ----------------------------------------------------------------

#define WM_LEFTTOPPOINT 0x600    // Cообщение окну карты из задачи
                                 // Запрос координат верхнего левого
                                 // угла окна карты в пикселах изображения
                                 // wparam : (POINT * point)        
                                 // lparam : 0
                                 // result : 0x600

#define WM_MOVEDOC      0x601    // Cообщение окну карты из задачи
                                 // Указание на необходимость
                                 // переместить изображение карты в заданную
                                 // точку, координаты верхнего левого угла окна
                                 // карты в пикселах изображения
                                 // wparam : (POINT * point)     
                                 // lparam : 0 или (POINT * marker)
                                 // координаты маркера,обрабатывать не обязательно
                                 // result : 0x601

#define WM_OPENDOC      0x602    // Cообщение окну приложения или карты из задачи
                                 // Указание на необходимость
                                 // открыть карту по имени файла (MAP,MTW,RSW,...)
                                 // wparam : (const char * name)
                                 // lparam : 0 или (DOUBLEPOINT * point),
                                 // координаты в метрах точки, которую нужно
                                 // показать в центре окна карты
                                 // result : 0x602

#define AW_CLOSEDIALOGNOTIFY 0x610 // Cообщение главному окну о закрытии диалога
                                   // Указание на необходимость вызвать функцию
                                   // закрытия диалога по указателю
                                   // wparam : 0                    
                                   // lparam : указатель диалога
                                   // result : 0x610   

// --------------------------------------------------------------
//  Структуры для вызова прикладных задач
// --------------------------------------------------------------

typedef struct TASKPARM        // ПАРАМЕТРЫ ПРИКЛАДНОЙ ЗАДАЧИ
{
  long int    Language;        // Код языка диалогов (1 - ENGLISH,
                               // 2 - RUSSIAN, ...)
#if defined(_M_X64) || defined(BUILD_DLL64)      
#ifdef WIN32API
  long int    Zero;            // Выравнивание указателей
#endif
#endif
  HINSTANCE   Resource;        // Модуль ресурсов приложения
  const char* HelpName;        // Полное имя файла ".hlp"
  const char* IniName;         // Полное имя файла ".ini" приложения
  const char* PathShell;       // Каталог приложения (exe,dll,...)
  const char* ApplicationName; // Имя приложения
#ifdef WIN32API
  HWND        Handle;          // Идентификатор главного окна приложения
#else
  MSGHANDLER  Handle;          // Идентификатор обработчика команд главного окна приложения
#endif
}
  TASKPARM;

typedef struct TASKPARMEX      // ПАРАМЕТРЫ ПРИКЛАДНОЙ ЗАДАЧИ
{
  long int    Language;        // Код языка диалогов (1 - ENGLISH,
                               // 2 - RUSSIAN, ...)
#if defined(_M_X64) || defined(BUILD_DLL64)     
#ifdef WIN32API
  long int    Zero;            // Выравнивание указателей
#endif
#endif
  HINSTANCE   Resource;        // Модуль ресурсов приложения
  const char* HelpName;        // Полное имя файла ".hlp"
  const char* IniName;         // Полное имя файла ".ini" приложения
  const char* PathShell;       // Каталог приложения (exe,dll,...)
  const char* ApplicationName; // Имя приложения
#ifdef WIN32API
  HWND        Handle;          // Идентификатор главного окна приложения
#else
  MSGHANDLER  Handle;          // Идентификатор обработчика команд главного окна приложения
#endif
  HWND        DocHandle;       // Идентификатор окна карты (документа)
  long int    StayOnTop;       // Признак выставления форме свойства StayOnTop  
}
  TASKPARMEX;

// --------------------------------------------------------------
//  Структуры для взаимодействия с ГИС-сервером
// --------------------------------------------------------------

#define HOSTALIAS    "HOST#"
#ifdef WIN32API                     // 18/02/13
#define WHOSTALIAS   L"HOST#"
#else
#ifdef BUILD_DLL
#define WHOSTALIAS   L"HOST#"
#else
#define WHOSTALIAS   "H\0O\0S\0T\0#\0"
#endif
#endif
#define HOSTSIZE     5

#define MAPALIAS     "ALIAS#"
#ifdef WIN32API                     // 18/02/13
#define WMAPALIAS    L"ALIAS#"
#else
#ifdef BUILD_DLL
#define WMAPALIAS    L"ALIAS#"
#else
#define WMAPALIAS    "A\0L\0I\0A\0S\0#\0"
#endif
#endif
#define ALIASSIZE    6

#define DOCALIAS     "DOC#"
#ifdef WIN32API                     // 18/02/13
#define WDOCALIAS    L"DOC#"
#else
#ifdef BUILD_DLL
#define WDOCALIAS    L"DOC#"
#else
#define WDOCALIAS    "D\0O\0C\0#\0"
#endif
#endif
#define DOCALIASSIZE 4

typedef struct TMCUSERPARM  // ПАРАМЕТРЫ ПОЛЬЗОВАТЕЛЯ
{
  char Name[32];            // Имя (Кодировка ANSI)
  char Password[64];        // Пароль
}
  TMCUSERPARM;

typedef struct TMCMAPLISTITEM
{
  unsigned int Level;      // Уровень вложенности элемента с 1 до MaxLevel    // 26/03/12
  unsigned int Flags;      // Флаг редактирования, 0 - нет прав редактировать
  unsigned int Type;       // Тип файла MAPFILES или 0 для промежуточного уровня
  unsigned int Size;       // Размер данных Кбайт                             // 26/06/14
  char Name[256];          // Имя элемента (Алиас)
}
  TMCMAPLISTITEM;

typedef struct TMCMAPLIST   // СПИСОК ДОСТУПНЫХ КАРТ
{
  TMCMAPLIST() { Init(); }

  void Init() {  Ident = 0x7F7F7F7F;
                 Length = sizeof(TMCMAPLIST);
                 Count = 0; MaxLevel = 1; }

  int Ident;           // Идентификатор записи 0x7F7F7F7F                         // 26/03/12
  int Length;          // Общая длина записи
  int Count;           // Число элементов в списке
  int MaxLevel;        // Максимальный уровень вложенности элементов (дерево)
  TMCMAPLISTITEM Item[1];   // Массив элементов списка доступных карт (в количестве Count)
}
  TMCMAPLIST;

#define TMCDATAIDENT  0x7FFF7FFF

typedef struct TMCDATALISTITEM
{
  unsigned int Ident;      // 0x7FFF7FFF
  unsigned int Length;     // Размер данной записи
  unsigned int Level;      // Уровень вложенности элемента с 1 до MaxLevel
  unsigned int Flags;      // Флаг редактирования, 0 - нет прав редактировать
  unsigned int Type;       // Тип файла MAPFILES или -1 - папка или 0 - комментарий
  unsigned int Size;       // Размер данных Кбайт
  unsigned int Date;       // Дата обновления в формате YYYYMMDD
  unsigned int Time;       // Время обновления - число секунд от 00:00:00
  WCHAR        Name[2];    // Имя элемента (папки или файла) переменной длины
}
  TMCDATALISTITEM;

typedef struct TMCDATALIST   // СПИСОК ДАННЫХ И ПАПОК                             // 26/06/14
{
  TMCDATALIST() { Init(); }

  void Init() {  Ident = 0x7F7F7F7F;
                 Length = sizeof(TMCDATALIST);
                 Count = 0; MaxLevel = 1; }

  int Ident;           // Идентификатор записи 0x7F7F7F7F       
  int Length;          // Общая длина записи
  int Count;           // Число элементов в списке
  int MaxLevel;        // Максимальный уровень вложенности элементов (дерево)
  TMCDATALISTITEM Item[1];   // Массив элементов списка доступных карт (в количестве Count)
}
  TMCDATALIST;

// Заголовок данных о состоянии подключений к ГИС Сервер
typedef struct GSMONITOR
{
  int Ident;          // 0x7F7F7F7F                          
  int Length;         // Общая длина записи в байтах
  int Version;        // Версия ГИС Сервиса 0x0T112233 11.22.33 (T=1,x64)
  int Count;          // Число подключенных клиентов
  int State;          // Состояние подключенных клиентов
  char BeginTime[32]; // Время старта ГИС Сервиса
  char ParmName[256]; // Имя файла параметров
  int MapEdit;        // Общее число карт, открытых на редактирование
  int MapRead;        // Общее число карт, открытых на чтение
  int MapCopy;        // Общее число карт, разрешенных к копированию
  int RswEdit;        // Общее число растров, открытых на редактирование
  int RswRead;        // Общее число растров, открытых на чтение
  int RswCopy;        // Общее число растров, разрешенных к копированию
  int MtwEdit;        // Общее число матриц, открытых на редактирование
  int MtwRead;        // Общее число матриц, открытых на чтение
  int MtwCopy;        // Общее число матриц, разрешенных к копированию
  int MapNotPrint;    // Общее число карт, не разрешенных к печати
  int RswNotPrint;    // Общее число растров, не разрешенных к печати
  int MtwNotPrint;    // Общее число матриц, не разрешенных к печати

//Если число пользователей > 32 - передается GSMUSERSHORT
//GSMUSEREX    User[1];    // Список данных о подключении пользователей
//GSMUSERSHORT  User[1];   // Список данных о подключении пользователей
}
  GSMONITOR;

// --------------------------------------------------------------
//  Структуры для списка врезок
// --------------------------------------------------------------

enum INSETPLACE      // СПОСОБ ПРИВЯЗКИ ВРЕЗКИ
{
   INP_TOTAL   = 0,  // Отображать все данные в заданном окне
   INP_FRAME   = 1,  // Отображать указанный фрагмент в заданном масштабе
};

typedef struct INSETDESC    // ОПИСАНИЕ ЭЛЕМЕНТА СПИСКА ВРЕЗОК
{
  INSETDESC()
  {
    Ident       = 0;
    Path[0]     = 0;
    Name[0]     = 0;
    Scale       = 100000;
    TopScale    = 40000000;
    BottomScale = 1;
    View        = 1;
    BackColor   = 0x07F7F7F;
    BorderColor = 0;
    Transparence= 0;
    BorderThick = 260;
    BorderView  = 1;
    PlaceFlag   = INP_TOTAL;
    FirstPlace  = 0;
    SecondPlace = 0;
    PlaceType   = PP_PLANE;
    Scheme      = 0; 
  }

  int         Length;       // Размер данной структуры
  int         Ident;        // Идентификатор элемента
  WCHAR       Path[1024];   // Полный путь к файлу данных
  WCHAR       Name[128];    // Условное название данных (первоначально из данных)

                            // Координаты в системе открытого документа (метры)
  DOUBLEPOINT FirstPlace;   // Координаты привязки левого верхнего угла
  DOUBLEPOINT SecondPlace;  // Координаты привязки правого нижнего угла
                            // Координаты в системе данных врезки для INP_FRAME (метры)
  DOUBLEPOINT FramePlace;   // Координаты точки привязки (центр фрагмента в системе врезки)
  int         PlaceFlag;    // Способ привязки (INP_TOTAL, INP_FRAME)
  int         PlaceType;    // Система координат привязки
                            // (PP_PLANE - прямоугольная, PP_GEO - геодезиченская WGS84)

  int         Scale;        // Масштаб отображения (первоначально из данных для INP_FRAME)
  int         TopScale;     // Знаменатель масштаба верхней границы видимости
  int         BottomScale;  // Знаменатель масштаба нижней границы видимости
  int         View;         // Признак видимости врезки

  int         Scheme;       // Признак отображения врезки в виде схемы
  int         BackColor;    // Цвет фона окна данных
  int         Transparence; // Признак прозрачности фона
  int         BorderColor;  // Цвет рамки окна
  int         BorderThick;  // Толщина рамки
  int         BorderView;   // Признак отображения рамки
}
  INSETDESC;


// --------------------------------------------------------------
// Выдать сообщение об ошибке
// code - код ошибки (см. MAPERR.RH)
// --------------------------------------------------------------
extern "C"
{
  _MAPIMP void _MAPAPI ErrorMessage(int code,const char * filename); 
}

enum MESSAGETYPE
{
  MT_INFO     = 0,    // Информационное сообщение
  MT_ERROR    = 1,    // Сообщение об ошибке
  MT_WARNING  = 2,    // Информационное сообщение
  MT_CONTINUE = 4,    // Продолжение предыдущего сообщения
					  // (на той же строке)
};

// --------------------------------------------------------------
// Описание структур для работы с меню
// --------------------------------------------------------------

#define MENUEXTENDIDENT   0x7FFF7FFF

typedef struct MENUEXTEND         // ОПИСАНИЕ РАЗДЕЛА МЕНЮ
{
  long int Ident;                 // Метка начала записи 0x7FFF7FFF
  long int Count;                 // Число пунктов меню
  struct
  {
   long int Command;              // Идентификатор команды (-1 для разделителя)
   long int Check;                // Признак выделения пункта галочкой (0/1)
   char     Text[32];             // Текст строки меню
  }
   Item[32];
}
  MENUEXTEND;

typedef struct COMMANDENABLER     // ЗАПРОС СОСТОЯНИЯ ПУНКТА МЕНЮ ИЛИ КНОПКИ
{
  const WCHAR * SetText(const WCHAR * text)
  { if (text) WcsCopy(Text, text, sizeof(Text)); else Text[0] = 0;
    return Text; }
 
  long int     Command;           // Идентификатор команды (-1 для разделителя)
  long int     Enable;            // Признак активности
  long int     Check;             // Признак выставления "галочки" - 1, снятия - 0
#if defined(_M_X64) || defined(BUILD_DLL64)
  long int     Zero;              // Выравнивание
#endif
  MENUEXTEND * Menu;              // Раздел меню, который добавляется за текущим
  WCHAR        Text[256];         // Новый текст пункта меню                         // 15/08/14
}
  COMMANDENABLER;

typedef struct VIEWHELPEX         // ЗАПРОС НА ОТОБРАЖЕНИЕ HELP (AW_VIEWHELPEX)
{
  const char * HelpName;          // Имя файла HELP
  long int     Topic;             // Номер раздела
}
  VIEWHELPEX;


typedef struct CONTROLMENU        // ДОБАВИТЬ/УДАЛИТЬ/ОТОБРАЗИТЬ/ЗАПРОСИТЬ... ПУНКТ ГЛАВНОГО МЕНЮ (AW_MENUCONTROL)
{                                 // WPARAM=MC_APEENDMENU,MC_DELETEMENU...,LPARAM=CONTROLMENU* 
  char Name[32];                  // Имя пункта меню (идентификатор)
  char Caption[64];               // Название пункта меню (отображается на панели)     
  char After[32];                 // Имя пункта меню, за которым вставляется данный пункт ("MenuFile","MenuView"...)
}                                 // Если равно 0, то в конец, если символ #, то за ним порядковый номер с 1,
  CONTROLMENU;                    // которым будет размещен данный пункт

typedef struct CONTROLITEM        // ДОБАВИТЬ/УДАЛИТЬ... ПОДПУНКТ ГЛАВНОГО МЕНЮ (AW_MENUCONTROL)
{                                 // WPARAM=MC_APPENDMENUITEM..,LPARAM=CONTROLITEM*
  char Name[32];                  // Имя пункта меню (идентификатор) ("MenuFile","MenuView"...,"MenuTask"...)
  char Caption[64];               // Название подпункта меню (отображается на панели)    
  long int Item;                  // Числовой идентификатор команды (посылается задаче при выборе пункта меню)
  long int SubItem;               // Числовой идентификатор команды (посылается задаче при выборе подпункта меню)
  long int After;                 // Идентификатор пункта/подпункта за которым добавляется пункт меню,
				  // 0 - в конце, -1 - в начале, -2 - вторым и т.д.
#if defined(_M_X64) || defined(BUILD_DLL64)
  long int     Zero;              // Выравнивание
#endif
}
  CONTROLITEM;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++  Описание структур классификатора (RSC)                   +++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ЛОКАЛИЗАЦИЯ ОБ'ЕКТА (ТИП)
enum OBJECT_LOCAL { LOCAL_LINE = 0,  LOCAL_SQUARE = 1, LOCAL_POINT = 2,
					LOCAL_TITLE = 3, LOCAL_VECTOR = 4, LOCAL_MIXED = 5 };

// НАПРАВЛЕНИЕ ЦИФРОВАНИЯ ОБ'ЕКТА
enum OBJECT_DIRECT       // Диапазон от 0 до OD_LAST    // 26/12/12
{
   OD_NONE      = 0,     // Произвольное. Используется:
						 // - для точечных;
						 // - для линейных объектов, направление контуров которых
						 //   не имеет значения (линейные дороги);
						 // - для векторных знаков, изображение которых
						 //   симметрично относительно базовой линии знака (мосты)

   OD_ALONG     = 1,     // Однозначное (вдоль контура). Используется:
						 // - для подписей;
						 // - для линейных объектов, имеющих однозначное направление
						 //   контуров (линейные реки - от истока к устью);
                         // - для векторных знаков, имеющих однозначное направление

   OD_UNDEFINED = 1,     // Не определено. Возвращают функции запроса направления
                         // метрики контура в случае, когда контур незамкнут или
                         // имеет нулевую площадь (вырожден в точку или имеет "петли")

   OD_RIGHT     = 2,     // Объект справа от контура. Используется для площадных:
                         //   основной контур - по часовой стрелке;
                         //   подобъекты      - против часовой стрелки

   OD_LOWRIGHT  = 3,     // Справа ниже. Используется для горизонталей.
                         // Замкнутая горизонталь с направлением:
                         // - по часовой стрелке ограничивает яму;
                         // - против часовой стрелки ограничивает холм

   OD_LEFT      = 4,     // Объект слева от контура. Используется для площадных:
                         //   основной контур - против часовой стрелки
                         //   подобъекты      - по часовой стрелке

   OD_CLOCKWISE = 5,     // Все контура объекта цифруются по часовой стрелке  //23/01/12

   OD_ANTICLOCKWISE = 6, // Все контура объекта цифруются против часовой стрелки

   OD_LAST      = 6,     // Последний допустимый код направления
};

// РЕЖИМЫ ЗАМЫКАНИЯ ОБ'ЕКТА      //01/03/01

enum { ABR_APPEND   = 0,  // Добавить точку в конце объекта
                            // не добавляет, если объект замкнут
       ABR_LAST     = 1,  // Обновить последнюю точку
       ABR_FIRST    = 2,  // Обновить первую точку
       ABR_ADDLAST  = 3,  // Добавить точку в конце объекта
       ABR_ADDFIRST = 4   // Добавить первую точку
     };

#define  OTHERSCALERSC  10000L
#define  MAXSCALERSC  40000000L

// СТРУКТУРА ДЛЯ СОЗДАНИЯ КЛАССИФИКАТОРА
typedef struct RSCCREATE
{
  char Name[32]; // Название классификатора
  char Type[32]; // Тип классификатора
  char Code[8];  // Код классификатора
  int  Scale;    // Знаменатель масштаба
  int  Language; // Язык (MAPAPILANGUAGE)
}
  RSCCREATE;

typedef struct RSCCREATEUN
{
  WCHAR Name[32]; // Название классификатора в UNICODE
  WCHAR Type[32]; // Тип классификатора в UNICODE
  WCHAR Code[8];  // Код классификатора в UNICODE
  int   Scale;     // Знаменатель масштаба
  int   Language;  // Язык (MAPAPILANGUAGE)
}
  RSCCREATEUN;

// ОПИСАНИЕ ОБ'ЕКТОВ
typedef struct RSCOBJECT
{
   unsigned long Length   ; // РАЗМЕР СТРУКТУРЫ
   unsigned long Code     ; // КЛАССИФИКАЦИОННЫЙ КОД
   unsigned long Local    ; // ХАРАКТЕР ЛОКАЛИЗАЦИИ  (OBJECT_LOCAL)
   unsigned long Segment  ; // Н0МЕР СЛОЯ ( НЕ БОЛЕЕ 255)
   unsigned long Scale    ; // 1 - ОБ'ЕКТ МАСШТАБИРУЕТСЯ
                            // 0 - ОБ'ЕКТ НЕ МАСШТАБИРУЕТСЯ
   unsigned long Direct   ; // НАПРАВЛЕНИЕ ЦИФРОВАНИЯ ОБ'ЕКТА (OBJECT_DIRECT)
   unsigned long Bot      ; // НИЖНЯЯ ГРАНИЦА ВИДИМОСТИ (ЗНАМЕНАТЕЛЬ МАСШТАБА)
   unsigned long Top      ; // ВЕРХНЯЯ ГРАНИЦА ВИДИМОСТИ (ЗНАМЕНАТЕЛЬ МАСШТАБА)
   char Name[32]          ; // НАЗВАНИЕ ( НЕ БОЛЕЕ 30)        // 04/12/14
}
   RSCOBJECT;

typedef struct RSCOBJECTEX                             
{
   unsigned long Length   ; // РАЗМЕР СТРУКТУРЫ
   unsigned long Code     ; // КЛАССИФИКАЦИОННЫЙ КОД
   unsigned long Local    ; // ХАРАКТЕР ЛОКАЛИЗАЦИИ  (OBJECT_LOCAL)
   unsigned long Segment  ; // Н0МЕР СЛОЯ ( НЕ БОЛЕЕ 255)
   unsigned long Scale    ; // 1 - ОБ'ЕКТ МАСШТАБИРУЕТСЯ
                            // 0 - ОБ'ЕКТ НЕ МАСШТАБИРУЕТСЯ
   unsigned long Direct   ; // НАПРАВЛЕНИЕ ЦИФРОВАНИЯ ОБ'ЕКТА (OBJECT_DIRECT)
   unsigned long Bot      ; // НИЖНЯЯ ГРАНИЦА ВИДИМОСТИ (ЗНАМЕНАТЕЛЬ МАСШТАБА)
   unsigned long Top      ; // ВЕРХНЯЯ ГРАНИЦА ВИДИМОСТИ (ЗНАМЕНАТЕЛЬ МАСШТАБА)
   unsigned char Name[32] ; // НАЗВАНИЕ
   unsigned char Key [32] ; // УНИКАЛЬНЫЙ СИМВОЛЬНЫЙ КЛЮЧ ОБЪЕКТА
}
   RSCOBJECTEX;


// ОПИСАНИЕ СЛОЕВ
typedef struct RSCSEGMENT
{
   unsigned long Order;     // ПОРЯДОК ВЫВОДА СЛОЯ НА ПЕЧАТЬ
   unsigned char Name[32];  // НАЗВАНИЕ ( НЕ БОЛЕЕ 30)
}
   RSCSEGMENT;

// ОПИСАНИЕ СЕМАНТИКИ
typedef struct RSCSEMANTICEX  // РАСШИРЕННОЕ ОПИСАНИЕ СЕМАНТИЧЕСКОЙ
                              // ХАРАКТЕРИСТИКИ ОБ'ЕКТА
{
  long int Code;         // внешний код
  long int Type;         // тип значения
  long int Reply;        // разрешается повторение (1)
  long int Enable;       // допустимая(1),обязательная(2)
                         // (при создании семантики поле не учитывается)
  long int Service;      // служебная,допустима для всех объектов (1)
  long int Reserv;       // резерв (должен быть равен 0)
  char     Name[32];     // название характеристики
  char     Unit[8];      // единица измерения
  double   Minimum;      // минимальное значение
  double   Default;      // умалчиваемое значение
  double   Maximum;      // максимальное значение
  long int Size;         // общий размер поля значения семантики
  long int Decimal;      // точность поля значения семантики
  char     ShortName[16];// короткое имя семантики (связь с полем БД)
}
  RSCSEMANTICEX;


// ОПИСАНИЕ ФОРМУЛЫ           // 14/11/14
typedef struct RSCFORMULE     // ОПИСАНИЕ ФОРМУЛЫ
{
  unsigned int Code;          // код
  unsigned int Semantic;      // код назначенной семантики или 0
  unsigned int Length;        // длина текста формулы
  int          Type;          // тип интерпретатора  1 - числовая формула,
                              // 2 - символьная, 4 - программируемая формула,
                              // 8 - формула на языке LUA
  int          Round;         // 0 - НЕТ ОКРУГЛЕНИЯ
                              // 1 - ОКРУГЛЕНИЕ ПО ГАУССУ
                              // 2 - ОКРУГЛЕНИЕ С УВЕЛИЧЕНИЕМ
                              // 3 - ОКРУГЛЕНИЕ С УМЕНЬШЕНИЕМ
  int         Precision;      // 3 - ДО ТЫСЯЧ
                              // 2 - ДО СОТЕН
                              // 1 - ДО ДЕСЯТЫХ
                              // 0 - ДО ЦЕЛЫХ
                              // -1 - ДО ОДНОГО ЗНАКА ПОСЛЕ ЗАПЯТОЙ
                              // -2 - ДО ДВУХ ЗНАКОВ ПОСЛЕ ЗАПЯТОЙ
                              // -3 - ДО ТРЕХ ЗНАКОВ ПОСЛЕ ЗАПЯТОЙ
  int         Index;          // НОМЕР БИБЛИОТЕКИ В СПИСКЕ ДЛЯ Flag=4 ИЛИ 0
  int         Number;         // НОМЕР ФОРМУЛЫ В БИБЛИОТЕКЕ ДЛЯ Flag=4 ИЛИ 0
  WCHAR        Name[128];     // название формулы
}
  RSCFORMULE;


#define SEMLIBIDENT   0x7E7E7E7E

typedef struct SEMLIBLIST
{
  int       Ident;      // Идентификатор записи 0x7E7E7E7E
  int       Count;      // Число видов объектов в описании
  struct
  {
    int       Type;     // Код вида функции
    int       Zero;     // Выравнивание на 8 
    WCHAR * Name[4];    // Условное название (назначение) функции
                        // Name[0]-ML_ENGLISH, Name[1]-ML_RUSSIAN...
  }
    Element[32];        // Реальное значение размерности массива задано в поле Count
}
  SEMLIBLIST;


enum FRAMEINCODE    
{
   FIRSTSERVEXCODE  = 1000000000l, // Начало внешних кодов служебных
   LINESERVEXCODE   = 1000000001l, // Внешний код служебного линейного
   LINESERVINCODE   = 1          , // Внутренний код служебного линейного
   SQUARESERVEXCODE = 1000000002l, // Внешний код служебного площадного
   SQUARESERVINCODE = 2          , // Внутренний код служебного площадного
   POINTSERVEXCODE  = 1000000003l, // Внешний код служебного  точечного
   POINTSERVINCODE  = 3          , // Внутренний код служебного точечного
   TEXTSERVEXCODE   = 1000000004l, // Внешний код служебного текста
   TEXTSERVINCODE   = 4          , // Внутренний код служебного текста
   VECTSERVEXCODE   = 1000000005l, // Внешний код служебного векторного
   VECTSERVINCODE   = 5          , // Внутренний код служебного векторного
   COMSERVEXCODE    = 1000000006l, // Внешний код служебного шаблона
   COMSERVINCODE    = 6          , // Внутренний код служебного шаблона
   SERVOBJECT       = 15         , // Количество служебных объектов
};

enum SEMTYPE           // ТИПЫ ЗНАЧЕНИЯ СЕМАНТИКИ
{
   TUNDEFINED = -1,    // Значение не установлено
   TSTRING    = 0,     // Символьная строка
   TNUMBER    = 1,     // Числовое значение
   TANYFILE   = 9,     // Имя файла зарегистрированного типа
   TBMPFILE   = 10,    // Имя файла BMP
   TOLEFILE   = 11,    // Имя файла,обрабатываемого OLE-сервером
   TREFER     = 12,    // Ссылка на произвольный объект карты
                       // (уникальный номер объекта)
   TMAPFILE   = 13,    // Имя файла-паспорта района
   TTXTFILE   = 14,    // Имя текстового файла
   TPCXFILE   = 15,    // Имя файла PCX
   TCODE      = 16,    // Значение в виде числового кода
                       // из классификатора значений
   TDATE      = 17,    // Значение даты в числовом виде (ГГГГММДД)
   TANGLE     = 18,    // Угловая величина в радианах
   TTIME      = 19,    // Значение времени в числовом виде (ЧЧММСС)
   TFONT      = 20,    // Имя шрифта ("Arial", "Courier"...)
   TCOLOR     = 21,    // Числовое значение цвета в RGB
   TFDIGITAL  = 22,    // Семантика типа формула числовая (с вычисляемыми значениями)   // 18/02/13
   TFSTRING   = 23,    // Семантика типа формула символьная (с вычисляемыми значениями) // 18/02/13
   TLAST      = 23,    // ВЕРХНЯЯ ГРАНИЦА СПИСКА КОДОВ

   TDOCREFMIN = 9,     // НИЖНЯЯ ГРАНИЦА ТИПА СЕМАНТИКИ - ССЫЛКА НА ДОКУМЕНТ (ГРАФИЧЕСКИЙ ФАЙЛ, КАРТА)
   TDOCREFMAX = 15,    // НИЖНЯЯ ГРАНИЦА ТИПА СЕМАНТИКИ - ССЫЛКА НА ДОКУМЕНТ (ГРАФИЧЕСКИЙ ФАЙЛ, КАРТА)
};

// ЗАРЕЗЕРВИРОВАННЫЕ КОДЫ СЕМАНТИЧЕСКИХ ХАРАКТЕРИСТИК,
// ВЛИЯЮЩИЕ НА ВИД ЗНАКА                                 
// ---------------------------------------------------
#define EXTSEMANTIC        31001  // НАЧАЛО ДИАПАЗОНА СЕРВИСНЫХ СЕМАНТИК
#define SEMIMAGESCALE      31001  // МАСШТАБ ОТОБРАЖЕНИЯ ЗНАКА В ПРОЦЕНТАХ
#define SEMIMAGECOLOR      31002  // ЦВЕТ ОТОБРАЖЕНИЯ ЗНАКА RGB
#define SEMIMAGEHIGHT      31003  // ВЫСОТА ШРИФТА         
#define SEMIMAGEFONT       31004  // НАЗВАНИЕ ШРИФТА       
#define SEMIMAGETHICK      31005  // ТОЛЩИНА ЛИНИИ В ММ          
#define SEMIMAGETHICK2     31006  // ТОЛЩИНА ОКОНЧАНИЯ ЛИНИИ В ММ 
#define SEMCOLORWEIGHT     31007  // ВЕС ЦВЕТА ОБЪЕКТА В ПРОЦЕНТАХ
#define SEMLINETHICK       31008  // ТОЛЩИНА ЛИНИИ В ПУНКТАХ      
#define SEMLINECOLOR       31009  // ЦВЕТ ЛИНИИ 
                                  // ЧИСЛОВОЕ ЗНАЧЕНИЕ ЦВЕТА RGB
#define SEMLINEPATTERN     31010  // НОМЕР СИМВОЛА ЗАЛИВКИ ЛИНИИ В MAPINFOW.PEN
                                  //  (1-255)
#define SEMMARKSIZE        31011  // РАЗМЕР СИМВОЛА ЗНАКА В ПУНКТАХ (1-48) 

#define SEMMARKCOLOR       31012  // ЦВЕТ ЗНАКА, ПЛОЩАДИ
                                  // ЧИСЛОВОЕ ЗНАЧЕНИЕ ЦВЕТА RGB
#define SEMFONCOLOR        31013  // ЦВЕТ ФОНА 
                                  // ЧИСЛОВОЕ ЗНАЧЕНИЕ ЦВЕТА RGB
#define SEMMARKSHAPE       31014  // НОМЕР СИМВОЛА ЗНАКА В СООТВЕТСТВУЮЩЕМ
                                  // TRUETYPE ШРИФТЕ (БОЛЬШЕ 31), 
                                  // УКАЗАННОМ В СЕМАНТИКЕ SEMIMAGEFONT
#define SEMSQUAREPATTERN   31015  // НОМЕР СИМВОЛА ЗАЛИВКИ ПЛОЩАДИ В MAPINFOW.PEN
                                  // (1-255)
#define SEMFONTSTYLE       31016  // СТИЛЬ ШРИФТА (СУММА ПРИЗНАКОВ:
                                  // 0 - ОБЫЧНЫЙ, 1 - TОЛСТЫЙ, 16 - В РАМКЕ,
                                  // 32 - С ТЕНЬЮ, 256 - БЕЛАЯ РАМКА)
#define EXTSEMANTICEND     31016  // КОНЕЦ ДИАПАЗОНА

// ЗАРЕЗЕРВИРОВАННЫЕ КОДЫ СЕМАНТИЧЕСКИХ ХАРАКТЕРИСТИК,
// ДЛЯ АДРЕСА                                               
// ---------------------------------------------------
#define ADRSEMANTIC        31201  // НАЧАЛО ДИАПАЗОНА АДРЕСНЫХ СЕМАНТИК
#define SEMADRSTREET       31201  // НАЗВАНИЕ УЛИЦЫ
#define SEMADRHOUSE        31202  // НОМЕР ДОМА
#define SEMADRTRUNK        31203  // НОМЕР КОРПУСА
#define SEMADRBUILD        31204  // НОМЕР СТРОЕНИЯ
#define SEMADRESTATE       31205  // НОМЕР ВЛАДЕНИЯ
#define SEMADRSTREETCODE   31206  // КОД УЛИЦЫ
#define SEMADRTOWN         31207  // НАЗВАНИЕ ГОРОДА (НАСЕЛЕННОГО ПУНКТА)
#define SEMADRTOWNCODE     31208  // КОД ГОРОДА (НАСЕЛЕННОГО ПУНКТА) ПО ОКАТО
#define SEMADRPOSTCODE     31209  // ПОЧТОВЫЙ ИНДЕКС        
#define ADRSEMANTICEND     31209  // КОНЕЦ ДИАПАЗОНА

// ЗАРЕЗЕРВИРОВАННЫЕ КОДЫ СЕМАНТИЧЕСКИХ ХАРАКТЕРИСТИК
// ---------------------------------------------------
#define OBJEXCODEINSEMANTIC 32800  // КЛАССИФИКАЦИОННЫЙ КОД ОБЪЕКТА
#define SEMOBJEXCODE        32800  // КЛАССИФИКАЦИОННЫЙ КОД ОБЪЕКТА

#define GROUPLEADER         32801  // ПРИЗНАК ОБЪЕКТА, ИМЕЮЩЕГО ССЫЛКУ
#define SEMGROUPLEADER      32801  // НА ПОДЧИНЕННЫЙ ОБЪЕКТ

#define GROUPSLAVE          32802  // ПРИЗНАК ОБЪЕКТА, НА КОТОРЫЙ
#define SEMGROUPSLAVE       32802  // ИМЕЕТСЯ ССЫЛКА ОТ ГЛАВНОГО ОБЪЕКТА;

#define GROUPPARTNER        32803  // ПРИЗНАК ОБЪЕКТА, ВХОДЯЩЕГО
#define SEMGROUPPARTNER     32803  // В ГРУППУ РАВНОПРАВНЫХ ОБЪЕКТОВ

#define SEMOBJECTTOTEXT     32804  // ССЫЛКА НА ПОДПИСЬ ОТ ОБЪЕКТА
                                   // (КЛЮЧ ОБЪЕКТА-ПОДПИСЬ НА ТОЙ ЖЕ КАРТЕ)
#define SEMOBJECTFROMTEXT   32805  // ССЫЛКА НА ОБЪЕКТ ОТ ПОДПИСИ
                                   // (КЛЮЧ ОСНОВНОГО ОБЪЕКТА НА ТОЙ ЖЕ КАРТЕ)
#define SEMOBJECTTOTEXTFAR  32806  // ССЫЛКА НА ПОДПИСЬ ОТ ОБЪЕКТА               // 31/05/16
                                   // (КЛЮЧ ОБЪЕКТА-ПОДПИСЬ НА ГЛАВНОЙ КАРТЕ)
#define SEMOBJECTFROMTEXTFAR 32807 // ССЫЛКА НА ОБЪЕКТ ОТ ПОДПИСИ                // 31/05/16
                                   // (КЛЮЧ ОСНОВНОГО ОБЪЕКТА НА ГЛАВНОЙ КАРТЕ)

#define SEMRSCNAME          32809  // ИМЯ ФАЙЛА КЛАССИФИКАТОРА RSC 
#define SEMLAYERSHORTNAME   32810  // КОРОТКОЕ ИМЯ СЛОЯ ОБЪЕКТА
#define SEMOBJECTSHORTNAME  32811  // КОРОТКОЕ ИМЯ ОБЪЕКТА (КЛЮЧ)
#define SEMOBJECTIDENT      32812  // ИДЕНТИФИКАТОР ОБЪЕКТА  
#define SEMOBJECTBASESCALE  32814  // БАЗОВЫЙ МАСШТАБ ИСХОДНОГО УСЛОВНОГО ЗНАКА ОБЪЕКТА  

// СЛУЖЕБНЫЕ СЕМАНТИКИ НЕ РЕДАКТИРУЮТСЯ !
#define SEMHIDEFIRST        32800  // НАЧАЛО ДИАПАЗОНА СЛУЖЕБНЫХ СЕМАНТИК
#define SEMHIDELAST         32800  // КОНЕЦ  ДИАПАЗОНА СЛУЖЕБНЫХ СЕМАНТИК

#define SEMLONWGS84         32201  // ГЕОДЕЗИЧЕСКАЯ КООРДИНАТА ПО ДОЛГОТЕ В ГРАДУСАХ НА WGS-84
#define SEMLATWGS84         32202  // ГЕОДЕЗИЧЕСКАЯ КООРДИНАТА ПО ШИРОТЕ В ГРАДУСАХ НА WGS-84

#define SEMOBJECTDATE       32850  // ДАТА СОЗДАНИЯ ОБЪЕКТА
#define SEMOBJECTTIME       32851  // ВРЕМЯ СОЗДАНИЯ ОБЪЕКТА
#define SEMOBJECTAUTHOR     32852  // ИМЯ ОПЕРАТОРА
#define SEMOBJECTREDATE     32853  // ДАТА ОБНОВЛЕНИЯ ОБЪЕКТА
#define SEMOBJECTRETIME     32854  // ВРЕМЯ ОБНОВЛЕНИЯ ОБЪЕКТА
#define SEMOBJECTREAUTHOR   32855  // ИМЯ ОПЕРАТОРА ОБНОВЛЕНИЯ

// DATUM PARAMETERS
#define SEMDATUMDX          32871  // СДВИГ ПО X
#define SEMDATUMDY          32872  // СДВИГ ПО Y
#define SEMDATUMDZ          32873  // СДВИГ ПО Z
#define SEMDATUMRX          32874  // ПОВОРОТ ПО X
#define SEMDATUMRY          32875  // ПОВОРОТ ПО Y
#define SEMDATUMRZ          32876  // ПОВОРОТ ПО Z
#define SEMDATUMM           32877  // МАСШТАБНЫЙ ЭЛЕМЕНТ
#define SEMDATUMTYPE        32878  // ТИП ПРЕОБРАЗОВАНИЯ (0, 3, 7, 14)
#define SEMSCALEFACTOR      32879  // МАСШТАБ НА ОСЕВОМ МЕРИДИАНЕ             // 13/05/15

// USER ELLIPSOID PARAMETERS
#define SEMUSELLAXIS        32880  // SEMIMAJORAXIS
#define SEMUSELLINFL        32881  // INVERSEFLATTENING

#define SEMCRSIDENT         32882  // CRS Ident (String)
#define SEMSHEETNAME        32883  // ИМЯ ЛИСТА КАРТЫ
#define SEMNOMENCLATURE     32884  // НОМЕНКЛАТУРА ЛИСТА КАРТЫ  // 16/10/14
#define SEMMAINNAME         32885  // НАЗВАНИЕ РАЙОНА           // 16/10/14
#define SEMDATAIDENT        32886  // GUID НАБОРА ДАННЫХ        // 11/01/16

// ЗАРЕЗЕРВИРОВАННЫЕ КОДЫ СЕМАНТИЧЕСКИХ ХАРАКТЕРИСТИК,
// ДЛЯ ГРАФОВ                                               
// ---------------------------------------------------

#define SEMNETRIB           32812  // Номер объекта-ребра (для узла)
#define SEMNETKNOTFIRST     32813  // Номер объекта-узла первой точки (для ребра)
#define SEMNETKNOTLAST      32814  // Номер объекта-узла последней точки (для ребра)
#define SEMNETNUMBER        32815  // Номер сети
#define SEMNETTOOBJECT      32816  // Ссылка на объект листа
#define SEMNETVDIRECT       32817  // Скорость прямая (для ребра)
#define SEMNETVUNDIRECT     32818  // Скорость обратная (для ребра)
#define SEMNETCOSTRIB       32819  // Стоимость ребра
#define SEMNETRANKRIB       32820  // Ранг ребра
#define SEMNETBAN           32821  // Номер ребра, на который запрещен переход (если ребра соединены в одном узле)
#define SEMNETTURN          32822  // Признак запрета разворота (1-в начале ребра,2-в конце,3-в начале и в конце)
#define SEMSELFID           32823  // Собственный идентификатор оъекта GDF
#define SEMNETCOSTRIBBACK   32824  // Стоимость ребра обратная
#define SEMNETRANKRIBBACK   32825  // Ранг ребра обратный
#define SEMNETBANBEGIN      32826  // Номер ребра, на который запрещен переход из начала ребра
#define SEMNETBANEND        32827  // Номер ребра, на который запрещен переход из конца ребра
#define SEMNETINLIST        32828  // Ссылка на рамку исходной карты
#define SEMNETNOMECLATURE   32829  // Номенклатура листа

#define SEMSERVICEFIRST     32800  // НАЧАЛО ДИАПАЗОНА СПЕЦИАЛЬНЫХ СЕМАНТИК
#define SEMSERVICELAST      33000  // КОНЕЦ  ДИАПАЗОНА СПЕЦИАЛЬНЫХ СЕМАНТИК

#define SHEETFRAMEINCODE    15l       // ВНУТРЕННИЙ КОД РАМКИ ЛИСТА 
#define SHEETFRAMEEXCODE    91000000l // ВНЕШНИЙ КОД РАМКИ ЛИСТА

enum SEMANTIC_FOR_OBJECT   // Использование семантики для объектов
{
  NONE_SEMANTIC     = 0,   // не используется
  POSSIBLE_SEMANTIC = 1,   // возможная
  MUST_SEMANTIC     = 2,   // обязательная
  IMAGE_SEMANTIC    = 3,   // влияет на вид отображения
  ALL_SEMANTIC      = 4    // все семантики
};


// ОПИСАНИЕ ИСПОЛЬЗОВАНИЯ СЕМАНТИКИ ДЛЯ ОБ'ЕКТОВ
typedef struct APPLYSEMANTIC
{                           // КОЛИЧЕСТВО ОБ'ЕКТОВ,ИСПОЛЬЗУЮЩИХ СЕМАНТИКУ
  long Possible;            // КАК ОБЯЗАТЕЛЬНУЮ
  long Must;                // КАК ВОЗМОЖНУЮ
  long Image;               // СЕМАНТИКА ИСПОЛЬЗУЕТСЯ ДЛЯ ИЗМЕНЕНИЯ ВИДА
}
  APPLYSEMANTIC;

// ОПИСАНИЕ ШРИФТА
typedef struct RSCFONT
{
  char    Font[32];     // НАЗВАНИЕ
  char    Name[32];     // УСЛОВНОЕ НАЗВАНИЕ
  long    CharSet ;     // КОДОВАЯ СТРАНИЦА
}
  RSCFONT;

// ОПИСАНИЕ ПОДПИСИ ОБ ЕКТА ПО ЗНАЧЕНИЮ СЕМАНТИКИ ("МЕТКА ОБ'ЕКТА")
typedef struct RSCRELATION                              
{
  unsigned long  ObjectCode   ; // ВНУТРЕННИЙ КОД ПОДПИСИ СЕМАНТИКИ
  unsigned long  SemanticCode ; // КОД ПОДПИСЫВАЕМОЙ СЕМАНТИКИ
  char           Prefix[7]    ; // ПОСТОЯННЫЙ ПРЕФИКС (ТЕКСТ) ДЛЯ ПОДПИСИ
  char           Decimal      ; // КОЛИЧЕСТВО ЗНАКОВ ПОСЛЕ ЗАПЯТОЙ
}
  RSCRELATION;

// РЕЖИМ ПОИСКА ОБЪЕКТОВ КЛАССИФИКАТОРА                 
enum SEEK_RSCOBJECT
{
   SEEK_EXCODE = 1,        // ПОИСК ПО КОДУ
   SEEK_KEY    = 2,        // ПОИСК ПО КЛЮЧУ
   SEEK_NAME   = 4,        // ПОИСК ПО ИМЕНИ
};

// Типы цветовой модели палитры
enum COLORMODEL
{
   MODEL_RGB       = 0,
   MODEL_CMYKINDEX = 1,     // отображение индексами цветов CMYK палитры
   MODEL_CMYKCOLOR = 2,     // отображение цветами CMYK палитры
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++ ВСПОМОГАТЕЛЬНЫЕ СТРУКТУРЫ +++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ОШИБОЧНАЯ ВЫСОТА
#define ERRORHEIGHT  -111111.0

// ОШИБОЧНАЯ МОЩНОСТЬ СЛОЯ
#define ERRORPOWER   -1111111.0

// ПАРАМЕТРЫ СОЗДАНИЯ МАТРИЦЫ ВЫСОТ(MTW), МАТРИЦЫ КАЧЕСТВ(MTQ), РАСТРА КАЧЕСТВ(RSW)
// Для функций mapCreateMtw, mapCreateMtq, mapBuildRsw должны быть заданы:
// StructSize, BeginX,BeginY,Width,Height,ElemSizeMeters,ElemSizeBytes,Scale.
// Для функции mapCreateMtw также должны быть заданы: Unit, ReliefType.
typedef struct BUILDMTW
{
  int StructSize;          // Размер данной структуры : sizeof (BUILDMTW)

  int NotCheckDiskFreeSpace;  // Флаг - не проверять наличие свободного места на диске  17/04/13
                              // при создании файла матрицы :
                              //   0 - проверять наличие свободного места
                              //   1 - не проверять наличие свободного места

  double   BeginX;         // Прямоугольные координаты начала
  double   BeginY;         // (юго-западного угла) матрицы в метрах

  double   Width;          // Ширина матрицы в метрах
  double   Height;         // Высота матрицы в метрах

  double   ElemSizeMeters; // Размер стороны элементарного участка
                           // в метрах на местности

  int ElemSizeBytes;       // Pазмер элемента матрицы в байтах
                           // (допустимые значения : 1,2,4,8)
                           // При создании матрицы высот MTW по векторной карте функцией
                           // mapBuildMtw значение данного поля должно быть равно 4
                           // Значение 1 соответствует типу "unsigned char" (код отсутствия данных = 0)
                           // Значение 2 соответствует типу "short int" (код отсутствия данных = -32767)
                           // Значение 4 соответствует типу "long int" (код отсутствия данных = -32767000)
                           // Значение 8 соответствует типу "double" (код отсутствия данных = -32767000)

  int Unit;                // Eдиница измерения высоты матрицы высот MTW
                           // (0 - метры, 1 - дециметры,
                           //  2 - сантиметры, 3 - миллиметры )

  int ReliefType;          // Тип матрицы высот MTW (0 - абсолютные высоты,
                           //  1 - абсолютные + относительные,
                           //  2 - относительные)

  int UserType;            // Произвольное число, связываемое
                           // с создаваемой матрицей

  int Scale;               // Знаменатель масштаба создаваемой
                           // матричной карты (при создании матрицы высот MTW
                           // по векторной карте функцией mapBuildMtw значение
                           // данного поля будет заменено на знаменатель
                           // масштаба векторной карты)

  int HeightSuper;         // Флаг занесения высоты в элемент при создании матрицы
                           // высот MTW по векторной карте функцией mapBuildMtw и при попадании
                           // в элемент более одного объекта с абсолютной высотой :
                           //  0 - заносить среднюю высоту
                           //  1 - заносить максимальную высоту
                           //  2 - заносить минимальную высоту

  int FastBuilding;        // УСТАРЕВШЕЕ ПОЛЕ
                           // Режим создания матрицы (при Method = 0) :
                           //  0 - средневзвешенная интерполяция по 16 направлениям,
                           //    без формирования локальных экстремумов
                           //  1 - средневзвешенная интерполяция по 8 направлениям,
                           //    без формирования локальных экстремумов
                           //  2 - средневзвешенная интерполяция по 16 направлениям,
                           //    с формированием локальных экстремумов
                           //  3 - средневзвешенная интерполяция по 8 направлениям,
                           //    с формированием локальных экстремумов

 int Method;          // Метод построения поверхности при создании матрицы высот MTW
                      // по векторной карте функцией mapBuildMtw :
                      //  0 - метод определяется режимом FastBuilding
                      //  1 - средневзвешенная интерполяция по 16 направлениям
                      //  2 - средневзвешенная интерполяция по 8 направлениям
                      //  3 - средневзвешенная интерполяция по 16 направлениям,
                      //      сглаживание поверхности
                      //  8 - линейная интерполяция по сетке треугольников
                      //     (в методе используются только точечные объекты)
                      //  16 - плоская поверхность с нулевой высотой
                      //       для ReliefType = 2

  int Extremum;       // Флаг формирования локальных экстремумов
                      //
                      // функцией mapBuildMtw (при Method = 1,2,3) :
                      //  0 - локальные экстремумы не формируются
                      //  1 - локальные экстремумы формируются

#ifdef _M_X64
  int Free;           // Выравнивание  // 21/07/16
#endif // _M_X64

  HOBJ Border;        // Идентификатор замкнутого объекта при создании
                      // матрицы высот MTW по векторной карте функцией mapBuildMtw.
                      // По данному объекту устанавливается рамка создаваемой матрицы
                      // (если объект расположен вне габаритов матрицы,
                      // определяемых значениями полей BeginX, BeginY, Width, Height
                      // данной структуры, то рамка не устанавливается)

  int LimitMatrixFrame;
                      // Флаг ограничения габаритов при создании матрицы MTW
                      // по векторной карте функцией mapBuildMtw при Method = 1,2,3,8
                      // (габариты матрицы определяются значениями полей
                      // BeginX, BeginY, Width, Height данной структуры) :
                      //  0 - ограничение не выполняется
                      //  1 - габариты матрицы ограничиваются
                      //      габаритами района
                      //  2 - габариты матрицы ограничиваются
                      //      габаритами области расположения
                      //      объектов с абсолютной высотой


  int NotUse3DMetric;
                      // Флаг - не использовать трехмерную метрику объектов
                      // при создании матрицы MTW по векторной карте функцией mapBuildMtw:
                      //  0 - трехмерная метрика используется
                      //  1 - трехмерная метрика не используется

  int SurfaceSquare3DObject;
                      // Флаг при создании матрицы MTW по векторной карте функцией mapBuildMtw -
                      // строить поверхность внутри площадного
                      // объекта по его трехмерной метрике :
                      //  0 - не строить поверхность
                      //  1 - строить поверхность
                      // Если параметр NotUse3DMetric равен 1,
                      // то поверхность не строится

  int AltitudeMarksNet;
                      // Флаг дополнительной обработки высотных точек при
                      // создании матрицы MTW по векторной карте функцией mapBuildMtw для
                      // средневзвешенной интерполяции (Method = 1,2,3) :
                      //  0 - для каждой высотной точки выполняется построение и
                      //      занесение в матрицу лучей влияния высоты (луч влияния
                      //      высоты - 3D-отрезок,выходящий из точки по одному из
                      //      16 радиальных направлений; высоты элементов отрезка
                      //      определяются высотой точки и высотой, найденной при
                      //      сканировании матрицы из точки по данному направлению.
                      //      Лучи влияния высоты компенсируют промахи мимо точки
                      //      с абсолютной высотой при поиске значащих высот в процессе
                      //      вычисления незаполненых элементов матрицы.
                      //  1 - по набору высотных точек создается триангуляция,
                      //      ребра триангуляции (или части ребер) заносятся в матрицу
                      //      в виде 3D-отрезков (если ребро не пересекает объекты
                      //      с абсолютной высотой, то оно выводится целиком,
                      //      если пересекает, то выводится часть ребра, ограниченная
                      //      его вершиной и точкой пересечения с объектом);
                      //  2 - дополнительная обработка высотных точек не выполняется,
                      //      высота точки заносится в один элемент матрицы,
                      //      обычно применяется при построении поверхности
                      //      по набору отметок высот

  int LimitMatrixByFramesOfSheets;
                      // Флаг ограничения матрицы рамками листов при создании
                      // матрицы MTW по векторной карте функцией mapBuildMtw (для Method = 1,2,3,8):
                      //  0 - ограничение матрицы не выполняется,
                      //      информативные элементы могут располагаться
                      //      вне рамок листов
                      //  1 - ограничение матрицы выполняется,
                      //      информативные элементы располагаются
                      //      только внутри габаритов рамок листов

  char Reserve[20];   // Должны быть нули
}
  BUILDMTW;


typedef struct MTRPROJECTIONDATA  //  ПАРАМЕТРЫ СОЗДАНИЯ МАТРИЦЫ ВЫСОТ
{                                 //   (ДАННЫЕ О ПРОЕКЦИИ)
  int  StructSize;              // Размер данной структуры : 128 байт
                                //  sizeof (MTRPROJECTIONDATA)

  int  Free;                    // Должен быть ноль

  int  MapType;                 // Тип карты (задавать в соответствии
                                //   с MAPTYPE, файл MAPCREAT.H)

  int  ProjectionType;          // Тип проекции (задавать в соответствии
                                //   с MAPPROJECTION, файл MAPCREAT.H)
                                // long  MaterialProjection; // Проекция исх. материала

                                // В радианах
  double    FirstMainParallel;  // Первая главная параллель
  double    SecondMainParallel; // Вторая главная параллель
  double    AxisMeridian;       // Осевой меридиан
  double    MainPointParallel;  // Параллель главной точки
  double    PoleLatitude;       // Широта полюса проекции          
  double    PoleLongitude;      // Долгота полюса проекции         

  int       EllipsoideKind;     // Вид эллипсоида           
  int       HeightSystem;       // Система высот            
  int       CoordinateSystem;   // Система координат        
  int       ZoneNumber;         // Номер зоны топокарты     

  double    FalseEasting;    // Смещение координат по оси Y            // 05/07/10
  double    FalseNorthing;   // Смещение координат по оси X            // 05/07/10
  double    ScaleFactor;     // Масштабный коэффициент                 // 05/07/10
  double    TurnAngle;       // Угол разворота осей для локальных систем (МСК)   // 05/07/10
  int       ZoneIdent;       // Идентификатор района (для МСК 63: A-X или 0)     // 26/04/11

  char      Reserve[12];     // Должны быть нули

}
  MTRPROJECTIONDATA;

// ПАРАМЕТРЫ ПОСТРОЕНИЯ МАТРИЦЫ ПОВЕРХНОСТИ (МАТРИЦЫ КАЧЕСТВ ИЛИ МАТРИЦЫ ВЫСОТ)

typedef struct BUILDSURFACE  // 15/06/11
{
 unsigned
     int StructSize; // Размер данной структуры : sizeof (BUILDSURFACE) = 320 байт
     int FileMtw;    // Флаг расширения создаваемого файла :
                     //   0 - создание файла матрицы качеств (*.mtq)
                     //   1 - создание файла матрицы высот (*.mtw)

 double BeginX;    // Прямоугольные координаты начала
 double BeginY;    // (юго-западного угла) матрицы в метрах

 double Width;     // Ширина матрицы в метрах
 double Height;    // Высота матрицы в метрах

 double MinValue;  // Диапазон значений характеристики качества создаваемой матрицы,
 double MaxValue;  // если MinValue >= MaxValue в матрицу заносится фактический диапазон значений

 double ElemSizeMeters;  // Размер стороны элементарного участка
                         // в метрах на местности (дискрет матрицы)

 int UserType;           // Произвольное число, связываемое с создаваемой матрицей
                         //  (тип, характеристика матрицы)

 int SearchSectorCount;  // Количество секторов для поиска соседних точек (для Method = 9,11,12) // 09/06/11
                             // 1 - по кругу
                             // 4 - по 4 секторам
                             // 8 - по 8 секторам

 char UserName[32];      // Условное имя матрицы (название характеристики качества)

 HOBJ Border;  // Идентификатор замкнутого объекта карты, ограничивающего область
               // заполняемых элементов матрицы (если равен нулю, то не используется)
               // Если объект расположен вне габаритов матрицы, определяемых значениями
               // полей BeginX, BeginY, Width, Height данной структуры, то не используется

 HWND Handle;  // Идентификатор окна диалога, которому посылается
               // сообщение 0x0581 о проценте выполненных работ (в WPARAM),
               // если процесс должен быть принудительно завершен, в ответ
               // должно вернуться значение 0x0581.
               // Если Handle равно нулю - сообщения не посылаются.

 COLORREF* Palette;      // Адрес палитры создаваемой матрицы качеств(*.mtq),
                         // если равен нулю - используется палитра по умолчанию
 int PaletteCount;       // Kоличество цветов в палитре (от 1 до 256)

 int Method;             // Метод построения поверхности :
                         //  8 - Линейная интерполяция по сетке треугольников
                         //      ( используются точки массива PointArray )
                         //  9 - Логарифмическая интерполяция по локальной поверхности
                         //       вокруг определяемого элемента
                         //      ( используются точки метрики объектов карты )
                         // 10 - Линейная интерполяция по сетке треугольников
                         //      ( используются точки метрики объектов карты )
                         // 11 - Кригинг
                         // 12 - Кокригинг // 19/11/12

 XYHDOUBLE* PointArray;  // Адрес массива значений характеристики качества (для Method = 8)
 int PointCount;    // Число точек в массиве PointArray (для Method = 8)

 int SemanticCode;  // Код семантики моделируемой характеристики
                         // если = 0, то не используется 

 int LocalSurfacePointCount;  // Kоличество точек для построения локальной поверхности
                              // вокруг определяемого элемента (для Method = 9,11,12)
                                   // если SearchSectorCount = 4, то LocalSurfacePointCount должно быть кратно 4
                                   // если SearchSectorCount = 8, то LocalSurfacePointCount должно быть кратно 8

 int LocalSurfaceRebuildPointCount;  // Kоличество обновленных точек при переходе
                                          // к очередному определяемому элементу,
                                          // при котором перестраивается локальная
                                          // поверхность (для Method = 9)
 double MaxMetricCutLength;  // Шаг добавления точек по контуру линейных и площадных объектов (для Method = 9,11,12)
                             // Используется если IsMetricCutLength = 1

 int Use3DMetric;            // Флаг использования высот из трехмерной метрики объектов :  // 06/11/07
                             //  0 - высоты из трехмерной метрики не используются
                             //  1 - высоты из трехмерной метрики используются
                             

 int SemanticCode2;          // Код семантики дополнительной характеристики для кокригинга (для Method = 12) // 13/02/13

 int FillBorderType;         // Тип заполнения матрицы внутри ограничивающего объекта Border
                             // 0 - заполняется весь объект
                             // 1 - заполняется объект без подобъектов
                             // 2 - заполняются объект и подобъекты
                             // 3 - заполняются только подобъекты

 int IsMetricCutLength; // Добавлять точки по контуру линейных и площадных объектов (для Method = 9,11,12) // 09/06/11
                             // Точки добавляются с шагом MaxMetricCutLength

 int IsAddPointsInEmptyRegion; // Добавлять точки в пустых областях (для Method = 9,11,12) // 09/06/11

 int IsLimitHeight;      // Если включено, то высота элемента матрицы не может быть больше или меньше высот (для Method = 9,11,12) // 09/06/11
                         // соседних точек, по которым строится поверхность, на LimitOffset

 double   DistBeforePointsInEmptyRegion; // Расстояние между точками в пустых областях (для Method = 9,11,12) // 09/06/11
                                         // Значение высоты (характеристики) в этих
                                         // точках определяется методом интерполяции по 8 направлениям.
                                         // Эти точки добавляются для того, чтобы в пустых областях избавится от появления
                                         // гор и ям в случаях когда на границе пустых областей есть значительные градиенты.
 double   LimitOffset; // Если включено IsLimitHeight то разрешенное отклонение вычисленного  (для Method = 9,11,12) // 09/06/11
                       // значения матрицы от минимального, максимального значения среди соседних точек

 char Reserve[128];    // Должны быть нули
}
  BUILDSURFACE;


// ПАРАМЕТРЫ ПОСТРОЕНИЯ ПРОФИЛЯ    // 10/02/10

typedef struct PROFBUILDPARM
{
  DOUBLEPOINT Point;               // Координаты текущей точки на объекте
  HOBJ        Object;              // Объект для построения профиля
  long int    ProfStepVertical;    // Шаг шкалы по вертикали (м)
  long int    ProfStepHorizontal;  // Шаг шкалы по горизонтали (м)

} 
  PROFBUILDPARM;

  
// ПАРАМЕТРЫ ПОСТРОЕНИЯ ПРОФИЛЯ    // 23/04/10

typedef struct PROFBUILDPARMEX
{
  DOUBLEPOINT Point;               // Координаты текущей точки на объекте
  HOBJ        Object;              // Объект для построения профиля
  long int    ProfStepVertical;    // Шаг шкалы по вертикали (м)
  long int    ProfStepHorizontal;  // Шаг шкалы по горизонтали (м)
  double      DeltaRight;          // Превышение в конечной точке
  double      DeltaCurrent;        // Превышение в текущей точке
  double      DeltaLeft;           // Превышение в начальной точке
  long        ColorProf;           // Цвет профиля
  long        ColorLine;           // Цвет линии
  long int    IsCurvatureEarth;    // Флаг учета кривизны  Земли  0 - не учитывать,
                                   //                             1 - учитывать;
  long int    IsMiddleHeight;      // Флаг учета усреднения высот 0 - не учитывать,
                                   //                             1 - учитывать;
  long int    IsLineFL;            // Флаг отображения линии (первая - последняя точка)
                                   // 0 - отображать,   1 - не отображать;
  long int    IsLineFCL;           // Флаг отображения линии (первая - текущая- последняя точка)
                                   // 0 - отображать,   1 - не отображать;
  long int    IsLineCross;         // Флаг отображения линии (перекрестье от текущей точки)
                                   // 0 - отображать,   1 - не отображать;
  long int    IsLineNet;           // Флаг отображения сетки профиля
                                   // 0 - отображать,   1 - не отображать;
  long int    IsLineRelief;        // Флаг отображения рельефа
                                   // 0 - заливка,   1 - контур;
}
  PROFBUILDPARMEX;


// СОКРАЩЕННОЕ ОПИСАНИЕ НАЙДЕННОГО ОБ'ЕКТА  // 01/02/02

typedef struct
{
   int         ListNumber;        // НОМЕР ЛИСТА ВЫБРАННОГО ОБ'ЕКТА
   int         ObjectNumber;      // НОМЕР ВЫБРАННОГО ОБ'ЕКТА В ЛИСТЕ
   HSITE       Map;               // ОПИСАНИЕ ЭЛЕКТРОННОЙ КАРТЫ
}
  MAPOBJDESCEX;

typedef struct
{
  DOUBLEPOINT XY;     // Координата точки пересечения
  double       H;     // Высота точки пересечения
                      // (из метрики)
  HOBJ Info1;
  long int Number1;   // За какой точкой объекта Info1
                      // находится точка пересечения
  long int Subject1;  // Номер объекта/подобъекта объекта info2
  HOBJ Info2;
  long int Number2;   // За какой точкой объекта Info2
                      // находится точка пересечения
  long int Subject2;  // Номер объекта/подобъекта объекта info2
}
  CROSSPOINT;


// СТРУКТУРА ОПИСЫВАЮЩАЯ ПАНЕЛЬ ПРИКЛАДНОЙ ЗАДАЧИ
// Указатель на нее используется в качестве второго
// параметра в сообщении AW_CREATETASKPANEL

typedef struct PANELINFO
{
  HINSTANCE hInst;     // Идентификатор библиотеки содержащей кнопки панели
  long int  Count;     // Количество передаваемых идентификаторов(кнопок) в
                       // первом параметре сообщения AW_CREATETASKPANEL
  char      Name[32];  // Собственное название панели
}
  PANELINFO;

typedef struct PANELPOS   // Параметр LPARAM сообщения AW_MOVEPANEL
{
  RECT      Rect;         // Координаты нового положения панели
  long int  Position;     // Расположение панели в окне (см. LOCAL)
  long int  Layout;       // Признак немедленного обновления окна
}
  PANELPOS;

typedef struct BUTTONINFO   // Параметр LPARAM сообщения AW_INSERTBUTTON
{
  long int Command;         // Идентификатор создаваемой кнопки,
                            // если Command = (-1) - вставляется разделитель
  HBITMAP  hBitmap;         // Идентификатор картинки на кнопке
                            // Для Linux - указатель на структуру BUTTONINFOLX
  long int Sibling;         // Идентификатор кнопки в панели инструментов,
                            // после которой вставляется создаваемая кнопка.
                            // Если Sibling = NULL - вставляется в конец
}
  BUTTONINFO;

  
typedef struct TASKBUTTONINFO // Параметр LPARAM сообщения AW_INSERTTASKBUTTON     // 05/06/10
{
  HINSTANCE hTask;          // Идентификатор библиотеки задачи, которая будет выполнять команду или 0
                            // Если задан идентификатор задачи, то Command будет
                            // передаваться на обработку конкретной задаче
  long int Command;         // Идентификатор кнопки,
                            // если Command = (-1) - вставляется разделитель
  long int Sibling;         // Идентификатор кнопки в панели инструментов,
                            // после которой вставляется создаваемая кнопка.
                            // Если Sibling равен 0, то при создании кнопка вставляется в конец списка
  HBITMAP  hBitmap;         // Идентификатор картинки на кнопке или 0
  long int Background;      // Признак прорисовки фона кнопки (если не равно 0),
                            // учитывается если hBitmap не ноль
  long int State;           // Состояние кнопки (0 - отжата, 1 - нажата, -1 - не меняется)
  long int Enable;          // Доступность кнопки (0 - недоступна для нажатия, 1 - может нажиматься)

  WCHAR    Hint[256];       // Короткая подсказка к кнопке в UNICODE или 0

  WCHAR    Comment[256];    // Длинная подсказка к кнопке в UNICODE или 0
}
  TASKBUTTONINFO;

#ifdef LINUXAPI 
typedef struct BUTTONINFOLX // Параметр hBitmap структуры BUTTONINFO
{
  HBITMAP  hBitmap;         // Идентификатор картинки на кнопке

  char     Hint[80];        // Подсказка
}
  BUTTONINFOLX;
#endif


typedef struct CONTRINFO  // Параметр LPARAM сообщения AW_INSERTCONTROL
{
  long int CtrlId;          // Идентификатор создаваемого элемента управления,
                            // если CtrlId = (-1) - вставляется разделитель
  HWND     hWindow;         // Идентификатор окна элемента управления

  long int Sibling;         // Идентификатор элемента управления в панели
                            // инструментов, после которого вставляется
                            // создаваемый элемент управления.
                            // Если Sibling = NULL - добавляется в конец
}
  CONTRINFO;

typedef struct HITCONTROLINFO // Параметр LPARAM событии MT_PANELRBUTTONUP
{
  POINT    Position;          // Положение курсора на экране

  long int CtrlID;            // Идентификатор элемента управления панели,
                              // над которым была нажата клавиша
}
  HITCONTROLINFO;

enum DBFFIELDTYPE         // ТИПЫ ПОЛЕЙ В ФАЙЛЕ DBF
{
  DBFCHAR     = 'C',      // Char (1 - 254)
  DBFDATE     = 'D',      // Date (10 символов : YYYYMMDD)
  DBFFLOAT    = 'F',      // Float  (1-20)
  DBFFIXED    = 'N',      // Numeric (1-20)
  DBFBOOL     = 'L',      // Logical (Yy, Tt, Nn, Ff, , ?)
  DBFMEMO     = 'M',      // Memo (10 цифр - ссылка в DBT или пробелы)
  DBFVARIABLE = 'V',      // Variable (4+4+1+1)
  DBFPICTURE  = 'P',      // Picture (10 цифр - ссылка в FTP или пробелы)
  DBFBINARY   = 'B',      // Binary (10 цифр - ссылка в DPT или пробелы)
  DBFGENERAL  = 'G',      // OLE-объекты (10 цифр - ссылка в DPT или пробелы)
  DBFDOUBLE   = 'O',      // Double
};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++ ВЕДЕНИЕ ПРОТОКОЛА ТРАНЗАКЦИЙ +++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef enum READLOG   // УСЛОВИЯ СЧИТЫВАНИЯ ТРАНЗАКЦИЙ
{
  READ_ANYACTION = 0,  // СЧИТАТЬ ПОСЛЕДНЮЮ ТРАНЗАКЦИЮ
  LOG_ANYACTION  = 0,  // ОБРАБОТАТЬ ПОСЛЕДНЮЮ ТРАНЗАКЦИЮ
  READ_MYACTION  = 1,  // СЧИТАТЬ СВОЮ ПОСЛЕДНЮЮ ТРАНЗАКЦИЮ
                       // (ПО ИМЕНИ ПОЛЬЗОВАТЕЛЯ)
  LOG_MYACTION   = 1,  // ОБРАБОТАТЬ СВОЮ ТРАНЗАКЦИЮ
}
 READLOG;

// ЗАРЕЗЕРВИРОВАННЫЕ ТИПЫ ТРАНЗАКЦИЙ
enum ACTION_TYPE
{
  AT_OPEN     = 60000,     // Журнал открыт
  AT_CLOSE    = 60001,     // Журнал закрыт
  AT_LOGON    = 60002,     // Запись в журнал включена
  AT_LOGOFF   = 60003,     // Запись в журнал отключена
  AT_CLEAR    = 60004,     // Файлы отката очищены (сортировка,...)
  AT_UNDO     = 60005,     // Отмена какой-либо предыдущей транзакции
  AT_NEXT     = 60006,     // Продолжение предыдущей транзакции
  AT_REDO     = 60007,     // Восстановление какой-либо предыдущей транзакции
  AT_SORT     = 60008,     // Выполнена сортировка карты
  AT_RSCSAVE  = 60009,     // Выполнено обновление классификатора
  AT_ERROR    = 60010,     // Число задач не соотвествует числу подключений, был сбой
  AT_FIRST    = AT_OPEN,   // Первый зарезервированный код
  AT_LAST     = AT_ERROR,  // Последний зарезервированный код
};

// Если Type == AT_OPEN, после заголовка идет
// описание источника транзакций, строка 16 байт
// (имя компьютера в сети).

typedef struct ACTIONHEAD  // ЗАГОЛОВОК ОПИСАНИЯ ТРАНЗАКЦИИ
{
  short int      Ident;    // 0x7FFF
  unsigned short Task;     // Идентификатор задачи,выполнившей транзакцию
  unsigned short Count;    // Число операций (Length = 16+Count*16)
  unsigned short Type;     // Тип транзакции (код задачи)
  unsigned int   Date;     // Дата выполнения транзакции
  unsigned int   Time;     // Время выполнения транзакции
}
  ACTIONHEAD;

// ТИПЫ ОПЕРАЦИЙ НАД ОБ'ЕКТОМ
enum OBJECT_OPERATION
{
   OO_APPEND    = 1,        // СОЗДАНИЕ ОБ' ЕКТА
   OO_DELETE    = 2,        // УДАЛЕНИЕ ОБ'ЕКТА
   OO_UPDATE    = 4,        // ОБНОВЛЕНИЕ ОБ'ЕКТА
   OO_UNDO      = 8,        // ВОССТАНОВЛЕНИЕ ОБ'ЕКТА
};

// МАСКА ОБРАБОТАННЫХ ЭЛЕМЕНТОВ ОБ'ЕКТА
enum ACTION_MASK
{
   AM_HEAD      = 1,        // ИЗМЕНЕНЫ СПРАВОЧНЫЕ ДАННЫЕ
   AM_METRIC    = 2,        // ИЗМЕНЕНА МЕТРИКА ОБ'ЕКТА
   AM_SEMANTIC  = 4,        // ИЗМЕНЕНА СЕМАНТИКА
   AM_DRAW      = 8,        // ИЗМЕНЕНО ГРАФИЧЕСКОЕ ОПИСАНИЕ
   AM_HEADREF   = 16,       // СПРАВОЧНЫЕ ДАННЫЕ ИЗМЕНЕНЫ ТОЛЬКО В ЧАСТИ ЦЕПОЧКИ ССЫЛОК
};

typedef struct ACTIONRECORD // ОПИСАНИЕ ОТДЕЛЬНОЙ ОПЕРАЦИИ
{                           // (Для AT_OPEN - это строка 16 байт)
  int      Key;             // Уникальный номер объекта в листе                // 09/04/13
  int      Number;          // Последовательный номер в момент обработки
  int      Back;            // Номер записи в файле отката (служебный)
  unsigned char Type;       // Тип операции
  unsigned char Mask;       // Маска обработанных элементов объекта
  short         List;       // Номер листа
}
 ACTIONRECORD;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++ ОПОВЕЩЕНИЕ О ИЗМЕНЕНИИ ОБ ЕКТА КАРТЫ  ++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32API
typedef long int (WINAPI * CHANGECALL)(void * parm, void * value);
#else
typedef long int (*CHANGECALL)(void * parm, void * value);
#endif

typedef struct CHANGEINFO
{
  HSITE    hSite;     // Идентификатор карты в документе
  short int List;     // Номер листа карты
  unsigned char Type; // Тип операции (OBJECT_OPERATION)
  unsigned char Mask; // Маска обработанных элементов объекта (ACTION_MASK)
  unsigned int  Key;    // Уникальный номер объекта в листе 
                        // (или -1, если обновлено большое число объектов)  // 12/12/13
  unsigned int  Object; // Последовательный номер объекта в листе
                        // (или -1, если обновлено большое число объектов)
}
  CHANGEINFO;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++ ПОДДЕРЖКА ОТОБРАЖАЕМЫХ В ПАМЯТЬ ФАЙЛОВ +++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct FILEMAPPING  // ПАРАМЕТРЫ ОТОБРАЖЕНИЯ ФАЙЛА В ПАМЯТИ
{
  FILEMAPPING() { hFile = 0; hMapping = 0; Address = 0; }

  HANDLE   hFile;           // Идентификатор открытого файла
  HANDLE   hMapping;        // Идентификатор отображения (в UNIX = 0)
  char *   Address;         // Адрес области памяти для отображения данных
  unsigned long int Offset; // Смещение в файле на первый байт данных
  unsigned long int Size;   // Размер отображаемой области в файле
  unsigned long int Shift;  // Поправка на первый байт в области памяти
                            // для учета кратности размеру страницы (64 Кб)
}
  FILEMAPPING;

// Флаг проверки входимости объекта, который режут, в объект - РЕЗАК
// (см. mapCreateObjectsCross(...) в seekapi.h)
#define FLAGINSIDEOBJECTS  32

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++ НАНЕСЕНИЕ ОБ ЕКТА НА КАРТУ  +++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct PUTTOMAPINFO
{
  long int Incode;     // Внутренний код объекта
  long int MapNumber;  // Номер карты
  long int Regime;     // Способ нанесения объекта

}
  PUTTOMAPINFO;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++ ИНФОРМАЦИЯ О ТОЧКЕ ПЕРЕСЕЧЕНИЯ ++++++++++++
// +++++++++++++  ОТРЕЗКА С МЕТРИКОЙ ПОДОБЪЕКТА ++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct NUMBERPOINT
{
  DOUBLEPOINT Point;   // Координаты точки
  long int Number;     // Номер точки метрики, за которой следует point
  long int Update;     // Предложение замены точки метрики
  long int Equal;      // Номер точки совпадения в метрике
  long int Reserve;
}
  NUMBERPOINT;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    ПАРАМЕТРЫ ОБРАБОТКИ КОНТУРОВ ОБЪЕКТОВ, ИСПОЛЬЗУЕМЫЕ
//        ПРИ СОХРАНЕНИИ ОБЪЕКТОВ С ДЕЛЕНИЕМ НА ЛИСТЫ
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct _DLLCLASS COMMITOBJECTPARM                         // 11/04/11
{
  COMMITOBJECTPARM () { memset(this, 0, sizeof(*this)); }

  double MinCutLength;          // Минимальная длина отрезка метрики объекта
                                // (в метрах на местности)

  // Используется для автоматической фильтрации точек метрики при сохранении
  // линейных и площадных объектов с делением по листам в функциях:
  // mapCommitWithPlace(), mapCommitWithPlaceAsNew().
  // При обнаружении отрезка, длина которого менее MinLineLength,
  // удаляется его вторая точка.

  double MinCutLengthOnBorder;  // Минимальная длина отрезка метрики объекта,
                                // выходящего на рамку (в метрах на местности)

  // Используется для автоматического удаления коротких линейных
  // и вырожденных площадных объектов, формируемых при обрезке
  // объектов по рамке в функциях:
  // mapCommitWithPlace(), mapCommitWithPlaceAsNew().
  // Под вырожденными площадными объектами понимаются объекты:
  // 1) треугольник, вытянутый вдоль рамки листа карты,
  //    длина одной из сторон которого менее MinCutLengthOnBorder;
  // 2) четырехугольник, вытянутый вдоль рамки листа карты,
  //    длины двух отрезков которого менее MinCutLengthOnBorder.

  char Reserve[48];             // Резерв
}
  COMMITOBJECTPARM;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Файл с набором записей фиксированной длины - 3D-вектора
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct VECDATA     // ВЕКТОР ПРИВЯЗКИ 3D-модели
{                          //
  int         Code;        // Код библиотеки (для уникальных изображений, иначе 0)
  int         Key;         // Ключ изображения (для уникальных изображений, иначе 0)
  XYHDOUBLE   Offset;      // Смещение формы модели относительно первой точки объекта
  double      Angle;       // Угол поворота оси X модели относительно оси X карты
}
  VECDATA;

typedef struct VECRECORD   // Запись вектора привязки (запись файла VEC)
{                          //
  int         Number;      // Номер объекта
  int         BaseKey;     // Ключ объекта
  VECDATA     Data;        // Вектор привязки
}
  VECRECORD;

// Первая точка вектора на объекте вычитаем из нее координаты первой точки объекта.
// Из полученного вычитаем координаты первой точки вектора модели. Полученное значение = Offset.
// Angle - Угол поворота оси X модели относительно оси X карты.

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++ Параметры КАЛЬКУЛЯТОРА ++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct SAVEMAPCOMPPARM
{
 char Titlsave[40];          // Название режима
 double MemoryRegister;      // Регистр
 double MemoryBuffer;        // Буфер памяти
 double Buffer;              // Буфер вычисленный
 double Memory;              // Память
 double Value;               // Передаваемое значение
 long int Flagcomp;          // Признак действия 1-(+),2-(-),3-(*),4-(/),
                             //                  0 - действия не было
 long int Index;             // Индекс для единиц измерения длины
 long int Indexold;          // Предыдущий индекс
 long int IndexSq;           // Индекс для единиц измерения площади
 long int Flagshow;          // Видимость диалога
 long int Regnow;            // Передаваемый режим
 long int Regold;            // Режим предыдущий-лин. или площ.
 long int Flagreg;           // Текущий режим: 0- длина, 1- площадь
 long int StopFlag;          // Флаг закрытия диалога
 long int Precision;         // Точность вычислений

} SAVEMAPCOMPPARM;


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// // Параметры загрузки Ole-объекта
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef struct OLELOADPARM
{
 double   PosX;                 // координата x в метрах на местности
 double   PosY;                 // координата y в метрах на местности 

 int      Height;               // высота Ole-объекта в мм на карте в единичном масштабе
 int      Width;                // ширина Ole-объекта в мм на карте в единичном масштабе

 char     PosRight;             // 0 - левый угол, 1 - правый угол
 char     PosBottom;            // 0 - верхний угол, 1 - нижний угол
 char     Reserve[50];          // выравнивание на 8

 char     Path[260];            // полный путь к данным

} 
  OLELOADPARM;


struct SETVIEWSCALE    // Параметры сообщения для масштабирования OLE-документов  
{
  POINT     center;    // Новые координаты центра
  int       percent;   // Процент увеличения изображения
  int       zero;      // Выравнивание
};


// Вегетационные индексы  // 31/08/11
#define	VIFIRST 1
#define	NDVI    1
#define	RVI     2
#define	IPVI    3
#define	DVI     4
#define	SAVI    5
#define	MSAVI2  6
#define	VILAST  6

// Данные, необходимые для отображения вегетационного индекса // 31/08/11
typedef struct VEGINDEX
{
   int      Index;         // используемый индекс вегетации (NDVI, RVI и т.д.)
   int      ColorCount;    // количество цветов в палитре отрисовки вегетационного индекса
   RGBQUAD  Palette[256];  // палитра отрисовки вегетационного индекса
   int      RedBandNum;    // номер красного канала
   int      NirBandNum;    // номер инфракрасного канала
   double   L;             // коэффициент для SAVI (0..1)
   double   Reserv[32];
}
  VEGINDEX;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Параметры создания TIFF растров 18/09/13
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Тип пикселя (каким числом кодируется цвет пикселя на уровне программного интерфейса,
// реальный размер пикселя в файле задается параметром CREATETIFPARM.BitInBand)
#define PT_MIN       0
#define PT_UNKNOWN   0
#define PT_BYTE      1    // 8-ми разрядное целое число без знака
#define PT_UINT16    2    // 16-ти разрядное целое число без знака
#define PT_INT16     3    // 16-ти разрядное целое число со знаком
#define PT_UINT32    4    // 32-х разрядное целое число без знака
#define PT_INT32     5    // 32-х разрядное целое число со знаком
#define PT_FLOAT32   6    // 32-х разрядное число c плавающей запятой
#define PT_FLOAT64   7    // 64-х разрядное число с плавающей запятой
#define PT_CINT16    8    // 16-ти разрядное комплексное число
#define PT_CINT32    9    // 32-х разрядное комплексное число
#define PT_CFLOAT32 10    // 32-х разрядное комплексное число c плавающей запятой
#define PT_CFLOAT64 11    // 64-х разрядное комплексное число с плавающей запятой
#define PT_MAX      11

// Интерпретация каналов для 3 (CMYK - 4) канальных растров
#define CS_RGB       2
#define CS_CMYK      5 
#define CS_YCBCR     6
#define CS_CIELAB    8
#define CS_ICCLAB    9
#define CS_ITULAB   10

// Условия создания BIGTIFF
#define BT_NO        0  // не создавать
#define BT_YES       1  // создавать
#define BT_IFNEEDED  2  // создавать, если размер больше 4 ГБ и нет сжатия
#define BT_IFSAFER   3  // создавать, если размер больше 2 ГБ и сжатие не проверяется

// Типы компрессии в TIF файлах
#define NOTCOMPRESS     1
#define LZW             5
#define CCITT           2
#define CCITTFAX3       3
#define CCITTFAX4       4
#define JPEG            6
#define DEFLATE         8
#define PACKBITS        32773

// Параметры афинного трансформирования      
typedef struct AFFINCOEF
{
  double A0, A1, A2, B0, B1, B2;
}
  AFFINCOEF;

// Параметры создавемого TIF файла
typedef struct CREATETIFPARM
{
  int     BandCount;    // количество каналов
  int     BitInBand;    // бит на канал
  int     PixelType;    // тип пикселя (PT_BYTE и т.д.)
  int     Compress;     // тип компрессии (см. NOTCOMPRESS и т.д.)
                        // JPEG - возможен только для 3 и 1 канальных 8, 16 битных растров
                        //      - для 3 канальных допустим ColorSpace = CS_RGB или CS_YCBCR (лучшее сжатие, но медленнее)
                        //      - при CS_YCBCR в буфер тайла записываются RGB
  int     IsPredictor;  // для LZW и DEFLATE компрессии - признак сжатия расхождений
                        // 0 - сжимаются сами цвета
                        // 1 - сжимаются расхождения между соседними пикселями в строке
  int     JPEGQuality;  // качество JPEG сжатия (1-100) (по умолчанию 75)
  int     ColorSpace;   // интерпретация каналов для 3 канальных растров (CMYK для 4 канальных) см. CS_RGB и т.д.
  int     DeflateLevel; // уровень DEFLATE сжатия (1-9) (по умолчанию 6)
  int     TileWidth;    // ширина тайла в пикселях
  int     TileHeight;   // высота тайла в пикселях
  int     BigTIFF;      // условия создания BIGTIFF (см. BT_NO и т.д.)
  int     IsPlane;      // признак хранения изображения по плоскостям (0 - RGB RGB ..., 1 - RR...BB...GG...)
                        // при JPEG сжатии с цветовой схемой YCBCR должно быть = 0
  int     IsAlpha;      // признак использования альфа канала
  RGBQUAD Palette[256]; // палитра 1,4,8 битных растров
  int     Reserved[64];
}
  CREATETIFPARM;

#pragma pack(pop)

#endif  // MAPTYPE_H

