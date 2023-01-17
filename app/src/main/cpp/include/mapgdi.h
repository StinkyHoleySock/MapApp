/****** MAPGDI.H *************** Kruzhkov A.E.   ****** 27/05/16 ***
******* MAPGDI.H *************** Belenkov O.V.   ****** 28/05/15 ***
******* MAPGDI.H *************** Panteleeva N.A. ****** 26/05/16 ***
******* MAPGDI.H *************** Kruzhkova E.A.  ****** 19/03/08 ***
******* MAPGDI.H *************** Dishlenko S.G.  ****** 26/03/12 ***
******* MAPGDI.H *************** Derevyagina Zh.A.***** 12/01/16 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*      ОПИСАНИЕ ПАРАМЕТРОВ ФУНКЦИЙ ВИЗУАЛИЗАЦИИ ПРОИЗВОЛЬНЫХ       *
*             ГРАФИЧЕСКИХ ОБ'ЕКТОВ ЭЛЕКТРОННОЙ КАРТЫ               *
*                                                                  *
*******************************************************************/

#ifndef MAPGDI_H
#define MAPGDI_H

#ifndef MAPTYPE_H
 #include "maptype.h"
#endif

#pragma pack(push,1)

// Опции масштабируемости объекта (игнорируются при Image равном IMG_OBJECT)

#define IM_PRESS       0x00000000  // Сжимать при уменьшении карты (по умолчанию)
#define IM_DONTPRESS   0x04000000  // Не сжимать при уменьшении

#define IM_SCALE       0x01000000  // Масштабировать
#define IM_DONTSCALE   0x02000000  // Не масштабировать

#define IM_SCALEDONTPRESS  (IM_SCALE | IM_DONTPRESS)  // Только увеличивать

#define IM_CLEARSCALE 0x0F8FFFFFF  // Убрать флаги масштабирования

//-----------------------------------------------------------------
// ОПИСАНИЕ СПОСОБА ИЗОБРАЖЕНИЯ ГРАФИЧЕСКОГО ОБ'ЕКТА
//-----------------------------------------------------------------

#if defined(_M_X64) || defined(BUILD_DLL64)       // 28/07/09

typedef struct PAINTPARM
{
  int Image;       // Вид функции отображения
  int Mode ;       // Условия отображения и масштабирования
                   //   R2_COPYPEN   - отображение цветом
                   //   R2_NOT       - отображение инверсией фона (цвет игнорируется)
                   //   IM_SCALE, IM_DONTSCALE, ... - признаки масштабируемости
                   //    (игнорируются при Image равном IMG_OBJECT)
                   // Пример использования: Mode = R2_NOT | IM_SCALE
  char *   Parm ;  // Параметры отображения для соответствующей функции.
                   // Игнорируется при Image равном IMG_OBJECT (параметры
                   // и номер функции устанавливается из классификатора)
}
  PAINTPARM;

#else

typedef struct PAINTPARM
{
  int Image;       // Вид функции отображения
  char *   Parm ;  // Параметры отображения для соответствующей функции.
                   // Игнорируется при Image равном IMG_OBJECT (параметры
                   // и номер функции устанавливается из классификатора)
  int Mode ;       // Условия отображения и масштабирования
                   //   R2_COPYPEN   - отображение цветом
                   //   R2_NOT       - отображение инверсией фона (цвет игнорируется)
                   //   IM_SCALE, IM_DONTSCALE, ... - признаки масштабируемости
                   //    (игнорируются при Image равном IMG_OBJECT)
                   // Пример использования: Mode = R2_NOT | IM_SCALE
}
  PAINTPARM;

#endif

//-----------------------------------------------------------------
// ОПИСАНИЕ ВЫВОДИМОГО ТЕКСТА
//-----------------------------------------------------------------
// РЕАЛЬНАЯ ДЛИНА ТЕКСТА МОЖЕТ БЫТЬ МЕНЬШЕ ДЛИНЫ ЗАПИСИ.
// МИНИМАЛЬНАЯ ДЛИНА ЗАПИСИ - 5 байт: 4 байта - длина = 5 и 1 байт = 0
// ЕСЛИ В ТЕКСТЕ ВСТРЕЧАЕТСЯ '\n',
// ТО ВЫПОЛНЯЕТСЯ ПЕРЕХОД НА СЛЕДУЮЩУЮ СТРОКУ
//-----------------------------------------------------------------

typedef struct TEXTRECORD
{
  int  Length ;                  // Длина записи, включая поле Length
  char Text[256];                // Текст, заканчивающийся двоичным 0
}                                // Текст всегда в кодировке WINDOWS
  TEXTRECORD;


//-----------------------------------------------------------------
// ОПИСАНИЕ КООРДИНАТ ГРАФИЧЕСКОГО ОБ'ЕКТА
//-----------------------------------------------------------------
// ЕДИНИЦА ИЗМЕРЕНИЯ И ВИД СИСТЕМЫ КООРДИНАТ
// ЗАДАЕТСЯ В ПАРАМЕТРАХ ФУНКЦИИ ВИЗУАЛИЗАЦИИ
//-----------------------------------------------------------------

typedef struct PLACEDATA         // КООРДИНАТЫ В ПРОИЗВОЛЬНОЙ СИСТЕМЕ
{
  DOUBLEPOINT * Points    ;      // Списки координат отдельных контуров
  int *         PolyCounts;      // Списки количества точек контуров
  int           Count     ;      // Количество контуров или структур TEXTRECORD
                                 // Для функций типа IMG_TEXT :
#if defined(_M_X64) || defined(BUILD_DLL64)       // 12/01/16
  int           Reserv    ;      // Должен быть равен 0
#endif
  TEXTRECORD *  PolyText  ;      // Область последовательно расположенных
                                 // записей типа TEXTRECORD, размеры записей
                                 // могут быть разными, но не меньше 5 байт,
                                 // число структур равно Count !
}
  PLACEDATA;


//-----------------------------------------------------------------
// ПАРАМЕТРЫ ОТОБРАЖЕНИЯ ОБРАЗЦА ВИДА ОБЪЕКТА
//-----------------------------------------------------------------

typedef struct PAINTEXAMPLE
{
  HDC       Hdc;       // Контекст устройства
  RECT*     Rect;      // Область отображения в пикселах

  int       Func;      // Номер функции (IMG_LINE, IMG_DOT, ...)
  char*     Parm;      // Указатель на параметры функции

  POLYDATAEX* Data;    // Метрика
                       //   (если Data != 0, то text игнорируется)
                       //   (если Data == 0, то создается автоматически)

  char*     Text;      // Текст всегда в кодировке WINDOWS
                       //   (если Text == 0, то создается автоматически)

  int       Local;     // Локализация (LOCAL_LINE, LOCAL_SQUARE, ...)
                       //   (используется при Data == 0)

  float     Factor;    // Коэффициент увеличения (Factor >= 0.0)
                       //  0.0 - рассчитывается автоматически

  int       Colors;    // Число цветов палитры
  COLORREF* Palette;   // Указатель на палитру

  int       VisualType;     // Тип визуализации (VT_SCREEN, VT_PRINT, ...)
                            //    0 - текущий

  int       FillIntensity;  // Интенсивность заливки (0,100)
                            //    0 - текущая
                            //  100 - в реальных цветах
  char      Reserve[16];
}
  PAINTEXAMPLE;


//******************************************************************
// ВИДЫ ФУНКЦИЙ ВИЗУАЛИЗАЦИИ ПРОИЗВОЛЬНЫХ ГРАФИЧЕСКИХ ОБ'ЕКТОВ НА ЭК
//******************************************************************

#define IMG_EMPTY             127  // Пустая функция - нет параметров

#define IMG_LINE              128  // Линия
#define IMG_DOT               129  // Пунктир

#define IMG_SQUARE            135  // Полигон

#define IMG_CIRCLE            140  // Окружность

#define IMG_TEXT              142  // Подпись

#define IMG_MULTIMARK         143  // Многоцветный точечный знак
#define IMG_MULTISQUAREMARK   144  // Полигон, заполненный точечными знаками

#define IMG_DRAW              147  // Набор различных примитивов

#define IMG_DOTSHIFT          148  // Смещенный пунктир

#define IMG_VECTOREX          149  // Векторный знак

#define IMG_TEMPLATE          150  // Шаблон (устаревший тип)

#define IMG_TRUETYPE          151  // Знак TrueType

#define IMG_TRUETEXT          152  // Подпись

#define IMG_HATCHSQUARE       153  // Заштрихованный полигон

#define IMG_SQUAREGLASS       154  // Полигон-стекло

#define IMG_SQUAREVECTOR      155  // Полигон, заполненный векторными знаками
#define IMG_VECTOREXTURN      156  // Векторный знак (служебный)

#define IMG_DECORATE          157  // Линия с оформлением векторными знаками
#define IMG_DASH              158  // Наборная линия
#define IMG_LINESHIFT         159  // Смещенная линия
#define IMG_VECTORTEXT        160  // Подпись в векторном знаке
#define IMG_VECTORNODE        161  // Векторные в узлах (IMGVECTOREX)
#define IMG_THICKENLINE       162  // Утолщающаяся линия
#define IMG_MIDDLETHICKENLINE 163  // Утолщающаяся линия к середине от концов
#define IMG_GRAPHICMARK       165  // Знак из файла (ВМР, Metafile )
#define IMG_HATCHSQUARESHIFT  166  // Штрихованый площадной со смещением штриховки // 31/03/08
#define IMG_POLYGONGLASS      167  // Полигон-стекло с величиной прозрачности  // 19/01/09
#define IMG_LINEGLASS         168  // Линия-стекло с величиной прозрачности    // 23/01/09
#define IMG_DOTGLASS          169  // Штриховая линия-стекло с величиной прозрачности // 15/02/09
#define IMG_MULTIMARKGLASS    170  // Точечный-стекло с величиной прозрачности // 14/02/09
#define IMG_PICTURE           171  // Графический знак (PNG) параметры в классификаторе

#define IMG_VECTORGRADIENT    199  // Служебная функция
#define IMG_VECTORMIX         200  // Служебная функция
#define IMG_DRAWMIX           201  // Служебная функция

#define IMG_DOCITEM           248  // Структура IMGDOCITEM для хранения документов
#define IMG_PERCENT           249  // Структура типа IMG_LONG
#define IMG_LIBRARY           250  // Знак IML

#define IMG_SEMANTIC          251  // Структура типа IMG_LONG
#define IMG_COLOR             252  // Структура типа IMG_LONG
#define IMG_DOUBLE            253  // Поле типа DOUBLE
#define IMG_LONG              254  // Поле типа LONG

#define IMG_OBJECT            255  // Номер функции устанавливается
                                   // из классификатора

// Прозрачный цвет (для IMG_SQUAREMARK, IMG_MULTISQUAREMARK,
//                      IMG_SQUARECROSS, IMG_FONT, IMG_SQUAREGLASSCOLOR)
#define IMGC_TRANSPARENT  0xFFFFFFFF
#define IMGC_NOMARK       0xFFFFFFFF   // Не выделять (для IMG_OBJECT)

// Флаги для определения формата цвета (индекс|IMGC_INDEX, RGB)
#define IMGC_INDEX        0x0F0000000  // Флаг цвета из палитры карты
#define IMGC_CLEARINDEX   0x0FF000000  // Для очистки флага цвета (& IMGC_CLEARINDEX)
#define IMGC_CLEARCOLOR   0x000FFFFFF  // Флаг очистки цвета      (& IMGC_CLEARCOLOR)

// Установить формат цвета
// color - номер цвета в палитре RSC, начиная с 0 !!!
// Если флаг IMGC_INDEX не установлен, цвет в формате RGB
#define SETCOLORINDEX(color)   (((int)(color))|IMGC_INDEX)

#define IMGC_SYSTEM       0x0F1000000L  // Флаг цвета из системной палитры (LINUX)  // 16/10/03
#define SETCOLORSYSTEM(color)  (((int)(color))|IMGC_SYSTEM)

// Определить формат цвета (возвращает: 0 - RGB, 1 - индекс из палитры)
#define TESTCOLORINDEX(color)  ((((int)(color))&IMGC_CLEARINDEX)==IMGC_INDEX)

// Определить формат цвета (возвращает: 0 - RGB, 1 - индекс из системной палитры)
#define TESTCOLORSYSTEM(color) ((((int)(color))&IMGC_CLEARINDEX)==IMGC_SYSTEM)  // 16/10/03

// Тип сетки заполнения 4x4 (16 бит) (для IMG_SQUAREMARK, IMG_MULTISQUAREMARK)
//  1 - заполняемые элементы сетки
//  0 - незаполняемые
// Основные типоы сеток:
//  0000(0)  1111(F)  1010(A)  0101(5)
//  0000(0)  1111(F)  0101(5)  1010(A)
//  0000(0)  1111(F)  1010(A)  0101(5)
//  0000(0)  1111(F)  0101(5)  1010(A)
#define IMGK_REGULAR0     0x00000000L  // Регулярная (заполнить все)
#define IMGK_REGULAR      0x0000FFFFL  // Регулярная (заполнить все)
#define IMGK_ONECHESS     0x0000A5A5L  // Шахматная
#define IMGK_TWOCHESS     0x00005A5AL  // Шахматная смещенная

// Локализация объекта (для IMG_OBJECT)
#define IMGL_LINE      0  // Линейный
#define IMGL_SQUARE    1  // Площадной
#define IMGL_POINT     2  // Точечный
#define IMGL_TITLE     3  // Подпись
#define IMGL_VECTOR    4  // Векторный
#define IMGL_MIXED     5  // Сложная подпись (шаблон)

// Тип штpиховки (для IMG_SQUARECROSS)
#define SC_HORIZONTAL  1  // Гоpизонтальными линиями
#define SC_VERTICAL    2  // Веpтикaльными линиями
#define SC_QUADRATIC   3  // Кубикaми
#define SC_CROSS45     4  // Под углом 45 градусов
#define SC_CROSS135    5  // Под углом 135 градусов

// Типы векторных цепочек (для SCR_VECTOR)
#define VT_THICK2      2  // Линия толщины 2
#define VT_THICK3      3  // Линия толщины 3
#define VT_THICK4      4  // Линия толщины 4
#define VT_THICK5      5  // Линия толщины 5
#define VT_CIRCLE2     8  // Окружность толщины 2
#define VT_CIRCLE3     9  // Окружность толщины 3
#define VT_CIRCLE4    10  // Окружность толщины 4
#define VT_CIRCLE5    11  // Окружность толщины 5

// Типы векторных цепочек (IMG_VECTOREX)
#define VT_SQUARE      0  // Площадь
#define VT_LINE        1  // Линия
#define VT_ROUND       6  // Круг (площадь)
#define VT_CIRCLE      7  // Окружность
#define VT_FREE       12  // Цепочка не определена

// Типы векторных цепочек (IMG_VECTOREX)
#define VT_ELLIPSE    64  // Эллипс
#define VT_OVAL       65  // Эллипс (площадь)
#define VT_ARC        66  // Цепочка типа дуга
#define VT_PIE        67  // Сектор (площадь)
#define VT_TEXT       68  // Цепочка типа текст (постоянный)
#define VT_STEXT      69  // Цепочка типа текст (из семантики)
#define VT_ARC_REVERT 70  // Цепочка типа дуга

// Типы служебных цепочек для описания структуры блока
#define VT_OBJECT    101  // Начало блока
#define VT_SUBJECT   102  // Начало подобъекта
#define VT_END       103  // Конец блока

// Операции пересчета пикселов в микроны и обратно
#define MKMINPIX     250  //  1000 * 25.4 / 96 = 264.583333
#define PIX2MKM(pixel)  (((int)(pixel))  * MKMINPIX)
#define MKM2PIX(metric) (((int)(metric)) / MKMINPIX)

//------------------------------------------------------------------
// Для IMG_FONT
//------------------------------------------------------------------

// Прозрачный цвет (фона или тени)
#define UNIC_TRANSPARENT  255

// Относительная ширина символа
#define UNIW_NORMAL       0  // Нормальный
#define UNIW_NARROW       1  // Суженый
#define UNIW_WIDE         2  // Широкий

// Выравнивание по вертикали
#define UNIA_BASELINE     0  // По базовой линии
#define UNIA_TOP          1  // По верху
#define UNIA_BOTTOM       2  // По низу

// Выравнивание по горизонтали
#define UNIA_LEFT         0  // По левому краю
#define UNIA_CENTER       1  // По центру
#define UNIA_RIGHT        2  // По правому краю

// Минимально допустимая высота символа в пикселах
#define CharMin  5  // Подписи высотой < CharMin, не отображаются
                    // 5 - 25/10/99

// Относительная ширина символа (mapgds.h)
//  UNIW_NORMAL     - Нормальный
//  UNIW_NARROW     - Суженый
//  UNIW_WIDE       - Широкий

#ifdef LINUXAPI  // 23/04/01
// Вес шрифта, толщина контура (Windows-константы)
//  FW_THIN         - Тонкий
//  FW_ULTRALIGTH   - Четкий
//  FW_NORMAL       - Средний
//  FW_MEDIUM       - Утолщенный
//  FW_BOLD         - Толстый
#define FW_DONTCARE         0
#define FW_THIN             100
#define FW_EXTRALIGHT       200
#define FW_LIGHT            300
#define FW_NORMAL           400
#define FW_MEDIUM           500
#define FW_SEMIBOLD         600
#define FW_BOLD             700
#define FW_EXTRABOLD        800
#define FW_HEAVY            900

#define FW_ULTRALIGHT       FW_EXTRALIGHT
#define FW_REGULAR          FW_NORMAL
#define FW_DEMIBOLD         FW_SEMIBOLD
#define FW_ULTRABOLD        FW_EXTRABOLD
#define FW_BLACK            FW_HEAVY

#endif // LINUXAPI


// Выравнивание по горизонтали (Windows-константы)
#define FA_LEFT      0     // Слева = TA_LEFT       // 14/09/00
#define FA_RIGHT     2     // Справа = TA_RIGHT     // 14/09/00
#define FA_CENTER    6     // По центру = TA_CENTER // 14/09/00

// Выравнивание по вертикали (Windows-константы)
#define FA_TOP       0     // Сверху = TA_TOP       // 14/09/00
#define FA_BOTTOM    8     // Снизу = TA_BOTTOM     // 14/09/00
#define FA_BASELINE  24    // По базовой линии = TA_BASELINE  // 14/09/00
#define FA_MIDDLE    4120  // По средней линии = (TA_BASELINE|0x01000) // 14/09/00

// Маска выборки выравнивания
#define FA_MASK     (FA_CENTER | FA_MIDDLE)         // 23/04/03


// Коды выравнивания текста (внутренние, в метрике подписи)
#define CA_BASELINE_LEFT          20
#define CA_BASELINE_RIGHT         21
#define CA_BASELINE_CENTER        22

#define CA_MIDDLE_LEFT            23
#define CA_MIDDLE_RIGHT           24
#define CA_MIDDLE_CENTER          25

#define CA_TOP_LEFT               26
#define CA_TOP_RIGHT              27
#define CA_TOP_CENTER             28

#define CA_BOTTOM_LEFT            29
#define CA_BOTTOM_RIGHT           30
#define CA_BOTTOM_CENTER          31

// Диапазон кодов выравнивания текста
#define CA_MINCODE  CA_BASELINE_LEFT
#define CA_MAXCODE  CA_BOTTOM_CENTER

// Кодовая страница (Windows-константы)
//  RUSSIAN_CHARSET - для России (ANSI/WINDOWS)
//  OEM_CHARSET     - для России (OEM/866/DOS)
//------------------------------------------------------------------

//******************************************************************
//               СТРУКТУРЫ ПАРАМЕТРОВ ВИЗУАЛИЗАЦИИ                 *
//******************************************************************
//      ВСЕ РАЗМЕРЫ (ТОЛЩИНА, ДЛИНА, РАССТОЯНИЕ ...) В МИКРОНАХ    *
//                 (В ТЫСЯЧНЫХ ДОЛЯХ МИЛЛИМЕТРА)                   *
//******************************************************************
// Примечания:                                                     *
//  1. Цвет в формате COLORREF (0x00BBGGRR) или номер индекса      *
//     палитры карты в младшем байте, если старший байт равен 0x0F *
//  2. Для описания размеров в точках экрана (пикселах)            *
//     использовать операцию PIX2MKM(число пикселов):              *
//     Parm->Thick = PIX2MKM(2);  (толщина 2 пиксела)              *
//******************************************************************

typedef struct IMGLINE           // (128) ЛИНИЯ
{                                //      (size = 8)
  unsigned int  Color;           // Цвет линии (COLORREF)
  unsigned int  Thick;           // Толщина линии
}
  IMGLINE;

typedef struct IMGTHICKENLINE    // (162,163) УТОЛЩАЮЩАЯСЯ ЛИНИЯ
{                                //      (size = 12)
  unsigned int  Color;           // Цвет линии (COLORREF)
  unsigned int  Thick;           // Толщина минимальная
  unsigned int  Thick2;          // Толщина максимальная (утолщения)
}
  IMGTHICKENLINE;

typedef struct IMGDOT            // (129) ПУНКТИРНАЯ ЛИНИЯ
{                                //      (size = 16)
  unsigned int  Color;           // Цвет линии
  unsigned int  Thick;           // Толщинa штpиха
  unsigned int  Dash ;           // Длина штpиха
  unsigned int  Blank;           // Длина пpобела
}
  IMGDOT;

typedef struct IMGSQUARE         // (135) ПЛОЩАДНОЙ ОБ'ЕКТ
{                                //      (size = 4)
  unsigned int  Color;           // Цвет площади
}
  IMGSQUARE;

typedef struct IMGCIRCLE         // (140) ОКРУЖНОСТЬ
{                                //      (size = 12)
  unsigned int  Color ;          // Цвет окружности
  unsigned int  Thick ;          // Толщина линии
  unsigned int  Radius;          // Радиус окружности
}
  IMGCIRCLE;


typedef struct IMGTEXT           // (142) ШРИФТ                   // 30/01/01
{                                //      (size = 32)
  unsigned int   Color       ;   // Цвет текста
  unsigned int   BkgndColor  ;   // Цвет фона (IMGC_TRANSPARENT - прозрачный)
  unsigned int   ShadowColor ;   // Цвет тени (IMGC_TRANSPARENT - прозрачный)
  unsigned int   Height      ;   // Высота (При растягивании по метрике
                                 // по умолчанию 1800 мкм)
  unsigned short Weight      ;   // Вес шрифта, толщина контура (FW_THIN, ...)
  unsigned char  Outline     ;   // Признак вывода тени в виде контура // 17/07/07
                                 // 0 - не выводить. 1 - тонкий контур, 2 - толстый
  unsigned char  Interval    ;   // Межстрочный интервал в процентах (от 50 до 255)  // 17/03/16
                                 // Если интервал не указан (0), то соответствует 100%
  unsigned short Align       ;   // Выравнивание (FA_LEFT|FA_BASELINE, ...)
  unsigned short Service     ;   // Равно 0

  unsigned char  Wide        ;   // Относительная ширина символа (UNIW_NORMAL, ...)
  unsigned char  Horizontal  ;   // Признак горизонтального расположения (0/1)
  unsigned char  Italic      ;   // Признак наклона символов (0/1)
  unsigned char  Underline   ;   // Признак подчеркивания    (0/1)
  unsigned char  StrikeOut   ;   // Признак перечеркивания   (0/1)
  unsigned char  Type        ;   // Тип шрифта (номер в таблице: 0,1,2,3...)
  unsigned char  CharSet     ;   // Кодовая страница (RUSSIAN_CHARSET, ...)
  unsigned char  Flag        ;   // Признаки отображения текста (IMGTEXT_SCALE, ...)
}
  IMGTEXT;

// Признаки отображения текста (для поля Flag структуры IMGTEXT)
#define IMGTEXT_SCALE       1    // Признак растягивания по метрике (высота вычисляется)
#define IMGTEXT_SHOWCODE    2    // Отобразить цифровое значение классификатора семантики
#define IMGTEXT_SHORTCLS    4    // Отобразить текстовое значение классификатора семантики
#define IMGTEXT_EXPANDED    8    // Отобразить текст со вставкой пробелов: "Abc" -> "A b c"
#define IMGTEXT_UPPERCASE  16    // Отобразить текст в верхнем регистре: Abc -> "ABC"  // 24/11/15

typedef IMGTEXT IMGFONT;


typedef struct IMGMARKCHAIN      // ТОЧЕЧНЫЙ РАСТРОВЫЙ ОБ'ЕКТ ЦЕПОЧКИ
{                                //      (size = 4 + 128 = 132)
  unsigned int  Color    ;       // Цвет знака
  unsigned char Bits[128];       // Растровое изображение 32x32 с верхнего угла
}
  IMGMARKCHAIN;


typedef struct IMGMULTIMARK      // (143) МНОГОЦВЕТНЫЙ ТОЧЕЧНЫЙ
{                                //      (size = 20 + 132*Count)
  unsigned int  Length;          // Полная длина записи описания объекта
  unsigned int  Count ;          // Число цветов в знаке
  unsigned int  Size  ;          // Размер в микронах (для печати)
  unsigned int  PosV  ;          // Точка привязки знака PosV,PosH  <  Size
  unsigned int  PosH  ;          //  (для IMGMULTISQUAREMARK - шаг сетки >= Size)
                                 // затем: Count структур типа IMGMARKCHAIN
}
  IMGMULTIMARK;


typedef struct IMGMULTISQUAREMARK// (144) ПЛОЩАДНОЙ + МНОГОЦВЕТНЫЙ ТОЧЕЧНЫЙ
{                                //      (size = 4 + 20+132*Count )
  unsigned short Kind    ;       // Тип сетки заполнения 4x4 (16 бит)
                                 //  (IMGK_REGULAR, ...)
  unsigned char  FullMark;       // Флаг заполнения
                                 //  0 - по всей площади
                                 //  1 - только полными знаками
  unsigned char  Weight  ;       // Вес знака, толщина контура   // 27/04/00
                                 //  0 - тонкий
                                 //  1 - нормальный
  IMGMULTIMARK   Mark    ;       // (143) МНОГОЦВЕТНЫЙ ТОЧЕЧНЫЙ
                                 // затем: Count структур типа IMGMARKCHAIN
}
  IMGMULTISQUAREMARK;


typedef struct IMGDECOR          // ЗАГОЛОВОК ЧАСТИ ПАРАМЕТРОВ
{
  unsigned short Length;         // Длина параметров >= 4
                                 // (включая sizeof(IMGDECOR))
  unsigned short Number;         // Номер функции
}
  IMGDECOR;

// Флаги для набора примитивов   // 28/07/00
#define IMGD_SPLINE       0x0001 // Флаг рисования по сглаженной метрике

typedef struct IMGDRAW           // (147) НАБОР ПРИМИТИВОВ
{
  unsigned int  Ident   ;        // Идентификатор записи 0x7FFF7FFE
  unsigned int  Length  ;        // Полная длина записи описания объекта
  unsigned short Count  ;        // Количество примитивов
  unsigned short Flags  ;        // Должен быть 0
  IMGDECOR      Image   ;        // Заголовок первого примитива
  unsigned int  Parm[1] ;        // Параметры примитива по типу
                                 // Далее заголовок и параметры
                                 // следующих примитивов
} IMGDRAW;

typedef IMGDRAW  SCRDRAW ;

typedef struct IMGDOTSHIFT       // (148) ПУНКТИР СМЕЩЕННЫЙ
{                                //
  IMGDOT    Dot   ;              // Пунктир
  int       Shift ;              // Направление и величина сдвига
                                 // 0 - пунктир лежит справа вплотную к оси
                                 // > 0 сдвигаемся вправо от оси,иначе влево
                                 // при сдвиге = - ширина пунктир лежит слева
                                 // вплотную к  оси
}
 IMGDOTSHIFT;

typedef IMGDOTSHIFT SCRDOTSHIFT;

typedef struct IMGVECTPOINT      // Для (149) описание точки объекта
{                                //      (size = 8)
  int Hor;                       // Координаты от точки привязки
  int Ver;                       // вправо и вниз в соответствии
                                 // с типом цепочки:
                                 //  Центр  Радиусы  Точки начала и окончания
                                 // Circle, Round  (h,v)  (dh,dh)
                                 // Ellipse, Oval  (h,v)  (dh,dv)
                                 // Arc,      Pie  (h,v)  (dh,dv) (h1,v1)(h2,v2)
}
  IMGVECTPOINT;


typedef struct IMGPOLYDESC       // Для (149) описание цепочки
{                                //      (size = 12 + 8*Count + Length)
  unsigned char  Type    ;       // Тип цепочки (VT_SQUARE, ...)
                                 // для выделения блока VT_OBJECT,
                                 // VT_SUBJECT, VT_END)
  unsigned char  Image   ;       // Тип параметров
                                 // (IMG_SQUARE, IMG_LINE, IMG_DOT,
                                 // IMG_VECTORTEXT)                 // 11/04/05
  unsigned short Length  ;       // Длина параметров (>= 4)
  unsigned int   Parm    ;       // Параметры цепочки по ее типу
  unsigned int   Count   ;       // Число точек в цепочке
  IMGVECTPOINT   Point[1];       // Координаты до 512 точек
}
  IMGPOLYDESC;

typedef struct IMGVECTOREX       // (149,156) ПАРАМЕТРЫ ВЕКТОРНОГО ОБ'ЕКТА
{                                //      (size = 20 + ...)
  int  Length      ;             // Полная длина записи описания объекта
  int  PosV        ;             // Точка привязки знака по вертикали
  int  PosH        ;             //  и горизонтали (положение первой точки)
  int  Base        ;             // Длина базовой осевой линии знака
  int  VLine1      ;             // Начало отметки по вертикали  (0)
  int  VLine2      ;             // Конец  отметки по вертикали  (VSize)
  int  VSize       ;             // Требуемый размер отметки по вертикали
  int  HLine1      ;             // Начало отметки по горизонтали (0)
  int  HLine2      ;             // Конец отметки по горизонтали  (HSize)
  int  HSize       ;             // Требуемый размер отметки по горизонтали
  char Static      ;             // 1 - горизонтальное расположение
                                 // 0 - по метрике
  char Mirror      ;             // Флаг зеркального отображения знака
                                 // в направлении второй точки метрики
  char NoPress     ;             // Признак запрета сжатия знака
  char Scale       ;             // Признак растягивания по метрике
  char Centre      ;             // Признак центрирования по метрике
  char  NoChangeColor ;  // 1 - не измененять цвет по семантике SEMIMAGECOLOR     
  char Reserve[2]  ;             // Резерв
  int  Border      ;             // Габариты знака от отсчета в мкм или 0
  int  Count       ;             // Число фрагментов,описывающих знак
  IMGPOLYDESC  Desc;             // Описание фрагмента
}
  IMGVECTOREX;                   // то же для IMG_VECTOREXTURN (156)

typedef struct TABLETEMPLATE     // Постоянная часть векторного шаблона
{
  int  Length  ;                 // длина структуры
  int Order[12];                 // Клетки с семантикой
                                 // 0-пустая клетка
                                 // -1 линия  -2 знак
                                 // >0  номер семантики подписи  (текст)
  int  Origin  ;                 // Номер клетки привязки
  int  Static  ;                 // 1 - горизонтальное расположение
                                 // 0 - по метрике
}
  TABLETEMPLATE;

typedef struct IMGTEMPLATE       // (150) ПАРАМЕТРЫ ВЕКТОРНОГО ШАБЛОНА
{                                //      (size = 20 + ...)
  int  Length               ;    // Полная длина записи описания объекта
  TABLETEMPLATE Table       ;    // Постоянная часть векторного шаблона
  int           Count       ;    // Количество примитивов
  IMGDECOR      Text        ;    // Заголовок текста
  unsigned int  Parmtext[1] ;    // Параметры текста
  IMGDECOR      Line        ;    // Заголовок линии
  unsigned int  Parmline[1] ;    // Параметры линии
  IMGDECOR      Mark        ;    // Заголовок знака
  unsigned int  Parmmark[1] ;    // Параметры знака
}
  IMGTEMPLATE;

typedef struct IMGTRUETYPE       // (151) ПАРАМЕТРЫ ЗНАКА из TRUETYPE ШРИФТА
{
  IMGTEXT Text     ;             // Параметры шрифта
  char FontName[32];             // Условное название шрифта
  int  Number      ;             // Номер знака в шрифте
  int  MinV        ;             // Габариты знака
  int  MinH        ;             //  относительно точки привязки
  int  MaxV        ;             // (0,0)
  int  MaxH        ;             // (H,H/2) - начальные значения
  int  PosV        ;             // Точка привязки знака по вертикали
  int  PosH        ;             // и горизонтали (0,0)
}
  IMGTRUETYPE;

typedef struct IMGTRUETEXT       // (152) ШРИФТ
{                                //      (size = 64)
  IMGTEXT Text     ;             // Параметры шрифта (Поле Type - произвольно)
  char FontName[32];             // Условное название шрифта
}
  IMGTRUETEXT;

typedef struct IMGSQUAREGLASS    // (154) ПЛОЩАДНОЙ ОБ'ЕКТ - СТЕКЛО
{                                //      (size = 8)
  signed int Bright;            // Процент изменения яркости (+/- 100)
  signed int Contrast;          // Процент изменения контраста (+/- 100)
}
  IMGSQUAREGLASS;

typedef struct IMGSQUAREVECTOR   // (155) ПЛОЩАДНОЙ + ВЕКТОРНЫЙ
{                                //
  int            Length  ;       // Длина структуры
  unsigned short Kind    ;       // Тип сетки заполнения 4x4 (16 бит)
                                 //  (IMGK_REGULAR, ...)
  unsigned char  FullMark;       // Флаг заполнения
                                 //  0 - по всей площади
                                 //  1 - только полными знаками
  unsigned char  Reserv  ;       // Должен быть 0
  int            StepHor ;       // Расстояние между знаками по горизонтали
  int            StepVer ;       // Расстояние между знаками по вертикали
  int            Angle   ;       // Угол поворота знака от вертикали по часовой
  IMGVECTOREX    Mark    ;       // Описание знака
}
  IMGSQUAREVECTOR;


#define DO_BASE      1           // базовый объект (линия либо площадь)
                                 // объекты заполняющие метрику
#define DO_FIRST     2           // на первой точке
#define DO_END       3           // на последней точке
#define DO_LEFT      4           // слева на каждой точке
#define DO_RIGHT     5           // справа на каждой точке
#define DO_MIDDLE1   6           // вдоль линии (1 - оформляющий)
#define DO_MIDDLE2   7           // вдоль линии (2 - заполняющий)


typedef struct IMGDECORHEAD      // ЗАГОЛОВОК ОФОРМЛЕННОГО ЛИНЕЙНОГО
{
  unsigned int  Length      ;    // Полная длина записи описания объекта
           int  Begin       ;    // Длина начала линии
           int  End         ;    // Длина конца линии
           int  Left        ;    // Длина начала отрезка
           int  Right       ;    // Длина конца отрезка
           int  First       ;    // Длина первого заполняющего знака
           int  Second      ;    // Длина второго заполняющего знака
           char Equidistant ;    // Равномерно размещать по всей линии
           char Reserv[3]   ;    // Резерв - равен 0
}
  IMGDECORHEAD;

typedef struct IMGDECORATE        // (157) ОФОРМЛЕННЫЙ ЛИНЕЙНЫЙ
{
  IMGDECORHEAD   Head         ;   // Заголовок
  IMGDECOR       Base         ;   // Заголовок базовой функции
  unsigned int   BaseParm[1]  ;   // Параметры базовой по типу
  IMGDECOR       Begin        ;   // Заголовок начального знака линии
  unsigned int   BeginParm[1] ;   // Параметры по типу
  IMGDECOR       End          ;   // Заголовок последнего знака линии
  unsigned int   EndParm[1]   ;   // Параметры по типу
  IMGDECOR       Left         ;   // Заголовок начального знака отрезка
  unsigned int   LeftParm[1]  ;   // Параметры по типу
  IMGDECOR       Rigth        ;   // Заголовок последнего знака отрезка
  unsigned int   RigthParm[1] ;   // Параметры по типу
  IMGDECOR       First        ;   // Заголовок первого знака вдоль отрезка
  unsigned int   FirstParm[1] ;   // Параметры по типу
  IMGDECOR       Second       ;   // Заголовок второго знака вдоль отрезка
  unsigned int   SecondParm[1];   // Параметры по типу
}
  IMGDECORATE;

typedef struct IMGSECTION     // Описание отдельной секции для (158)
{
  unsigned int  Length;       // Длина параметров (включая IMGSECTION)
  unsigned int  Number;       // Номер функции(IMG_EMPTY, IMG_LINE, IMG_VECTOREX,
                              //               IMG_TRUETYPE)
  unsigned int  Base;         // Длина отрезка, занимаемая секцией
  unsigned int  Parm[1];      // Параметры
}
  IMGSECTION;

typedef struct IMGDASH           // (158) НАБОРНАЯ ШТРИХОВАЯ ЛИНИЯ
{
       int    Length;            // Полная длина записи описания объекта,
       int    Count;             // Число отдельных секций (штрихов, пробелов, знаков)
       int    Last;              // Порядковый номер секции (от 0 до Count-1),
                                 // Завершающей контур объекта
       int    Border;            // Максимальная толщина секций
       int    Reserve;           // Резерв
  IMGSECTION  Section[1];        // Описание отдельной секции
}
  IMGDASH;

typedef struct IMGLINESHIFT      // (159) CМЕЩЕННАЯ ЛИНИЯ
{
       int    Length;            // Полная длина записи описания объекта,
       int    Shift;             // Направление и величина сдвига
                                 // = 0 - центр линии вдоль оси метрики
                                 // > 0 - сдвиг вправо от оси, иначе влево
  IMGDECOR    Head;              // Заголовок линии
       int    Parm[1];           // Параметры линии по ее типу
}
  IMGLINESHIFT;

typedef struct IMGVECTORTEXT     // (160) ПАРАМЕТРЫ ЦЕПОЧКИ ВЕКТОРНОГО (VT_TEXT)
{                                // (size = 68)
  IMGTEXT  Text     ;            // Параметры шрифта
       int Code     ;            // Код семантики объекта, для выбора переменного
                                 // текста или 0 для постоянного текста
  char     Title[32];            // Постоянный текст
}
  IMGVECTORTEXT;

typedef struct IMGGRAPHICMARKEX    // (165) ТОЧЕЧНЫЙ ЗНАК - ГРАФИЧЕСКОЕ ИЗОБРАЖЕНИЕ
{                                  // (size = 312)
       int Length   ;              // Полная длина записи описания объекта
       int Ident    ;              // Идентификатор изображения = 0
  char     Name[256];              // Имя файла изображения
       int Type     ;              // Тип файла (1 - BMP, 2 - JPEG, 3 - Metafile)
       int Height   ;              // Габариты знака (высота, ширина)
       int Width    ;              //
       int PosH     ;              // Точка привязки знака по горизонтали
       int PosV     ;              // и вертикали (от начала габаритов знака)
       int Shift        ;          // Смещение тени
  IMGLINE  Contour      ;          // Параметры контура
  char     ContourFlag  ;          // Флаг наличия контура
  char     ShadeFlag    ;          // Флаг наличия тени
  char     TransparentFlag;        // Флаг прозрачности фонового цвета     // 29/10/09
                                   // (первый пиксел картинки - левый нижний угол BMP)
  char     Reserve[13]  ;          // Резерв
}
 IMGGRAPHICMARKEX;

typedef IMGGRAPHICMARKEX IMGGRAPHICMARK;   

typedef struct IMGHATCHSQUARESHIFT  // (166) ШТРИХОВАННЫЙ ПЛОЩАДНОЙ СО СМЕЩЕНИЕМ ШТРИХОВКИ
{                                   // 04/04/08
       int Length;               // Полная длина записи описания объекта,
                                 // включая параметры линий штриховки
       int Angle ;               // Угол наклона штриховки в градусах (0,45,90,135,180,225,270,315)
                                 // против часовой стрелки от горизонтали
       int Step  ;               // Расстояние между осями линий штриховки
       int Number;               // Номер функции линии штриховки (IMGLINE, IMGDOT, IMGDASH)
       int Shift ;               // Смещение оси первой линии штриховки
                                 // затем: параметры линии штриховки
} IMGHATCHSQUARESHIFT;

typedef struct IMGPOLYGONGLASS   // (167) ПЛОЩАДНОЙ ОБ'ЕКТ - ЦВЕТНОЕ СТЕКЛО C ВЕЛИЧИНОЙ ПРОЗРАЧНОСТИ
{                                // (size = 16) (вместо IMGSQUAREGLASSCOLOR)
         int  Color;             // Цвет тонирования (IMGC_TRANSPARENT - бесцветный, тень)
  signed int Bright;            // Процент изменения яркости (+/- 100)
  signed int Contrast;          // Процент изменения контраста (+/- 100)
         int  Transparency;      // Степень прозрачности (от 0 - не отображается, до 100 - сплошной цвет)
}
  IMGPOLYGONGLASS;


typedef struct IMGLINEGLASS      // (168) ЛИНЕЙНЫЙ ОБ'ЕКТ - ЦВЕТНОЕ СТЕКЛО C ВЕЛИЧИНОЙ ПРОЗРАЧНОСТИ
{                                // (size = 20) (вместо IMGSQUAREGLASSCOLOR)
           int  Color;           // Цвет тонирования (IMGC_TRANSPARENT - бесцветный, тень)
  unsigned int  Thick;           // Толщина линии
  signed   int  Bright;          // Процент изменения яркости (+/- 100)
  signed   int  Contrast;        // Процент изменения контраста (+/- 100)
           int  Transparency;    // Степень прозрачности (от 0 - не отображается, до 100 - сплошной цвет)
}
  IMGLINEGLASS;

typedef struct IMGDOTGLASS       // (169) ШТРИХОВАЯ ЛИНИЯ - ЦВЕТНОЕ СТЕКЛО C ВЕЛИЧИНОЙ ПРОЗРАЧНОСТИ
{                                // (size = 28)
           int  Color;           // Цвет тонирования (IMGC_TRANSPARENT - бесцветный, тень)
  unsigned int  Thick;           // Толщинa штpиха
  unsigned int  Dash ;           // Длина штpиха
  unsigned int  Blank;           // Длина пpобела
  signed   int  Bright;          // Процент изменения яркости (+/- 100)
  signed   int  Contrast;        // Процент изменения контраста (+/- 100)
           int  Transparency;    // Степень прозрачности (от 0 - не отображается, до 100 - сплошной цвет)
}
  IMGDOTGLASS;

typedef struct IMGMULTIMARKGLASS // (170) МНОГОЦВЕТНЫЙ ТОЧЕЧНЫЙ - ЦВЕТНОЕ СТЕКЛО
{                                //      (size = 20 + 132*Count)
  signed   int  Bright;          // Процент изменения яркости (+/- 100)
  signed   int  Contrast;        // Процент изменения контраста (+/- 100)
           int  Transparency;    // Степень прозрачности (от 0 - не отображается, до 100 - сплошной цвет)
  IMGMULTIMARK  Mark;            // Параметры точечного знака
}
  IMGMULTIMARKGLASS;

typedef struct IMGPICTURE        // 171  ГРАФИЧЕСКИЙ ЗНАК параметры в классификаторе
{
  int  Length;                   // Полная длина записи описания объекта
  int  Height;                   // Габариты знака (высота, ширина)
  int  Width;                    //
  int  PosH;                     // Точка привязки знака по горизонтали
  int  PosV;                     // и вертикали (от начала габаритов знака)
  int  Shift;                    // Смещение тени

  int  PixelHeight;              // Габариты знака (высота, ширина)
  int  PixelWidth;               // справочные в пикселях

  IMGLINE Contour;               // Параметры контура

  char ContourFlag;              // Флаг наличия контура
  char ShadeFlag;                // Флаг наличия тени
  char TransparentFlag;          // Флаг прозрачности фонового цвета
                                 // (первый пиксел картинки - левый нижний угол BMP)
  char Type;                     // Тип файла (0 - PNG, 1 - BMP, 2 - JPG,
                                 //   3 - TIF, 4 - ICO, 5 - WMF, 6 - EMF)
                                 // должен соответствовать формату изображения
  char Reserve[16];              // Резерв

  // Далее содержимое файла изображения в соответствии с типом Type
}
 IMGPICTURE;

#define IMGDOCITEMIDENT  0x7F7F7F7E

typedef struct IMGDOCITEM
{
  unsigned int  Ident   ;        // Идентификатор записи 0x7F7F7F7E
  unsigned int  Length  ;        // Полная длина записи
  unsigned int  HeadLength;      // Длина заголовка - смещение на начало документа (кратно 4)
  unsigned int  DocLength;       // Длина документа, начиная со смещения на документ, или 0 (если ссылка)
  unsigned int  ImageLength;     // Длина изображения документа или 0 (если графический файл)
  unsigned int  NameLength;      // Длина названия документа, включая замыкающий 0 (кратно 4, название дополняется нулевыми байтами)
  unsigned int  Date;            // Дата создания\обновления исходного файла документа (YYYYMMDD)
  unsigned int  Time;            // Время создания\обновления исходного файла документа (число мск с 00:00)
  unsigned int  Flags;           // Управляющие флажки (1 - документ вставлен по ссылке, Name содержит путь или алиас)
  unsigned int  ImageFormat;     // Формат изображения или 0 (FILE_PNG, FILE_JPEG, FILE_EMF, FILE_BMP) 
  WCHAR         Name[1];         // Название документа (поле переменной длины)
// Далее размещается документ и его изображение в заданном формате, если есть изображение 
}
  IMGDOCITEM;

typedef struct IMGLIBRARY        // (250) ПАРАМЕТРЫ ОБ'ЕКТА(РИСУЕТ ПОЛЬЗОВАТЕЛЬ)
{
       int Length  ;             // Полная длина записи описания объекта
       int Code    ;             // Код DLL (IML)
       int Index   ;             // Индекс DLL в списке библиотек
       int Function;             // Код функции
  IMGDRAW  Draw    ;             // Параметры
}
  IMGLIBRARY;

typedef struct IMGDOUBLE         // (253) СВОБОДНЫЕ ПАРАМЕТРЫ ТИПА DOUBLE
{
  double Value;
}
  IMGDOUBLE;

typedef struct IMGLONG           // (254) СВОБОДНЫЕ ПАРАМЕТРЫ ТИПА LONG
{
  int  Value;
}
  IMGLONG;

typedef struct IMGOBJECT         // (255) ОТОБРАЗИТЬ/ВЫДЕЛИТЬ ОБ'ЕКТ
{                                //  В УСЛОВНЫХ ЗНАКАХ КЛАССИФИКАТОРА
                                 //      (size = 16)
  unsigned int  Color ;          // Цвет выделения объекта на карте
                                 //  (IMGC_NOMARK - не выделять)
  unsigned int  Incode;          // Внутренний код объекта
                                 //  (0 - устанавливается по Excode и Local)
  unsigned int  Excode;          // Внешний код объекта
  unsigned int  Local ;          // Локализация объекта (IMGL_LINE, ...)

  // Для функций типа PaintMapObject(...) Incode устанавливается
  // автоматически. Если выделение цветом не нужно, то вместо
  // адреса структуры можно указывать ноль.
}
  IMGOBJECT;



typedef struct IMGVECTORGRADIENT    // ВСПОМОГАТЕЛЬНАЯ СТРУКТУРА
{
  unsigned int  Color;        // Цвет градиентной заливки (COLORREF)
  unsigned int  Left;         // Процент заполнения габаритов слева
  unsigned int  Right;        // Процент заполнения габаритов справа
  unsigned int  Bright;       // Процент изменения яркости заливки
}
  IMGVECTORGRADIENT;

typedef struct VECTORMIX
{
  unsigned int  Color;        // Цвет знака (COLORREF) или 0xFFFFFFFF
  signed   int  Bright;       // Процент изменения яркости (+/- 100) или 0
  signed   int  Contrast;     // Процент изменения контраста (+/- 100) или 0
  signed   int  Transparency; // Степень прозрачности (от 0 - не отображается, до 100 - сплошной цвет)
  unsigned int  Exclusion;    // Цвет, который не меняется (COLORREF, обычно 0x0FFFFFF) или 0xFFFFFFFF
  unsigned int  Scale;        // Процент изменения размеров знака или 1
}
  VECTORMIX;

typedef struct IMGVECTORMIX   // ВСПОМОГАТЕЛЬНАЯ СТРУКТУРА (200)    // 12/04/12
{
  IMGVECTOREX * Vector;       // Указатель на параметры векторного знака
  VECTORMIX     Mixing;       // Параметры модификации изображения знака
}
  IMGVECTORMIX;

typedef struct IMGDRAWMIX     // ВСПОМОГАТЕЛЬНАЯ СТРУКТУРА  (201)   // 18/04/12
{
  IMGDRAW *     Draw;         // Указатель на параметры векторного знака
  VECTORMIX     Mixing;       // Параметры модификации изображения знака
}
  IMGDRAWMIX;

typedef struct LEGENDESC      // Описание элемента легенды          // 21/04/15
{      
  COLORREF    FillColor;      // Цвет окна, на котором расположены элементы легенды (IMGC_TRANSPARENT - бесцветный)
  COLORREF    ContourColor;   // Цвет контура элемента изображения (IMGC_TRANSPARENT - бесцветный)    
  int         ColumnCount;    // Число столбцов (1 и более)
  int         ElHeight;       // Размер элемента легенды по высоте в мкм на карте в базовом масштабе
  int         ElWidth;        // Размер элемента легенды по ширине (изображение плюс подпись) в мкм
  int         ElHeightImage;  // Размер изображения условного знака по высоте в мкм
  int         ElWidthImage;   // Размер изображения условного знка по ширине в мкм
  int         Reserve;        // Резерв, равен нулю
  IMGTRUETEXT ImageFont;      // Описание шрифтов
}
  LEGENDESC;  

//**********************************************************************
// УСТАРЕВШИЕ СТРУКТУРЫ ПАРАМЕТРОВ ВИЗУАЛИЗАЦИИ (заменены на IMGDRAW,...)
//**********************************************************************

#define IMG_LINEDOT           130
#define IMG_DOUBLELINE        131
#define IMG_DOTDLINE          132
#define IMG_DOUBLEDOT         133
#define IMG_SHIFTLINEDOT      134
#define IMG_SQUARECROSS       136
#define IMG_MARK              137
#define IMG_SQUAREMARK        138
#define IMG_LEFTLINEDOT       145
#define IMG_DECORATELINE      146
#define IMG_SQUAREGLASSCOLOR  164  


typedef struct IMGLINEDOT        // (130) ПУНКТИР ПО ЛИНИИ
{                                //      (size = 24)
  IMGLINE Line;                  // Нижняя линия
  IMGDOT  Dot ;                  // Верхняя линия
}
  IMGLINEDOT;


typedef struct IMGDOUBLELINE     // (131) ЛИНИЯ ПО ЛИНИИ
{                                //      (size = 16)
  IMGLINE OneLine;               // Нижняя линия
  IMGLINE TwoLine;               // Верхняя линия
}
  IMGDOUBLELINE;


typedef struct IMGDOTDLINE       // (132) ДВОЙНАЯ ЛИНИЯ ПО ПУНКТИРУ
{                                //      (size = 32)
  IMGDOT        Dot   ;          // Нижняя линия
  IMGDOUBLELINE Double;          // Верхняя линия
}
  IMGDOTDLINE;


typedef struct IMGDOUBLEDOT      // (133) ДВОЙНОЙ ПУНКТИР
{                                //      (size = 32)
  IMGDOT OneDot;                 // Нижняя линия
  IMGDOT TwoDot;                 // Верхняя линия
}
  IMGDOUBLEDOT;


typedef struct IMGSHIFTLINEDOT   // (134) ПУНКТИР, СМЕЩЕННЫЙ ВПРАВО, ПО ЛИНИИ
{                                //      (size = 24)
  IMGLINE Line;                  // Нижняя линия
  IMGDOT  Dot ;                  // Верхняя линия-пунктир
}
  IMGSHIFTLINEDOT;


typedef struct IMGSQUARECROSS    // (136) ШТРИХОВАННАЯ ПЛОЩАДЬ
{                                //      (size = 16)
  unsigned int  Color;           // Цвет штpиховки (IMGC_TRANSPARENT - прозрачный)
  unsigned int  Kind ;           // Тип штpиховки (SC_HORIZONTAL, ...)
  unsigned int  Thick;           // Толщинa штpихa
  unsigned int  Blank;           // Толщинa пpобелa
}
  IMGSQUARECROSS;

typedef struct IMGMARK           // (137) ТОЧЕЧНЫЙ РАСТРОВЫЙ ОБ'ЕКТ
{                                //      (size = 16 + 128 = 144)
  unsigned int  Color    ;       // Цвет знака
  unsigned int  Size     ;       // Размер в микронах (для печати)
  unsigned int  PosV     ;       // Точка привязки знака PosV,PosH < Size
  unsigned int  PosH     ;       //  (для IMGSQUAREMARK - шаг сетки >= Size
  unsigned char Bits[128];       // Растровое изображение 32x32 с верхнего угла
}
  IMGMARK;

typedef struct IMGSQUAREMARK     // (138) ПЛОЩАДНОЙ + ТОЧЕЧНЫЙ
{                                //      (size = 8 + 144 = 152)
  unsigned int   Color   ;       // Цвет фона (IMGC_TRANSPARENT - прозрачный)
  unsigned short Kind    ;       // Тип сетки заполнения 4x4 (16 бит)
                                 //  (IMGK_REGULAR, ...)
  unsigned char  FullMark;       // Флаг заполнения
                                 //  0 - по всей площади
                                 //  1 - только полными знаками
  unsigned char  Weight  ;       // Вес знака, толщина контура   // 27/04/00
                                 //  0 - тонкий
                                 //  1 - нормальный
  IMGMARK        Mark    ;       // Описание знака
}
  IMGSQUAREMARK;



typedef struct IMGLEFTLINEDOT    // (145) ПУНКТИР, СМЕЩЕННЫЙ ВЛЕВО, ПО ЛИНИИ
{                                //      (size = 24)
  IMGLINE Line;                  // нижняя линия
  IMGDOT  Dot ;                  // верхняя линия-пунктир
}
  IMGLEFTLINEDOT;


typedef struct IMGDECHEAD        // ЗАГОЛОВОК ОФОРМЛЕННОГО ЛИНЕЙНОГО
{
  unsigned int  Length   ;       // Полная длина записи описания объекта
           int  Distance ;       // Расстояние между заполняющими векторн.
}
 IMGDECHEAD;
                                 // работа со структурой в классе
                                 //
typedef struct IMGDECORATELINE   // (146) ОФОРМЛЕННЫЙ ЛИНЕЙНЫЙ
{
  IMGDECHEAD     Head        ;   // Заголовок
  IMGDECOR       Base        ;   // Заголовок базовой функции
  unsigned int   BaseParm[1] ;   // Параметры базовой по типу
  IMGDECOR       First       ;   // Заголовок начального
  unsigned int   FirstParm[1];   // Параметры по типу
  IMGDECOR       End         ;   // Заголовок конечного
  unsigned int   EndParm[1]  ;   // Параметры по типу
  IMGDECOR       Left        ;   // Заголовок левого
  unsigned int   LeftParm[1] ;   // Параметры по типу
  IMGDECOR       Rigth       ;   // Заголовок правого
  unsigned int   RigthParm[1];   // Параметры по типу
  IMGDECOR       Middle1     ;   // Заголовок первого оформляющего
  unsigned int   Mid1Parm[1] ;   // Параметры по типу
  IMGDECOR       Middle2     ;   // Заголовок второго оформляющего
  unsigned int   Mid2Parm[1] ;   // Параметры по типу
}
  IMGDECORATELINE;

                                 // ( ) ОФОРМЛЕННЫЙ ЛИНЕЙНЫЙ
typedef IMGDECORATELINE SCRDECORATELINE;
                                 // Цвета векторных заданы индексами
                                 // Экранные параметры линий

typedef struct IMGHATCHSQUARE    // (153) ШТРИХОВАННЫЙ ПЛОЩАДНОЙ
{
       int Length;               // Полная длина записи описания объекта,
                                 // включая параметры линий штриховки
       int Angle ;               // Угол наклона штриховки в градусах (0,45,90,135)
                                 // против часовой стрелки от горизонтали
       int Step  ;               // Расстояние между осями линий штриховки
       int Number;               // Номер функции штриховой линии (IMGLINE или IMGDOT)
                                 // затем: параметры линии штриховки
} IMGHATCHSQUARE ;


typedef struct IMGSQUAREGLASSCOLOR // (164) ПЛОЩАДНОЙ ОБ'ЕКТ - ЦВЕТНОЕ СТЕКЛО
{                                  // (size = 12)
         int  Color;               // Цвет тонирования (IMGC_TRANSPARENT - бесцветный)
  signed int Bright;               // Процент изменения яркости (+/- 100)
  signed int Contrast;             // Процент изменения контраста (+/- 100)
}
  IMGSQUAREGLASSCOLOR;

#pragma pack(pop)

#endif  // MAPGDI_H

