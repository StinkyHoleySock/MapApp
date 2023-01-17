/******* QDMWINA.CPP *********** Dishlenko S.G.  ****** 05/06/15 ***
 ******* QDMWINA.CPP *********** Железняков В.   ****** 15/09/14 ***
 ******* QDMWINA.CPP *********** Savelov A.S.    ****** 07/05/15 ***
 ******* QDMWINA.CPP *********** Korjilov A.     ****** 20/11/14 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2017              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*                      FOR GIS DESIGNER                            *
*                                                                  *
********************************************************************
*                                                                  *
*              ПРОТОТИПЫ ФУНКЦИЙ ОБРАТНОГО ВЫЗОВА                  *
*              ОТРИСОВКИ ГРАФИЧЕСКИХ ПРИМИТИВОВ                    *
*                                                                  *
*******************************************************************/

#ifndef VISFUNCS_H
#define VISFUNCS_H

typedef struct DRAWFUNCPARM
{
    DRAWFUNCPARM()
    {
        memset(this, 0, sizeof(*this));
        Alpha = 0xff;
        MarkColor = -1;
    }

    unsigned char * Begin;  // Указатель на начало области DIB

    int Width;              // Ширина области DIB (в пикселях)
    int Height;             // Ширина области DIB (в пикселях)

    int DrawWidth;          // Ширина области рисования
    int DrawHeight;         // Высота области рисования

    int DrawPointX;         // Расположение отображаемой области в окне
    int DrawPointY;         // (левый верхний угол)

    float Dx, Dy;           // Смещение и поворот
    float Angle;            // области рисования

    int Color;              // Цвет рисования (пера, кисти, подписи)
    int Alpha;              // Альфа-канал (в младшем байте)

    int MarkColor;          // Цвет тени (оконтуривания)
                            // (-1 - не рисуем)
    int MarkType;           // Тип подсветки
                            // (0 - тень, 1,2 - тонкий и толстый контур прочистки)
    int MarkThick;          // Смещение тени или толщина контура прочистки

    HWIDGET hWidget;        // Виджет рисования
}
DRAWFUNCPARM;

// Сплошная линия
typedef void (*DRAWFUNC_SOLIDLINE)(DRAWFUNCPARM& dfp, const POLYDATAEX *data, float thick);

// Штриховая линия
typedef void (*DRAWFUNC_DASHLINE)(DRAWFUNCPARM& dfp, const POLYDATAEX *data, float thick, float dash, float space);

// Площадной объект
typedef void (*DRAWFUNC_POLYGON)(DRAWFUNCPARM& dfp, const POLYDATAEX *data);

// Запросить ширину текста
typedef int (*DRAWFUNC_TEXTWIDTH)(DRAWFUNCPARM& dfp, const WCHAR *text,
                                  float height, int italic, int weight, int wide, const char *fontName);

// Подпись
typedef void (*DRAWFUNC_TEXT)(DRAWFUNCPARM& dfp, const WCHAR *text,
                              float height, int italic, int weight, int wide, const char *fontName);

// Эллипс
typedef void (*DRAWFUNC_ELLIPSE)(DRAWFUNCPARM& dfp, int x, int y, int width, int height,
                                 float thick, float dash, float space,
                                 int imagetype);

// Дуга
typedef void (*DRAWFUNC_ARC)(DRAWFUNCPARM& dfp, int x, int y, int width, int height,
                             int startAngle, int spanAngle,
                             float thick, float dash, float space,
                             int imagetype);

// Сектор
typedef void (*DRAWFUNC_PIE)(DRAWFUNCPARM& dfp, int x, int y, int width, int height,
                             int startAngle, int spanAngle);

// Сохранение изображения в файл
typedef void (*DRAWFUNC_SAVEIMAGE)(const char *filename, const char *image, int w, int h);

// Сохранение изображения заполненного указанным цветом прямоугольника  в файл
typedef void (*DRAWFUNC_SAVEFILLRECTANGLE)(const char *filename, int w, int h, COLORREF color);

// Запрос параметров шрифта для правильного расчета отображения подписей
typedef int (*DRAWFUNC_FONTMETRICS)(const char *fontName, int height, int italic,
                                    unsigned long* ascent, unsigned long* descent, unsigned long* resultHeight);

// Рисование в пямяти картинки
typedef void (*DRAWFUNC_PICTURE)(DRAWFUNCPARM& dfp, IMGPICTURE *imgpict, const char *fileaddr, int picturesize);

extern "C"
{
// Установить виджет рисования      // 14/11/14
_MAPIMP void _MAPAPI mapSetHWidget(HMAP hMap, HWIDGET hwidget);

// Установить функции рисования:
// Сплошная линия
_MAPIMP void _MAPAPI mapDrawSolidLine(DRAWFUNC_SOLIDLINE func);

// Штриховая линия
_MAPIMP void _MAPAPI mapDrawDashLine(DRAWFUNC_DASHLINE func);

// Площадной объект
_MAPIMP void _MAPAPI mapDrawPolygon(DRAWFUNC_POLYGON func);

// Подпись
_MAPIMP void _MAPAPI mapDrawText(DRAWFUNC_TEXT func);

// 28/10/14 Савелов
// Эллипс
_MAPIMP void _MAPAPI mapDrawEllipse(DRAWFUNC_ELLIPSE func);

// Дуга
_MAPIMP void _MAPAPI mapDrawArc(DRAWFUNC_ARC func);

// Сектор
_MAPIMP void _MAPAPI mapDrawPie(DRAWFUNC_PIE func);

// Параметры шрифта            // 05/04/16
_MAPIMP void _MAPAPI mapDrawFontMetrics(DRAWFUNC_FONTMETRICS call);

// Картинка                    // 27/06/16
_MAPIMP void _MAPAPI mapDrawPicture(DRAWFUNC_PICTURE call);
}

#endif // VISFUNCS_H
