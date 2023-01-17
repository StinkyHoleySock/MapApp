/******* MAPRSCEX.H  *********** Panteleeva N.A. ****** 14/04/16 ***
******** MAPRSCEX.H  *********** Kruzhkov A.E.   ****** 23/04/06 ***
******** MAPRSCEX.H  *********** Belenkov O.V.   ****** 13/07/15 ***
******** MAPRSCEX.H  *********** Dishlenko S.G.  ****** 04/04/12 ***
******** MAPRSCEX.H  *********** Shabakov D.A.   ****** 09/12/13 ***
******** MAPRSCEX.H  *********** Tarakanova I.R. ****** 04/02/14 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*       ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К СЕРИИ ОБ'ЕКТОВ                  *
*       ДОПОЛНИТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ КЛАССИФИКАТОРА                  *
*       ДЕРЕВО ДАННЫХ                                              *
*        ИНТЕРФЕЙС ДЛЯ ПРОГРАММ НА ЯЗЫКАХ C, PASCAL, BASIC         *
*                                                                  *
********************************************************************
*                                                                  *
*                 ИМПОРТИРОВАНИЕ ФУНКЦИЙ :                         *
*                                                                  *
*    // Загрузка библиотеки                                        *
*    HINSTANCE libInst = ::LoadLibrary("gisacces.dll");            *
*                                                                  *
*    // Вызов функции                                              *
*    long int (WINAPI * lpfn) (HRSC hRsc, long int code,           *
*                                        long int local);          *
*    (FARPROC)lpfn = GetProcAddress(libInst, "mapSeriaDelete");    *
*    long int retcode = (*lpfn)(hRsc, code, local);                *
*                                                                  *
*    // Выгрузка библиотеки                                        *
*    ::FreeLibrary(libInst);                                       *
*                                                                  *
*******************************************************************/

#ifndef MAPRSCEX_H
#define MAPRSCEX_H

#ifndef MAPMACRO_H
 #include "mapmacro.h"
#endif

#ifndef RSCAPI_H
   #include "rscapi.h"
#endif

typedef struct ELEMTREEE
{
   int  Ident;           // Идентификатор объекта (или 0, у слоя)                  // 04/04/12
   int  Depth;           // Глубина вложенности в дерево
   int  LayerNumber;     // Номер слоя
   int  ParentNumber;    // Номер родителя в общем массиве элементов
   int  PictureIndex;    // Индекс в массиве изображений
   char Code[28];        // Форматированное представление кода(как сортировали)
                         // с указанием принадлежности к серии и локализации
}
  ELEMTREEE;


 typedef struct SERIALIMIT          // Интервалы значений семантики,соответст-
                                    // вующие определенным видам объектов
  {
    int    Code        ;            // Код семантики                              // 04/04/12
    int    Count       ;            // Количество ограничителей (>=2)
    double   Value[256];            // Значения ограничителей
  }
   SERIALIMIT;


 typedef struct SERIATYPE           // Справочные данные о серии
  {
    unsigned int Excode      ; // Классификационный код об'ектов серии            // 04/04/12
    int           Local      ; // Характер локализации  об'ектов серии
    int           Count      ; // Количество объектов серии
    int           FirstCode  ; // Код первой семантики
    int           FirstCount ; // Количество ограничителей по 1 семантике
    int           SecondCode ; // Код второй семантики
    int           SecondCount; // Количество ограничителей по 2 семантике
#if defined(_M_X64) || defined(BUILD_DLL64)
    int           SeriaTypeZero; // Выравнивание                                // 04/12/13
#endif
  }
   SERIATYPE;

 typedef struct SERIAPLACE          // Место объекта в серии
  {
    int FirstCode   ;          // Код семантики                                    // 04/04/12
    int FirstNumber ;          // Номер ограничителя (с 1)
    int SecondCode  ;          // Код семантики ,если серия по одной
                               // семантике,то равен нулю
    int SecondNumber;          // Номер ограничителя (с 1)
  }
   SERIAPLACE;

// Для создания серии объектов необходимо:
// - создать серию по описанию первого объекта и интервалов значений семантик
//   ( mapSeriaCreate );
// - добавить остальные объекты в созданную серию с указанием их места
//   ( mapSeriaAppendObject );
// - при необходимости назначить места объектов в серии
//   ( mapSeriaSetObject );
// - задать вид (по внутреннему коду для каждого объекта серии)
//   ( mapSetRscObjectImage см. RSCAPI.H);

extern "C"
{

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К СЕРИИ ОБ'ЕКТОВ    ++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 // Создать серию
 // hRsc - идентификатор классификатора карты
 // RSCOBJECT -  описание объекта (см. maptype.h)
 // first, second - интервалы значений семантик серии
 // Если серия по одной семантике, second = 0
 // При ошибке возвращает 0, иначе - внутренний код объекта

_MAPIMP long int _MAPAPI mapSeriaCreate(HRSC hRsc, SERIALIMIT * first,
                                       SERIALIMIT * second, RSCOBJECT * object);

 // Обновить серию
 // hRsc - идентификатор классификатора карты
 // code - классификационный код объектов входящих в cерию
 // local - локализация объектов серии ( LOCAL_POINT, ...)
 // first, second - интервалы значений семантик серии
 // Если серия по одной семантике, second = 0
 // Если серии с таким кодом и локализацией нет , она создается
 // Если такой объект или серия уже была - по возможности согласуются
 // с новой(у новой приоритет), при сильных расхождениях - старая серия
 // удаляется, новая создается - все объекты сохраняются
 // Если серия по одной семантике second = 0
 // При ошибке возвращает 0, иначе - внутренний код объекта

_MAPIMP long int _MAPAPI mapSeriaUpdate(HRSC hRsc, long int code,
                                 long int local, SERIALIMIT * first,
                                 SERIALIMIT * second);


 // Добавить объект в существующую серию
 // hRsc - идентификатор классификатора карты
 // RSCOBJECT -  описание объекта (см. maptype.h)
 // SERIAPLACE - место объекта в серии
 // При ошибке возвращает 0, иначе - внутренний код объекта

_MAPIMP long int _MAPAPI mapSeriaAppendObject(HRSC hRsc, RSCOBJECT * object,
                                                 SERIAPLACE * place);

 // Удалить серию
 // hRsc - идентификатор классификатора карты
 // code - классификационный код объектов входящих в cерию
 // local - локализация объектов серии ( LOCAL_POINT, ...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSeriaDelete (HRSC hRsc, long int code, long int local);

 // Запросить информацию по серии
 // hRsc - идентификатор классификатора карты
 // code - классификационный код объектов входящих в cерию
 // local - локализация объектов серии ( LOCAL_POINT, ...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSeriaDescribe(HRSC hRsc, long int code,long int local,
                                          SERIATYPE * seria);

 // Запросить информацию по значениям ограничителей семантики серии
 // hRsc - идентификатор классификатора карты
 // incode - внутренний код ( индекс ) любого объекта серии
 // limit->Code - код запрашиваемой семантики
 // При ошибке возвращает ноль, иначе номер семантики в серии (1 или 2)
 // по адресу limit - должно быть памяти  sizeof(SERIALIMIT)

_MAPIMP long int _MAPAPI mapSeriaDescribeLimit(HRSC hRsc, long int incode,
                                               SERIALIMIT * limit);

  // Назначить место объекта в серии (объект может повторяться)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код ( индекс ) объекта
  // При ошибке возвращает ноль , иначе - внутренний код объекта

_MAPIMP long int _MAPAPI mapSeriaSetObject(HRSC hRsc, long int incode,
                                           SERIAPLACE * place);

  // Запросить внутренний код объекта по месту в серии
  // hRsc - идентификатор классификатора карты
  // code - классификационный код объектов входящих в cерию
  // local - локализация объектов серии ( LOCAL_POINT, ...)
  // При ошибке возвращает ноль , иначе - внутренний код объекта

_MAPIMP long int _MAPAPI mapSeriaGetObject(HRSC hRsc, long int code,long int local,
                                           SERIAPLACE * place);

  // назначить семантику всем объектам серии (не видовую)      //10/12/01
  // hRsc - идентификатор классификатора карты
  // code - классификационный код объектов входящих в cерию
  // local - локализация объектов серии ( LOCAL_POINT, ...)
  // semanticcode - код семантики
  // importance значимость семантики  см. maptype.h
  // (POSSIBLE_SEMANTIC или MUST_SEMANTIC) )

_MAPIMP long int _MAPAPI mapSeriaSetObjectSemantic(HRSC hRsc, long int code,long int local,
                                                   long int semanticcode, long int importance);

 // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // +++++ ОПИСАНИЕ ДОПОЛНИТЕЛЬНЫХ ФУНКЦИЙ ДОСТУПА К ПАЛИТРЕ    ++++++
 // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Добавить палитру                                                // 27/11/01
  // hRsc - идентификатор классификатора карты
  // palette - цвета палитры,count - количество цветов в палитре (16,32,64,256)
  // name    - название палитры
  // При ошибке возвращает ноль иначе 1

_MAPIMP long int _MAPAPI mapRscAppendPaletteUn(HRSC hRsc, COLORREF * palette,
                                        long int count,WCHAR *wname);

_MAPIMP long int _MAPAPI mapRscAppendPalette(HRSC hRsc, COLORREF * palette,
                                             long int count,const char *name);

  // Установить количество цветов в классификаторе            // 27/11/01
  // hRsc - идентификатор классификатора карты
  // count - количество цветов в палитре (16,32,64,256)
  // При ошибке возвращает ноль иначе 1

_MAPIMP long int _MAPAPI mapRscSetColorCount(HRSC hRsc,long int count);

 // Запросить палитру классификатора по порядковому номеру (с 1)    // 27/11/01
 // hRsc - идентификатор классификатора карты
 // number - порядковый номер палитры
 // Возвращает указатель на палитру
 // При ошибке возвращает ноль

_MAPIMP COLORREF * _MAPAPI mapRscGetPalette(HRSC hRsc,long int number);

 // Проверить использование данного цвета в параметрах объекта     // 22/03/16
 // hRsc - идентификатор классификатора карты
 // color -  младший байт содержит номер индекса палитры карты (c 0 до 255),
 //          старший байт равен 0x0F (признак указания индексного цвета)
 // function - номер функции отображения (mapgdi.h)
 // param - адрес параметров
 // Возвращает: 1 - используется, 0 - не используется 

 _MAPIMP long int _MAPAPI mapRscCheckParamColor(HRSC hRsc, COLORREF color, long int function,
                                           const char * param);


 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // ++++  ОПИСАНИЕ ДОПОЛНИТЕЛЬНЫХ ФУНКЦИЙ ДОСТУПА К БИБЛИОТЕКАМ  +++
 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 // Запросить количество библиотек пользователя                 // 16/10/04
 // hRsc - идентификатор классификатора карты
 // При ошибке или отсутствии подключенных библиотек возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscImlCount(HRSC hRsc);

 // Запросить порядковый номер библиотеки пользователя(с 1)       // 30/01/02
 // hRsc - идентификатор классификатора карты
 // соde - код библиотеки
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscImlOrder(HRSC hRsc,long code);

 // Запрос индекса библиотеки (с 1) по порядковому номеру
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscImlIndex(HRSC hRsc,int number);  // 23/04/06

 // Запрос кода по порядковому номеру библиотеки (c 1)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscImlCode(HRSC hRsc,int number);   // 23/04/06

 // Запросить имя библиотеки по порядковому номеру (c 1)
 // hRsc - идентификатор классификатора карты
 // При ошибке возвращает пустую строку

_MAPIMP long int _MAPAPI mapGetRscImlNameDLLUn(HRSC hRsc,long int number,
                                               WCHAR * wname,long int size);
_MAPIMP const char * _MAPAPI mapGetRscImlNameDLL(HRSC hRsc,long int number);

 // Запросить условное имя библиотеки по порядковому номеру (c 1)
 // hRsc - идентификатор классификатора карты
 // При ошибке возвращает пустую строку

_MAPIMP long int _MAPAPI mapGetRscImlNameUn(HRSC hRsc,long int number,
                                     WCHAR * wname,long int size);
_MAPIMP char * _MAPAPI mapGetRscImlName(HRSC hRsc,long number);

 // Добавить библиотеку
 // hRsc - идентификатор классификатора карты
 // nameDll - имя библиотеки
 // nameUser - условное имя
 // Возвращает код библиотеки
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapAppendRscImlUn(HRSC hRsc, WCHAR * wnameDll, WCHAR * wnameUser);
_MAPIMP long int _MAPAPI mapAppendRscIml(HRSC hRsc, char * nameDll,char * nameUser);


 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // +++++ ОПИСАНИЕ ФУНКЦИЙ ПОСТРОЕНИЯ ДЕРЕВА ОБЪЕКТОВ КЛАССИФИКАТОРА  ++++
 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 // Для использования дерева объектов необходимо:
 // Запросить максимальный размер
 // Выделить память под размещение нужного количества структур ELEMTREEE,
 // Вызвать функцию построения дерева
 // Заполнять элементы по порядку. Для слоя incode равно 0
 // У каждого элемента указан номер элемента - родителя

  // Посчитать максимальное число элементов дерева объектов
_MAPIMP long int _MAPAPI mapCountRscTreeObject(HRSC hRsc);

  // Выгрузить дерево построенное по Rsc и фильтру в выделенную область памяти
  // hRsc - идентификатор классификатора карты
  // hSelect - идентификатор фильтра (может быть 0)
  // size    - размер выделенной области для размещения дерева(в байтах)
  // elemtree - указатель на выделенную область
  // Возвращает точное количество элементов дерева
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapBuildRscTree(HRSC hRsc, HSELECT hSelect, long size,
                               ELEMTREEE * elemtree);


 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // +++++ РЕДАКТИРОВАНИЕ ПАРАМЕТРОВ ФУНКЦИИ ОТОБРАЖЕНИЯ ОБЪЕКТОВ      ++++
 // +++++                 НАБОР ПРИМИТИВОВ DRAW                       ++++
 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Создать идентификатор параметров набора примитивов        // 24/01/08
  // HDRW - maptype.h
  // Выделяет память для работы с набором примитивов
  // После использования освободить с помощью функции mapFreeDraw()
  // hDrw - идентификатор набора примитивов

_MAPIMP HDRAW _MAPAPI mapCreateDraw();

  // Освободить параметры набора примитивов
  // hDrw - идентификатор набора примитивов
  // Освобождает память для работы с набором примитивов
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapFreeDraw(HDRAW hDrw);

  // Загрузить параметры в набор примитивов
  // hDrw - идентификатор набора примитивов
  // param - память, где находятся параметры
  // number - номер функции типа IMG_XXXXXXX (см. MAPGDI.H)
  // length - длина загружаемых параметров
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCopyMemoryToDraw(HDRAW hDrw, long int number,
                                             const char * param);

  // Выгрузить набор примитивов в память
  // hDrw - идентификатор набора примитивов
  // buffer - адрес памяти куда выгружаются параметры
  // length - размер памяти для выгрузки не менее mapDrawLength()
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCopyDrawToMemory(HDRAW hDrw, char * buffer,
                                             long int length);

  // Запросить общую длину набора примитивов
  // hDrw - идентификатор набора примитивов
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapAllDrawLength(HDRAW hDrw);

  // Запросить количество элементов набора примитивов
  // hDrw - идентификатор набора примитивов
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapRscDrawCount(HDRAW hDrw);       // 17/09/09

  // Запросить вид элемента набора примитивов
  // по его номеру ( от 1 до mapDrawCount())
  // hDrw - идентификатор набора примитивов
  // Возвращает номер функции типа IMG_XXXXXXX (см. MAPGDI.H)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDrawElementImage(HDRAW hDrw, long int number);

  // Запросить адрес параметров элемента набора примитивов
  // по его номеру ( от 1 до mapDrawCount())
  // hDrw - идентификатор набора примитивов
  // Возвращает адрес структуры типа IMGXXXXXX, в соответствии
  // с видом элемента (см. MAPGDI.H)
  // При ошибке возвращает 0

_MAPIMP char * _MAPAPI mapDrawElementParameters(HDRAW hDrw, long int number);

  // Запросить длину параметров элемента набора примитивов
  // по его номеру ( от 1 до mapDrawCount())
  // hDrw - идентификатор набора примитивов
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDrawElementLength(HDRAW hDrw, long int number);

  // Добавить элемент в набор примитивов
  // hDrw - идентификатор набора примитивов
  // image - вид элемента (см. MAPGDI.H)
  // param - адрес параметров элемента
  // При ошибке возвращает ноль,иначе - число элементов в записи

_MAPIMP long int _MAPAPI mapAppendElementDraw(HDRAW hDrw, long int image,
                                              const char * parm);

  // Добавить элемент в набор примитивов c инициализацией параметров // 09/02/16
  // hDrw - идентификатор набора примитивов
  // image - вид элемента (см. MAPGDI.H)
  // При ошибке возвращает ноль,иначе - число элементов в записи

_MAPIMP long int _MAPAPI mapAppendElementDrawWithInit(HDRAW hDrw, long int image);

  // Заменить примитив                                               // 09/02/16
  // hDrw - идентификатор набора примитивов
  // number - номер заменяемого примитива ( от 1 до mapDrawCount())
  // image - вид элемента (см. MAPGDI.H)
  // param - адрес параметров элемента
  // При ошибке возвращает ноль,иначе номер примитива в записи

_MAPIMP long int _MAPAPI mapReplaceElementDraw(HDRAW hDrw, long int number,
                             long int image, const char * parm);

  // Заменить примитив  по номеру  c инициализацией параметров     13/04/16
  // hDrw - идентификатор набора примитивов
  // number - номер заменяемого примитива ( от 1 до mapDrawCount())
  // image - вид элемента (см. MAPGDI.H)
  // При ошибке возвращает ноль,иначе номер примитива в записи

_MAPIMP long int _MAPAPI mapReplaceElementWithInitDraw(HDRAW hDrw, long int number,
                             long int image);


  // Передвинуть примитив                                           // 09/02/16
  // hDrw - идентификатор набора примитивов
  // oldnumber - место откуда взять примитив ( от 1 до mapDrawCount())
  // newnumber - место куда положить примитив ( от 1 до mapDrawCount())
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapMoveElementDraw(HDRAW hDrw, long int oldnumber,
                             long int newnumber);

  // Удалить все элементы графического описания объекта
  // hDrw - идентификатор набора примитивов
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscClearDraw(HDRAW hDrw);                // 17/09/09

  // Удалить элемент графического описания объекта
  // hDrw - идентификатор набора примитивов
  // number - номер элемента (начиная с 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteElementDraw(HDRAW hDrw, long int number);

  // Добавить элементом графического описания объекта точечный знак IMGMULTIMARK
  // созданный по ВМР - файлу
  // name - имя файла
  // В BMP не более 256 цветов (не сжатое)
  // Размеры более 32*32 - обрезаются
  // При ошибке возвращает ноль                  // 31/01/08
_MAPIMP long int _MAPAPI mapAppendBMPtoDraw(HDRAW hDrw, const char * name);

 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // +++++                 Дополнительные функции                      ++++
 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Запрос индекса верхнeй границы видимости по внутреннему коду   // 14/04/16
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код
  // Возвращает индекс от 0 до 15

_MAPIMP long int _MAPAPI mapGetRscObjectTop(HRSC hRsc,long int incode);

  // Запрос индекса нижней границы видимости по внутреннему коду    // 14/04/16
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код 
  // Возвращает индекс от 0 до 15

_MAPIMP long int _MAPAPI mapGetRscObjectBottom(HRSC hRsc,long int incode);



} // extern "C"

#endif
