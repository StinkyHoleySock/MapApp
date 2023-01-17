/****  ALSAPI.H  **************  Belenkov  O.V.  *****  09/06/14 ***
*****  ALSAPI.H  **************  Derevyagina Zh.A.****  29/11/10 ***
*****  ALSAPI.H  **************  Shabakov D.A.   *****  14/05/13 ***
*****  ALSAPI.H  **************  Tarakanova I.R. *****  08/04/14 ***
*****  ALSAPI.H  **************  Kruzhkova E.A.  *****  02/12/15 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2015              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*                FOR  WINDOWS ME & WINDOWS XP, LINUX               *
*                                                                  *
********************************************************************
*                                                                  *
*               СПИСОК РАЙОНОВ ЭЛЕКТРОННОЙ КАРТЫ                   *
*                                                                  *
*******************************************************************/

#ifndef ALSAPI_H
#define ALSAPI_H

#ifndef MAPTYPE_H
 #include "maptype.h"
#endif

typedef struct ALSITEM     // ОПИСАНИЕ ЭЛЕМЕНТА СПИСКА РАЙОНОВ РАБОТ
{
  long Ident;              // Идентификатор элемента
  char Path[260];          // Полный путь к паспорту района
  char Name[32];           // Имя района (берется из паспорта)
  long Scale;              // Масштаб района (берется из паспорта)
  long Minimum;            // Нижняя граница перехода к району (например, 100 000)
  long Maximum;            // Верхняя граница перехода к району (например, 500 000)
  long Priority;           // Приоритет выбора для одинаковых масштабов
}
  ALSITEM;

typedef struct ALSITEMEX   // ОПИСАНИЕ ЭЛЕМЕНТА СПИСКА РАЙОНОВ РАБОТ
{                                                                     // 21/11/06
  long Ident;              // Идентификатор элемента
  long Scale;              // Масштаб района (берется из паспорта)
  long Minimum;            // Нижняя граница перехода к району (например, 100 000)
  long Maximum;            // Верхняя граница перехода к району (например, 500 000)
  long Priority;           // Приоритет выбора для одинаковых масштабов
  long Reserve;            // Резерв = 0
}
  ALSITEMEX;

typedef HMAP HALS;         // Идентификатор списка районов работ

// Список служебных семантик, описывающих район работ (элемент атласа)
#define SEMALSPATH     7701  // Путь к карте
#define SEMALSNAME     7702  // Имя района работ
#define SEMALSSCALE    7703  // Масштаб района работ
#define SEMALSMIN      7704  // Нижняя граница перехода
#define SEMALSMAX      7705  // Верхняя граница перехода
#define SEMALSMODE     7706  // Номер состояния данных
#define SEMALSPRIOR    7707  // Приоритет выбора
#define SEMALSGROUP    7708  // Имя группы данных атласа     // 28/10/09
#define SEMALSNOTE     7709  // Примечание                   // 27/11/09
#define SEMALSORDER    7710  // Порядок следования                   // 05/04/10
#define SEMALSPATHTILE 7711  // Путь к паспорту растризованной карты // 05/04/10
#define SEMALSCOUNT    7712  // Число элементов в группе (SEMGROUPLEADER + SEMGROUPSLAVE) // 31/10/11
#define SEMALSSHEET    7713  // Имя/Номенклатура листа       // 31/10/11

#define ALS_UP          1  // Поиск более мелкого масштаба
#define ALS_DOWN        2  // Поиск более крупного масштаба
#define ALS_ANY         0  // Поиск любой карты
#define ALS_BACK        4  // Вернуться к карте

extern "C"
{

// Открыть список районов работ
// listname - путь к списку районов работ (файлу "ALS")
// При ошибке возвращает ноль, иначе - идентификатор списка районов

_MAPIMP HALS _MAPAPI alsOpenAreaList(const char * listname);
_MAPIMP HALS _MAPAPI alsOpenAreaListUn(const WCHAR * listname);

// Создать список районов работ
// listname - путь к списку районов работ (файлу "ALS")
// listtype - тип списка районов (0 - глобальный с поддержкой геодезических
//            координат, 1 - локальный в местной системе координат
// rscname -  имя классификатора для создания атласа (штатно - "arealist.rsc")
// При ошибке возвращает ноль, иначе - идентификатор списка районов

_MAPIMP HALS _MAPAPI alsCreateAreaList(const char * listname, long int listtype);
_MAPIMP HALS _MAPAPI alsCreateAreaListEx(const char * listname,
                                         const char * rscname, long int listtype);
_MAPIMP HALS _MAPAPI alsCreateAreaListUn(const WCHAR * listname,
                                         const WCHAR * rscname, long int listtype);

// Закрыть список районов работ
// hals - идентификатор списка районов

_MAPIMP void _MAPAPI alsCloseAreaList(HALS hals);

// Запросить является ли атлас глобальным
// hals - идентификатор списка районов (атласа)
// Если нет - возвращает ноль

_MAPIMP long int _MAPAPI alsIsGlobalAreaList(HALS hals);

// Запросить число элементов в списке
// hals - идентификатор списка районов

_MAPIMP long int _MAPAPI alsGetItemCount(HALS hals);

// Запросить описание элемента списка районов
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// item   - заполняемая структура описания элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItem(HALS hals, long int number, ALSITEM * item);

// Запросить описание элемента списка районов
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// item   - заполняемая структура описания элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemEx(HALS hals, long int number, ALSITEMEX * item);

// Запросить описание элемента списка районов с именами в Unicode
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// itemex - заполняемая структура описания элемента
// path   - заполняемый путь к паспорта элемента в Unicode
// sizepath - размер памяти под путь (в байтах)
// name   - заполняемое имя района элемента в Unicode
// sizename - размер памяти под имя района элемента (в байтах)
_MAPIMP long int _MAPAPI alsGetItemExUn(HALS hals, long int number, ALSITEMEX * itemex,
                                        WCHAR * path, long int sizepath,
                                        WCHAR * name, long int sizename);       // 02/12/15

// Запросить имя элемента списка районов
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// name   - адрес памяти для записи результата
// size   - длина области памяти в байтах (не менее MAX_PATH байт)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemName(HALS hals, long int number, char * name, long int size);
_MAPIMP long int _MAPAPI alsGetItemNameUn(HALS hals, long int number, WCHAR * name,
                                          long int size);                       // 02/12/15

// Запросить путь к паспорту для элемента из списка районов
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// path   - адрес памяти для записи результата
// size   - длина области памяти в байтах (не менее MAX_PATH)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemPath(HALS hals, long int number, char * path,
                                        long int size);                         // 02/12/15

// Запросить путь к паспорту для элемента из списка районов
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// path   - адрес памяти для записи результата
// size   - длина области памяти в байтах (не менее MAX_PATH_LONG)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemPathUn(HALS hals, long int number, WCHAR * path,
                                          long int size);                       // 02/12/15

// Запросить путь к паспорту данных для WebServer по идентификатору элемента
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// pathtile   - адрес памяти для записи результата
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetPathTileByIdent(HALS hals, long int ident, char * pathtile);
_MAPIMP long int _MAPAPI alsGetPathTileByIdentUn(HALS hals, long int ident,
                                                 WCHAR * pathtile, long int size); // 02/12/15

// Запросить путь к паспорту данных для WebServer по номеру
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// pathtile   - адрес памяти для записи результата(размер буфера должен быть
//              не менее MAX_PATH_LONG)
// При ошибке возвращает ноль
_MAPIMP long int _MAPAPI alsGetPathTileUn(HALS hals, long int number,
                                          WCHAR * pathtile, long int size);     // 02/12/15

// Запросить путь к паспорту данных для WebServer по номеру
// hals   - идентификатор списка районов
// number - порядковый номер элемента, начиная с 1
// pathtile   - адрес памяти для записи результата(размер буфера должен быть
//              не менее MAX_PATH)
// При ошибке возвращает ноль
_MAPIMP long int _MAPAPI alsGetPathTile(HALS hals, long int number,
                                        char * pathtile);                       // 02/12/15

// Установить путь к паспорту данных для WebServer
// hals     - идентификатор списка районов
// ident    - идентификатор элемента, см. ALSITEM
// pathtile     - дополнительная информация (примечание)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemPathTile(HALS hals, long int ident, char * pathtile);
_MAPIMP long int _MAPAPI alsSetItemPathTileUn(HALS hals, long int ident,
                                              WCHAR * pathtile, long int size);  // 02/12/15

// Запросить описание элемента списка районов
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// item   - заполняемая структура описания элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemByIdent(HALS hals, long int ident, ALSITEM * item);

// Запросить описание элемента списка районов с именами в Unicode
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// itemex - заполняемая структура описания элемента
// path   - заполняемый путь к паспорта элемента в Unicode
// sizepath - размер памяти под путь (в байтах)
// name   - заполняемое имя района элемента в Unicode
// sizename - размер памяти под имя района элемента (в байтах)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemByIdentExUn(HALS hals, long int ident,
                                               ALSITEMEX * itemex,
                                               WCHAR * path, long int sizepath,
                                               WCHAR * name, long int sizename); // 02/12/15

// Запросить имя группы данных списка районов
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// name   - адрес памяти для записи результата (размер памяти должен быть
//          не менее MAX_PATH)                                                  
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetNameGroupByIdent(HALS hals, long int ident, char * name);
_MAPIMP long int _MAPAPI alsGetNameGroupByIdentUn(HALS hals, long int ident,
                                                  WCHAR * name, long int size); // 02/12/15

// Установить имя группы данных атласа
// hals     - идентификатор списка районов
// ident    - идентификатор элемента, см. ALSITEM
// name     - имя группы данных атласа
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemGroupName(HALS hals, long int ident, char * name);
_MAPIMP long int _MAPAPI alsSetItemGroupNameUn(HALS hals, long int ident,
                                               WCHAR * name, long int size);    // 02/12/15

// Запросить дополнительную информацию (примечание) для элемента списка районов
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// note   - адрес памяти для записи результата (размер буфера должен быть
//          не менее MAX_PATH)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetNoteByIdent(HALS hals, long int ident, char * note);
_MAPIMP long int _MAPAPI alsGetNoteByIdentUn(HALS hals, long int ident,
                                             WCHAR * note, long int size);      // 02/12/15

// Установить дополнительную информацию (примечание) для элемента списка районов
// hals     - идентификатор списка районов
// ident    - идентификатор элемента, см. ALSITEM
// note     - дополнительная информация (примечание)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemNote(HALS hals, long int ident, char * note);
_MAPIMP long int _MAPAPI alsSetItemNoteUn(HALS hals, long int ident,
                                          WCHAR * note, long int size);         // 02/12/15

// Запросить порядок отображения в дереве для элемента списка районов
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// order   - адрес памяти для записи результата
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetOrderByIdent(HALS hals, long int ident, long int * order);

// Установить порядок отображения в дереве для элемента списка районов
// hals     - идентификатор списка районов
// ident    - идентификатор элемента, см. ALSITEM
// order    - порядок отображения в дереве
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemOrder(HALS hals, long int ident, long int order);

// Запросить описание элемента списка районов
// hals   - идентификатор списка районов
// ident  - идентификатор элемента
// item   - заполняемая структура описания элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemByIdentEx(HALS hals, long int ident, ALSITEMEX * item);

// Добавить элемент в список районов
// hals - идентификатор списка районов
// path - полный путь к паспорту района работ
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsAppendItem(HALS hals, const char * path);
_MAPIMP long int _MAPAPI alsAppendItemUn(HALS hals, const WCHAR * path);  // 02/12/15

// Установить границы масштабов для автоматического перехода в район
// hals    - идентификатор списка районов
// ident   - идентификатор элемента, см. ALSITEM
// minimum - нижняя граница перехода к району (например, 100 000)
// maximum - верхняя граница перехода к району (например, 500 000)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemMiniMax(HALS hals, long int ident,
                                           long int minimum,
                                           long int maximum);

// Установить приоритет выбора карты
// hals     - идентификатор списка районов
// ident    - идентификатор элемента, см. ALSITEM
// priority - приоритет выбора района
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsSetItemPriority(HALS hals, long int ident,
                                            long int priority);

// Установить границы масштабов для автоматического перехода в район
// hals   - идентификатор списка районов
// ident   - идентификатор элемента, см. ALSITEM
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsDeleteItem(HALS hals, long int ident);


// Найти элемент расположенный в заданной точке ниже указанного
// элемента (в более крупном масштабе)
// hals   - идентификатор списка районов
// ident   - идентификатор элемента, см. ALSITEM
// point  - геодезические координаты заданной точки в радианах
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemDown(HALS hals, long int ident,
                                       DOUBLEPOINT * point, ALSITEM * item);

// Найти элемент расположенный в заданной точке ниже заданного
// элемента (в более крупном масштабе)
// hals   - идентификатор списка районов
// scale  - масштаб текущего элемента
// point  - геодезические координаты заданной точки в радианах
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemDownByScale(HALS hals, long int scale,
                                              DOUBLEPOINT * point, ALSITEM * item);

// Найти элемент расположенный в заданной точке ниже заданного
// элемента (в более крупном масштабе)
// hals   - идентификатор списка районов
// scale  - масштаб текущего элемента
// frame  - координаты заданной области на атласе
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemDownByScaleEx(HALS hals, long int scale,
                                                HOBJ frame, ALSITEM * target);

// Найти элемент расположенный в заданной точке выше указанного
// элемента (в более мелком масштабе)
// hals   - идентификатор списка районов
// ident   - идентификатор элемента, см. ALSITEM
// point  - геодезические координаты заданной точки в радианах
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemUp(HALS hals, long int ident,
                                     DOUBLEPOINT * point, ALSITEM * item);

// Найти элемент расположенный в заданной точке выше указанного
// элемента (в более мелком масштабе)
// hals   - идентификатор списка районов
// scale  - масштаб текущего элемента
// point  - геодезические координаты заданной точки в радианах
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemUpByScale(HALS hals, long int scale,
                                            DOUBLEPOINT * point, ALSITEM * item);

// Найти элемент расположенный в заданной точке выше заданного
// элемента (в более мелком масштабе)
// hals   - идентификатор списка районов
// scale  - масштаб текущего элемента
// frame  - геодезические координаты заданной области на атласе
// item   - заполняемая структура описания найденного элемента
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGoItemUpByScaleEx(HALS hals, long int scale,
                                              HOBJ frame, ALSITEM * target);

// Установить масштаб отображения с учетом размещения изображения
// данных в пределах заданной области
// width  - ширина области в пикселах,
// height - высота области в пикселах

_MAPIMP void _MAPAPI alsSetPictureSize(HALS hals, long int width,
                                       long int height);

// Выделить на схеме указанный элемент атласа
// hdc   - контекст окна отображения атласа
// rect  - координаты окна в области полного изображения атласа
// ident - идентификатор элемента, см. ALSITEM
// color - цвет выделения элемента на атласе
// thick - толщина линии выделения в пикселах

_MAPIMP void _MAPAPI alsPaintSelectItem(HALS hals, HDC hdc,
                                        RECT * rect, long int ident,
                                        COLORREF color, int thick);

// Запросить положение района в изображении атласа
// ident - идентификатор элемента (района), см. ALSITEM
// point - заполненные координаты в пикселах для текущего масштаба изображения
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetItemPosition(HALS hals, long int ident, POINT * point);

// Выбрать в точке все районы и заполнить список по адресу item
// hals   - идентификатор списка районов
// item   - адрес области памяти для заполнения списка найденных элементов
// count  - число элементов, для которых зарезервирована память
// point  - геодезические координаты заданной точки в радианах
// При ошибке возвращает ноль иначе - число найденных элементов

_MAPIMP long int _MAPAPI alsItemListEx(HALS hals, ALSITEM * item, long int count, DOUBLEPOINT * point);


// Запросить - есть ли в точке карта
// direct    - направление поиска (ALS_UP, ALS_DOWN, ALS_ANY)
// point     - геодезические координаты заданной точки в радианах
// Если карта есть - возвращает ненулевое значение
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsEnableItem(HALS hals, long int scale,
                                       DOUBLEPOINT * point, long int direct);

// Запросить - есть ли в заданной области карта
// direct    - направление поиска (ALS_UP, ALS_DOWN, ALS_ANY)
// frame     - координаты заданной области на атласе
// Если карта есть - возвращает ненулевое значение
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsEnableItemEx(HALS hals, long int scale,
                                         HOBJ frame, long int direct);

// Установить область перехода
// hals - идентификатор списка районов
// path - полный путь к паспорту района работ
// frame  - координаты заданной области в метрах на карте
// Если frame == 0, то устанавливаются габариты района
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsBorderChange(HALS hals, const char * path, HOBJ frame);
_MAPIMP long int _MAPAPI alsBorderChangeUn(HALS hals, const WCHAR * path, HOBJ frame);  // 02/12/15

// Определить вхождение карты в атлас
// hals - идентификатор списка районов
// path - полный путь к паспорту района работ
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsMapOccurrence(HALS hals, const char * path);
_MAPIMP long int _MAPAPI alsMapOccurrenceUn(HALS hals, const WCHAR * path);     // 02/12/15

// Запросить область перехода
// hals - идентификатор списка районов
// path - полный путь к паспорту района работ
// frame  - координаты заданной области в метрах на карте
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsGetMapBorder(HALS hals, const char * path, HOBJ frame);
_MAPIMP long int _MAPAPI alsGetMapBorderUn(HALS hals, const WCHAR * path, HOBJ frame);   // 02/12/15

// Найти в атласе карту, содержащую лист с указанной номенклатурой (условным
// названием)
// hals  - идентификатор списка районов (атласа)
// sheet - номенклатура листа
// name  - адрес буфера для размещения результата (пути к карте)
// size  - размер буфера, в который будет записан путь к карте (буфер не менее
//         MAX_PATH_LONG)
// При успешном выполнении в буфер записывается путь к паспорту карты
// или имя алиаса карты на ГИС Сервере
// Для ускорения выполнения функции формируется служебный файл со
// списком карт и номенклатур. Он имеет имя файла атласа и расширение "shl"
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI alsFindMapBySheetName(HALS hals, const char * sheet,
                                               char * name, int size);
_MAPIMP long int _MAPAPI alsFindMapBySheetNameUn(HALS hals, const WCHAR * sheet,
                                                 WCHAR * name, int size);         // 02/12/15

}  // extern "C"


#endif  //  AREALIST_H

