/****** MAPAPI.H *************** Belenkov O.V.    ***** 02/08/16 ***
******* MAPAPI.H *************** Tarakanova I.R.  ***** 12/01/15 ***
******* MAPAPI.H *************** Kruzhkova E.A.   ***** 12/01/16 ***
******* MAPAPI.H *************** Dishlenko S.G.   ***** 21/04/14 ***
******* MAPAPI.H *************** Borzov A.U.      ***** 02/03/15 ***
******* MAPAPI.H *************** Derevyagina Zh.A.***** 21/06/16 ***
******* MAPAPI.H *************** Kruzhkov A.E.    ***** 22/06/16 ***
******* MAPAPI.H *************** Shabakov D.A.    ***** 26/08/13 ***
******* MAPAPI.H *************** Gheleznyakov V.A.***** 07/06/16 ***
******* MAPAPI.H *************** Shishkova L.A.   ***** 05/05/14 ***
******* MAPAPI.H *************** Osipov    D.A.   ***** 05/06/15 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*     ОПИСАНИЕ ИНТЕРФЕЙСА ДОСТУПА К ОБЪЕКТУ "ЭЛЕКТРОННАЯ КАРТА"    *
*                                                                  *
********************************************************************
*                                                                  *
*     Импортирование функций В Windows :                           *
*                                                                  *
*    // Загрузка библиотеки                                        *
*    HINSTANCE libInst = ::LoadLibrary("gisacces.dll");            *
*                                                                  *
*    // Вызов функции                                              *
*    HMAP (WINAPI * lpfn_OpenData)(const char*, long int);         *
*    (FARPROC)lpfn_OpenMap = GetProcAddress(libInst, "OpenData");  *
*    HMAP hMap = (*lpfn_OpenData)("Noginsk.map",0);                *
*                                                                  *
*    // Выгрузка библиотеки                                        *
*    ::FreeLibrary(libInst);                                       *
*                                                                  *
*******************************************************************/

/******************************************************************
********************  СИСТЕМЫ КООРДИНАТ ***************************
*******************************************************************

    Xд Xm
  ^
  | (0,0) (Начало изображени "Picture")
  +---------------------------------------------------> Xи
  |                                                 ^
  |        (Координаты окна "Client")               |
  |   (0,0)                      Xо                 |
  |     ----------------------->                    |
  |     |               |                           |
  |     |        x..... x  Окно визуализации        |
  |     |        ....A. |                           |
  |     |        ...+.. |                           |
  |     |        ...... |                         Габариты
  |     ---------x------x                    района ("Region")
  |     |          ^                                |
  |     |          |                                |
  |     |         Область текущего вывода           |
  |     V                  ("Draw")                 |
  |       Yо                                        |
  |                                                 V
  +----------------------------------------------------> Yд Ym
  |
  |  0,0 (Начало района "Map" в дискретах и метрах,
  |       дискрет м.б. равен метру)
  |
  V        координаты т.A в районе Xд,Yд; в изображении Xи,Yи
    Yи

******************************************************************/

//   Программное обеспечение, применяющее интерфейс "MAPAPI",
// может выполняться в различных операционных системах
// (Windows, Linux, QNX, OC - PB и т.д.).
//   Все строковые параметры API - функций имеют кодировку
// ANSI для Windows и KOИ - 8 для UNIX-подобных систем.
// Параметры типа HWND и HDC в Windows являются идентификаторами
// окна и графического контекста соответственно.
//   Для систем, применяющих графику X-Window,
// параметры HWND и HDC определены как long int (mapsyst.h),
// но содержат указатели на структуру типа XCONTEXT.
//  typedef struct XCONTEXT
//  {
//   Display     xcDisplay;  //  Связь с Х - сервером
//   Window      xcWindow;   //  Идентификатор окна
//   GC          xcContext;  //  Графический контекст окна
//   DRAWPOINT   xcPoint;    //  Расположение области вывода в окне :
//                           //  верхний, левый  угол в пикселах
// }
//   XCONTEXT;


#ifndef MAPAPI_H
#define MAPAPI_H

#if defined(__BORLANDC__)
#ifndef MAPVERSN_H
 #include "mapversn.h"
#endif
#endif

#ifndef MAPGDI_H
  #include "mapgdi.h"    // Параметры функций визуализации
#endif

#ifndef MAPTYPE_H
  #include "maptype.h"   // Описание структур интерфейса MAPAPI
#endif

#ifndef MAPCREAT_H
  #include "mapcreat.h"   // Описание структур данных для создания карты
#endif

// Версия библиотеки MapAccess
// (если применяется "mapacces.h" и т.п.)
#define MAPACCESSVERSION  20160605

// Версия интерфейса MAPAPI и ядра библиотеки
#define MAPAPIVERSION  0x0111304

//enum PPLACE             // ПРИМЕНЯЕМАЯ СИСТЕМА КООРДИНАТ
//    {
//      PP_MAP     = 1,    // КООРДИНАТЫ ТОЧЕК В СИСТЕМЕ КАРТЫ В ДИСКРЕТАХ
//      PP_PICTURE = 2,    // КООРДИНАТЫ ТОЧЕК В СИСТЕМЕ ИЗОБРАЖЕНИЯ В ПИКСЕЛАХ
//      PP_PLANE   = 3,    // КООРДИНАТЫ ТОЧЕК В ПЛОСКОЙ ПРЯМОУГОЛЬНОЙ СИСТЕМЕ
//                         // НА МЕСТНОСТИ В МЕТРАХ
//      PP_GEO     = 4,    // КООРДИНАТЫ ТОЧЕК В ГЕОДЕЗИЧЕСКИХ КООРДИНАТАХ
//                         // В РАДИАНАХ
//    };

//enum VTYPE              // ТИП ОТОБРАЖЕНИЯ КАРТЫ
//    {
//      VT_SCREEN        = 1, // ЭКРАННЫЙ (ЧЕРЕЗ DIB)
//      VT_SCREENCONTOUR = 2, // ЭКРАННЫЙ КОНТУРНЫЙ
//      VT_PRINT         = 3, // ПРИНТЕРНЫЙ (ЧЕРЕЗ WIN API)
//      VT_PRINTGLASS    = 4, // ПРИНТЕРНЫЙ БЕЗ ЗАЛИВКИ ПОЛИГОНОВ
//      VT_PRINTCONTOUR  = 5, // ПРИНТЕРНЫЙ КОНТУРНЫЙ, БЕЗ УСЛОВНЫХ ЗНАКОВ
//      VT_PRINTRST      = 6, // ПРИНТЕРНЫЙ РАСТРИЗОВАННЫЙ (ЧЕРЕЗ WIN API)
//    };

#ifndef SEEKAPI_H
  #include "seekapi.h"     // Описание методов поиска и отбора объектов карты
#endif

#ifndef SITAPI_H
  #include "sitapi.h"      // Описание методов доступа к обстановке
#endif

#ifndef MTRAPI_H
  #include "mtrapi.h"      // Описание методов доступа к матрицам
#endif

#ifndef RSTAPI_H
  #include "rstapi.h"      // Описание методов доступа к растрам
#endif

#ifndef RSCAPI_H
  #include "rscapi.h"      // Описание методов доступа к классификатору
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К ЭЛЕКТРОННОЙ КАРТЕ ++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern "C"
{

  /*********************************************************
  *                                                        *
  *          ОТКРЫТЬ/ЗАКРЫТЬ ДОСТУП К ДАННЫМ КАРТЫ         *
  *                                                        *
  *********************************************************/

  // Открыть данные с автоматическим определением их типа
  // (векторные,растровые,матричные...)
  // name - имя открываемого файла (MAP, SIT, MTW, RSW, MPT) в кодировке UNICODE
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение, при этом не открываются
  // файлы \Log\name.log и \Log\name.tac - протокол работы и журнал транзакций
  // error - после выполнения функции переменная содержит код ошибки
  //        (когда HMAP равен 0) или 0; коды ошибок приведены в maperr.rh
  // password - пароль доступа к данным из которого формируется 256-битный код
  //            для шифрования данных (при утрате пароля данные не восстанавливаются)
  // size     - длина пароля в байтах !!!
  // Передача пароля необходима, если при создании карты он был указан.
  // Если пароль не передан, а он был указан при создании,
  // то автоматически вызывается диалог scnGetMapPassword из mapscena.dll (gisdlgs.dll)
  // Если выдача сообщений запрещена (mapIsMessageEnable()), то диалог
  // не вызывается, а при отсутствии пароля происходит отказ открытия данных
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenAnyDataPro(const WCHAR *name, long int mode,
                                       long int *error,
                                       const WCHAR *password, long int size);

_MAPIMP HMAP _MAPAPI mapOpenAnyData(const WCHAR *name, long int mode = 0,
                                    long int *error = 0);

_MAPIMP HMAP _MAPAPI mapOpenDataUn(const WCHAR * name, long int mode = 0);

  // Открыть векторные данные (аналогично mapOpenData)
  // Возвращает идентификатор открытой векторной карты
  // mapname - имя файла карты MAP(многолистовая), SIT (пользовательская),
  //           SITX (хранилище данных с поддержкой шифрования данных)
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE, или 0)
  // GENERIC_READ - все данные только на чтение, при этом не открываются
  // файлы \Log\name.log и \Log\name.tac - протокол работы и журнал транзакций
  // password - пароль доступа к данным из которого формируется 256-битный код
  //            для шифрования данных (при утрате данные не восстанавливаются)
  // size     - длина пароля в байтах !!!
  // Передача пароля необходима, если при создании карты он был указан.
  // Если пароль не передан, а он был указан при создании,
  // то автоматически вызывается диалог scnGetMapPassword из mapscena.dll (gisdlgs.dll)
  // Если выдача сообщений запрещена (mapIsMessageEnable()), то диалог
  // не вызывается, а при отсутствии пароля происходит отказ открытия данных
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenMapPro(const WCHAR *name,long int mode,
                                   const WCHAR *password, long int size);
_MAPIMP HMAP _MAPAPI mapOpenMapUn(const WCHAR *mapname, long int mode = 0);

  // Запросить является ли идентификатор данных корректным           // 28/02/14
  // hMap -  идентификатор открытых данных
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsMapHandleCorrect(HMAP hMap);

  // Запрос/установка разрешения выполнять структурный контроль карты
  // после сбоев программы
  // flag - нулевое значение запрещает выполнение контроля структуры
  //        при открытии карты, ненулевое - разрешает
  // Возвращает старое значение флага

_MAPIMP long int _MAPAPI mapSetStructureControlFlag(long int flag);

  // Установить режим добавления данных к карте
  // hMap -  идентификатор открытых данных
  // mode - режим добавления данных (1 - ускоренный, 0 - стандартный)
  // При ускоренном режиме не пересчитываются габариты документа по
  // всем открытым данным и не обновляется палитра, что существенно
  // ускоряет процесс добавления данных потоком
  // По окончанию добавления данных рекомендуется вернуть режим добавления
  // к стандартному для обновления габаритов и палитры
  // Габариты обновляются автоматически и при масштабировании документа,
  // а палитра нужна при формировании изображений с ограниченным диапазоном
  // цветов
  // После вызова mapOpenProject или mapAppendProject автоматически устанавливает
  // стандартный режим обновления данных
  // Возвращает текущее значение режима

_MAPIMP long int _MAPAPI mapSetAppendDataMode(HMAP hMap, long int mode);

  // Добавить данные к открытой карте (карту, растр, матрицу)
  // hMap -  идентификатор открытых данных
  // name - имя открываемого файла (SIT, MTW, MTQ, RSW, MPT)
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // Возвращает идентификатор типа данных (FILE_MAP - для пользовательской
  // карты, FILE_RSW - для растра, FILE_MTW - для матрицы, FILE_MTL - для
  // матрицы слоев, FILE_MTQ - для матрицы качеств), данные добавляются в
  // список последними, если данные уже были открыты, число открытых данных
  // (карт, растров, матриц) не меняется
  // transform - признак трансформирования пользовательской карты
  //             к ранее открытым данным (если проекции разные):
  //             0 - не трансформировать данные (преобразовывать "на лету"),
  //             1 - трансформировать данные при открытии и сохранить карту
  //                 в новой проекции,
  //            -1 - задать вопрос пользователю.
  // В серверной версии (-1) обрабатывается, как 0.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendData(HMAP hMap, const char *name,
                                       long int mode = 0);

_MAPIMP long int _MAPAPI mapAppendDataEx(HMAP hMap, const char *name,
                                         long int mode = 0,
                                         long int transform = -1);

  // Добавить данные к открытой карте (карту, растр, матрицу)
  // hMap -  идентификатор открытых данных
  // name - имя открываемого файла (SITX, SIT, MTW, MTQ, RSW, MPT) в кодировке UNICODE
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // transform - признак трансформирования пользовательской карты
  //             к ранее открытым данным (если проекции разные):
  //             0 - не трансформировать данные (преобразовывать "на лету"),
  //             1 - трансформировать данные при открытии и сохранить карту
  //                 в новой проекции,
  //            -1 - задать вопрос пользователю.
  // В серверной версии (-1) обрабатывается, как 0.
  // password - пароль доступа к данным из которого формируется 256-битный код
  //            для шифрования данных (при утрате данные не восстанавливаются)
  // size     - длина пароля в байтах !!!
  // Передача пароля необходима, если при создании карты он был указан.
  // Если пароль не передан, а он был указан при создании,
  // то автоматически вызывается диалог scnGetMapPassword из mapscena.dll (gisdlgs.dll)
  // Если выдача сообщений запрещена (mapIsMessageEnable()), то диалог
  // не вызывается, а при отсутствии пароля происходит отказ открытия данных
  // Возвращает идентификатор типа данных (FILE_MAP - для пользовательской
  // карты, FILE_RSW - для растра, FILE_MTW - для матрицы, FILE_MTL - для
  // матрицы слоев, FILE_MTQ - для матрицы качеств), данные добавляются в
  // список последними, если данные уже были открыты, число открытых данных
  // (карт, растров, матриц) не меняется
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendAnyData(HMAP hMap, const WCHAR *name,
                                          long int mode = 0, long int transform = -1);

_MAPIMP long int _MAPAPI mapAppendDataUn(HMAP hMap, const WCHAR *name,
                                         long int mode = 0);

_MAPIMP long int _MAPAPI mapAppendAnyDataPro(HMAP hMap, const WCHAR *name,
                                             long int mode = 0, long int transform = -1,
                                             const WCHAR *password = 0, long int size = 0);

  // Запросить размер данных по имени файла
  // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapGetDataSize(const char *name);
_MAPIMP double _MAPAPI mapGetDataSizeUn(const WCHAR *name);

  // Запросить имя главной карты в документе или имя проекта,
  // если открыт проект
  // hMap -  идентификатор открытых данных
  // При ошибке возвращает пустую строку

_MAPIMP const char* _MAPAPI mapGetMainName(HMAP hMap);
_MAPIMP const WCHAR * _MAPAPI mapGetMainNameEx(HMAP hMap);             // 17/02/14  

  // Запросить в кодировке UNICODE имя главной карты в документе или имя проекта,
  // если открыт проект
  // hMap -  идентификатор открытых данных
  // name - адрес буфера для записи результата
  // size - длина строки в байтах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMainNameUn(HMAP hMap, WCHAR *name, long int size);

  // Запросить имя (полный путь к файлу) главной карты в документе
  // или в проекте (MPT)
  // hMap -  идентификатор открытых данных
  // При ошибке возвращает пустую строку

_MAPIMP const char* _MAPAPI mapGetMainMapName(HMAP hMap);

  // Запросить имя (полный путь к файлу) главной карты в документе
  // или в проекте (MPT)
  // hMap     -  идентификатор открытых данных
  // name     - буфер для возвращаемой строки
  // namesize - размер буфера в БАЙТАХ
  // При ошибке возвращает 0
  
_MAPIMP long int _MAPAPI mapGetMainMapNameUn(HMAP hMap, WCHAR *name, int namesize);

  // Проверка корректности паспортных данных и, если надо, то заполнение
  // координат по признаку приоритета 0 - расчет прямоугольных координат
  //                                  1 - геодезических
  // Структуры MAPREGISTEREX, LISTREGISTER описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckAndUpdate(MAPREGISTEREX *mapreg, LISTREGISTER *listreg,
                                           long int priority);

  // Заполнение справочных данных в зависимости от типа карты
  // Структуры MAPREGISTEREX, LISTREGISTER описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterFromMapType(int maptype, MAPREGISTEREX *mapreg);

  // Запросить допустимые параметры для проекции
  // code - номер проекции из MAPPROJECTION (mapcreat.h)
  // Возвращает комбинацию флагов PROJECTIONPARAMETERS (mapcreat.h)
  // Например: значение 49 = EPP_AXISMERIDIAN|EPP_FALSEEASTING|EPP_FALSENORTHING
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetProjectionParameters(long int code);

  // Создать новый район (новую векторную карту)
  // mapname - полное имя файла карты (MAP, SIT, SITX)
  // rscname - полное имя файла ресурсов (RSC)
  // Возвращает идентификатор открытой векторной карты
  // Структуры MAPREGISTEREX и LISTREGISTER описаны в mapcreat.h
  // sheetnames - название (UTF-16) листа карты, номенклатуры и файлов даных (для многолистовой карты),
  // для пользовательской карты (не ограниченной рамкой) название листа и номенклатуры совпадает,
  // а название файлов данных совпадает с названием паспорта карты
  // mainname - главное название (UTF-16) многолистовой карты (MAP),
  // для пользовательской карты совпадает с названием листа карты
  // (Запросить главное название карты можно функцией mapGetSiteNameUn)
  // password  - пароль доступа к данным из которого формируется 256-битный код
  //             для шифрования данных (при утрате данные не восстанавливаются)
  //             (поддерживается для карт с расширением SITX - хранилище в
  //              одном файле) или 0
  // size      - длина пароля в байтах !!! или 0
  // Возвращает идентификатор открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateMapPro(const WCHAR * mapname,        // 14/10/14
                                     const WCHAR * rscname,
                                     MAPREGISTEREX * mapreg,
                                     LISTREGISTER * listreg,
                                     SHEETNAMES  * sheetnames,
                                     const WCHAR * mainname,
                                     const WCHAR * password, long int size);

_MAPIMP HMAP _MAPAPI mapCreateMapUn(const WCHAR * mapname,
                                    const WCHAR * rscname,
                                    MAPREGISTEREX * mapreg,
                                    LISTREGISTER * listreg,
                                    const WCHAR * mainname,
                                    const WCHAR * password, long int size);

_MAPIMP HMAP _MAPAPI mapCreateMapEx(const char * mapname,
                                    const char * rscname,
                                    MAPREGISTEREX * mapreg,
                                    LISTREGISTER * listreg);

  // Создать план
  // (карта в местной системе координат)
  // mapname - полное имя файла карты
  // rscname - полное имя файла ресурсов
  // Струтктуры MAPREGISTEREX, LISTREGISTER описаны в mapcreate.h
  // Возвращает идентификатор открытой векторной карты
  // Структура CREATEPLAN описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreatePlane(const char * mapname,
                                    const char * rscname,
                                    CREATEPLANE * createplane);

_MAPIMP HMAP _MAPAPI mapCreatePlaneUn(const WCHAR * mapname,                    // 05/05/14
                                      const WCHAR * rscname,
                                      CREATEPLANE * createplane);

  // Создать пользовательскую карту
  // mapname - полное имя файла карты в кодировке UNICODE
  // rscname - полное имя файла ресурсов в кодировке UNICODE
  // Возвращает идентификатор открытой векторной карты
  // Структура CREATESITEUN описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateSiteUn(const WCHAR * mapname,
                                     const WCHAR * rscname,
                                     CREATESITEUN * createsite);

  // Создать пользовательскую карту по открытой карте
  // hmap -  идентификатор открытых данных
  // mapname - полное имя файла карты в кодировке UNICODE
  // rscname - полное имя файла ресурсов в кодировке UNICODE
  // Структура CREATESITEUN описана в maptype.h
  // Возвращает идентификатор открытой пользовательской карты
  // При ошибке возвращает ноль

_MAPIMP HSITE _MAPAPI mapCreateAndAppendSiteUn(HMAP hmap,
                                               const WCHAR * mapname,
                                               const WCHAR * rscname,
                                               CREATESITEUN * createsite);

  // Создать временную пользовательскую карту
  // rscname - полное имя файла ресурсов
  // Файлы карты размещаются в рабочей директории системы
  // и имеют уникальные имена, генерируемые автоматически
  // При закрытии карты все файлы данных удаляются
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateTempSite(const char * rscname);
_MAPIMP HMAP _MAPAPI mapCreateTempSiteUn(const WCHAR * rscname);

  // Создать временную пользовательскую карту
  // rscname - полное имя файла ресурсов
  // mapreg  - параметры проекции создаваемой временной карты
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateTempSiteEx(const char *rscname, MAPREGISTEREX *mapreg);
_MAPIMP HMAP _MAPAPI mapCreateTempSiteExUn(const WCHAR *rscname, MAPREGISTEREX *mapreg,
                                           DATUMPARAM *datum, ELLIPSOIDPARAM *ellipsoid);

  // Создать временную пользовательскую карту по открытой карте
  // hmap    -  идентификатор открытых данных
  // rscname - полное имя файла ресурсов, если равно 0 - выбирается
  // из открытой карты
  // Файлы карты размещаются в рабочей директории системы
  // и имеют уникальные имена, генерируемые автоматически
  // При закрытии пользовательской карты все файлы данных автоматически удаляются
  // Возвращает идентификатор открытой пользовательской карты
  // При ошибке возвращает ноль

_MAPIMP HSITE _MAPAPI mapCreateAndAppendTempSite(HMAP hmap, const char *rscname);
_MAPIMP HSITE _MAPAPI mapCreateAndAppendTempSiteUn(HMAP hMap, const WCHAR *rscname);

  // Установить параметры Datum для карты
  // Может выполняться или до записи объектов на карту или
  // в другой момент - для карты хранящей геодезические координаты объектов
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetSiteDatum(HMAP hmap, HSITE hSite, DATUMPARAM * parm);

  // Запросить параметры Datum для карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetSiteDatum(HMAP hmap, HSITE hSite, DATUMPARAM * parm);

  // Установить параметры эллипсоида для карты
  // Может выполняться или до записи объектов на карту или
  // в другой момент - для карты хранящей геодезические координаты объектов
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetSiteEllipsoidParameters(HMAP hMap, HSITE hSite,
                                                       ELLIPSOIDPARAM * parm);

  // Запросить параметры эллипсоида для карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetSiteEllipsoidParameters(HMAP hMap, HSITE hSite,
                                                       ELLIPSOIDPARAM * parm);

  // Переоткрыть главную карту
  // Для специальных случаев. HMAP остается прежний, а
  // карта открывается новая
  // name     - имя открываемой карты
  // password - пароль доступа к данным из которого формируется 256-битный код
  //            для шифрования данных (при утрате данные не восстанавливаются)
  // size     - длина пароля в байтах !!!
  // При ошибке возвращает ноль (в этом случае может остаться
  // открытой прежняя карта или никакой)

_MAPIMP long int _MAPAPI mapReOpenMapPro(HMAP hMap, const WCHAR * name, const WCHAR * password, long int size);
_MAPIMP long int _MAPAPI mapReOpenMap(HMAP hMap, const char * name);

  // Закрыть все данные электронной карты
  // hmap -  идентификатор открытых данных
  // Идентификатор HMAP становится недействительным !

_MAPIMP void _MAPAPI mapCloseData(HMAP hMap);

  // Закрыть все данные электронной карты
  // hmap -  идентификатор открытых данных
  // Идентификатор HMAP становится недействительным !

_MAPIMP void _MAPAPI mapCloseMap(HMAP hMap);

  // Копирование района работ         // 09/01/07
  // oldname - старое имя района
  // newname - новое имя района
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCopyMap(const char * oldname, const char * newname);
_MAPIMP long int _MAPAPI mapCopyMapUn(const WCHAR *oldname, const WCHAR *newname);

  // Закрыть и Удалить векторную карту (все файлы данных)
  // hmap - идентификатор открытых данных
  // После удаления идентификатор hMap не должен использоваться,
  // как после mapCloseData()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMap(HMAP hMap);

  // Удаление района работ
  // name      - имя удаляемой карты
  // rscdelete - признак удаления файла классификатора вместе с картой
  // Классификатор необходимо удалить отдельно
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMapByName(const char *name);
_MAPIMP long int _MAPAPI mapDeleteMapByNameUn(const WCHAR *name);
_MAPIMP long int _MAPAPI mapDeleteMapByNameEx(const WCHAR *name, long int rscdelete);  // 11/05/15

  // Открыть проект данных (карта, обстановки, растры, матрицы...)
  // name - имя файла проекта (MPT : структура, как в INI)
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenProject(const char *name);
_MAPIMP HMAP _MAPAPI mapOpenProjectUn(const WCHAR *name);

  // Добавить проект данных (карта, обстановки, растры, матрицы...)
  // name - имя файла проекта (MPT : структура, как в INI)
  // При ошибке возвращает ноль

_MAPIMP int _MAPAPI mapAppendProject(HMAP hMap, const char *name);
_MAPIMP int _MAPAPI mapAppendProjectUn(HMAP hMap, const WCHAR *name);

  // Запросить имя открытого проекта
  // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI mapGetProjectName(HMAP hMap);

  // Запросить имя открытого проекта
  // hmap     -  идентификатор открытых данных
  // name     - буфер для размещения возвращаемой строки
  // namesize - размер буфера в БАЙТАХ
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetProjectNameUn(HMAP hMap, WCHAR *name, long int namesize);

  // Сохранить текущее описание данных в проекте данных
  // hmap -  идентификатор открытых данных
  // name - имя файла проекта (MPT : структура, как в INI)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveProjectUn(HMAP hMap, const WCHAR * name);
_MAPIMP long int _MAPAPI mapSaveProject(HMAP hMap, const char * name);

  // Запросить - является ли документ проектом (MPT)          // 29/07/15
  // hmap -  идентификатор открытых данных
  // Если это проект - возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsDocProject(HMAP hMap);

  // Сохранить текущие параметры окна карты в INI-файл карты,
  // имя файла можно запросить через mapGetMapIniName()
  // Вызывается перед закрытием окна карты
  // Сохраняет описание открытых данных, масштаб, палитру, признаки видимости,
  // редактируемости, состав отображаемых объектов...
  // hmap  -  идентификатор открытых данных
  // point -  координаты центра окна в метрах (может быть 0)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveMapState(HMAP hMap, DOUBLEPOINT *point);

  // Восстановить параметры окна карты из INI-файла карты,
  // имя файла можно запросить через mapGetMapIniName()
  // Вызывается после открытия карты
  // Восстанавливает описание списка данных, масштаб, палитру, признаки видимости,
  // редактируемости, состав отображаемых объектов...
  // hmap  -  идентификатор открытых данных
  // point -  координаты центра окна в метрах (может быть 0)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRestoreMapState(HMAP hMap, DOUBLEPOINT *point);

  // Запросить - есть ли какие-либо открытые данные
  // Данные - векторные, растровые, матричные...
  // hmap -  идентификатор открытых данных
  // Если открытых данных нет или ошибка - возвращает ноль

_MAPIMP long int _MAPAPI mapIsActive(HMAP hMap);

  // Запросить - есть ли какие-либо открытые векторные карты
  // Если открытых векторных карт нет или ошибка - возвращает ноль

_MAPIMP long int _MAPAPI mapIsVectorMapActive(HMAP hMap);

  // Запросить - есть ли какие-либо открытые векторные карты,
  // доступные для редактирования
  // Если доступных для редактирования векторных карт нет
  // или ошибка - возвращает ноль

_MAPIMP long int _MAPAPI mapIsVectorMapEdit(HMAP hMap);

  // Запросить - есть ли какие-либо открытые векторные карты
  // Если открытых векторных карт нет или ошибка - возвращает ноль

_MAPIMP long int _MAPAPI mapIsVectorActive(HMAP hMap);

  // Выполнить согласование данных электронной карты
  // в памяти и на диске (при многопользовательском доступе
  // к данным)
  // hmap -  идентификатор открытых данных
  // Если состояние данных в памяти изменилось (по данным
  // с диска) - возвращает ненулевое значение (1), иначе 0
  // Если карта должна быть закрыта - возвращает 2
  // (доступ на ГИС Сервер прекращен!)
  // Если состояние изменилось - необходимо перерисовать
  // изображение карты
  // Опрос состояния целесообразно выполнять периодически
  // в процессе работы приложения

_MAPIMP long int _MAPAPI mapAdjustData(HMAP hMap);

  // Установить доступность для выполнения команды Adjust
  // mode - признак доступности обработки команды Adjust,
  //        если равен 0 - команда не обрабатывается.
  // При выполнении длительных процедур (отмена длинных
  // транзакций, трансформирование данных и т.п.) целесообразно
  // отключать команду Adjust, если она может быть вызвана из
  // других потоков приложения. Команда Adjust может вызывать
  // переоткрытие карт и перераспределение памяти.
  // Возвращает прежнее значение

_MAPIMP long int _MAPAPI mapSetAdjustMode(HMAP hMap, long int mode);

  // Сохранить данные на диск
  // при работе на Sparc-платформе также сохраняет данные в файлы Intel-платформы
  // hMap - идентификатор базовой карты
  // hSite - идентификатор пользовательской карты
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapFlushData(HMAP hMap, HSITE hSite);


  // Создать(добавить) новый лист в районе
  // hmap -  идентификатор открытых данных
  // Структура LISTREGISTER описана в mapcreat.h
  // При ошибке возвращает ноль, иначе - номер созданного листа

_MAPIMP long int _MAPAPI mapCreateList(HMAP hMap, LISTREGISTER *sheet);

_MAPIMP long int _MAPAPI mapCreateListPro(HMAP hMap, LISTREGISTER *sheet,
                                          SHEETNAMES * sheetnames);      // 07/11/14

  // Удалить указанный лист карты
  // list - номер листа (с 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteList(HMAP hmap, long int list);

  // Добавить листы из другого района работ в данный
  // hmap -  идентификатор открытых данных
  // mapname - добавляемый район,
  // handle - идентификатор окна,которое будет извещаться
  // о ходе процесса (0x585 - 0x588)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendMapToMap(HMAP hMap, const char *mapname, HWND handle);
_MAPIMP long int _MAPAPI mapAppendMapToMapUn(HMAP hMap, const WCHAR *name, HWND handle);

  // Cоздать пустой объект векторной карты
  // (создание подобъекта - см. редактирование метрики)
  // По умолчанию метрика FLOAT 2-ух мерная
  // hmap - идентификатор открытых данных
  // sheetnumber - номер листа в котором будет расположен
  // kind - тип создаваемой метрики, описан в maptype.h
  // создаваемый объект
  // text - признак метрики с текстом (объекты типа "подпись")
  // (устанавливается автоматически при вызове mapPutText(...))
  // После вызова функций типа What...() и Seek...() все параметры
  // полученного объекта могут измениться (text,kind,list и т.п.)
  // При ошибке возвращает ноль

#ifndef HIDEFORMAT
_MAPIMP HOBJ _MAPAPI mapCreateObject(HMAP hMap, long int sheetnumber = 1,
                                     long int kind = IDDOUBLE2,
                                     long int text = 0);
#endif

  // Очистить содержимое объекта
  // (для повторного заполнения, как пустого объекта)
  // hmap - идентификатор открытых данных
  // sheetnumber - номер листа в котором будет расположен
  // kind - тип создаваемой метрики, описан в maptype.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapClearObject(HOBJ info,
                                        long int sheetnumber = 1,
                                        long int kind = IDDOUBLE2);

  // Cоздать копию объекта векторной карты
  // hmap - идентификатор открытых данных
  // info - идентификатор объекта карты в памяти
  // Для каждого полученного и больше не используемого
  // идентификатора HOBJ необходим вызов функции mapFreeObject()
  // При ошибке возвращает ноль

_MAPIMP HOBJ _MAPAPI mapCreateCopyObject(HMAP hMap, HOBJ info);

  // Считать копию объекта векторной (src) карты в другой объект (dest)
  // Для каждого полученного и больше не используемого
  // идентификатора HOBJ необходим вызов функции mapFreeObject()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReadCopyObject(HOBJ dest, HOBJ src);

  // Считать копию подобъекта векторной (src) карты в другой объект (dest)   // 20/08/13
  // Для каждого полученного и больше не используемого
  // идентификатора HOBJ необходим вызов функции mapFreeObject()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReadCopySubject(HOBJ dest, HOBJ src, long int subject);

  // Cоздать копию объекта векторной карты, как нового объекта (!)
  // hmap - идентификатор открытых данных
  // info - идентификатор объекта карты в памяти
  // Для каждого полученного и больше не используемого
  // идентификатора HOBJ необходим вызов функции FreeObject()
  // При ошибке возвращает ноль

_MAPIMP HOBJ _MAPAPI mapCreateCopyObjectAsNew(HMAP hMap, HOBJ info);

  // Cоздать копию объекта векторной карты, как нового объекта
  // Для каждого полученного и больше не используемого
  // идентификатора HOBJ необходим вызов функции FreeObject()
  // src  - исходный объект,
  // dest - копия объекта (при сохранении - будет создан новый)
  // Предполагается, что до сохранения в копии что-то поменяют.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCopyObjectAsNew(HOBJ dest, HOBJ src);

  // Удалить описание объекта векторной карты из памяти
  // info - идентификатор объекта карты в памяти
  // Для сохранения объекта на карте необходимо
  // до вызова mapFreeObject(...) выполнить функцию
  // mapCommitObject(...)
  // При ошибке возвращает ноль

_MAPIMP void _MAPAPI mapFreeObject(HOBJ info);

  // Запросить код ошибки последней операции доступа к данным
  // Коды ошибок - см. maperr.rh

_MAPIMP long int _MAPAPI mapGetAccessError();

  // Считать объект, который отображался последним перед возникновением
  // сбоя отображения карт
  // Применяется при попадании в секцию catch(..) при вызове отображения
  // карты для вывода диагностической информации
  // hmap - идентификатор открытых данных;
  // info - идентификатор существующего объекта,
  // в котором будет размещен результат поиска.
  // Если такой объект не установлен - возвращает ноль
 
_MAPIMP long int _MAPAPI mapReadLastViewObject(HMAP hMap, HOBJ info);

  // Запросить, растягивается ли объект по метрике
  // Данный признак может быть установлен у подписей
  // и векторных объектов
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsObjectStretch(HOBJ info);


  /*********************************************************
  *                                                        *
  *    ФУНКЦИИ ОТОБРАЖЕНИЯ И ПЕЧАТИ ЭЛЕКТРОННОЙ КАРТЫ      *
  *  ПАРАМЕТРЫ КОНТЕКСТА ПОДГОТОВЛЕНЫ (HDC) И УСТАНОВЛЕНО  *
  *  СМЕЩЕНИЕ НА ОБЛАСТЬ ВЫВОДА В ОКНЕ ::SetViewportOrgEx()*
  *                                                        *
  *  Размер картинки, рисуемой за один вызов на экран или  *
  *  в память, не более текущих размеров экрана!           *
  *  Произвольный размер - см. PaintToDib,PaintToImage,    *
  *                            PaintToXImage               *
  *                                                        *
  *********************************************************/

 // Установить общие параметры проекции документа для отображения,
 // печати и расчета координат
 // hmap  - идентификатор открытых данных (документа)
 // Структуры MAPREGISTER, DATUMPARAM и ELLIPSOIDPARAM описаны в mapcreat.h
 // Устанавливать общие параметры проекции можно для документа
 // поддерживающего пересчет геодезических координат (mapIsGeoSupported() != 0)
 // После установки общих параметров проекции изображение карты формируется
 // в заданной проекции. Векторные карты, имеющие другие параметры
 // проекции, трансформируются в процессе отображения.
 // Матрицы и растры не трансформируются при отображении и должны иметь 
 // те же параметры проекции, что и общие параметры документа
 // Все операции с координатами (mapPlaneToGeo, mapGeoToPlane, 
 // mapPlaneToGeoWGS84, mapAppendPointPlane, mapInsertPointPlane,
 // mapUpdatePointPlane, mapAppendPointGeo и другие) выполняются
 // в системе координат документа, определяемой общими параметрами проекции
 // При чтении\записи координат в конкретной карте выполняется пересчет
 // из системы координат документа
 // Например, при записи координат из WGS84 на карту в СК-42 можно
 // установить общие параметры документа, как "Широта/Долгота на WGS-84"
 // и выполнить запись координат функцией mapAppendPointGeo, не заботясь
 // о дополнительном пересчете координат, или считать координаты функцией
 // mapGetGeoPoint (или функцией mapGetGeoPointWGS84, игнорирующей параметры
 // документа).
 // Чтобы установить текущие параметры проекции и системы координат, как у первой
 // карты в документе можно передать в качестве параметров (кроме hMap) нули,
 // или вызвать mapClearDocProjection.
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetDocProjection(HMAP hMap,
                                             MAPREGISTEREX * map,
                                             DATUMPARAM * datum,
                                             ELLIPSOIDPARAM * ellparm);

_MAPIMP long int _MAPAPI mapClearDocProjection(HMAP hMap);
                                             
 // Запросить общие параметры проекции документа для отображения,
 // печати и расчета координат
 // hmap  - идентификатор открытых данных (документа)
 // Структуры MAPREGISTER, DATUMPARAM и ELLIPSOIDPARAM описаны в mapcreat.h
 // Если параметры не устанавливались функцией mapSetMapInfoEx,
 // то они соответсвуют параметрам карты, открытой в документе первой
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetDocProjection(HMAP hMap,
                                             MAPREGISTEREX * map,
                                             DATUMPARAM * datum,
                                             ELLIPSOIDPARAM * ellparm);


 // Запросить - устанавливались ли общие параметры проекции документа 
 // для отображения, печати и расчета координат
 // hmap  - идентификатор открытых данных (документа)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsDocProjection(HMAP hMap);

 // Запросить размеры общего изображения карты в пикселах
 // для текущего масштаба
 // hmap - идентификатор открытых данных
 // В переменную width заносится ширина изображения (dx),
 // в переменную height - высота (dy)

_MAPIMP void _MAPAPI mapGetPictureSize(HMAP hmap, long int *width,
                                       long int *height);
_MAPIMP void _MAPAPI mapGetPictureSizePro(HMAP hmap, double * width,
                                          double * height);
                                          
_MAPIMP long int _MAPAPI mapGetPictureHeight(HMAP hMap);
_MAPIMP long int _MAPAPI mapGetPictureWidth(HMAP hMap);

 // Запросить ширину пиксела изображения карты в метрах на местности
 // для текущего масштаба изображения
 // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapGetPixelWidth(HMAP hMap);

 // Запросить высоту пиксела изображения карты в метрах на местности
 // для текущего масштаба изображения
 // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapGetPixelHeight(HMAP hMap);

 // Запросить идентификатор логической палитры карты
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP HPALETTE _MAPAPI mapGetMapPalette(HMAP hmap);

 // Запросить описание цвета палитры по индексу (index)
 // hmap - идентификатор открытых данных
 // Номер первого индекса равен нулю

_MAPIMP COLORREF _MAPAPI mapGetMapColor(HMAP hmap, long int index);

 // Запросить число цветов логической палитры
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetColorCount(HMAP hmap);

 // Отобразить фрагмент карты на заданном устройстве
 // в текущих масштабе и составе объектов
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // erase - признак стирания фона перед выводом,
 // (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
 //  для экранного способа вывода (VT_SCREEN) всегда стирает
 //  цветом фона, кроме значения -2 (минус 2))
 // rect  - координаты фрагмента карты (Draw)
 // в изображении (Picture).
 // С изображениями больше 32000х32000 работает только
 // под Windows NT (Windows 95 имеет 16-битовую графику)
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...

_MAPIMP void _MAPAPI mapPaint(HMAP hmap, HDC hdc, long int erase, RECT *rect);

 // Создать/Удалить контекст отображения для многопоточного вызова   // 26/10/15
 // функций mapPaintByFramePro или mapPaintByFrameToXImagePro
 // hmap - идентификатор открытых данных
 // Для каждого потока создается свой контекст и передается в качестве параметра функции
 // В каждом контексте создается свой буфер отображения и выделяется память под служебные области
 // Размер резервируемой памяти помимо буфера отображения может занимать 1-2 Мбайта,
 // внутренний буфер отображения для размера 1920x1080 занимает 8 Мбайт 
 // Размер может устанавливаться программно - mapSetMaxScreenImageSize
 // При ошибке возвращает ноль

_MAPIMP HPAINT _MAPAPI mapCreatePaintControl(HMAP hmap);
_MAPIMP void _MAPAPI mapFreePaintControl(HPAINT hPaint);

 // Сменить идентификатор открытых данных в контексте отображения     // 26/10/15
 // Применяется для последовательной отрисовки в многопоточном варианте 
 // в буфер изображения одного контекста из нескольких HMAP
 // для наложения слоев
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetPaintControlMapHandle(HPAINT hPaint, HMAP hmap);

 // Установить параметры системы координат документа в контексте отображения      // 05/11/15
 // Применяется для установки системы координат формируемого изображения
 // по коду EPSG
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // epsgcode - код EPSG для требуемой системы координат (например, 3395, 3857, 4326)
 // Для геодезических систем координат возвращает 2,
 // для плоских прямоугольных возвращает 1.
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetPaintControlProjection(HPAINT hPaint, long int epsgcode);

 // Запросить идентификатор открытых данных, для которых создан контекст отображения
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // При ошибке возвращает ноль

 _MAPIMP HMAP _MAPAPI mapGetPaintControlMapHandle(HPAINT hPaint);

#ifdef WIN32API
 // Запросить контекст устройства отображения для внутренниего буфера  // 26/10/15
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // Контекст устройства вывода может применяться для отрисовки фонового изображения,
 // поверх которого будет отрисована карта (растр, матрица) и
 // для считывания готового изображения
 // При ошибке возвращает ноль

 _MAPIMP HDC _MAPAPI mapGetPaintControlDC(HPAINT hPaint);
#endif 

 // Скопировать содержимое внутренниего буфера в заданную область      // 26/10/15
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // imagedesc - описание выходного буфера изображения
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapCopyPaintControlToXImage(HPAINT hPaint, XIMAGEDESC * imagedesc);
 
 // Отобразить фрагмент карты, смасштабировав до заданной ширины и высоты
 // в текущем составе объектов (применяется для обработки OGC WMS-запросов)
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 //         если контекст устройства вывода равен 0, то отрисовка выполняется
 //         во внутренний буфер контекста отображения (hPaint) 
 // erase - признак стирания фона перед выводом,
 //        (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
 //         для экранного способа вывода (VT_SCREEN) всегда стирает
 //         цветом фона, кроме значения -2 (минус 2))
 //         При последовательном отображении нескольких HMAP в один контекст
 //         отображения, для второго HMAP и далее значение erase = -2 при hdc = 0
 // frame  - координаты фрагмента карты в системе координат документа в метрах
 // (см. mapSetDocProjection)
 // width  - ширина изображения в пикселах
 // height - высота изображения в пикселах
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // filename  - полное имя создаваемого файла формата png
 // viewselect - условия отбора отображаемых объектов, если равно 0, то применяются
 //           условия обобщенного поиска\выделения (внутренние)
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // Данная функция может изменять текущий масштаб отображения документа (если hPaint равен 0),
 // для сохранения текущего масштаба можно применить функции mapGetRealShowScale/mapSetRealShowScale
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintByFramePro(HMAP hMap,HDC hDC,          // 26/10/15
                                            long int erase, DFRAME * frame,
                                            long int width, long int height, long int alpha, 
                                            HSELECT viewselect, HPAINT hPaint);

_MAPIMP long int _MAPAPI mapPaintByFrame(HMAP hMap, HDC hDC,
                                         long int erase, DFRAME *frame,
                                         long int width, long int height);

_MAPIMP long int _MAPAPI mapPaintByFrameEx(HMAP hMap,HDC hDC,
                                           long int erase, DFRAME *frame,
                                           long int width, long int height, long int alpha);

_MAPIMP long int _MAPAPI mapPaintByFrameSelectEx(HMAP hMap,HDC hDC,
                                           long int erase, DFRAME *frame,
                                           long int width, long int height, long int alpha, 
                                           HSELECT viewselect);

_MAPIMP long int _MAPAPI mapPaintByFrameToFile(HMAP hMap,  const char *filename,
                                               long int erase, DFRAME * frame,
                                               long int width, long int height);
_MAPIMP long int _MAPAPI mapPaintByFrameToFileUn(HMAP hMap, const WCHAR *filename,
                                                 long int erase, DFRAME *frame,
                                                 long int width, long int height);

 // Отобразить фрагмент карты на заданном устройстве
 // в текущих масштабе и составе объектов
 // hmap  - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // erase - признак стирания фона перед выводом,
 //        (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
 //        для экранного способа вывода (VT_SCREEN) всегда стирает
 //        цветом фона, кроме значения -2 (минус 2))
 // rect - координаты фрагмента карты (Draw) в изображении (Picture)
 // Корректно работает с большими изображениями под Windows95 и NT,
 // но требует перед вызовом установки
 //              ::SetViewportOrgEx(hDC, dx , dy, 0),
 // где dx,dy - положение отображаемого фрагмента в клиентной
 // области !
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // filename  - полное имя создаваемого файла формата png
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // Если image != 0 и object != 0 дополнительно вызвается фукция
 // mapPaintMapObject95Ex  (Отобразить произвольный объект в пределах фрагмента
 // в условных знаках пользователя)
 // image - описание вида объекта (см. MAPGDI.H),
 // object - идентификатор объекта

_MAPIMP long int _MAPAPI mapPaint95(HMAP hmap, HDC hdc, long int erase, RECT *rect);

_MAPIMP long int _MAPAPI mapPaint95Ex(HMAP hmap, HDC hdc, long int erase,
                                      RECT *rect, long int alpha);

_MAPIMP long int _MAPAPI mapPaint95ToFile(HMAP hMap, const char *filename,
                                          long int erase, RECT *rect, long int alpha,
                                          PAINTPARM *image, HOBJ object);

_MAPIMP long int _MAPAPI mapPaint95ToFileUn(HMAP hMap, const WCHAR *filename,
                                            long int erase, RECT *rect, long int alpha,
                                            PAINTPARM *image, HOBJ object);

 // Отобразить фрагмент карты на заданном устройстве
 // в текущих масштабе и составе объектов и
 // выделить на карте объекты, удовлетворющие заданным условим
 // hMap   - идентификатор открытых данных
 // hdc    - контекст устройства
 // erase  - признак стирания фона перед выводом,
 //          (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
 //          для экранного способа вывода (VT_SCREEN) всегда стирает
 //          цветом фона, кроме значения -2 (минус 2))
 // rect   - координаты фрагмента карты (Draw) в изображении (Picture)
 // select - условия отбора объектов, если равны 0, то применяются
 //          условия обобщенного поиска\выделения (см. mapTotalPaintSelect95).
 // color  - цвет, которым будут выделяться объекты на карте
 // Корректно работает с большими изображениями под Windows95 и NT,
 // но требует перед вызовом установки
 //              ::SetViewportOrgEx(hDC, dx , dy, 0),
 // где dx,dy - положение отображаемого фрагмента в клиентной области !
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана!
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // filename  - полное имя создаваемого файла формата png
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // Если image != 0 и object != 0 дополнительно вызвается фукция
 // mapPaintMapObject95Ex  (Отобразить произвольный объект в пределах фрагмента
 // в условных знаках пользователя)
 // image - описание вида объекта (см. MAPGDI.H),
 // object - идентификатор объекта

_MAPIMP void _MAPAPI mapPaint95AndSelect(HMAP hMap, HDC hDC, long int erase,
                                         RECT *rect, HSELECT select, COLORREF color);

_MAPIMP void _MAPAPI mapPaint95AndSelectEx(HMAP hMap, HDC hDC, long int erase,
                                           RECT *rect, HSELECT select,
                                           COLORREF color, long int alpha);

_MAPIMP long int _MAPAPI mapPaint95AndSelectToFile(HMAP hMap, const char *filename,
                                                   long int erase, RECT *rect,
                                                   HSELECT select, COLORREF color,
                                                   long int alpha, PAINTPARM *image,
                                                   HOBJ object);

_MAPIMP long int _MAPAPI mapPaint95AndSelectToFileUn(HMAP hMap, const WCHAR *filename,
                                                     long int erase, RECT *rect,
                                                     HSELECT select, COLORREF color, long int alpha,
                                                     PAINTPARM *image, HOBJ object);

 // Установить толщину линии для отрисовки выделенных на карте
 // объектов (при вызове mapPaint95AndSelect и т.п.)
 // thick - толщина линии в mkm (из пикселов - PIX2MKM(pixel))
 // Возвращает установленное ранее значение

_MAPIMP long int _MAPAPI mapSetSelectLineThick(HMAP hmap, long int thick);

 // Отобразить отдельный объект карты в пределах фрагмента
 // в условных знаках карты
 // Может использоваться для вывода шаблонов объектов
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // rect  - координаты фрагмента карты (Draw)
 // info - идентификатор объекта карты в памяти
 // При ошибке в параметрах возвращает ноль
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...

_MAPIMP long int _MAPAPI mapPaintObject(HMAP hmap, HDC hdc, RECT *rect, HOBJ info);

 // Отобразить отдельный объект карты в пределах фрагмента
 // в условных знаках карты
 // Может использоваться для вывода шаблонов объектов
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // rect  - координаты фрагмента карты (Draw)
 // info - идентификатор объекта карты в памяти
 // erase -  признак стирания фона перед выводом,
 // (0 - фон не стирать, !=0 - очистить фрагмент цветом фона),
 // dontclip - признак отмены обрезки объектов по рамке при отображении
 // (1 - не обрезать, 0 - выполнить обрезку)
 // При ошибке в параметрах возвращает ноль
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...

_MAPIMP long int _MAPAPI mapPaintObjectEx(HMAP hmap, HDC hdc, RECT *rect,
                                          HOBJ info, long int erase,
                                          long int dontclip);

 // Отобразить отдельный объект карты в пределах фрагмента
 // в условных знаках карты
 // Может использоваться для вывода шаблонов объектов
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // rect  - координаты фрагмента карты (Draw)
 // info - идентификатор объекта карты в памяти
 // Корректно работает с большими изображениями под Windows95 и NT,
 // При ошибке в параметрах возвращает ноль
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать

 _MAPIMP long int _MAPAPI mapPaintObject95(HMAP hmap, HDC hdc, RECT *rect,
                                          HOBJ info);

 _MAPIMP long int _MAPAPI mapPaintObject95Ex(HMAP hmap, HDC hdc, RECT *rect,
                                             HOBJ info, long int alpha);

 // Отобразить произвольный объект в пределах фрагмента окна (карты)
 // в условных знаках пользователя
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // rect  - координаты фрагмента карты (Draw)
 // image - описание вида объекта (см. MAPGDI.H), если объект
 // должен рисоваться своим условным знаком - значение параметра
 // можно установить в ноль
 // info - идентификатор объекта карты в памяти
 // При ошибке в параметрах возвращает ноль
 // Размер картинки, рисуемой за один вызов, не более
 // текущих размеров экрана! Иначе - см. PaintToDib,PaintToImage...
 // filename  - полное имя создаваемого файла формата png
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать

_MAPIMP long int _MAPAPI mapPaintMapObject95(HMAP hmap, HDC hdc, RECT *rect,
                                             PAINTPARM *image, HOBJ info);

_MAPIMP long int _MAPAPI mapPaintMapObject95Ex(HMAP hMap, HDC hdc, RECT *rect,
                                               PAINTPARM *image, HOBJ object,
                                               long int alpha);

_MAPIMP long int _MAPAPI mapPaintMapObject95ToFile(HMAP hMap, const char *filename,
                                                   RECT *rect, PAINTPARM *image,
                                                   HOBJ object, long int alpha);

_MAPIMP long int _MAPAPI mapPaintMapObject95ToFileUn(HMAP hMap, const WCHAR *filename,
                                                     RECT *rect, PAINTPARM *image,
                                                     HOBJ object, long int alpha);

 // Отобразить произвольный объект в пределах фрагмента
 // в условных знаках пользователя
 // hmap - идентификатор открытых данных
 // hdc   - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw) в изображении (Picture)
 // image - описание вида объекта (см. MAPGDI.H), если объект
 // должен рисоваться своим условным знаком - значение параметра
 // можно установить в ноль,
 // info - идентификатор объекта карты в памяти
 // offset - смещение координат объекта (в соответствии с place)
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintOffsetMapObject95(HMAP hmap, HDC hdc, RECT *rect,
                                                   PAINTPARM *image, HOBJ info,
                                                   DOUBLEPOINT *offset,
                                                   long int place = PP_MAP);


 // Отобразить образец вида объекта по номеру записи      
 // в классификаторе объектов (incode)
 // hmap - идентификатор открытой карты
 // hrsc - идентификатор классификатора открытой карты
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты клиентской области окна вывода (размер окна)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleRscObject(HMAP hmap, HRSC hrsc,
                                                  HDC hdc, RECT *rect,
                                                  long int incode);

// Отобразить образец вида объекта
// При ошибке возвращает ноль
_MAPIMP long int _MAPAPI mapPaintExample(HMAP hMap, PAINTEXAMPLE *parm);

#ifdef WIN32API
 // Вывести изображение карты в DIB
 // Данная функция реализована только для платформы Windows !
 // Выводится фрагмент карты, заданный параметром rect.
 // Заполнется палитра,расположенная за BITMAINFOHEADER
 // Изображение строится в текущем масштабе и составе объектов
 // Размер точки 1,2,3 или 4 байт.
 // Если размеры заданного DIB меньше размеров фрагмента -
 // изображение строится в пределах размеров DIB.
 //  hmap      - идентификатор открытых данных
 //  dibinfo   - описатель создаваемого DIB в памяти
 //  lpDibBits - адрес первого байта битовой области.
 //  rect      - координаты фрагмента карты (Draw)
 //              в изображении (Picture).
 // Размер области DIB, задаваемый параметром dibinfo->biSizeImage,
 // должен учитывать то, что каждая строка DIB должна быть
 // кратна 4 байтам (32 битам):
 //  dibinfo->biSizeImage = dibinfo->biHeight *
 //    ((dibinfo->biWidth * dibinfo->biBitCount + 31) / 32) * 4;
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintToDib(HMAP hmap, BITMAPINFOHEADER *dibinfo,
                                       char *lpDibBits, RECT *rect);

#ifndef WINCE
 // Вывести изображение карты в Image (массив)
 // Данная функция реализована только для платформы Windows !
 // Выводится фрагмент карты, заданный параметром rect.
 // Заполняется палитра в поле palette.
 // Изображение строится в текущем масштабе и составе объектов
 // Палитра Image только 256 цветов, размер точки 1 байт !
 // Если размеры заданного Image меньше размеров фрагмента -
 // изображение строится в пределах размеров Image.
 //  hmap          - идентификатор открытых данных
 //  palette       - адрес палитры (256 RGBQUAD-цветов)
 //  lpImage       - адрес первого байта области изображения.
 //  width, height - ширина и высота Image.
 //  rect          - координаты фрагмента карты (Draw)
 //                  в изображении (Picture).
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintToImage(HMAP hmap, RGBQUAD *palette,
                                         char *lpImage, long int width,
                                         long int height, RECT *rect);
#endif  // !WINCE
#endif  // WIN32API

 // Вывести изображение объекта в DIB
 //  hmap      - идентификатор открытых данных
 //  lpDibBits - адрес первого байта буфера изображения (32 бита)
 //  width     - ширина изображения (кратно 32),
 //  height    - высота изображения,
 //  rect      - координаты фрагмента карты (Draw)
 //              в изображении (Picture).
 //  image     - описание вида объекта (см. MAPGDI.H), если объект
 //              должен рисоваться своим условным знаком - значение параметра
 //              можно установить в ноль,
 //  info      - идентификатор объекта карты в памяти
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintMapObjectToDib(HMAP hmap, char *lpImage,
                                                int width, int heigth, RECT *rect,
                                                PAINTPARM *image, HOBJ object);

 // Отобразить фрагмент карты, смасштабировав до заданной ширины и высоты
 // в текущем составе объектов
 // hmap  - идентификатор открытых данных
 // imagedesc - описание выходного буфера изображения
 // erase - признак стирания фона перед выводом,
 // (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
 //  для экранного способа вывода (VT_SCREEN) всегда стирает
 //  цветом фона, кроме значения -2 (минус 2))
 // frame  - координаты фрагмента карты в системе координат документа в метрах
 // (см. mapSetDocProjection)
 // width  - ширина изображения в пикселах
 // height - высота изображения в пикселах
 // Описание структуры XIMAGEDESC в maptype.h
 // alpha - флаг использования альфа канала 0 - не использовать 1 - использовать
 // viewselect - условия отбора объектов, если равны 0, то применяются
 // условия обобщенного поиска\выделения
 // hPaint - идентификатор контекста отображения для многопоточного вызова функции отображения
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintByFrameToXImagePro(HMAP hMap, XIMAGEDESC * imagedesc,  // 26/10/15
                                                    long int erase,
                                                    DFRAME *frame, long int width,
                                                    long int height, long int alpha, 
                                                    HSELECT viewselect, HPAINT hPaint);

_MAPIMP long int _MAPAPI mapPaintByFrameToXImage(HMAP hMap, XIMAGEDESC * imagedesc,
                                                 DFRAME *frame, long int width,
                                                 long int height, long int alpha);

_MAPIMP long int _MAPAPI mapPaintByFrameSelectToXImage(HMAP hMap, XIMAGEDESC * imagedesc,
                                                       DFRAME *frame, long int width,
                                                       long int height, long int alpha, HSELECT viewselect);


 // Вывести изображение карты в XImage (массив)
 // Описание структуры XIMAGEDESC в maptype.h
 // hmap - идентификатор открытых данных
 // x,y - координаты левого верхнего угла внутри
 // битовой области XImage для размещения изображения
 // rect - выводимый фрагмент карты
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintToXImage(HMAP hmap, XIMAGEDESC *imagedesc,
                                           long int x, long int y, RECT *rect);

  // Вывести изображение карты в XImage (массив) и выделить
  // на карте отобранные объекты
  // Описание структуры XIMAGEDESC в maptype.h
  // x,y - координаты левого верхнего угла внутри
  // битовой области XImage для размещения изображения
  // rect - выводимый фрагмент карты
  // select - условия отбора объектов, если равны 0, то применяются
  //          условия обобщенного поиска\выделения (см. mapTotalPaintSelect95).
  // color  - цвет, которым будут выделяться объекты на карте
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintAndSelectToXImage(HMAP hMap, XIMAGEDESC *imagedesc,
                                                    long int x, long int y, RECT *rect,
                                                    HSELECT select, COLORREF color);

  // Вывести изображение карты и объекта в XImage (массив)
  // Описание структуры XIMAGEDESC в maptype.h
  // x,y - координаты левого верхнего угла внутри
  // битовой области XImage для размещения изображения
  // rect - выводимый фрагмент карты
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintMapAndObjectToXImage(HMAP hMap, HOBJ hObj,
                                                       XIMAGEDESC *imagedesc,
                                                       long int x, long int y,
                                                       RECT *rect);

  // Вывести изображение карты в XImage (массив)
  // Описание структуры XIMAGEDESC в maptype.h
  // x,y - координаты левого верхнего угла внутри
  // битовой области XImage для размещения изображения
  // rect - фрагмент для вывода отображения
  // func - функция отображения объекта
  // parm - параметры изображения
  // data - метрика для отображения
  // colors - количество цветов
  // palette - палитра
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectByFuncDataToXImage(HMAP hMap, XIMAGEDESC *imagedesc,
                                                                 long int x, long int y,
                                                                 RECT *rect, long int func,
                                                                 const char *parm, POLYDATAEX* data,
                                                                 long int colors, COLORREF *palette);

  // Вывести изображение карты в XImage (массив)
  // Данная функция реализована для XWindow !
  // Описание структуры XIMAGEDESC в maptype.h
  // rect - фрагмент для вывода отображения
  // func - функция отображения объекта
  // parm - параметры изображения
  // colors - количество цветов
  // palette - палитра
  // text - необходимый текст
  // local - локализация
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectByFuncToXImage(HMAP hMap, XIMAGEDESC *imagedesc,
                                                             RECT *rect, long int func, const char *parm,
                                                             long int colors, COLORREF *palette,
                                                             const char *text, long int local);

_MAPIMP long int _MAPAPI mapPaintExampleObjectByFuncToXImageUn(HMAP hMap, XIMAGEDESC *imagedesc,
                                                               RECT *rect, long int func, const char *parm,
                                                               long int colors, COLORREF *palette,
                                                               const WCHAR *text, long int local);


#ifdef WIN32API
 // Вывести изображение карты в метафайл
 // Данные функции реализованы только для платформы Windows !
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintToEmf(HMAP hMap, const char *name,
                                       METAFILEBUILDPARM *parm);
_MAPIMP long int _MAPAPI mapPaintToEmfUn(HMAP hMap, const WCHAR *name,
                                         METAFILEBUILDPARM *parm);
_MAPIMP long int _MAPAPI mapPaintToEmfEx(HMAP hMap, const char *name,
                                         METAFILEBUILDPARMEX *parm);
_MAPIMP long int _MAPAPI mapPaintToEmfExUn(HMAP hMap, const WCHAR *name,
                                           METAFILEBUILDPARMEX *parm);

#endif  // WIN32API

 // Отобразить произвольный объект в пределах фрагмента
 // в условных знаках пользователя
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture).
 // image - описание вида объекта (см. MAPGDI.H),
 // data  - координаты объекта.
 // Координаты объекта заданы в соответствии с параметром place
 // С изображениями больше 32000х32000 работает только
 // под Windows NT (Windows 95 имеет 16-битовую графику)
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintUserObject(HMAP hmap, HDC hdc, RECT *rect,
                                            PAINTPARM *image, PLACEDATA *data,
                                            long int place/* = PP_MAP*/);

 // Отобразить произвольный объект в пределах фрагмента
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture).
 // image - описание вида объекта (см. MAPGDI.H),
 // data  - координаты объекта.
 // offset - смещение координат объекта (в соответствии с place)
 // Координаты объекта заданы в соответствии с параметром place
 // Корректно работает с большими изображениями под Windows95 и NT,
 // При ошибке в параметрах возвращает ноль

_MAPIMP  long int _MAPAPI mapPaintOffsetUserObject95(HMAP hmap, HDC hdc,
                                                     RECT *rect, PAINTPARM *image,
                                                     PLACEDATA *data,
                                                     DOUBLEPOINT *offset,
                                                     long int place);

 // Отобразить произвольный объект в пределах фрагмента
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture),
 // image - описание вида объекта (см. MAPGDI.H),
 // data  - координаты объекта.
 // Координаты объекта заданы в соответствии с параметром place
 // Корректно работает с большими изображениями под Windows95 и NT,
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapPaintUserObject95(HMAP hmap, HDC hdc,
                                              RECT *rect, PAINTPARM *image,
                                              PLACEDATA *data, long int place);

 // Отобразить образец вида объекта по номеру записи
 // в классификаторе объектов (incode)
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture),
 // Используется в диалогах выбора вида объекта
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObject(HMAP hmap, HDC hdc,
                                               RECT *rect, long int incode);

 // Отобразить образец вида объекта c учетом типа визуализации по
 // номеру записи в классификаторе объектов (incode)
 // Используется в диалогах выбора вида объекта
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture),
 // visualtype - тип визуализации
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectEx(HMAP hmap, HDC hdc, RECT *rect,
                                                 long int incode, long int visualtype);

 // Отобразить образец вида объекта c учетом типа визуализации по      // 23/02/14
 // номеру записи в классификаторе объектов (incode)
 // Используется в диалогах выбора вида объекта
 // hmap - идентификатор открытых данных
 // hdc  - идентификатор контекста устройства вывода,
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture),
 // visualtype - тип визуализации (VT_SCREEN, VT_PRINT)
 // text - текст для отображения знака типа подпись
 // factor - процент изменения размера знака для отображения больших знаков
 //          в маленьких окошках (50 - сжать в 2 раза, 150 - увеличить в 1,5 раза)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectUn(HMAP hMap, HDC hdc,RECT* rect,
                                                 long int incode, long int visualtype,
                                                 const WCHAR * text, long int factor);

 // Отобразить образец вида объекта по номеру функции
 // hmap       - идентификатор открытых данных
 // hdc        - контекст устройства
 // rect       - область отображения
 // func       - номер функции
 // parm       - указатель на параметры функции
 // colors     - число цветов палитры
 // palette    - указатель на палитру
 // visualtype - тип визуализации
 // text       - текст всегда в кодировке WINDOWS, кроме функции IMG_TEXT
 //              (кодировка задана в параметрах структуры IMGTEXT)
 // Параметры colors, palette, visualtype, text - могут быть нулевыми :
 //  colors,palette - если в параметрах функции индексы палитры не используются;
 //  visualtype     - если отображение выполняется в текущем режиме визуализации;
 //  text           - при отображении стандартного текста ("AaBbCc").
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectFunc(HMAP hmap, HDC hdc, RECT rect,
                                                   long int func, char* parm,
                                                   long int colors, COLORREF* palette,
                                                   long int visualtype, const char* text);
_MAPIMP long int _MAPAPI mapPaintExampleObjectFuncUn(HMAP hMap, HDC hdc, RECT rect,
                                                     long int func, char* parm,
                                                     long int colors, COLORREF* palette,
                                                     long int visualtype, const WCHAR *text);

 // Отобразить образец вида объекта по номеру функции     // 09/02/16
 //  hdc        - контекст устройства
 //  rect       - область отображения
 //  func       - номер функции
 //  local      - локализация для задания вида метрики
 //  parm       - указатель на параметры функции
 //  colors     - число цветов палитры
 //  palette    - указатель на палитру
 //  visualtype - тип визуализации
 //  erase      - 0 - не стирать, 1 - стирать изображение
 //  factor - коэффициент увеличения образца ( по умолчанию 0.0)
 //  text       - текст всегда в кодировке WINDOWS, кроме функции IMG_TEXT
 //               (кодировка задана в параметрах структуры IMGTEXT)
 // Параметры colors, palette, visualtype, text - могут быть нулевыми :
 //  colors,palette - если в параметрах функции индексы палитры не используются;
 //  visualtype     - если отображение выполняется в текущем режиме визуализации;
 //  text           - при отображении стандартного текста ("AaBbCc").
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleFunc(HMAP hMap, HDC hdc, RECT rect,
                                      long int func, const char * parm,
                                      long int colors, COLORREF* palette,
                                      long int visualtype, const WCHAR * text,
                                      long int local,HRSC hRsc, long int erase);

 // Отобразить образец вида объекта по заданной метрике
 // в реальных цветах (с интенсивностью заливки - 100%)
 // hmap       - идентификатор открытых данных,
 // hdc        - контекст устройства
 // rect       - область отображения
 // func       - номер функции
 // parm       - указатель на параметры функции
 // data       - метрика изображения в точках устройства вывода
 // colors     - число цветов палитры
 // palette    - указатель на палитру
 // visualtype - тип визуализации
 // text - текст всегда в кодировке WINDOWS
 //        (если data != 0, то text игнорируется)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintExampleObjectRealByFuncData(HMAP hmap, HDC hdc, RECT* rect,
                                                             long int func, const char* parm, POLYDATA* data,
                                                             long int colors, COLORREF* palette,
                                                             long int visualtype, const char* text);

_MAPIMP long int _MAPAPI mapPaintExampleObjectRealByFuncDataUn(HMAP hMap, HDC hdc, RECT* rect,
                                                               long int func, const char* parm, POLYDATA* data,
                                                               long int colors, COLORREF* palette,
                                                               long int visualtype, const WCHAR *text);

 // Выделить на карте объекты, удовлетворяющие заданным условиям
 // hmap   - идентификатор открытых данных
 // hdc    - контекст устройства
 // rect   - область отображения
 // select - условия отбора объектов
 // color  - цвет, которым будут выделяться объекты на карте
 // Максимальный размер картинки, равен разрешению экрана (рабочего стола)

_MAPIMP void _MAPAPI mapPaintSelect95(HMAP hmap, HDC hdc, RECT *rect,
                                      HSELECT select, COLORREF color);

 // Установить/Запросить способ выделения площадных и линейных
 // объектов на карте
 // type - способ выделения (STF_CONTOUR - контур объекта,
 // STF_OBJECT - весь объект)
 // Возвращает значение, которое было ранее установлено

_MAPIMP long int _MAPAPI mapSetSelectType(long int type);
_MAPIMP long int _MAPAPI mapGetSelectType();

#ifndef LINUXAPI
 // Отобразить фрагмент карты на заданном устройстве с учетом калибровки
 // в текущих масштабе и составе объектов
 // hmap   - идентификатор открытых данных
 // rect - координаты фрагмента карты (Draw) в изображении (Picture)
 // parm - параметры печати, указатель на PRINTPARM (см. prnapi.h)

_MAPIMP void _MAPAPI mapPrint(HMAP hmap, HDC hDC, RECT *rect, const char *parm);
#endif

 // Установить пошаговый вывод DIB в окно
 // millisec - интервал вывода промежуточных изображений в контекст вывода
 //            в миллисекундах (при нулевом значении изображение выводится один раз по готовности)
 // Возвращает предыдущее значение

_MAPIMP long int _MAPAPI mapSetPaintStepEx(HMAP hmap, long int millisec);

 // Запросить значение пошагового вывода DIB в окно
 // Возвращает интервал вывода промежуточных изображений в контекст вывода
 // в миллисекундах (при нулевом значении изображение выводится один раз по готовности)
 // Если hmap равно нулю, то возвращает ноль

_MAPIMP long int _MAPAPI mapGetPaintStepEx(HMAP hmap);

 // Установить режим качественного отображения подписей
 // для строгого соблюдения размеров подписи при печати
 // flag - ненулевое значение устанавливает режим качественного вывода
 //        (при этом время отображения отдельной подписи увеличивается
 //         примерно в 2 раза)
 // При старте программы установлен режим качественного отображения
 // Возвращает предыдущее значение

_MAPIMP long int _MAPAPI mapSetTextQuality(long int flag);

 // Запросить режим качественного отображения подписей
 // для строгого соблюдения размеров подписи при печати

_MAPIMP long int _MAPAPI mapGetTextQuality();

  /*********************************************************
  *                                                        *
  *      ФУНКЦИИ ОТОБРАЖЕНИЯ И ПЕЧАТИ ЭЛЕКТРОННОЙ КАРТЫ    *
  *   ЗАДАН ИДЕНТИФИКАТОР ОКНА, И ПОЗИЦИЯ ФРАГМЕНТА ВЫВОДА *
  *                ЭЛЕКТРОННОЙ КАРТЫ В ОКНЕ                *
  *                                                        *
  *********************************************************/

 // Выделить на карте объекты, удовлетворющие заданным условим
 // hmap   - идентификатор открытых данных
 // hwnd   - идентификатор окна вывода
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // select - условия отбора объектов,
 // color  - цвет, которым будут выделяться объекты на карте

_MAPIMP void _MAPAPI mapViewSelect(HMAP hmap, HWND hwnd, DOUBLEPOINT *point,
                                   HSELECT select, COLORREF color,
                                   long int place);

 // Установить/Запросить выделение объектов на карте с разрядкой     // 29/07/14
 // hmap - идентификатор открытых данных
 // flag - признак отображения выделенных объектов с разрядкой,
 // позволяет отображать выделенные объекты выборочно, когда
 // выделяются тысячи объектов
 // Возвращает установленное значение

_MAPIMP long int _MAPAPI mapSetLiteSelect(HMAP hmap, long int flag);
_MAPIMP long int _MAPAPI mapGetLiteSelect(HMAP hmap);

 // Отобразить фрагмент карты в клиентной области окна
 // в текущих масштабе и составе объектов
 // erase - признак стирания фона перед выводом,
 // (0 - фон не стирать, !=0 - очистить фрагмент цветом фона),
 // point - координаты верхнего левого угла фрагмента
 // в соответсвующей параметру place системе координат

_MAPIMP void _MAPAPI mapView(HMAP hmap, HWND hwnd, long int erase,
                             DOUBLEPOINT *point, long int place);

#ifdef WIN32API
 // Отобразить фрагмент карты в клиентной области окна
 // в текущих масштабе и составе объектов
 // hMap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // erase - признак стирания фона перед выводом,
 // (0 - фон не стирать, !=0 - очистить фрагмент цветом фона),
 // rect - координаты фрагмента карты (Draw)
 // в изображении (Picture),
 // point - смещение фрагмента в отображаемом окне

_MAPIMP void _MAPAPI mapViewEx(HMAP hmap, HWND hwnd, long int erase,
                               RECT *rect, POINT *point);
#endif  // WIN32API

 // Отобразить произвольный объект в пределах
 // клиентной области окна
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // info - идентификатор объекта карты в памяти
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapViewObject(HMAP hmap, HWND hwnd,
                                       DOUBLEPOINT *point,
                                       HOBJ info, long int place);

 // Отобразить существующий объект карты в пределах фрагмента окна (карты)
 // в условных знаках пользователя
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // image - описание вида объекта (см. MAPGDI.H), если объект
 // должен рисоваться своим условным знаком - значение параметра
 // можно установить в ноль,
 // info - идентификатор объекта карты в памяти
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapViewMapObject(HMAP hmap, HWND hwnd,
                                          DOUBLEPOINT *point,
                                          PAINTPARM *image,
                                          HOBJ info, long int place);

 // Отобразить существующий объект карты в пределах фрагмента окна (карты)
 // в условных знаках пользователя
 // hmap   - идентификатор открытых данных
 // image - описание вида объекта (см. MAPGDI.H), если объект
 // должен рисоваться своим условным знаком - значение параметра
 // можно установить в ноль,
 // info - идентификатор объекта карты в памяти
 // offset - смещение координат объекта (в соответствии с place)
 // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapViewOffsetMapObject(HMAP hmap, HWND hwnd,
                                                DOUBLEPOINT *point,
                                                PAINTPARM *image,
                                                HOBJ info, DOUBLEPOINT *offset,
                                                long int place);

  // Отобразить произвольный объект в пределах фрагмента окна (карты)
  // hmap   - идентификатор открытых данных
  // hwnd  - идентификатор окна,
  // point - координаты верхнего левого угла окна на карте
  // в соответсвующей параметру place системе координат
  // image - описание вида объекта (см. MAPGDI.H),
  // data  - координаты объекта.
  // offset - смещение координат объекта (в соответствии с place)
  // Координаты объекта заданы в соответствии с параметром place

_MAPIMP long int _MAPAPI mapViewOffsetUserObject(HMAP hmap, HWND hwnd,
                                                 DOUBLEPOINT *point,
                                                 PAINTPARM *image,
                                                 PLACEDATA *data,
                                                 DOUBLEPOINT *offset,
                                                 long int place);

 // Отобразить произвольный объект в пределах фрагмента окна (карты)
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // image - описание вида объекта (см. MAPGDI.H),
 // data  - координаты объекта.
 // Координаты объекта заданы в соответствии с параметром place

_MAPIMP long int _MAPAPI mapViewUserObject(HMAP hmap, HWND hwnd,
                                           DOUBLEPOINT *point,
                                           PAINTPARM *image,
                                           PLACEDATA *data,
                                           long int place);

 // Отобразить курсор в виде перекрестья в заданной точке
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте в пикселах
 // linethick - толщина линии курсора в пикселах (рекомендуется 1 - 2)
 // size  - размер курсора (обычно 16 или 32 - в зависимости от экрана)
 // x,y   - координаты центра курсора (соответсвуют place)

_MAPIMP void _MAPAPI mapViewMarkerEx(HMAP hMap, HWND hwnd, POINT *point,
                                     long int linethick, long int size,
                                     long int x, long int y);

 // Отобразить курсор в виде перекрестья в заданной точке
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // linethick - толщина линии курсора в пикселах (рекомендуется 1 - 2)
 // x,y   - координаты центра курсора (соответсвуют place)

_MAPIMP  void _MAPAPI mapViewMarker(HMAP hMap, HWND hwnd, DOUBLEPOINT *point,
                                    long int linethick, double x, double y,
                                    long int place);

 // Отобразить линию в виде заданного примитива
 // hmap   - идентификатор открытых данных
 // hwnd  - идентификатор окна,
 // point - координаты верхнего левого угла окна на карте
 // в соответсвующей параметру place системе координат
 // image - описание вида объекта (см. MAPGDI.H),
 // first - координаты первой точки,
 // second - координаты второй точки.

_MAPIMP  void _MAPAPI mapViewUserLine(HMAP hMap, HWND hwnd, DOUBLEPOINT *point,
                                      PAINTPARM *image, DOUBLEPOINT *first,
                                      DOUBLEPOINT *second, long int place);

  /*********************************************************
  *                                                        *
  *        ФУНКЦИИ УПРАВЛЕНИЯ ПРОЦЕССОМ ОТОБРАЖЕНИЯ        *
  *                                                        *
  *********************************************************/

  // Установить адрес функции, которая будет периодически
  // вызываться при построении изображения карты и выводе
  // его в заданный контекст отображения
  // Устанавливать адрес рекомендуется перед каждым вызовом Paint,
  // а по окончании отрисовки карты - отключать
  // (устанавливать нулевой адрес).
  // Вызываемая функция не должна сама вызывать Paint !
  // call - адрес вызываемой функции (см. maptype.h),
  // parm - параметр, который будет передан вызываемой функции.
  // Кроме параметра parm вызываемой функции передается признак
  // перерисовки карты (0/1) для ее пошагового обновления на экране
  // Если вызываемая функция вернет ненулевое значение, то
  // процесс отображения будет прерван (например, при анализе
  // очереди сообщений вызванной функцией найден ввод с
  // клавиатуры команды Esc, функция возвращает 1, отображение
  // прерывается).

_MAPIMP void _MAPAPI mapSetBreakCallAndParmEx(HMAP hMap, BREAKCALLEX call,
                                              void *parm);

  // Установить адрес функции, которая будет вызываться
  // перед формированием изображения карты в области
  // памяти или перед началом отображения карты на экран.
  // Вызываемая функция не должна сама вызывать Paint !
  // hmap   - идентификатор открытых данных
  // call - адрес вызываемой функции (см. maptype.h),
  // parm - параметр, который будет передан вызываемой функции.
  // В вызываемой функции можно рисовать "под картой"
  // с применением графических функций системы или функций
  // типа PaintUserObject.

_MAPIMP void _MAPAPI mapSetBeforePaintCallAndParm(HMAP hmap, BEFOREPAINT call,
                                                  void *parm);

  // Установить адрес функции, которая будет периодически
  // вызываться при построении изображения карты и выводе
  // его на экран. 
  // Устаревшая функция. Обратный вызов выполняется только после
  // отрисовки некоторого числа объектов, заданного функцией
  // mapSetPaintStep, или других данных (матриц, растров).
  // Позволяет прервать изображение карты,
  // сменить параметры отображения (масштаб, позиция карты...)
  // и запустить новый процесс отображения. Устанавливать
  // адрес рекомендуется перед каждым вызовом Paint,
  // а по окончании отрисовки карты - отключать
  // (устанавливать нулевой адрес).
  // Вызываемая функция не должна сама вызывать Paint !
  // hmap   - идентификатор открытых данных
  // call - адрес вызываемой функции (см. maptype.h),
  // parm - параметр, который будет передан вызываемой функции.
  // Если вызываемая функция вернет ненулевое значение, то
  // процесс отображения будет прерван (например, при анализе
  // очереди сообщений вызванной функцией найден ввод с
  // клавиатуры команды Esc, функция возвращает 1, отображение
  // прерывается).

_MAPIMP void _MAPAPI mapSetBreakCallAndParm(HMAP hmap, BREAKCALL call,
                                            void *parm);

  /*********************************************************
  *                                                        *
  *     ФУНКЦИИ НАСТРОЙКИ ОТОБРАЖЕНИЯ ЭЛЕКТРОННОЙ КАРТЫ    *
  *                     С ПОВОРОТОМ                        *
  *                                                        *
  *********************************************************/

 // Функция настройки отображения карты с поворотом
 //  hmap     - идентификатор открытых данных
 //  angle    - угол поворота карты в плане с вершиной в
 //             юго-западном углу карты (от -Pi до Pi)
 //  fixation - угол сектора фиксации поворота отображения карты
 //             относительно предыдущего положения (от 0 до Pi/6).
 //             По умолчанию = Pi/18 (10 градусов)
 // Угол fixation используется для минимизации дрожания изображения 
 // при движении по повернутой карте по прямой (или почти по прямой), 
 // когда при последовательном вызове функции подаются близкие 
 // значения угла поворота (angle). В случае, если разность между 
 // текущим углом поворота и требуемым будет меньше fixation, 
 // то новый угол поворота не устанавливается.
 // Возвращает значение установленного угла поворота.
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSetupTurn(HMAP hmap, double angle, double fixation);

 // Активен ли поворот ?
 // hmap - идентификатор открытых данных
 // Возвращает (1 - активен, 0 - нет)

_MAPIMP long int _MAPAPI mapTurnIsActive(HMAP hmap);

 // Запросить угол поворота
 // hmap - идентификатор открытых данных
 // Возвращает значения от -Pi до Pi

_MAPIMP double _MAPAPI mapGetTurnAngle(HMAP hmap);

  /*********************************************************
  *                                                        *
  *    ФУНКЦИИ УПРАВЛЕНИЯ ОТОБРАЖЕНИЕМ ЭЛЕКТРОННОЙ КАРТЫ   *
  *                                                        *
  *    ПРИ ИЗМЕНЕНИИ МАСШТАБА ОТОБРАЖЕНИЯ АВТОМАТИЧЕСКИ    *
  *       ИЗМЕНЯЮТСЯ ГАБАРИТЫ ИЗОБРАЖЕНИЯ В ПИКСЕЛАХ !     *
  *                                                        *
  *********************************************************/

  // Установить масштаб отображения (знаменатель масштаба)
  //  Параметры:
  //          hmap - идентификатор открытых данных
  //          x, y - координаты предполагаемого "центра изображения"
  //                 (любой точки привязки) в окне в текущем масштабе
  //         scale - реальный масштаб отображения, который желают получить
  //  Возвращает:   0 - масштаб не изменился,  1 - масштаб изменился
  //          x, y - координаты предполагаемого "центра изображения"
  //                 в окне относительно всей картинки
  //                 в новом масштабе отображения

_MAPIMP long int _MAPAPI mapSetViewScale(HMAP hmap, long int *x,
                                         long int *y, float scale);

  // Изменить масштаб отображения относительно текущего
  //  Параметры:
  //          hmap - идентификатор открытых данных
  //          x, y - координаты предполагаемого "центра изображения"
  //                 (любой точки привязки) в окне в текущем масштабе
  //        change - коэффициент изменения текущего масштаба изображения
  //                 0 < change < 1 , для сжатия
  //                 1 < change < N , для увеличения
  //  Возвращает:   0 - масштаб не изменился,  1 - масштаб изменился
  //          x, y - координаты предполагаемого "центра изображения"
  //                 в окне относительно всей картинки
  //                 в новом масштабе отображения

_MAPIMP long int _MAPAPI mapChangeViewScale(HMAP hmap, long int *x,
                                            long int *y, float change);

  // Запросить округленный масштаб отображения карты
  // hmap - идентификатор открытых данных
  // Возвращает значение знаменателя масштаба

_MAPIMP long int _MAPAPI mapGetShowScale(HMAP hmap);

  // Запросить точный масштаб отображения карты
  // hmap - идентификатор открытых данных
  // Возвращает значение знаменателя масштаба

_MAPIMP double _MAPAPI mapGetRealShowScale(HMAP hmap);

  // Установить точный масштаб отображения карты
  // Возвращает значение знаменателя масштаба

_MAPIMP double _MAPAPI mapSetRealShowScale(HMAP hMap, double scale);

  // Запросить текущий коэффициент масштабирования карты
  // Например: 5 - растянута в 5 раз относительно базового масштаба,
  //         0.1 - сжата в 10 раз.

_MAPIMP double _MAPAPI mapGetDrawScale(HMAP hmap);

  // Подобрать "стандартный" масштаб, ближайший к заданному (scale)

_MAPIMP long int _MAPAPI mapScaleToRoundScale(double scale);

  // Подобрать "стандартный" масштаб, ближайший к заданному (scale)
  // С учетом состава карты (для открытых данных WMST могут быть
  // другие стандартные масштабы)
  // hmap - идентификатор открытых данных
  // Возвращает новое значение знаменателя масштаба

_MAPIMP long int _MAPAPI mapScaleToRoundScaleEx(HMAP hmap, double scale);

  // Подобрать "стандартный" реальный масштаб, ближайший к заданному (scale)
  // с учетом состава карты (для открытых данных WMTS могут быть
  // другие стандартные масштабы)
  // hmap - идентификатор открытых данных
  // Возвращает новое реальное (неокругленное) значение знаменателя масштаба

_MAPIMP double _MAPAPI mapScaleToRoundScaleReal(HMAP hmap, double scale);   // 12/11/15

  // Запросить количество "стандартных" масштабов

_MAPIMP long int _MAPAPI mapScaleTableCount();

 // Запросить элемент таблицы "стандартных" масштабов по порядковому
 // номеру в таблице(с 1)

_MAPIMP long int _MAPAPI mapScaleTableItem(long int number);

  // Установить способ масштабирования объектов карты при отображении
  // method - способ масштабирования
  // (0 - картографический "с запаздыванием увеличения",
  //  1 - чертежный)
  // Возвращает ранее установленное значение

_MAPIMP long int _MAPAPI mapSetScaleMethod(long int method);
_MAPIMP long int _MAPAPI mapGetScaleMethod();

  // Пересчитать общие габариты района работ
  // hmap - идентификатор открытых данных
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRegion(HMAP hMap);

  // Запросить состав отображаемых объектов
  // hmap - идентификатор открытых данных
  // select - идентификатор контекста поиска/отображения
  // в который будут помещены текущие условия отображения
  // см. mapCreateMapSelectContext(...)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetViewSelect(HMAP hmap, HSELECT select);

  // Установить состав отображаемых объектов
  // hmap - идентификатор открытых данных
  // select - идентификатор устанавливаемого контекста поиска/отображения

_MAPIMP void _MAPAPI mapSetViewSelect(HMAP hmap, HSELECT select);

  // Запросить/Установить цвет фона отображаемой карты
  // hmap - идентификатор открытых данных
  // color - цвет фона
  // При изменении цвета фона необходимо перерисовать весь экран
  // При ошибке возвращает 0x0FFFFFF (белый)

_MAPIMP COLORREF _MAPAPI mapGetBackColor(HMAP hmap);

_MAPIMP COLORREF _MAPAPI mapSetBackColor(HMAP hmap, COLORREF color);

  // Запросить/Установить цвет фона отображаемой карты
  // при выводе в принтерном режиме на экран
  // При изменении цвета фона необходимо перерисовать весь экран
  // При ошибке возвращает 0x0FFFFFF (белый)

_MAPIMP COLORREF _MAPAPI mapGetBackPrintColor(HMAP hmap);

_MAPIMP COLORREF _MAPAPI mapSetBackPrintColor(HMAP hmap, COLORREF color);

  // Запросить/Установить способ отображения карты
  // hmap - идентификатор открытых данных
  // При установке нового способа отображения возвращается
  // предыдущее значение
  // (см. Maptype.h : VT_SCREEN(1), VT_PRINT(3), VT_PRINTRST(6),...)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetViewType(HMAP hmap);
_MAPIMP long int _MAPAPI mapSetViewType(HMAP hmap, long int type);

  // Запросить яркость карты (от -16 до +16)
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapGetBright(HMAP hmap);

  // Установить яркость карты (от -16 до +16)
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapSetBright(HMAP hmap, long int bright);

  // Запросить контрастность (от -16 до +16)
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapGetContrast(HMAP hmap);

  // Установить контрастность (от -16 до +16)
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapSetContrast(HMAP hmap, long int contrast);

  // Запросить интенсивность заливки площадей полигонов для
  // принтерного отображения (от 0 до 100)
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapGetIntensity(HMAP hmap);

  // Установить интенсивность заливки площадей полигонов для
  // принтерного отображения (от 0 до 100)
  // Если mapGetTransparentSquare возвращает 0, то значения интенсивности 
  // соответствуют расслаблению цветов (кроме черного):
  //   0 - заливок нет, 50 - полурасслабленные цвета, 100 - нормальные цвета
  // Если mapGetTransparentSquare возвращает 1, то значения интенсивности
  // соответствуют степени непрозрачности цветов:
  //   0 - заливок нет, 50 - полупрозрачные заливки, 100 - нормальные цвета
  // hmap - идентификатор открытых данных

_MAPIMP long int _MAPAPI mapSetIntensity(HMAP hmap, long int intensity);

  // Запросить флаг прозрачности заливки площадей полигонов
  // hMap  - идентификатор открытой карты
  // Возвращает значение флага

_MAPIMP long int _MAPAPI mapGetTransparentSquare(HMAP hMap);   // 21/06/16

  // Установить флаг прозрачности заливки площадей полигонов.
  // Степень прозрачности устанавливается функцией mapSetIntensity
  // hMap  - идентификатор открытой карты
  // flag  - признак прозрачности:
  //         0 - прозрачность отключена (заливки сплошным цветом),
  //             использовать при обычном отображении или печати карты;
  //             в принтерных режимах (VT_PRINT, VT_PRINTRST)
  //         1 - прозрачность включена (заливки в виде цветного стекла),
  //             использовать при отображении или печати карты поверх растров
  //             в принтерном растровом режиме (VT_PRINTRST)

_MAPIMP long int _MAPAPI mapSetTransparentSquare(HMAP hMap, long int flag);

  // Выдать вид отображения карты
  // 1 - с узлами, 0 - без узлов

_MAPIMP long int _MAPAPI mapGetNodeView(HMAP hMap);

  // Установить отображение узлов на карте
  // 0 - не отображать, 1 - отображать

_MAPIMP void _MAPAPI mapSetNodeView(HMAP hMap, long int mode);


  /*********************************************************
  *                                                        *
  *          ФОРМИРОВАНИЕ БУФЕРА ОКНА КАРТЫ                *
  *                                                        *
  *********************************************************/

  // Создать буфер карты в памяти для исключения мигания
  // перемещаемых по карте объектов
  // width  - ширина клиентской части окна карты в точках,
  // height - высота клиентской части окна карты в точках.
  // Создается первый буфер экрана, второй создается при первом вызове
  // функции отображения объекта в буфер (Draw) -  для оптимального
  // применения функций при отображении карты и без перемещаемых объектов
  // Размер буфера =  (width * height * 4)
  // Всего может быть параллельно открыто до 256 образов экранов одновременно
  // Может применяться в паре с функцией mapChangeImageSizeEx
  // При успешном выполнении возвращает идентификатор образа экрана
  // При ошибке возвращает ноль

_MAPIMP HIMAGE _MAPAPI mapCreateImageEx(long int width, long int height);

  // Создание буфера окна
  // hwnd - идентификатор окна, для определения размеров клиентской области
  // Может применяться в паре с функцией mapChangeImageSize
  // При ошибке возвращает ноль

_MAPIMP HIMAGE _MAPAPI mapCreateImage(HWND hwnd);

  // Удалить буфер окна
  // himage - идентификатор буфера окна

_MAPIMP void _MAPAPI mapCloseImage(HIMAGE image);

  // Запросить контекст буфера окна
  // himage - идентификатор буфера окна
  // При ошибке возвращает ноль

_MAPIMP HDC _MAPAPI mapGetImageDC(HIMAGE image);

  // Запросить контекст буфера окна для отображения объектов
  // himage - идентификатор буфера окна
  // При ошибке возвращает ноль

_MAPIMP HDC _MAPAPI mapGetImageObjectDC(HIMAGE image);

  // Отобразить содержимое буфера окна в заданный контекст
  // himage - идентификатор буфера окна
  // hdc - контекст области отображения (окна),
  // rect - координаты область отображения в буфере и контексте
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapViewImage(HIMAGE image, HDC hdc, RECT *rect);

  // Отобразить содержимое буфера окна в заданный контекст
  // himage - идентификатор буфера окна
  // hwnd - идентификатор окна, в которое выводится изображение,
  // rect - координаты область отображения в буфере и контексте
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapViewImageEx(HIMAGE image, HWND hwnd, RECT *rect);

  // Отобразить содержимое буфера окна в заданный контекст со смещением
  // himage - идентификатор буфера окна
  // hwnd - идентификатор окна, в которое выводится изображение,
  // rect - координаты область отображения в буфере и контексте
  // offset - смещение изображения в окне
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapViewImageExOffset(HIMAGE himage, HWND hwnd,
                                              RECT *rect, POINT *offset);

  // Сместить изображение карты во вспомогательном буфере
  // himage - идентификатор буфера окна
  // offset - смещение изображения в окне
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapViewImageOffset(HIMAGE himage, POINT *offset);

  // Обновить размеры буфера окна
  // himage - идентификатор буфера окна
  // erase  - признак очистки окна, если равен 0 - содержимое сохраняется
  // width  - новая ширина буфера
  // height - новая высота буфера
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapChangeImageSizeEx(HIMAGE image, long int erase,
                                              long int width, long int height);

  // Обновить размеры буфера окна
  // himage - идентификатор буфера окна
  // Срабатывает правильно только, если буфер создан через mapCreateImage(HWND hwnd);
  // Содержимое буфера теряется !
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapChangeImageSize(HIMAGE image);

  // Скроллинг буфера окна
  // himage - идентификатор буфера окна
  // dx     - величина смещения окна по горизонтали (> 0 - слева направо,
  //                                                 < 0 - справа налево)
  // dy     - величина смещения окна по вертикали   (> 0 - сверху вниз,
  //                                                 < 0 - снизу вверх)
  // onlymap - признак обновления только буфера карты (оптимизация скорости отрисовки
  //           при дальнейшем вызове mapClearImageObjects)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapScrollImageEx(HIMAGE himage, long int dx, long int dy, long int onlymap);

_MAPIMP long int _MAPAPI mapScrollImage(HIMAGE himage, long int dx, long int dy);

  // Обновить изображение заданного фрагмента карты в образе экрана
  // После обновления карты изображение перемещаемых объектов стирается
  // в пределах заданного фрагмента (но для стирания объектов текущим
  // видом карты достаточно вызвать mapClearImageObjects).
  // himage   - идентификатор буфера окна
  // hMap     - идентификатор открытых данных
  // rect     - обновляемый фрагмент карты, задается в пикселах в
  //            системе координат полного изображения карты (PICTURE)
  // position - положение верхнего левого угла фрагмента в клиентской
  //            области окна карты (и образа экрана)
  // onlymap  - признак обновления только буфера карты (оптимизация скорости отрисовки
  //            при дальнейшем вызове mapClearImageObjects)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageMapEx(HIMAGE himage, HMAP hMap, RECT *rect,
                                           POINT *position, long int onlymap);

_MAPIMP long int _MAPAPI mapDrawImageMap(HIMAGE image, HMAP hMap, RECT *rect,
                                         POINT *position);

  // Обновить изображение заданного фрагмента карты в образе экрана
  // После обновления карты изображение перемещаемых объектов стирается
  // в пределах фрагмента, заданного габаритами объекта
  // himage  - идентификатор буфера окна
  // object  - идентификатор объекта, в пределах габаритов которого будет обновлена карта
  // onlymap - признак обновления только буфера карты (оптимизация скорости отрисовки
  //           при дальнейшем вызове mapClearImageObjects)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReDrawImageMapExUnderObject(HIMAGE himage, HMAP hMap,
                                                        HOBJ object, long int onlymap);

  // Отобразить объект поверх карты местности в буфере экрана
  // himage  - идентификатор буфера окна
  // hMap - идентификатор открытых данных
  // parm - параметры отображения объекта карты в буфере экрана (второй буфер)
  // object - идентификатор описания объекта в памяти
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageMapObject(HIMAGE himage, HMAP hMap,
                                               PAINTPARM *parm, HOBJ object);

  // Отобразить объект поверх карты местности в буфере экрана c учетом
  // заданного сдвига метрики
  // himage  - идентификатор буфера окна
  // hMap - идентификатор открытых данных
  // offset - величина смещения изображения объекта в буфере от его реальных
  //          координат в метрах в текущей системе координат документа
  // parm - параметры отображения объекта карты в буфере экрана (второй буфер)
  // object - идентификатор описания объекта в памяти
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageOffsetMapObject(HIMAGE himage, HMAP hMap,
                                                     DOUBLEPOINT *offset,
                                                     PAINTPARM *parm, HOBJ object);

  // Отобразить объект поверх карты местности в буфере экрана
  // himage  - идентификатор буфера окна
  // hMap - идентификатор открытых данных
  // parm - параметры отображения объекта карты в буфере экрана (второй буфер)
  // data - список координат объекта в системе координат, заданной параметром place
  // place - вид системы координат (в точках экрана - PP_PICTURE, в метрах в
  //         системе координат документа - PP_PLANE, в радианах на эллипсоиде
  //         документа - PP_GEO)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageUserObject(HIMAGE himage, HMAP hMap,
                                                PAINTPARM *parm,
                                                PLACEDATA *data,
                                                long int place);

  // Отобразить объект поверх карты местности в буфере экрана c учетом
  // заданного сдвига метрики
  // himage  - идентификатор буфера окна
  // hMap - идентификатор открытых данных
  // offset - величина смещения изображения объекта в буфере от его реальных
  //          координат в системе координат, заданной параметром place
  // parm - параметры отображения объекта карты в буфере экрана (второй буфер)
  // data - список координат объекта в системе координат, заданной параметром place
  // place - вид системы координат (в точках экрана - PP_PICTURE, в метрах в
  //         системе координат документа - PP_PLANE, в радианах на эллипсоиде
  //         документа - PP_GEO)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageOffsetUserObject(HIMAGE himage, HMAP hMap,
                                                      DOUBLEPOINT *offset,
                                                      PAINTPARM *parm,
                                                      PLACEDATA *data,
                                                      long int place);

  // Отобразить графические данные
  // hScreen  - идентификатор образа экрана,
  // points   - координаты в пикселах
  // count    - число координат
  // image    - тип графического примитива (см. mapgdi.h)
  // parm     - параметры графического примитива

_MAPIMP long int _MAPAPI mapDrawImageGraphics(HIMAGE image, HMAP hMap,
                                              DRAWPOINT *points, long int count,
                                              long int type, const char *parm);

  // Отобразить текстовую строку (Arial)
  // image    - идентификатор образа экрана,
  // points   - координаты в пикселах
  // count    - число координат
  // text     - текст подписи
  // height   - высота подписи в мкм
  // color    - цвет подписи RGB
  // align    - флажки выравнивания текста (для -1 - FA_BASELINE|FA_LEFT)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageText(HIMAGE image, HMAP hMap,
                                          DRAWPOINT *points, long int count,
                                          const char *text, long int height,
                                          long int color, long int align);
_MAPIMP long int _MAPAPI mapDrawImageTextUn(HIMAGE image, HMAP hMap,
                                            DRAWPOINT *points, long int count,
                                            const WCHAR *text, long int height,
                                            long int color, long int align);

  // Отобразить BMP в образ экрана
  // image     - идентификатор образа экрана,
  // point     - координаты в пикселах,
  // bmpmemory - адрес массива байт, содержащего образ BMP-файла
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDrawImageBitMap(HIMAGE image, HMAP hMap,
                                            DRAWPOINT *point,
                                            const char *bmpmemory,
                                            long int transparent);

  // Стереть изображение объектов на заданном фрагменте образа
  // экрана (используется копия фона карты в отдельном буфере)
  // При ошибке возвращает ноль

_MAPIMP void _MAPAPI mapClearImageObjects(HIMAGE image, RECT *rect);


#ifdef WIN32API
  // Создать BITMAP с помощью функции CreateDIBSection
  // Размер элемента - 4 байта
  // hdc    - произвольный контекст
  // width  - ширина изображения
  // height - высота изображения
  // memory - адрес изображения в памяти
  // При ошибке возвращает ноль

_MAPIMP HBITMAP _MAPAPI mapCreateBitmap(HDC hdc, long int width, long int height,
                                        void ** memory);

_MAPIMP HBITMAP _MAPAPI mapCreateBitmapEx(long int width, long int height);

  // Удалить BITMAP, созданный mapCreateBitmap

_MAPIMP void _MAPAPI mapCloseBitmap(HBITMAP hbitmap);

  // Отрисовать карту в Bitmap
  // Размер изображения не больше размеров экрана
  // Для увеличения размеров отрисовываемого изображения
  // (до разумных размеров) можно вызвать mapSetMaxScreenImageSize()
  // erase - признак стирания фона перед выводом,
  //        (0 - фон не стирать, !=0 - очистить фрагмент цветом фона,
  //        для экранного способа вывода (VT_SCREEN) всегда стирает
  //        цветом фона, кроме значения -2 (минус 2))
  // rect - координаты фрагмента карты (Draw) в изображении (Picture)
  // При ошибке в параметрах возвращает 0

_MAPIMP long int _MAPAPI mapPaintToBitmap(HMAP hmap, HBITMAP hbitmap, int erase,
                                          RECT *rect);

  // Сохранить изображение с заданного контекста в файл BMP
  // Данная функция реализована только для платформы Windows !
  // hdc     - контекст устройства
  // rect    - размер сохраняемой области
  // bmpname - имя создаваемого файла BMP, 32 бита на пиксел
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapSaveImageToBMP(HDC hdc, RECT *rect, const char *bmpname);
_MAPIMP long int _MAPAPI mapSaveImageToBMPUn(HDC hdc, RECT *rect, const WCHAR *bmpname);

  // Считать изображение из файла BMP
  // rect    - размер считанной области
  // bmpname - имя считываемого файла BMP, 32 бита на пиксел
  // При ошибке в параметрах возвращает ноль

_MAPIMP HBITMAP _MAPAPI mapLoadFromBMP(RECT * rect, const char *bmpname);
_MAPIMP HBITMAP _MAPAPI mapLoadFromBMPUn(RECT *rect, const WCHAR *bmpname);

#endif

  /*********************************************************
  *                                                        *
  *          ОТОБРАЖЕНИЕ СЕТКИ НА ИЗОБРАЖЕНИИ КАРТЫ        *
  *                                                        *
  *********************************************************/

  // Запросить признак отображения сетки
  // hmap - идентификатор открытых данных
  // Если отображение сетки включено - возвращает ненулевое значение
  // 1 - отображение над картой, -1 - отображение под картой
  // Если сетка не отображается - возвращает ноль

_MAPIMP long int _MAPAPI mapIsGridActive(HMAP hmap);

  // Установить признак отображения сетки
  // hmap - идентификатор открытых данных
  // 1 - отображение над картой, -1 - отображение под картой
  // 0 - не отображать сетку
  // Если сетка не отображается - возвращает ноль

_MAPIMP long int _MAPAPI mapSetGridActive(HMAP hmap, long int active);

  // Запросить текущий шаг сетки
  // hmap - идентификатор открытых данных
  // Текущее значение может изменяться с учетом масштаба
  // предыдущего отображения документа
  // (маленькие ячейки меньше 8 пикселов, кратно увеличиваются)

_MAPIMP double _MAPAPI mapGetGridStep(HMAP hmap);

  // Запросить параметры сетки
  // hmap - идентификатор открытых данных
  // parm - указатель на область размещения результата
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetGridParm(HMAP hmap, GRIDPARM *parm);
_MAPIMP long int _MAPAPI mapGetGridParmEx(HMAP hmap, GRIDPARMEX *parm);

  // Установить параметры сетки
  // hmap - идентификатор открытых данных
  // parm - новые значения параметров отображения сетки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetGridParm(HMAP hmap, GRIDPARM *parm);
_MAPIMP long int _MAPAPI mapSetGridParmEx(HMAP hmap, GRIDPARMEX *parm);

  /*********************************************************
  *                                                        *
  *       ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ О ВЕКТОРНОЙ КАРТЕ     *
  *                                                        *
  *********************************************************/

  // Запросить базовый масштаб карты
  // hmap - идентификатор открытых данных
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMapScale(HMAP hmap);

  // Запросить название карты
  // hmap - идентификатор открытых данных
  // name - адрес буфера для результата запроса
  // size - размер буфера
  // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI mapGetMapName(HMAP hmap);
_MAPIMP long int _MAPAPI mapGetMapNameEx(HMAP hmap, char *name, long int size);

  // Запросить название карты в формате UNICODE
  // name - строка в кодировке UNICODE (2 байта на символ)
  // size - размер строки в байтах
  // При ошибке возвращает пустую строку

_MAPIMP long int _MAPAPI mapGetMapNameUn(HMAP hMap, WCHAR *name, long int size);

  // Запросить полный путь к паспорту главной карты
  // (функция вызывает mapGetMainMapName)
  // hmap - идентификатор открытых данных
  // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI mapGetMapPath(HMAP hmap);

  // Запросить полный путь к паспорту главной карты в формате UNICODE
  // (функция вызывает mapGetMainMapName)
  // hmap - идентификатор открытых данных
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMapPathUn(HMAP hmap, WCHAR *name, long int size);       // 24/10/14

  // Запросить тип карты
  // hmap - идентификатор открытых данных
  // Типы карт - см. mapcreat.h, MAPTYPE
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMapType(HMAP hmap);

  // Запросить - является ли карта пользовательской
  // Пользовательская карта состоит из одного листа переменных
  // размеров без рамки
  // В структуре MAPREGISTEREX пользовательская карта в поле
  // FlagRealPlace имеет ненулевое значение
  // Если карта пользовательская - возвращается ненулевое значение

_MAPIMP long int _MAPAPI mapIsMapSite(HMAP hmap);

  // Запросить прямоугольные координаты габаритов основной векторной карты
  // в метрах (система координат PLANE)
  // X - снизу вверх, Y - слева направо
  // т.1 - нижний левый угол,
  // т.2 - верхний правый
  // hmap - идентификатор открытых данных
  // При ошибке возвращает ноль (например, если первым открыт растр)

_MAPIMP double _MAPAPI mapGetMapX1(HMAP hmap);
_MAPIMP double _MAPAPI mapGetMapY1(HMAP hmap);
_MAPIMP double _MAPAPI mapGetMapX2(HMAP hmap);
_MAPIMP double _MAPAPI mapGetMapY2(HMAP hmap);

  // Запросить паспортные данные векторной карты
  // Структуры MAPREGISTER и LISTREGISTER описаны в mapcreat.h
  // hmap - идентификатор открытых данных
  // sheetnumber - номер листа карты для
  // которого запрашиваются паспортные данные
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMapInfo(HMAP hmap, long int sheetnumber,
                                       MAPREGISTER *map, LISTREGISTER *sheet);
_MAPIMP long int _MAPAPI mapGetMapInfoEx(HMAP hmap, long int sheetnumber,
                                         MAPREGISTEREX *map, LISTREGISTER *sheet);
_MAPIMP long int _MAPAPI mapGetMapInfoPro(HMAP hmap, long int sheetnumber,
                                          MAPREGISTEREX *map, LISTREGISTER *sheet,
                                          SHEETNAMES * sheetnames);         // 07/11/14

  // Запросить паспортные данные векторной карты
  // по имени файла - паспорта карты (MAP,SIT) (name)
  // Структуры MAPREGISTER и LISTREGISTER описаны в mapcreat.h
  // sheetnumber - номер листа карты для
  // которого запрашиваются паспортные данные
  // При ошибке возвращает ноль,
  // иначе - число объектов на карте (если на карте нет объектов, возвращает -1)

_MAPIMP long int _MAPAPI mapGetMapInfoByName(const char *name,
                                             long int sheetnumber,
                                             MAPREGISTER *map,
                                             LISTREGISTER *sheet);
_MAPIMP long int _MAPAPI mapGetMapInfoByNameUn(const WCHAR *name,
                                               long int sheetnumber,
                                               MAPREGISTER *mapreg,
                                               LISTREGISTER *sheet);
_MAPIMP long int _MAPAPI mapGetMapInfoByNameEx(const char *name,
                                               long int sheetnumber,
                                               MAPREGISTEREX *map,
                                               LISTREGISTER *sheet);
_MAPIMP long int _MAPAPI mapGetMapInfoByNameExUn(const WCHAR *name,
                                                 long int sheetnumber,
                                                 MAPREGISTEREX *mapreg,
                                                 LISTREGISTER *sheet);


  // Запросить имя классификатор карты по имени карты   
  // name    - имя файла паспорта карты (MAP,SIT)
  // rscname - имя файла RSC
  // size    - размер буфера для размещения имени RSC
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscByName(const char *name, char *rscname, long int size);
_MAPIMP long int _MAPAPI mapGetRscByNameUn(const WCHAR *name, WCHAR *rscname, long int size);

  // Контроль номенклатуры карты
  // nomenclature - строка с номенклатурой
  // length - длина строки
  // type - тип карты (из MAPTYPE)
  // scale - масштаб  (1000000,500000,200000 и т.д.
  //                   соответствующий типу карты)

_MAPIMP long int _MAPAPI mapCheckNomenclature(const char *nomenclature,
                                              long int length,
                                              long int type, long int scale);
_MAPIMP long int _MAPAPI mapCheckNomenclatureUn(const WCHAR *nomenclature,
                                                long int type, long int scale);

  // Формирование имени файла по номенклатуре (удаляет точки, пробелы, -)
  // filename - место под имя файла
  // filelength - длина строки под имя файла
  // nomenclature - номенклатура листа
  // при ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetFileNameFromNomenclature(char *filename, int filelength,
                                                        const char *nomenclature);

  // Формирование имени файла по номенклатуре (удаляет точки, пробелы, -)
  // filename - буфер для имени файла
  // namesize - размер буфера в БАЙТАХ
  // nomenclature - номенклатура листа
  // при ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetFileNameFromNomenclatureUn(WCHAR *filename, int namesize,
                                                          const WCHAR *nomenclature);

  // Расчет данных на лист топографической карты
  // Равноугольная проекция Гаусса-Крюгера
  // hmap - идентификатор открытых данных
  // Структуры MAPREGISTER и LISTREGISTER описаны в mapcreat.h
  // входные данные   заполнены  поля в mapreg:
  //                - тип карты
  //                - масштаб
  //                - проекции исходного материала
  //                  заполнены  поля в sheet:
  //                - номенклатура
  // выходные данные  заполнены  поля в mapreg:
  //                 - осевой меридиан (при создании карты( hMap = 0))
  //                  заполнены  поля в sheet:
  //                 - геодезические координаты,
  //                 - прямоугольные координаты,
  //                 - сближение меридианов
  // Если mapreg и sheet заполняются для создания карты (самый первый лист)
  //  - hmap = 0
  // Если mapreg заполняется для добавления листа в карту
  //  - hmap != 0

_MAPIMP long int _MAPAPI mapCalcTopographicSheet(HMAP hmap,
                                                 MAPREGISTER *mapreg,
                                                 LISTREGISTER *sheet);

_MAPIMP long int _MAPAPI mapCalcTopographicSheetEx(HMAP hmap,
                                                   MAPREGISTEREX *mapreg,
                                                   LISTREGISTER *sheet);

  // Рассчитать осевой меридиан (от 0 до 360) по номенклатуре топокарты
  // nomenclature - номенклатура листа
  // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapGetAxisMeridianFromNomenclature(const char *nomenclature);

                                                   
  // Установить признак повышенной точности хранения координат
  // Вызывается после создания карты (Map) или плана (Plane)
  // для установки максимальной точности хранения координат
  // hmap -  идентификатор открытых данных
  // Объем метрики вдвое больше, точность хранения - максимальная
  // Нормальная точность хранения зависит от масштаба - для 100 000 - не
  // хуже 0,5 метра, для 10 000 - не хуже 0,05 метра
  // Все пользовательские карты, карты и планы местности
  // масштабов 5 000 и крупнее - всегда повышенной точности
  // Степень точности устанавливается при создании карты в поле DeviceCapability
  // структуры MAPREGISTEREX отрицательным значением (см. mapcreat.h),
  // в этом случае вызов mapSetMapPrecision не нужен

_MAPIMP void _MAPAPI mapSetMapPrecision(HMAP hmap);

  // Запросить признак повышенной точности хранения координат
  // hmap -  идентификатор открытых данных
  // Возвращает значения:
  // 1 - максимальная точность хранения,
  // 2 - с точностью 2 знака (сантиметры),
  // 3 - с точностью 3 знака (миллиметры)
  // При ошибке или нормальной точности хранения координат возвращает ноль

_MAPIMP long int _MAPAPI mapGetMapPrecision(HMAP hmap);


  /*********************************************************
  *                                                        *
  *     ФУНКЦИИ ЗАПРОСА ХАРАКТЕРИСТИК МЕТАДАННЫХ КАРТЫ     *
  *                                                        *
  *********************************************************/

  // Запросить количество эллипсоидов в списке

_MAPIMP long int _MAPAPI mapGetEllipsoidCount();

  // Запросить название эллипсоида по коду
  // Параметры  code  - код эллипсоида
  //            name  - адрес строки для размещения названия эллипсоида
  //            size  - длина выделенной области под строку в БАЙТАХ
  // При ошибке возвращает 0
  // name содержит значение "Не установлено"

_MAPIMP long int _MAPAPI mapGetEllipsoidNameByCode(long int code, char *name,
                                                   long int size);

_MAPIMP long int _MAPAPI mapGetEllipsoidNameByCodeUn(long int code, WCHAR* name,
                                                     long int size);

  // Запрос кода и названия эллипсоида по номеру в таблице
  // Параметры  number  - номер строки таблицы эллипсоидов (номер начинается с 1)
  //            code    - код эллипсоида
  //            name    - адрес строки для размещения названия эллипсоида
  //            size    - длина выделенной области под строку в БАЙТАХ
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetEllipsoidByNumber(long int number, long int &code,
                                                 char *name, long int size);

_MAPIMP long int _MAPAPI mapGetEllipsoidByNumberUn(long int number, long int *code,
                                                   WCHAR *name, long int size);

  // Запросить по коду EPSG номер эллипсоида (см. MAPCREAT.H)
  // для заполнения структуры MAPREGISTEREX
  // code - код EPSG
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetEllipsoidByEPSGCode(long int code);

  // Запросить код EPSG эллипсоида по его коду (см. MAPCREAT.H)
  // ellipsoid - номер эллипсоида (см. MAPCREAT.H, ELLIPSOIDKIND)
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetEllipsoidEPSGCode(long int code);

  // Запрос количества типов карт в списке

_MAPIMP long int _MAPAPI mapGetMapTypeCount();

  // Запрос названия типа карты по коду
  // Параметры  code  - код типа карты
  //            name  - адрес строки для размещения названия типа карты
  //            size  - длина выделенной области под строку в БАЙТАХ
  // При ошибке возвращает 0
  // name содержит значение "Не установлено"

_MAPIMP long int _MAPAPI mapGetMapTypeByCode(long int code, char *name,
                                             long int size);

_MAPIMP long int _MAPAPI mapGetMapTypeByCodeUn(long int code, WCHAR* name,
                                               long int size);

  // Запрос кода и названия типа карты по номеру в таблице
  // Параметры  number  - номер строки таблицы типа карты (номер начинается с 1)
  //            code    - код типа карты
  //            name    - адрес строки для размещения названия типа карты
  //            size    - длина выделенной области под строку в БАЙТАХ
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMapTypeByNumber(long int number, long int &code,
                                               char *name, long int size);

_MAPIMP long int _MAPAPI mapGetMapTypeByNumberUn(long int number, long int *code,
                                                 WCHAR *name, long int size);


  // Запрос количества проекций в списке

_MAPIMP long int _MAPAPI mapGetProjectionCount();

  // Запрос названия проекции по коду
  // Параметры  code  - код проекции
  //            name  - адрес строки для размещения названия проекции
  //            size  - длина выделенной области под строку в байтах
  // При ошибке возвращает 0
  // name содержит значение "Не установлено"
_MAPIMP long int _MAPAPI mapGetProjectionNameByCode(long int code, char *name,
                                                    long int size);

_MAPIMP long int _MAPAPI mapGetProjectionNameByCodeUn(long int code, WCHAR *name,
                                                      long int size);

  // Запрос кода и названия проекции по номеру в таблице
  // Параметры  number  - номер строки таблицы проекций (номер начинается с 1)
  //            code    - код проекции
  //            name    - адрес строки для размещения названия проекции
  //            size    - длина выделенной области под строку в БАЙТАХ
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetProjectionByNumber(long int number, long int &code,
                                                  char *name, long int size);

_MAPIMP long int _MAPAPI mapGetProjectionByNumberUn(long int number, long int *code,
                                                    WCHAR *name, long int size);

  /*********************************************************
  *                                                        *
  *       ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ О СЛОЯХ КАРТЫ         *
  *                                                        *
  *********************************************************/

 // Запросить число слоев на карте
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetLayerCount(HMAP hmap);

 // Запросить название слоя по его номеру (number)
 // hmap - идентификатор открытых данных
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // Номер первого слоя 0
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetLayerName(HMAP hmap, long int number);

_MAPIMP long int _MAPAPI mapGetLayerNameEx(HMAP hmap, long int number,
                                           char *name, long int size);

_MAPIMP long int _MAPAPI mapGetLayerNameUn(HMAP hmap, long int number,
                                           WCHAR *name, long int size);

  /*********************************************************
  *                                                        *
  *       ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ О ЛИСТАХ КАРТЫ        *
  *                                                        *
  *********************************************************/

 // Определить собственный номер листа по заданным координатам (x,y).
 // Система координат задана переменной place.
 // Если лист не найден - возвращает ноль.
 // Если в одной точке несколько листов :
 // hmap - идентификатор открытых данных
 // number - порядковый номер листа в перекрытии (начина с 1).
 // Поиск всегда дает одинаковый порядок листов
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapWhatListNumber(HMAP hmap, double  x,double y,
                                           long int number, long int place);

 // Запросить номенклатуру листа по заданным координатам (x,y).
 // hmap - идентификатор открытых данных
 // name - адрес буфера для результата запроса
 // size - размер буфера
 // Система координат задана переменной place.
 // Если лист не найден - возвращает ноль

_MAPIMP const char* _MAPAPI mapWhatListName(HMAP hmap, double x, double y,
                                            long int number, long int place);

_MAPIMP long int _MAPAPI mapWhatListNameEx(HMAP hmap, double x, double y,
                                           long int number, long int place,
                                           char *name, long int size);

_MAPIMP long int _MAPAPI mapWhatListNameUn(HMAP hmap, double x, double y,
                                           long int number, long int place,
                                           WCHAR *name, long int size);

 // Запросить имя листа по его номеру (number)
 // hmap - идентификатор открытых данных
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetSheetName(HMAP hmap, long int number);

_MAPIMP long int _MAPAPI mapGetSheetNameEx(HMAP hmap, long int number,
                                           char *name, long int size);

_MAPIMP long int _MAPAPI mapGetSheetNameUn(HMAP hmap, long int number,
                                           WCHAR *name, long int namesize);

 // Запросить номенклатуру листа по его номеру (number)
 // hmap - идентификатор открытых данных
 // name - адрес буфера для результата запроса
 // size - размер буфера
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetListName(HMAP hmap,long int number);

_MAPIMP long int _MAPAPI mapGetListNameEx(HMAP hmap,long int number,
                                          char *name,long int size);

_MAPIMP long int _MAPAPI mapGetListNameUn(HMAP hmap, long int number,
                                          WCHAR *name, long int size);

 // Запросить общее число листов в районе
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetListCount(HMAP hmap);

 // Запросить общее число объектов в листе
 // hmap - идентификатор открытых данных
 // list - номер листа
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetObjectCount(HMAP hmap, long int list);

 // Запросить общее число объектов в листе, исключая удаленные
 // number - номер листа
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRealObjectCount(HMAP hMap, long int number);

 // Запросить номер листа по его номенклатуре
 // hmap - идентификатор открытых данных
 // name - имя листа
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetListNumberByName(HMAP hmap, const char *name);

_MAPIMP long int _MAPAPI mapGetListNumberByNameUn(HMAP hMap, const WCHAR *name);

 // Определить по номенклатуре листа его принадлежность карте
 // hmap - идентификатор открытых данных
 // listname - имя листа (номенклатура)
 // Возвращает номер карты в цепочке карт,которой принадлежит
 // лист по имени listname
 // (0-фоновая карта, 1-первая пользовательская карта и т.д.)
 // При ошибке возвращает "-1"

_MAPIMP long int _MAPAPI mapWhatListLayoutIs(HMAP hmap, const char *listname);

_MAPIMP long int _MAPAPI mapWhatListLayoutIsUn(HMAP hMap, const WCHAR *listname);

 // Запросить объект "Рамка листа"
 // hmap - идентификатор открытых данных
 // list - номер листа (c 1)
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetListFrameObject(HMAP hmap, long int list,
                                               HOBJ info);

 // Запросить габариты объекта "Рамка листа" (если рамки нет -
 // заполняются по габаритам из паспорта)
 // hmap  - идентификатор открытых данных
 // list  - номер листа
 // frame - указатель на габариты листа в метрах
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetListFrame(HMAP hmap, int list, DFRAME *frame);

 // Создать объект "Рамка листа"
 // hmap - идентификатор открытых данных
 // list - последовательный номер листа карты (c 1)
 // info - идентификатор объекта карты в памяти
 // HOBJ должен быть создан вызовом mapCreateObject
 // при успешном выполнении HOBJ будет содержать созданную
 // или существующую рамку листа
 // Для пользовательской карты рамка создается, но не записывается
 // Для карты местности созданная рамка сохраняется на карте
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateListFrameObject(HMAP hmap, long int list,
                                                  HOBJ info);

 // Установить ограничение на число листов, открытых одновременно 
 // Применяется при работе с многолистовыми картами местности в
 // ограниченной области памяти
 // hmap - идентификатор открытых данных
 // islimited - признак установки ограничения числа открытых листов, 
 // обычно от 8 до 32, но не менее числа потоков, обрабатывающих листы параллельно
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetActiveListCountLimit(HMAP hmap, long int islimited = 32);

  /*********************************************************
  *                                                        *
  * ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ ИЗ КЛАССИФИКАТОРА ОБ'ЕКТОВ  *
  *                                                        *
  *********************************************************/

 // Запросить число объектов описанных в классификаторе
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscObjectCount(HMAP hmap);

 // Запросить число объектов описанных в классификаторе
 // в заданном слое
 // hmap - идентификатор открытых данных
 // layer - номер слоя в классификаторе
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscObjectCountInLayer(HMAP hmap, long int layer);

 // Запросить название объекта по порядковому номеру (number)
 // в заданном слое
 // hmap - идентификатор открытых данных
 // layer - номер слоя в классификаторе
 // size - размер строки в БАЙТАХ
 // При ошибке возвращает ноль или пустую строку

_MAPIMP const char* _MAPAPI mapRscObjectNameInLayer(HMAP hmap, long int layer,
                                                    long int number);

_MAPIMP long int _MAPAPI mapRscObjectNameInLayerEx(HMAP hMap, long int layer,
                                                   long int number,
                                                   char *name, long int size);

_MAPIMP long int _MAPAPI mapRscObjectNameInLayerUn(HMAP hMap, long int layer,
                                                   long int number, WCHAR *name,
                                                   long int size);

 // Запросить классификационный код объекта
 // по порядковому номеру в заданном слое (number)
 // hmap - идентификатор открытых данных
 // layer - номер слоя в классификаторе
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscObjectExcodeInLayer(HMAP hmap, long int layer,
                                                   long int number);

 // Запросить код локализации объекта
 // по порядковому номеру в заданном слое (number)
 // hmap - идентификатор открытых данных
 // layer - номер слоя в классификаторе
 // При ошибке возвращает ноль (ноль допустим)

_MAPIMP long int _MAPAPI mapRscObjectLocalInLayer(HMAP hmap, long int layer,
                                                  long int number);

 // Запросить внутренний код (индекс) объекта
 // по порядковому номеру в заданном слое (number)
 // hmap - идентификатор открытых данных
 // layer - номер слоя в классификаторе
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscObjectCodeInLayer(HMAP hmap, long int layer,
                                                 long int number);

 // Запросить внутренний код (индекс) объекта
 // по внешнему коду (excode) и локализации (local)
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscObjectCode(HMAP hMap, long int excode, long int local);

 // Запросить число семантических характеристик в классификаторе
 // hmap - идентификатор открытых данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscSemanticCount(HMAP hmap);

 // Запросить код семантической характеристики объекта
 // hmap - идентификатор открытых данных
 // number - последовательный номер характеристики в классификаторе (c 1)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscSemanticCode(HMAP hmap, long int number);

 // Запросить код семантической характеристики объекта
 // по короткому имени семантики
 // hmap - идентификатор открытых данных
 // key  - короткое имя семантики (ключ) в классификаторе
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRscSemanticCodeByKey(HMAP hmap, const char *key);

_MAPIMP long int _MAPAPI mapRscSemanticCodeByKeyUn(HMAP hMap, const WCHAR *key);

 // Запросить название семантической характеристики объекта
 // hmap - идентификатор открытых данных
 // code    - код характеристики
 // При ошибке возвращает ноль или пустую строку

_MAPIMP const char* _MAPAPI mapRscSemanticName(HMAP hmap, long int code);

_MAPIMP long int _MAPAPI mapRscSemanticNameUn(HMAP hmap, long int code,
                                              WCHAR *semname, long int size);

  /*********************************************************
  *                                                        *
  *           ПРЕОБРАЗОВАНИЕ КООРДИНАТ ТОЧКИ               *
  *                                                        *
  *********************************************************/

 // Запросить габариты района (всех видов карт)
 // hmap - идентификатор открытых данных
 // dframe - указатель на заполняемую структуру
 // Запрашиваются координаты углов района в метрах или радианах на местности
 // в картографической системе или в пикселах относительно верхнего левого угла
 // района
 // place  - запрашиваемая система координат (PP_PICTURE, PP_PLANE, PP_GEO)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetTotalBorder(HMAP hmap, DFRAME *dframe,
                                           long int place);

 // Запросить габариты района (всех видов карт) в системе координат,
 // заданной кодом EPSG, в метрах и радианах на местности
 // hmap - идентификатор открытых данных
 // dframeplane - указатель на заполняемую структуру в метрах
 // dframegeo   - указатель на заполняемую структуру в радианах
 // epsgcode    - код системы координат (3395, 3857, 4326 и т.д.)
 // Для геодезических систем координат возвращает 2,
 // для плоских прямоугольных возвращает 1.
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetTotalBorderByEPSG(HMAP hmap, DFRAME *dframeplane,
                                                 DFRAME *dframegeo, long int epsgcode);

 // Преобразование из дискретов на карте (районе работ)
 // в пикселы на изображении
 // hmap - идентификатор открытых данных
 // x,y  - преобразуемые координаты
 // на входе дискреты, на выходе - пикселы.
 // Применение :
 // xpix = xdis; ypix = ydis;
 // mapMapToPicture(xpix,ypix);

_MAPIMP void _MAPAPI mapMapToPicture(HMAP hmap, double *x, double *y);

 // Преобразование из дискретов на карте (районе работ)
 // в метры на местности
 // на входе дискреты, на выходе - метры.

_MAPIMP void _MAPAPI mapMapToPlane(HMAP hmap, double *x, double *y);

 // Преобразование из пикселов в изображении в координаты
 // карты (района работ) в дискретах
 // hmap - идентификатор открытых данных
 // x,y  - преобразуемые координаты

_MAPIMP void _MAPAPI mapPictureToMap(HMAP hmap, double *x, double *y);

 // Преобразование из пикселов в изображении в координаты
 // на местности в метрах
 // Применение :
 // xmet = xdis; ymet = ydis;
 // mapPictureToPlane(xmet,ymet);
 // hmap - идентификатор открытых данных
 // x,y  - преобразуемые координаты

_MAPIMP void _MAPAPI mapPictureToPlane(HMAP hmap, double *x, double *y);

 // Преобразование из метров на местности в дискреты
 // на карте (районе работ)
 // hmap - идентификатор открытых данных
 // x,y  - преобразуемые координаты

_MAPIMP void _MAPAPI mapPlaneToMap(HMAP hmap, double *x, double *y);

 // Преобразование из метров на местности в пикселы на
 // изображении
 // hmap - идентификатор открытых данных
 // x,y  - преобразуемые координаты

_MAPIMP void _MAPAPI mapPlaneToPicture(HMAP hmap, double *x, double *y);

 // Преобразование из метров на местности в геодезические
 // координаты в радианах в соответствии с проекцией карты
 // (поддерживаетс не для всех карт !)
 // Применение :
 // if (mapIsGeoSupported())   |  или :
 //   {                        |  if (mapIsGeoSupported())
 //     B = Xmet; L = Ymet;    |    {
 //     mapPlan2Geo(B,L);      |      mapPlan2Geo(B=Xmet,L=Ymet);
 //   }                        |    }
 // hmap - идентификатор открытых данных
 // Bx,Ly  - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneToGeo(HMAP hmap, double *Bx, double *Ly);

 // Преобразование из геодезических координат в радианах
 // в метры на местности в соответствии с проекцией карты
 // (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGeoToPlane(HMAP hmap, double *Bx, double *Ly);

 // Преобразование координат в метрах на местности из одной зоны в другую
 // source - номер исходной зоны системы 1942г
 // target - нужной зоны
 // x,y  - преобразуемые координаты
 // на входе метры в одной зоне 42г.,на выходе - другой.
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneToPlaneByZone(long int source, long int target,
                                               double *x, double *y);


 // Преобразование координат в метрах на местности из заданной зоны
 // в геодезические координаты в системе 42г.
 // zone - номер исходной зоны системы 1942г
 // x,y  - преобразуемые координаты
 // на входе метры в одной зоне 42г.,на выходе - радианы.
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlane42ToGeo42ByZone(long int zone, double *x, double *y);
_MAPIMP long int _MAPAPI mapPlaneToGeo42ByZone(long int zone, double *x, double *y);

 // Преобразование координат в метрах на местности из заданной зоны UTM
 // в геодезические координаты в системе WGS-84.
 // zone - номер исходной зоны системы UTM
 // x,y  - преобразуемые координаты
 // на входе метры в одной зоне UTM, на выходе - радианы WGS-84.
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneUTMToGeoWGS84ByZone(long int zone, double *x, double *y);

 // Преобразование координат в метрах на местности из одной зоны в другую
 // source - номер исходной зоны системы UTM
 // target - нужной зоны системы UTM
 // x,y  - преобразуемые координаты
 // на входе метры в одной зоне UTM, на выходе - метры в другой зоне UTM.
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneUTMToPlaneUTMByZone(long int source, long int target,
                                                     double *x, double *y);

 // Запрос - поддерживается ли пересчет к геодезическим
 // координатам из плоских прямоугольных и обратно
 // hmap - идентификатор открытых данных
 // Если нет - возвращает ноль

_MAPIMP long int _MAPAPI mapIsGeoSupported(HMAP hmap);

 // Преобразование из метров на местности (проекция карты)
 // в геодезические координаты в радианах (общеземной эллипсоид WGS84)
 // (поддерживается не для всех карт !)
 // Наличие высоты повышает точность расчетов,
 // функция mapPlaneToGeoWGS84() пытается
 // определить высоту из матрицы
 // Применение :
 // if (mapIsGeoSupported())
 //   {
 //     B = Xmet; L = Ymet;
 //     mapPlaneToGeoWGS84(hMap,B,L);
 //   }
 // hmap  - идентификатор открытых данных
 // Bx,Ly - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // H     - высота в точке (метры)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneToGeoWGS84(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapPlaneToGeoWGS843D(HMAP hmap, double *Bx, double *Ly,
                                              double *H);

 // Преобразование из метров на местности (проекция карты)
 // в геодезические координаты в радианах (эллипсоид Красовского, СК-42)
 // (поддерживается не для всех карт !)
 // hmap  - идентификатор открытых данных
 // Bx,Ly - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // H     - высота в точке (метры)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPlaneToGeo42(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapPlaneToGeo423D(HMAP hmap, double *Bx, double *Ly,
                                           double *H);

 // Преобразование из метров на местности (проекция карты)
 // в геодезические координаты в радианах (эллипсоид Красовского, СК-95)
 // (поддерживается не для всех карт !)
 // Если исходная карта в СК-42, то геодезические координаты выдаются в СК-42
 // без преобразования 
 // hmap  - идентификатор открытых данных
 // Bx,Ly - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // H     - высота в точке (метры)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPlaneToGeo953D(HMAP hmap, double *Bx, double *Ly,
                                           double *H);

                                          
 // Преобразование координат из геодезической системы координат карты к
 // в геодезические координаты в радианах (общеземной эллипсоид WGS84)
 // (поддерживается не для всех карт !)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoToGeoWGS843D(HMAP hMap, double *Bx, double *Ly,
                                            double *H);

 // Преобразование координат из геодезической системы координат в радианах
 // (общеземной эллипсоид WGS84) в геодезическую систему координат карты
 // (поддерживается не для всех карт !)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoWGS84ToGeo3D(HMAP hMap, double *Bx, double *Ly,
                                            double *H);

                                           
 //  Преобразование из геодезических координат в радианах
 // (общеземной эллипсоид WGS84)
 //  в метры на местности в системе 1942г (эллипсоид Красовского)
 // (поддерживается не для всех карт !)
 // Применение :
 // if (mapIsGeoSupported())
 //   {
 //     B = Xmet; L = Ymet;
 //     mapGeo84ToPlane42(hMap,B,L);
 //   }
 // hmap - идентификатор открытых данных
 // Bx,Ly  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoWGS84ToPlane42(HMAP hmap, double *Bx, double *Ly);

 //  Преобразование из геодезических координат в радианах
 // (общеземной эллипсоид WGS84)
 //  в метры на местности в проекции карты
 // (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoWGS84ToPlane3D(HMAP hmap, double *Bx, double *Ly,
                                              double *H);

 //  Преобразование из геодезических координат в радианах
 // (эллипсоид Красовского, СК-42)
 //  в метры на местности в проекции карты
 // (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeo42ToPlane3D(HMAP hmap, double *Bx, double *Ly,
                                           double *H);

 //  Преобразование из геодезических координат в радианах
 // (эллипсоид Красовского, СК-95)
 //  в метры на местности в проекции карты
 // (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeo95ToPlane3D(HMAP hmap, double *Bx, double *Ly,
                                           double *H);

 //  Преобразование из геодезических координат в радианах
 // (общеземной эллипсоид ПЗ-90.02)
 //  в метры на местности в проекции карты
 // (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoEP90ToPlane3D(HMAP hmap, double *Bx, double *Ly,
                                             double *H);

 // Преобразование геодезических координаты в радианах из системы 1942г
 // (эллипсоид Красовского) в геодезические координаты в радианах
 // (общеземной эллипсоид WGS84) (поддерживается не для всех карт !)
 // hmap - идентификатор открытых данных
 // Bx,Ly  - преобразуемые координаты
 // на входе радианы в 42г., на выходе - радианы в WGS84
 // H     - высота в точке (метры)
 // Параметр hmap может быть равен нулю

_MAPIMP void _MAPAPI mapGeo42ToGeoWGS84Ex(double *Bx, double *Ly, double *H);
_MAPIMP long int _MAPAPI mapGeo42ToGeoWGS84(HMAP hmap,double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeo42ToGeoWGS843D(HMAP hmap,double *Bx, double *Ly,
                                              double *H);

 // Преобразование геодезических координат в радианах из системы NAHRWAN
 // (эллипсоид Кларка 1880г) в геодезические координаты в радианах
 // системы WGS84 (общеземной эллипсоид WGS-84)
 // на входе радианы в NAHRWAN, на выходе - радианы в WGS84
 // H     - высота в точке (метры)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoNahrwanToGeoWGS843D(double *B, double *L, double *H);


 // Преобразование геодезических координаты в радианах из системы WGS 84
 // (общеземной эллипсоид WGS84) (поддерживается не для всех карт !)
 // в геодезические координаты в радианах в систему 1942 г (эллипсоид Красовского)
 // hmap - идентификатор открытых данных
 // Bx,Ly  - преобразуемые координаты
 // на входе радианы в WGS84, на выходе - радианы в 42г.
 // H     - высота в точке (метры)
 // Параметр hmap может быть равен нулю

_MAPIMP long int _MAPAPI mapGeoWGS84ToGeo42Ex(double *Bx, double *Ly, double *H);
_MAPIMP long int _MAPAPI mapGeoWGS84ToGeo42(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeoWGS84ToGeo423D(HMAP hmap, double *Bx, double *Ly,
                                              double *H);

 // Преобразование геодезических координат в радианах из системы ПЗ-90
 // (общеземной эллипсоид ПЗ-90.02) в геодезические координаты в радианах
 // системы 1942 г (эллипсоид Красовского)
 // Параметр hmap может быть равен нулю

_MAPIMP void _MAPAPI mapGeoEP90ToGeo42Ex(double *Bx, double *Ly, double *H);
_MAPIMP long int _MAPAPI mapGeoEP90ToGeo42(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeoEP90ToGeo423D(HMAP hmap, double *Bx, double *Ly,
                                             double *H);

 // Преобразование геодезических координат в радианах из системы ПЗ-90
 // (общеземной эллипсоид ПЗ-90.02) в геодезические координаты в радианах
 // системы 1995 г (эллипсоид Красовского)

_MAPIMP void _MAPAPI mapGeoEP90ToGeo95Ex(double *Bx, double *Ly, double *H);

 // Преобразование геодезических координат в радианах из системы 1995 г
 // (эллипсоид Красовского) в геодезические координаты в радианах
 // системы ПЗ-90(общеземной эллипсоид ПЗ-90.02)

_MAPIMP void _MAPAPI mapGeo95ToGeoEP90Ex(double *Bx, double *Ly, double *H);

 // Преобразование геодезических координат в радианах из системы 1942 г
 // (эллипсоид Красовского) в геодезические координаты в радианах
 // системы ПЗ-90(общеземной эллипсоид ПЗ-90.02)
 // Параметр hmap может быть равен нулю

_MAPIMP void     _MAPAPI mapGeo42ToGeoEP90Ex(double *Bx, double *Ly, double *H);
_MAPIMP long int _MAPAPI mapGeo42ToGeoEP90(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeo42ToGeoEP903D(HMAP hmap, double *Bx, double *Ly,
                                             double *H);

 // Преобразование геодезических координат в радианах из системы ПЗ-90
 // (общеземной эллипсоид ПЗ-90.02) в геодезические координаты в радианах
 // в системе WGS-84 (общеземной эллипсоид WGS84)

_MAPIMP void _MAPAPI mapGeoEP90ToGeoWGS843D(double *Bx, double *Ly, double *H);

 // Преобразование прямоугольных координат в метрах из системы координат карты
 // в геодезические координаты в радианах системы ПЗ-90(общеземной эллипсоид ПЗ-90.02)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPlaneToGeoEP903D(HMAP hmap, double *Bx, double *Ly,
                                             double *H);

 // Преобразование геодезических координат в радианах из системы 1942 г
 // (эллипсоид Красовского) в прямоугольные в метрах в системе NAHRWAN в
 // проекции UTM (эллипсоид Кларка 1880г.) - NAD27

_MAPIMP void     _MAPAPI mapGeo42ToPlaneUTMEx(double *Bx, double *Ly, double H);
_MAPIMP long int _MAPAPI mapGeo42ToPlaneUTM(HMAP hmap, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeo42ToPlaneUTM3D(HMAP hmap, double *Bx, double *Ly,
                                              double *H);


 // Преобразование геодезических координат в радианах из системы 1942 г
 // (эллипсоид Красовского) в прямоугольные в метрах с учетом ближайшей
 // зоны к координатам точки
 // Bx,Ly - преобразуемые координаты

_MAPIMP long int _MAPAPI mapGeo42ToPlaneByOwnZone(double *Bx, double *Ly);


 // Преобразование набора точек из одной системы
 // координат в другую
 // hmap  - идентификатор открытых данных
 // src,tag - указатели на области размещения точек,
 // могут указывать на одну и ту же область памяти;
 // source,target - типы входной и выходной метрики (PP_MAP,PP_PLANE ...);
 // count - число преобразуемых точек.
 // Пересчет связанный с геодезическими координатами
 // будет выполняться только,если IsGeoSupported() != 0.

_MAPIMP void _MAPAPI mapTransformPoints(HMAP hmap, DOUBLEPOINT *src,
                                        long int source, DOUBLEPOINT *tag,
                                        long int target, long int count);

 // Преобразование координат из градусов в радианы
 // (для положительного значения)
 // degree - структура, содержащая координаты в градусах, минутах,
 // секундах. Описана в maptype.h
 // radian - значение в радианах

_MAPIMP void _MAPAPI mapDegreeToRadian(GEODEGREE *degree, double *radian);

 // Преобразование координат из радиан в градусы
 // (для положительного значения)
 // radian - значение в радианах
 // degree - структура, содержащая координаты в градусах, минутах,
 // секундах. Описана в maptype.h

_MAPIMP void _MAPAPI mapRadianToDegree(double *radian, GEODEGREE *degree);

 // Преобразование координат из градусов в радианы с учетом знака
 // degree - структура, содержащая координаты в градусах, минутах,
 // секундах. Описана в maptype.h
 // radian - значение в радианах

_MAPIMP void _MAPAPI mapSignDegreeToRadian(SIGNDEGREE *degree, double *radian);

 // Преобразование координат из радиан в градусы со знаком
 // radian - значение в радианах
 // degree - структура, содержащая координаты в градусах, минутах,
 // секундах. Описана в maptype.h

_MAPIMP void _MAPAPI mapRadianToSignDegree(double *radian, SIGNDEGREE *degree);

 // Вычисление осевого маридиана по номеру зоны для
 // топокарт системы 42 года
 // zone - номер зоны системы 42 года
 // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapGetAxisMeridianByZone(long int zone);

 // Вычисление номера зоны по геодезической долготе в радианах
 // (меридиану) для топокарт системы 42 года
 // meridian - значение меридиана в радианах
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetZoneByMeridian(double meridian);

 // Заполнение осевого меридиана по геодезической долготе в
 // радианах для топографических карт
 // hmap - идентификатор открытых данных
 // meridian - значение меридиана в радианах
 // Не рекомендуется применять для карт уже содержащих объекты
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetAxisMeridianByMeridian(HMAP hmap, double meridian);

 // Заполнение осевого меридиана по координате Y для
 // топографических карт системы 42 года
 // Не рекомендуется применять для карт уже содержащих объекты
 // hmap  - идентификатор открытых данных
 // y     - координата Y в метрах произвольной точки,
 //         попадающей на заданный лист
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetAxisMeridianByPlaneY(HMAP hmap, double y);

  /*********************************************************
  *                                                        *
  *     ПЕРЕСЧЕТ КООРДИНАТ В РАБОЧЕЙ СИСТЕМЕ КООРДИНАТ     *
  *                                                        *
  *********************************************************/

 // Запросить параметры эллипсоида по его номеру (см. MAPCREAT.H)
 // ellipsoid - номер эллипсоида (см. MAPCREAT.H, ELLIPSOIDKIND)
 // parm      - параметры заданного эллипсоида
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetEllipsoidParameters(long int ellipsoid,
                                                   ELLIPSOIDPARAM *parm);

 // Установить текущие параметры рабочей системы координат
 // hMap      - идентификатор открытой основной карты
 // parm      - параметры рабочей системы координат
 // datum     - параметры пересчета с эллипсоида рабочей системы координат
 //             к WGS-84 (datum может быть 0)
 // ellipsoid - параметры пользовательского эллипсоида для рабочей
 //             системы координат, когда поле EllipsoideKind в
 //             MAPREGISTEREX равно USERELLIPSOID (ellipsoid может быть 0)
 // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapSetWorkSystemParametersEx(HMAP hMap,
                                                       MAPREGISTEREX *parm,
                                                       DATUMPARAM *datum,
                                                       ELLIPSOIDPARAM *ellipsoid);
                                                    
_MAPIMP long int _MAPAPI mapSetWorkSystemParameters(HMAP hmap,
                                                    MAPREGISTEREX * parm);

 // Запросить текущие параметры рабочей системы координат
 // hMap      - идентификатор открытой основной карты
 // parm      - параметры рабочей системы координат
 // datum     - параметры пересчета с эллипсоида рабочей системы координат
 //             к WGS-84
 // ellipsoid - параметры пользовательского эллипсоида для рабочей 
 //             системы координат
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetWorkSystemParametersEx(HMAP hmap,
                                                      MAPREGISTEREX *parm,
                                                      DATUMPARAM *datum,
                                                      ELLIPSOIDPARAM *ellipsoid);

_MAPIMP long int _MAPAPI mapGetWorkSystemParameters(HMAP hmap,
                                                    MAPREGISTEREX *parm);

 // Запросить - установлены ли параметры рабочей системы координат
 // (Если mapSetWorkSystemParameters не вызывалась - возвращает 0)
 // hMap   - идентификатор открытой основной карты
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsWorkSystemParameters(HMAP hmap);

 // Преобразование из плоских прямоугольных координат в метрах на эллипсоиде карты
 // в метры на местности в соответствии с параметрами рабочей системы координат
 // hmap   - идентификатор открытых данных
 // X,Y,H  - преобразуемые координаты
 // на входе метры, на выходе - метры
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneToWorkSystemPlane(HMAP hMap, double *X,
                                                   double *Y, double *H);

 // Преобразование из плоских прямоугольных координат в метрах на эллипсоиде карты
 // в геодезические координаты  в соответствии с параметрами рабочей системы координат
 // hmap   - идентификатор открытых данных
 // X,Y,H  - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPlaneToWorkSystemGeo(HMAP hMap, double *X,
                                                 double *Y, double *H);

 // Преобразование из геодезических координат в радианах на эллипсоиде карты
 // в метры на местности в соответствии с параметрами рабочей системы координат
 // hmap     - идентификатор открытых данных
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGeoToWorkSystemPlane(HMAP hmap, double *Bx,
                                                 double *Ly, double *H);

 // Преобразование в геодезические координаты в радианах на эллипсоиде карты
 // из метров на местности в рабочей системе координат в соответствии с ее параметрами
 // hmap    - идентификатор открытых данных
 // Bx,Ly,H - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapWorkSystemPlaneToGeo(HMAP hmap, double *Bx,
                                                 double *Ly, double *H);

  /**********************************************************
  *                                                         *
  * ПЕРЕСЧЕТ КООРДИНАТ В ПОЛЬЗОВАТЕЛЬСКОЙ СИСТЕМЕ КООРДИНАТ *
  *                                                         *
  **********************************************************/

 // Установить текущие параметры пользовательской системы координат
 // parm      - параметры рабочей системы координат (см. MAPCREAT.H)
 // datum     - параметры пересчета с эллипсоида рабочей системы координат
 //             к WGS-84 (datum может быть 0)
 // ellipsoid - параметры пользовательского эллипсоида для рабочей 
 //             системы координат, только когда поле EllipsoideKind в
 //             MAPREGISTEREX равно USERELLIPSOID (ellipsoid может быть 0)
 // code      - код EPSG (4326 - WGS-84),
 //             для СК-42 зоны 1-60 : 28401-28460, для СК-95 зоны 4-32: 20004-20032
 // Возвращает идентификатор пользовательской системы координат
 // По завершении использования необходимо вызвать mapDeleteUserSystemParameters
 // При ошибке возвращает ноль

_MAPIMP HANDLE _MAPAPI mapCreateUserSystemParameters(MAPREGISTEREX *parm,
                                                     DATUMPARAM *datum,
                                                     ELLIPSOIDPARAM *ellipsoid);

_MAPIMP HANDLE _MAPAPI mapCreateUserSystemParametersByEpsg(long int code);

 // Запросить/Установить тип системы координат
 // (плоская прямоугольная - 1 или геодезическая - 2)
 // При отсутствии данных возвращает ноль

_MAPIMP long int _MAPAPI mapGetUserSystemType(HANDLE huser);
_MAPIMP long int _MAPAPI mapSetUserSystemType(HANDLE huser, long int type);

 // Преобразование из геодезических координат (радианы) в пользовательской проекции
 // в геодезические координаты в радианах (общеземной эллипсоид WGS84)
 // huser - идентификатор пользовательской системы координат
 // Bx,Ly - преобразуемые координаты
 // на входе радианы, на выходе - радианы
 // H     - высота в точке (метры)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUserGeoToGeoWGS84(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapUserGeoToGeoWGS843D(HANDLE huser, double *Bx, double *Ly,
                                                double *H);

 // Преобразование из геодезических координат (радианы) в пользовательской проекции
 // в метры в пользовательской проекции
 // huser - идентификатор пользовательской системы координат
 // Bx,Ly - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUserGeoToUserPlane(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapUserGeoToPlane(HANDLE huser, double *Bx, double *Ly);

 // Преобразование из метров на местности в пользовательской проекции
 // в геодезические координаты в радианах (общеземной эллипсоид WGS84)
 // Наличие высоты повышает точность расчетов
 // huser - идентификатор пользовательской системы координат
 // Bx,Ly - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // H     - высота в точке (метры)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUserPlaneToGeoWGS84(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapUserPlaneToGeoWGS843D(HANDLE huser, double *Bx, double *Ly,
                                                  double *H);

 // Преобразование из метров на местности в пользовательской проекции
 // в геодезические координаты в радианах на эллипсоиде в пользовательской проекции
 // Наличие высоты повышает точность расчетов
 // huser - идентификатор пользовательской системы координат
 // Bx,Ly - преобразуемые координаты
 // на входе метры, на выходе - радианы
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUserPlaneToUserGeo(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapUserPlaneToGeo(HANDLE huser, double *Bx, double *Ly);

 // Преобразование из геодезических координат в радианах
 // (общеземной эллипсоид WGS84)
 // в геодезические координаты в радианах на эллипсоиде в пользовательской проекции
 // huser    - идентификатор пользовательской системы координат
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - радианы
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoWGS84ToUserGeo(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeoWGS84ToUserGeo3D(HANDLE huser, double *Bx, double *Ly,
                                                double *H);

 // Преобразование из геодезических координат в радианах
 // (общеземной эллипсоид WGS84)
 // в метры на местности в пользовательской проекции
 // huser    - идентификатор пользовательской системы координат
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - метры
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoWGS84ToUserPlane(HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeoWGS84ToUserPlane3D(HANDLE huser, double *Bx, double *Ly,
                                                  double *H);

 // Преобразование из геодезических координат (радианы) в проекции документа
 // в геодезические координаты в радианах на эллипсоиде в пользовательской проекции
 // hmap    - идентификатор открытых данных
 // huser    - идентификатор пользовательской системы координат
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - радианы
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeoToUserGeo(HMAP hmap, HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapGeoToUserGeo3D(HMAP hmap, HANDLE huser, double *Bx, double *Ly,
                                           double *H);
                                                                                    
 // Преобразование из геодезических координат (радианы) на эллипсоиде
 // в пользовательской проекции в геодезические координаты (радианы) в проекции документа
 // hmap    - идентификатор открытых данных
 // huser    - идентификатор пользовательской системы координат
 // Bx,Ly,H  - преобразуемые координаты
 // на входе радианы, на выходе - радианы
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUserGeoToGeo(HMAP hmap, HANDLE huser, double *Bx, double *Ly);
_MAPIMP long int _MAPAPI mapUserGeoToGeo3D(HMAP hmap, HANDLE huser, double *Bx, double *Ly,
                                           double *H);

  // Освободить ресурсы пользовательской системы координат

_MAPIMP void _MAPAPI mapDeleteUserSystemParameters(HANDLE huser);

 // Сравнить параметры двух систем координат                   // 22/01/14
 // parm1      - параметры первой системы координат (см. MAPCREAT.H)
 // datum1     - параметры пересчета с эллипсоида первой системы координат
 //              к WGS-84 (datum может быть 0)
 // ellipsoid1 - параметры пользовательского эллипсоида для первой
 //              системы координат, только когда поле EllipsoideKind в
 //              MAPREGISTEREX равно USERELLIPSOID (ellipsoid может быть 0)
 // parm2      - параметры второй системы координат (см. MAPCREAT.H)
 // datum2     - параметры пересчета с эллипсоида второй системы координат
 //              к WGS-84 (datum может быть 0)
 // ellipsoid2 - параметры пользовательского эллипсоида для второй
 //              системы координат
 // При несовпадении каких-либо значений параметров возвращает ненулевое значение
 // Некоторые несовпадающие параметры могут считаться идентичными
 // (например, топографическая карта UTM и обзорно-географическая карта UTM)

_MAPIMP long int _MAPAPI mapCompareSystemParameters(MAPREGISTEREX * parm1,
                                                    DATUMPARAM * datum1,
                                                    ELLIPSOIDPARAM * ellipsoid1,
                                                    MAPREGISTEREX * parm2,
                                                    DATUMPARAM * datum2,
                                                    ELLIPSOIDPARAM * ellipsoid2);

  /*********************************************************
  *                                                        *
  *     ОТОБРАЖЕНИЕ КООРДИНАТ В ЗАДАННОМ ФОРМАТЕ           *
  *                                                        *
  *********************************************************/

 // Установить формат отображения текущих координат курсора
 // hmap    - идентификатор открытых данных
 // format -  номер формата отображения координат (см. maptype.h - CURRENTPOINTFORMAT)
 // При ошибке возвращает ноль, иначе - установленное значение

_MAPIMP long int _MAPAPI mapSetCurrentPointFormat(HMAP hmap, long int format);
  
 // Запросить формат отображения текущих координат курсора
 // hmap    - идентификатор открытых данных
 // Возвращает номер формата отображения координат (см. maptype.h - CURRENTPOINTFORMAT)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetCurrentPointFormat(HMAP hmap);
  
 // Пересчитать значение координат из плоских прямоугольных координат документа (метры)
 // в систему, определяемую форматом отображения текущих координат
 // hmap    - идентификатор открытых данных
 // x, y    - координаты точки в радианах в соответствии с текущими параметрами
 //           проекции (mapGetDocProjection) - пересчитываются в новые значения
 // h       - высота точки (указатель может быть равен нулю), если задано значение -
 //           пересчитывается в новое значение
 // maptype - тип карты, соответствующий координатам, если не равен нулю, 
 //           то добавляется строка с обозначением системы координат: "(СК42)", "(CR95)"...
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPlaneToPointFormat(HMAP hmap, double *x,
                                               double *y, double *h);

 // Пересчитать значение координат из плоских прямоугольных координат документа (метры)
 // в систему, определяемую форматом отображения текущих координат
 // и сформировать строку
 // hmap    - идентификатор открытых данных
 // x, y    - координаты точки в радианах в соответствии с текущими параметрами
 //           проекции (mapGetDocProjection)
 // h       - высота точки (указатель может быть равен нулю)
 // place   - адрес строки для записи результата
 // size    - размер выделеной строки (не менее 256 байт)
 // Пример строки:
 // B= -73° 27' 04.53"  L= 175° 51' 21.07"  H= 109.51 m (WGS84)
 // X= 6 309 212.12 м   Y= 7 412 249.25 м (СК42)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPlaneToPointFormatString(HMAP hmap, double *x,
                                                     double *y, double *h,
                                                     char *place, long int size);
_MAPIMP long int _MAPAPI mapPlaneToPointFormatStringUn(HMAP hmap, double *x,
                                                       double *y, double *h,
                                                       WCHAR *place, long int size);

 // Вывод координат точки в строку
 // x,y   - плоские прямоугольные координаты точки в метрах
 // h     - высота в метрах или нулевой указатель
 // place - адрес строки для размещения результата
 // size  - размер строки в байтах (не менее 80 байт)
 // maptype - тип карты, если не равен нулю, то добавляется строка
 //           с обозначением системы координат: "   (СК42)", "   (CК95)",...
 // Пример результата:
 // "X=  438 145.27 m  Y= 6 230 513.03 m  H=  54.12 m"

_MAPIMP void _MAPAPI mapPlaneToString(double *x, double *y, double *h,
                                      char *place, long int size, long int maptype);
_MAPIMP long int _MAPAPI mapPlaneToStringUn(double *x, double *y, double *h,
                                            WCHAR *place, long int size, long int maptype);

 // Запись числа с плавающей точкой в символьном виде со вставкой разделяющих пробелов
 // (разделение на тройки символов от конца строки к началу)
 // Например: 7 390 621.458
 // value  - значение числа, записываемого в строку
 // string - адрес строки для размещения результата
 // size   - длина строки в байтах (не менее 16 !)
 // precision - число знаков после запятой
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDoubleToString(double value, char* string,
                                           long int size, long int precision);
_MAPIMP long int _MAPAPI mapDoubleToStringUn(double value, WCHAR *string,
                                             long int size, long int precision);

  // Запись целого числа в символьном виде со вставкой разделяющих пробелов
  // (разделение на тройки символов от конца строки к началу)
  // size - длина строки в байтах (не менее 16 !)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapLongToString(long int number, char *string, long int size);
_MAPIMP long int _MAPAPI mapLongToStringUn(long int number, WCHAR* string,long int size);

  // Запись целого числа типа __int64 в символьном виде
  // со вставкой разделяющих пробелов
  // (разделение на тройки символов от конца строки к началу)
  // size - длина строки в байтах (не менее 32 !)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapInt64ToString(__int64 number, char *string, long int size);
_MAPIMP long int _MAPAPI mapInt64ToStringUn(__int64 number, WCHAR *string, long int size);

  // Округлить дробную часть числа до заданного числа знаков
  // Целая часть остается без изменения
  // value - исходное значение
  // count - число знаков после запятой (от 0 до 9)

_MAPIMP double _MAPAPI mapRoundDouble(double value, int count);

  // Запись масштаба в символьном виде со вставкой разделяющих пробелов
  // (например: "1 : 50 000","2 : 1" - если scale < 1)
  // (разделение на тройки символов от конца строки к началу)
  // size - длина строки в байтах (не менее 20 !)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapScaleToString(double scale, char * string, long int size);
_MAPIMP long int _MAPAPI mapScaleToStringUn(double scale, WCHAR* string, long int size);


/*************************************************************************
*                                                                        *
*             ФУНКЦИИ ДОСТУПА К БАЗЕ ДАННЫХ EPSG                         *
*  ДЛЯ ВЫПОЛНЕНИЯ НЕОБХОДИМО НАЛИЧИЕ В ПАПКЕ ПРИЛОЖЕНИЯ ФАЙЛОВ           *
*  EPSG.CSG, EPSG.CSP, EPSG.CSU, КОТОРЫЕ ВХОДЯТ В СОСТАВ ГИС КАРТА 2011. *
*  ОБНОВИТЬ ФАЙЛЫ МОЖНО ИСПОЛЬЗУЯ ПРОГРАММУ EPSGReader, ВХОДЯЩУЮ В       *
*  ГИС КАРТА 2011.                                                       *
*************************************************************************/

  // Запросить параметры проекции и системы координат по коду EPSG
  // Если код EPSG задает геодезическую систему координат,
  // то устанавливается проекция Широта\Долгота и соответствующие
  // параметры эллипсоида и датум
  // Если код EPSG задает плоскую прямоугольную систему координат,
  // то все параметры устанавливаются из базы EPSG
  // epsgcode  - код EPSG, для СК-42 зоны 2-32 : 28402-28432, для СК-95 зоны 4-32: 20004-20032
  // mapreg    - параметры системы координат и проекции
  // datum     - параметры пересчета с эллипсоида рабочей системы координат
  //             к WGS-84
  // ellipsoid - параметры пользовательского эллипсоида для рабочей
  //             системы координат
  // Для геодезических систем координат возвращает 2,
  // для плоских прямоугольных возвращает 1.
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetParametersForEPSG(long int epsgcode,
                                                 MAPREGISTEREX  *mapreg,
                                                 DATUMPARAM     *datum,
                                                 ELLIPSOIDPARAM *ellipsoid);

  // Открыть базу данных EPSG
  // При успешном выполнении возвращает идентификатор открытой базы данных EPSG
  // При ошибке возвращает 0

_MAPIMP HANDLE _MAPAPI mapOpenEPSGDatabase();

  // Закрыть базу данных EPSG
  // epsgdata - идентификатор открытой базы данных EPSG

_MAPIMP void _MAPAPI mapCloseEPSGDatabase(HANDLE epsgdata);

  // Считать данные на текущую прямоугольную систему координат в базе данных EPSG
  // После вызова функции текущей становится следующая система координат в базе данных EPSG
  // epsgdata  - идентификатор открытой базы данных EPSG
  // mapreg    - параметры проекции
  // ellipsoid - эллипсоид
  // datum     - датум
  // rectsys   - параметры прямоугольной системы координат
  // geodsys   - параметры базовой геодезической системы координат
  // unit      - единицы измерения
  // При ошибке или при выходе за границы набора данных возвращает 0

_MAPIMP long int _MAPAPI mapReadEPSGProjectedSystem(HANDLE epsgdata, MAPREGISTEREX *mapreg, ELLIPSOIDPARAM *ellipsoid,
                                                    DATUMPARAM *datum, EPSGRECTSYS *rectsys,
                                                    EPSGGEODSYS *geodsys, EPSGMEASUNIT *unit);

  // Считать данные на прямоугольную систему координат по номеру записи в базе данных EPSG   // 24/03/11
  // После вызова функции текущей становится следующая система координат в базе данных EPSG
  // epsgdata    - идентификатор открытой базы данных EPSG
  // coordsysnum - номер записи в списке прямоугольных систем координат (от 1)
  // mapreg      - параметры проекции
  // ellipsoid   - эллипсоид
  // datum       - датум
  // rectsys     - параметры прямоугольной системы координат
  // geodsys     - параметры базовой геодезической системы координат
  // unit        - единицы измерения
  // При ошибке или при выходе за границы набора данных возвращает 0

_MAPIMP long int _MAPAPI mapReadEPSGProjectedSystemByNumber(HANDLE epsgdata, long int coordsysnum,
                                                            MAPREGISTEREX *mapreg,
                                                            ELLIPSOIDPARAM *ellipsoid,
                                                            DATUMPARAM *datum, EPSGRECTSYS *rectsys,
                                                            EPSGGEODSYS *geodsys, EPSGMEASUNIT *unit);

  // Запросить параметры геодезической системы координат по коду в базе данных EPSG
  // epsgcode  - код геодезической системы координат в базе данных EPSG
  // ellipsoid - параметры эллипсоида
  // datum     - параметры датума
  // geodsys   - параметры геодезической СК
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetEPSGGeodeticSystem(long int epsgcode, ELLIPSOIDPARAM *ellipsoid,
                                                  DATUMPARAM *datum, EPSGGEODSYS *geodsys);

  // Запросить параметры прямоугольной системы координат по коду в базе данных EPSG
  // epsgcode - код прямоугольной системы координат в базе данных EPSG
  // mapreg   - параметры проекции
  // rectsys  - параметры прямоугольной СК
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetEPSGProjectedSystem(long int epsgcode, MAPREGISTEREX *mapreg,
                                                   EPSGRECTSYS *rectsys);


  // Запросить параметры единицы измерения по коду в базе данных EPSG
  // epsgcode - код единицы измерения в базе данных EPSG
  // unit     - параметры единицы измерения
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetEPSGUnit(long int epsgcode, EPSGMEASUNIT *unit);

  // Заполнить WKT строку из MAPREGISTEREX, ELLIPSOIDPARAM, DATUMPARAM
  // mapreg    - параметры проекции
  // ellipsoid - параметры эллипсоида
  // datum     - параметры датума
  // wktstr    - заполняемая строка с описанием системы координат
  // wktstrsize - зарезервированный размер строки (4 Кбайта достаточно)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetWKTString(MAPREGISTEREX *mapreg, ELLIPSOIDPARAM *ellipsoid, DATUMPARAM *datum,
                                         char *wktstr, long int wktstrsize);


  /***************************************************************
  *                                                              *
  *  ФУНКЦИИ ЧТЕНИЯ ПАРАМЕТРОВ СИСТЕМ ОТСЧЕТА ИЗ СПИСКА В ФАЙЛЕ  *
  *                                                              *
  ****************************************************************
  * <?xml version="1.0" encoding="UTF-8"?>                       *
  * <ProjectList Version="1.0">                                  *
  * <Project Name="ITM"                                          *
  * Comment="Israel Transverse Mercator"                         *
  * EPSG="2039">                                                 *
  * <Projection Type="Transverse Mercator"                       *
  * EPSG="9807"                                                  *
  * CentralMeridian="31 44 3.816999999992845"                    *
  * LatitudeOfOrigin="35 12 16.260999999993260"                  *
  * ScaleFactor="1.0000067"                                      *
  * FalseEasting="219529.584"                                    *
  * FalseNothing="626907.3899999999"/>                           *
  * <Spheroid Type="GRS 1980" Parm="6378137.0 298.257222101"/>   *
  * </Project>                                                   *
  * </ProjectList>                                               *
  ****************************************************************/

 // Открыть список параметров систем отсчета
 // name - имя файла списка параметров
 // При успешном выполнении возвращает идентификатор списка в памяти
 // При ошибке возвращает ноль

_MAPIMP HMAPREG _MAPAPI mapOpenMapRegisterList(const char *name);

_MAPIMP HMAPREG _MAPAPI mapOpenMapRegisterListUn(const WCHAR *name);

 // Создать список параметров систем отсчета
 // name - имя файла списка параметров
 // При успешном выполнении возвращает идентификатор списка в памяти
 // При ошибке возвращает ноль

_MAPIMP HMAPREG _MAPAPI mapCreateMapRegisterList(const char *name);

_MAPIMP HMAPREG _MAPAPI mapCreateMapRegisterListUn(const WCHAR *name);

 // Закрыть список параметров систем отсчета
 // hmapreg - идентификатор списка параметров систем отсчета

_MAPIMP void _MAPAPI mapCloseMapRegisterList(HMAPREG hmapreg);

 // Запросить число систем отсчета, хранящихся в списке
 // в виде файла XML (<ProjectList Version="1.0">
 // Одной системе соответствует один тэг "Project"
 // hmapreg - идентификатор списка параметров систем отсчета
 // При успешном выполнении возвращает число записей параметров
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListCount(HMAPREG hmapreg);

 // Запросить название системы отсчета по заданному порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // name    - адрес строки для размещения результата
 // size    - размер выделенной строки в байтах
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListName(HMAPREG hmapreg, long int number,
                                                char *name, int size);

_MAPIMP long int _MAPAPI mapMapRegisterListNameUn(HMAPREG hmapreg, long int number,
                                                  WCHAR *name, int size);

 // Запросить комментарий для системы отсчета по заданному порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // name    - адрес строки для размещения результата
 // size    - длина выделенной строки для размещения результата
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListComment(HMAPREG hmapreg, long int number,
                                                   char *name, int size);

_MAPIMP long int _MAPAPI mapMapRegisterListCommentUn(HMAPREG hmapreg, long int number,
                                                     WCHAR *name, int size);

 // Запросить порядковый номер в списке (начиная с 1)               // 05/04/14
 // по коду EPSG для системы отсчета (CRS)
 // hmapreg - идентификатор списка параметров систем отсчета
 // epsg    - код EPSG для системы отсчета (CRS)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSeekMapRegisterListByEPSG(HMAPREG hmapreg, long int epsg);

 // Запросить код EPSG для системы отсчета по заданному порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // Если код не задан - возвращает "-1"
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListEPSG(HMAPREG hmapreg, long int number);

 // Запросить идентификатор для системы отсчета
 // по заданному порядковому номеру в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // ident   - адрес строки для размещения идентификатора
 // size    - длина выделенной строки для размещения идентификатора
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListCrsIdent(HMAPREG hmapreg, long int number,
                                                    char *ident, int size);

_MAPIMP long int _MAPAPI mapMapRegisterListCrsIdentUn(HMAPREG hmapreg, long int number,
                                                      WCHAR *ident, int size);

 // Запросить параметры системы отсчета по заданному порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // mapreg  - параметры проекции <Projection ...>
 // datum   - параметры датума <Datum ...>
 // ellparm - параметры эллипсоида <Spheroid ...>
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMapRegisterListParameters(HMAPREG hmapreg, long int number,
                                                      MAPREGISTEREX  *mapreg,
                                                      DATUMPARAM     *datum,
                                                      ELLIPSOIDPARAM *ellparm);

 // Добавить запись параметров системы отсчета
 // hmapreg - идентификатор списка параметров систем отсчета
 // name     - уникальное название системы отсчета (обязательно)
 // comment  - комментарий для системы отсчета (или ноль)
 // epsgcode - код EPSG (или ноль)
 // ident   - идентификатор системы отсчета (или ноль)
 // mapreg   - описание параметров системы отсчета (обязательно)(см. mapcreat.h)
 // datum    - описание параметров датума (или ноль) (см. mapcreat.h)
 // ellparam - описание параметров эллипсоида (или ноль) (см. mapcreat.h)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendMapRegisterListParameters(HMAPREG hmapreg,
                                                            const char *name,
                                                            const char *comment,
                                                            long int epsgcode,
                                                            MAPREGISTEREX *mapreg,
                                                            DATUMPARAM *datum,
                                                            ELLIPSOIDPARAM *ellparm);

_MAPIMP long int _MAPAPI mapAppendMapRegisterListParametersEx(HMAPREG hmapreg,
                                                              const char *name,
                                                              const char *comment,
                                                              long int epsgcode,
                                                              const char *ident,
                                                              MAPREGISTEREX *mapreg,
                                                              DATUMPARAM *datum,
                                                              ELLIPSOIDPARAM *ellparm);

_MAPIMP long int _MAPAPI mapAppendMapRegisterListParametersUn(HMAPREG hmapreg,
                                                              const WCHAR *name,
                                                              const WCHAR *comment,
                                                              long int epsgcode,
                                                              const WCHAR *ident,
                                                              MAPREGISTEREX *mapreg,
                                                              DATUMPARAM *datum,
                                                              ELLIPSOIDPARAM *ellparm);

 // Удалить запись параметров систем отсчета по порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // Для немедленного изменения данных в файле нужно вызвать
 // функцию mapCommitMapRegisterList
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMapRegisterListParameters(HMAPREG hmapreg, long int number);

 // Обновить название, комментарий и код системы отсчета по заданному порядковому номеру
 // в списке (начиная с 1)
 // hmapreg - идентификатор списка параметров систем отсчета
 // number  - порядковый номер записи параметров
 // name    - название системы отсчета или 0 (не менять)
 // comment - комментарий к системе отсчета или 0 (не менять)
 // code    - код EPSG или 0 (не менять)
 // ident   - идентификатор системы отсчета или 0 (не менять)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateMapRegisterListName(HMAPREG hmapreg, long int number,
                                                      const char *name,
                                                      const char *comment,
                                                      long int code);

_MAPIMP long int _MAPAPI mapUpdateMapRegisterListNameEx(HMAPREG hmapreg, long int number,
                                                        const char *name,
                                                        const char *comment,
                                                        long int code,
                                                        const char *ident);

_MAPIMP long int _MAPAPI mapUpdateMapRegisterListNameUn(HMAPREG hmapreg, long int number,
                                                        const WCHAR *name,
                                                        const WCHAR *comment,
                                                        long int code, const WCHAR *ident);

 // Сохранить изменения списка параметров систем отсчета в файле
 // hmapreg - идентификатор списка параметров систем отсчета
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitMapRegisterList(HMAPREG hmapreg);

 // Отменить изменения списка параметров систем отсчета в памяти
 // Отмена изменений может быть выполнена до вызова mapCommitMapRegisterList
 // hmapreg - идентификатор списка параметров систем отсчета
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUndoMapRegisterList(HMAPREG hmapreg);

  /*********************************************************
  *                                                        *
  *     ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ ОБ ОБЪЕКТЕ КАРТЫ        *
  *                                                        *
  *********************************************************/

 // Запросить название листа на котором расположен объект
 // info - идентификатор объекта карты в памяти
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapListName(HOBJ info);
_MAPIMP long int _MAPAPI mapListNameEx(HOBJ info, char *name, long int size);
_MAPIMP long int _MAPAPI mapListNameUn(HOBJ info, WCHAR *name, long int size);

 // Запросить идентификатор классификатора карты, содержащей объект
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP HRSC _MAPAPI mapGetRscIdentByObject(HOBJ info);

 // Запросить уникальный номер объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapObjectKey(HOBJ info);

 // Установить уникальный номер объекта
 // info - идентификатор объекта карты в памяти
 // number - уникальный номер объекта в листе
 // Программа, вызывающая данную функцию должна обеспечить
 // уникальность номеров в листе !
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetObjectKey(HOBJ info, long int number);

 // Запросить классификационный код объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает 0 (ноль допустим для нового объекта)

_MAPIMP long int _MAPAPI mapObjectExcode(HOBJ info);

 // Запросить характер локализации объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает 0  (ноль допустим)

_MAPIMP long int _MAPAPI mapObjectLocal(HOBJ info);

 // Запросить условное название объекта
 // info - идентификатор объекта карты в памяти
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // При ошибке возвращает ноль

_MAPIMP const char * _MAPAPI mapObjectName(HOBJ info);

_MAPIMP long int _MAPAPI mapObjectNameEx(HOBJ info, char *name, long int size);

_MAPIMP long int _MAPAPI mapObjectNameUn(HOBJ info, WCHAR *name, long int size);

 // Запросить максимальный размер условного названия объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectNameSize(HOBJ info);

 // Запросить направление цифрования объекта
 //  info - идентификатор объекта карты в памяти
 // Возвращает:
 //  OD_UNDEFINED (1) - не определено (незамкнутый контур или контур,
 //                     вырожденный в точку или контур, имеющий "петли")
 //  0D_RIGHT     (2) - объект справа (основной контур замкнутого объекта
 //                     по часовой стрелке)
 //  0D_LEFT      (4) - объект слева (основной контур замкнутого объекта
 //                     против часовой стрелки)
 // (см. OBJECT_DIRECT в Maptype.h)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectDirect(HOBJ info);

 // Запросить направление цифрования объекта
 //  info    - идентификатор объекта карты в памяти
 //  subject - номер подобъекта (для объекта - равен нулю)
 // Возвращает:
 //  OD_UNDEFINED (1) - не определено (незамкнутый контур или контур,
 //                     вырожденный в точку или контур, имеющий "петли")
 //  0D_RIGHT     (2) - объект справа (замкнутый контур объекта
 //                     по часовой стрелке)
 //  0D_LEFT      (4) - объект слева (замкнутый контур объекта
 //                     против часовой стрелки)
 // (см. OBJECT_DIRECT в Maptype.h)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSubjectDirect(HOBJ info, long int subject);


 // Запросить номер слоя объекта ("Layer" = "Segment")
 // Номера слоев начинаются с ноля !
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSegmentNumber(HOBJ info);

 // Запросить название слоя объекта ("Layer" = "Segment")
 // info - идентификатор объекта карты в памяти
 // name - адрес буфера для результата запроса
 // size - размер буфера
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapSegmentName(HOBJ info);
_MAPIMP long int _MAPAPI mapSegmentNameEx(HOBJ info, char *name, long int size);
_MAPIMP long int _MAPAPI mapSegmentNameUn(HOBJ info, WCHAR *name, long int size);

 // Запросить максимальный размер названи слоя объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSegmentNameSize(HOBJ info);

 // Запросить индекс (внутренний код) объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает 0 (ноль допустим для нового объекта)

_MAPIMP long int _MAPAPI mapObjectCode(HOBJ info);

 // Установить значение границ видимости по классификатору
 // объектов
 // info - идентификатор объекта карты в памяти

_MAPIMP void _MAPAPI mapClearBotTop(HOBJ info);

 // Запросить/Установить диапазон масштабов видимости объекта
 // scale - масштаб отображения от 1:1 до 1:40 млн.
 // info - идентификатор объекта карты в памяти

_MAPIMP long int _MAPAPI mapObjectTopScale(HOBJ info);
_MAPIMP long int _MAPAPI mapSetObjectTopScale(HOBJ info, long int scale);
_MAPIMP long int _MAPAPI mapObjectBotScale(HOBJ info);
_MAPIMP long int _MAPAPI mapSetObjectBotScale(HOBJ info, long int scale);

 // Запросить - являются ли значения границ видимости у объекта
 // уникальными (то есть установленными не из классификатора,
 // а персональными для данного экземпляра)
 // Если границы видимости беруться из классификатора - возвращает 0

_MAPIMP long int _MAPAPI mapObjectBotTopUniqueness(HOBJ info);

 // Сформировать описание нового объекта по внешнему коду и локализации
 // или изменить код существующего объекта на карте
 // info - идентификатор объекта карты в памяти
 // excode - внешний код объекта (числовой),
 // local  - локализация (LOCAL_LINE, LOCAL_POINT...)
 // Обычно вызывается после mapCreateObject(...) и добавления семантики 
 // (если она есть)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterObject(HOBJ info, long int excode,
                                           long int local);

 // Сформировать описание нового объекта по короткому имени объекта 
 // (ключу) или изменить код существующего объекта на карте
 // info - идентификатор объекта карты в памяти
 // name - символьный код объекта в классификаторе (до 31 символа)
 // Обычно вызывается после mapCreateObject(...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterObjectByKey(HOBJ info, const char *name);

 // Сформировать описание нового объекта
 // по внутреннему коду объекта (см. mapRscObjectCode() и т.п.)
 // или изменить код существующего объекта на карте
 // info - идентификатор объекта карты в памяти
 // Обычно вызывается после mapCreateObject(...) и добавления
 // семантики (если она есть)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDescribeObject(HOBJ info, long int code);

 // Сформировать описание нового графического объекта
 // по номеру слоя (из классификатора карты) и локализации
 // info - идентификатор объекта карты в памяти
 // layer - порядковый номер слоя в классификаторе
 // local  - локализация (LOCAL_LINE, LOCAL_POINT..., см. maptype.h)
 // Вызывается после mapCreateObject(...)
 // Для формирования условного знака необходимо
 // использовать функцию mapAppendDraw(...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterDrawObject(HOBJ info, long int layer,
                                               long int local);

 // Установить номер листа для нового объекта
 // info - идентификатор объекта карты в памяти
 // list - последовательный номер листа (с 1)
 // Обнуляет последовательный и уникальный номера объекта
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetObjectListNumber(HOBJ info, long int list);

 // Запросить номер листа для объекта
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetListNumber(HOBJ info);

 //  Запросить формат хранения метрики (IDSHORT2,...,IDDOUBLE3)
 //  При ошибке возвращает ноль, иначе - тип формата хранения метрики

_MAPIMP long int _MAPAPI mapGetObjectKind(HOBJ info);

 // Установить тип и размерность метрики объекта
 // info - идентификатор объекта карты в памяти
 // kind - тип метрики, см. maptype.h
 // (например : IDFLOAT2,IDDOUBLE3, и т.п.,
 // объекты пользовательской карты и рамка листа всегда имеют тип DOUBLE)
 // Пересчет выполняется с сохранением существующих координат
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetObjectKind(HOBJ info, long int kind);

 // Запросить описание объекта в виде записи
 // info - идентификатор объекта карты в памяти
 // buffer - адрес памяти для размещения результата,
 // size   - размер выделенной памяти для контроля.
 // Может применяться для переноса объекта на другую карту
 // той же проекции (!) (ограничение данной версии)
 // Передача объекта может выполняться между различными
 // потоками, процессами, компьютерами
 // по соответствующим протоколам.
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetObjectRecord(HOBJ info, char *buffer,
                                            long int size);

 // Запросить длину описания объекта в виде записи
 // info - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetObjectRecordLength(HOBJ info);

 // Создать объект на указанной карте из записи объекта
 // hmap  - идентификатор открытых данных
 // hsite - идентификатор открытой пользовательской карты
 // buffer - адрес области памяти с записью объекта, созданной в mapGetObjectRecord
 // mode  - режим создания
 //  0 - записать,как новый;
 //  1 - заменить объект при совпадении Key();
 //  4 - создать в памяти,как новый,
 //  5 - заменить объект при совпадении Key() в памяти;
 // Для режимов 4 и 5 требуется последующий
 // вызов mapCommitObject()
 // При ошибке возвращает ноль, иначе - идентификатор созданного объекта
 // Если объект не нужен, нужно освободить ресурсы функцией mapFreeObject

_MAPIMP HOBJ _MAPAPI mapPutObjectRecord(HMAP hmap, HSITE hsite,
                                        const char *buffer, long int size,
                                        long int mode);

 // Запросить порядковый номер объекта в карте
 // info  - идентификатор объекта карты в памяти
 // Если объект только создан и метод mapCommit... не вызывался -
 // возвращает ноль
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetObjectNumber(HOBJ info);

 // Запросить уникальный идентификатор вида объекта в классификаторе
 // (короткое имя - строка длиной до 31 символа)
 // info  - идентификатор объекта карты в памяти
 // key  - адрес буфера для записи результата
 // size - длина строки
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapObjectRscKey(HOBJ info);

_MAPIMP long int _MAPAPI mapObjectRscKeyUn(HOBJ info, WCHAR *key, long int size);

 // Установить/сбросить/запросить масштабируемость объекта
 // Применяется ТОЛЬКО для графических объектов, имеющих внутренний код равный нулю
 // Для объектов из классификатора значение игнорируется
 // scale = 1 для установки масштабируемости при увеличении карты
 //         0 для сброса признака масштабируемости
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetObjectScale(HOBJ info, long int scale);
_MAPIMP long int _MAPAPI mapGetObjectScale(HOBJ info);

 // Установить/сбросить/запросить признак "Не сжимать" объекта
 // Применяется ТОЛЬКО для графических объектов, имеющих внутренний код равный нулю
 // press = 1 для установки признака "Не сжимать" при сжатии карты
 //           относительного базового масштаба карты
 //         0 для сброса признака
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetObjectPress(HOBJ info, long int press);
_MAPIMP long int _MAPAPI mapGetObjectPress(HOBJ info);  

 // Установить/Запросить способ отображения метрики объекта в виде
 // динамического сплайна
 // type - тип сплайна (SPLINETYPE_SMOOTH, SPLINETYPE_POINTS)
 // При ошибке или отмене рисования сплайна возвращает ноль

_MAPIMP long int _MAPAPI mapSetObjectSpline(HOBJ info, long int type);
_MAPIMP long int _MAPAPI mapGetObjectSpline(HOBJ info);

 // Установить/Запросить признак выравнивания подобъекта по вертикали
 // При отображении первая точка метрики подобъекта выравнивается по вертикали
 // по первой точке метрики объекта для векторных знаков и подписей
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapSetObjectVerticalAlignment(HOBJ info, long int flag);
 _MAPIMP long int _MAPAPI mapGetObjectVerticalAlignment(HOBJ info);

 // Установить/Запросить признак формы объекта при его создании          // 09/07/15
 // Установка формы не меняет метрику и не выполняет контроль реальной формы метрики,
 // но может применяться в интерактивном редакторе для подбора инструментов редактирования
 // 1 (OBM_RECTANGLE) - метрика имеет форму горизонтального (вертикального) прямоугольника
 // 2 (OBM_CIRCLE) - метрика имеет форму окружности
 // Если форма не установлена возвращает ноль

 _MAPIMP long int _MAPAPI mapSetObjectFormType(HOBJ info, long int type);
 _MAPIMP long int _MAPAPI mapGetObjectFormType(HOBJ info);

 // Запросить/Установить признак использования семантики при отображении объекта
 // info   - идентификатор объекта карты в памяти
 // isview - признак использования семантики при отображении объекта
 // При отображении объектов классификатора и присвоении служебных семантик,
 // влияющих на вид объекта, использование семантики происходит автоматически
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapSetObjectViewSemantic(HOBJ info, long int isview);
 _MAPIMP long int _MAPAPI mapGetObjectViewSemantic(HOBJ info);



 // ************************************************************
 //                                                            *
 //         ЗАПРОС СЕМАНТИКИ (АТРИБУТОВ) ОБЪЕКТА               *
 //                                                            *
 // ************************************************************

 // Запросить число семантических характеристик у объекта
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticAmount(HOBJ info);

 // Запросить значение семантической характеристики объекта
 // Значение преобразуется в символьный вид без раскодирования
 // info   - идентификатор объекта карты в памяти
 // number - последовательный номер характеристики (c 1),
 // value  - адрес размещения строки,
 // size   - максимальная длина строки
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticValue(HOBJ info, long int number,
                                          char *value, long int size);

 // Запросить - не является ли семантика строкой UNICODE
 // info   - идентификатор объекта карты в памяти
 // number - последовательный номер характеристики (c 1)
 // Если семантика в кодировке Unicode - возврвщает ненулевое значение

_MAPIMP long int _MAPAPI mapIsSemanticUnicode(HOBJ info, long int number);
_MAPIMP long int _MAPAPI IsSemanticUnicode(HOBJ info, long int number);

 // Запросить значение семантической характеристики объекта в UNICODE
 // Значение преобразуется в символьный вид без раскодирования
 // info   - идентификатор объекта карты в памяти
 // number - последовательный номер характеристики (c 1),
 // value  - адрес размещения строки,
 // size   - максимальная длина строки в байтах
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticValueUn(HOBJ info, long int number,
                                            WCHAR *value, long int size);

_MAPIMP long int _MAPAPI mapSemanticValueUnicode(HOBJ info, long int number,
                                                 WCHAR *value, long int size);

 // Запросить значение семантической характеристики объекта
 // в виде числа с плавающей точкой двойной точности
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1)
 // Если значение семантики не может быть преобразовано
 // к числовому виду или не найдено - возвращает ноль

_MAPIMP double _MAPAPI mapSemanticDoubleValue(HOBJ info, long int number);

 // Запросить значение семантической характеристики объекта
 // в виде целого числа
 // number  - последовательный номер характеристики
 // Если значение семантики не может быть преобразовано
 // к числовому виду или - возвращает ноль

_MAPIMP double _MAPAPI mapSemanticLongValue(HOBJ info, long int number);

 // Запросить значение семантической характеристики объекта
 // в символьном раскодированном виде
 // Например: Для семантики "СОСТОЯНИЕ" значение "5"
 // заменется на "жилой"
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1),
 // place   - адрес размещения строки,
 // maxsize - максимальная длина строки в байтах
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticValueName(HOBJ info, long int number,
                                              char *place, long int maxsize);
_MAPIMP long int _MAPAPI mapSemanticValueNameUnicode(HOBJ info, long int number,
                                                     WCHAR *place, long int maxsize);
_MAPIMP long int _MAPAPI mapSemanticValueNameUn(HOBJ info, long int number,
                                                WCHAR *place, long int maxsize);

 // Запросить значение семантической характеристики объекта
 // в символьном раскодированном виде с добавлением единицы
 // измерения в символьном виде
 // Например: Для семантики "СОСТОЯНИЕ" значение "5"
 // заменется на "жилой";
 // Дл семантики "ВЫСОТА" значение "205,5" заменется
 // на "205,5 м"
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1),
 // place   - адрес размещения строки,
 // maxsize - максимальная длина строки
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticValueFullName(HOBJ info,long int number,
                                                  char *place,long int maxsize);
_MAPIMP long int _MAPAPI mapSemanticValueFullNameUn(HOBJ info,long int number,
                                                    WCHAR *place,long int maxsize);

 // Запросить значение семантической характеристики объекта
 // Значение преобразуется в символьный вид
 // info    - идентификатор объекта карты в памяти
 // code    - код характеристики,для которой ищется значение,
 // place   - адрес размещения строки,
 // maxsize - максимальная длина строки
 // number  - последовательный номер найденного значения,
 //  не равен последовательному номеру характеристики !
 //  например : код code имеют 3-я и 6-я характеристики,
 //             соответственно для них number = 1 и 2,
 //             а при number = 3  - код возврата будет ноль.
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер
 // найденной характеристики

_MAPIMP long int _MAPAPI mapSemanticCodeValue(HOBJ info, long int code,
                                              char *place, long int maxsize,
                                              long int number);

_MAPIMP long int _MAPAPI mapSemanticCodeValueUn(HOBJ info, long int code, WCHAR *place,
                                                long int maxsize, long int number);

 // Запросить значение семантической характеристики объекта
 // в виде числа с плавающей точкой двойной точности
 // info    - идентификатор объекта карты в памяти
 // code    - код характеристики,для которой ищется значение,
 // number  - последовательный номер найденного значения,
 //  не равен последовательному номеру характеристики !
 //  например : код code имеют 3-я и 6-я характеристики,
 //             соответственно для них number = 1 и 2,
 //             а при number = 3  - код возврата будет ноль.
 // Если значение семантики не может быть преобразовано
 // к числовому виду или не найдено - возвращает ноль

_MAPIMP double _MAPAPI mapSemanticCodeDoubleValue(HOBJ info, long int code,
                                                  long int number);

 // Запросить значение семантической характеристики объекта
 // в символьном раскодированном виде
 // info    - идентификатор объекта карты в памяти
 // code    - код характеристики,для которой ищется значение,
 // place   - адрес размещения строки,
 // maxsize - максимальная длина строки
 // number  - последовательный номер найденного значения,
 //  не равен последовательному номеру характеристики !
 //  например : код code имеют 3-я и 6-я характеристики,
 //             соответственно для них number = 1 и 2,
 //             а при number = 3  - код возврата будет ноль.
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер
 // найденной характеристики

_MAPIMP long int _MAPAPI mapSemanticCodeValueName(HOBJ info, long int code,
                                                  char *place, long int maxsize,
                                                  long int number);

_MAPIMP long int _MAPAPI mapSemanticCodeValueNameUn(HOBJ info, long int code,
                                                    WCHAR *place, long int maxsize,
                                                    long int number);

 // Запросить название семантической характеристики объекта
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1)
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapSemanticName(HOBJ info, long int number);
_MAPIMP long int _MAPAPI mapSemanticNameEx(HOBJ info, long int number,
                                           char *name, long int size);
_MAPIMP long int _MAPAPI mapSemanticNameUn(HOBJ info, long int number,
                                           WCHAR *name, long int size);

 // Запросить код семантической характеристики объекта
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticCode(HOBJ info, long int number);

 // Запросить последовательный номер кода семантической
 // характеристики объекта (c 1)
 // info    - идентификатор объекта карты в памяти
 // code    - код семантической характеристики в классификаторе
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticNumber(HOBJ info, long int code);

 // Запросить общий размер строки для символьной семантики с учетом повтора
 // кода семантики в нескольких характеристиках
 // При записи строки длиной более 255 символов она делится на несколько
 // характеристик, содержащих части строки менее 255 символов
 // Повтор характеристики может быть связан с применением повторяемой
 // характеристики (имеющей несколько значений для одного объекта,
 // см. в RSCSEMANTICEX поле Reply)
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1)
 // Если строка будет запрошена в UNICODE, то размер строки нужно умножить на 2
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticStringLength(HOBJ info, long int number);

 // Запросить общую строку для символьной семантики с учетом повтора
 // кода семантики в нескольких характеристиках
 // При записи строки (mapAppendSemantic) длиной более 255 символов
 // она делится на несколько характеристик, содержащих части строки
 // менее 255 символов.
 // Если для редактирования семантики запрошена строка длиной более 255 символов,
 // то запись обратно нужно делать через mapAppendSemantic (для автоматического
 // удаления старого набора строк неповторяемой семантики).
 // Повтор характеристики может быть связан с применением повторяемой
 // характеристики (имеющей несколько значений для одного объекта,
 // см. в RSCSEMANTICEX поле Reply)
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики (c 1)
 // value   - адрес строки, куда будет записана строка в кодировке ANSI
 // length  - максимальный размер выходной строки для контроля
 // При ошибке возвращает ноль, иначе - длину записанной строки

_MAPIMP long int _MAPAPI mapSemanticString(HOBJ info, long int number,
                                           char *value, long int length);

_MAPIMP long int _MAPAPI mapSemanticStringUnicode(HOBJ info, long int number,
                                                  WCHAR *value, long int length);

 // Заполнить структуру описания семантической характеристики
 // по коду семантики
 // info    - идентификатор объекта карты в памяти
 // code    - внешний код семантики
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticDescribeEx(HOBJ info, RSCSEMANTICEX * semtype,
                                               long int code);

 // Запросить количество записей в классификаторе
 // семантики по коду семантики
 // info    - идентификатор объекта карты в памяти
 // code - код семантики (семантика типа "TCODE")
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticClassificatorCount(HOBJ info, long int code);

 // Запросить название значения характеристики из
 // классификатора семантики по коду и
 // последовательному номеру в классификаторе
 // info    - идентификатор объекта карты в памяти
 // number - последовательный номер в классификаторе(1,2,3...)
 // code - код семантики
 // name - адрес буфера для результата запроса
 // size - размер буфера в байтах
 // При ошибке возвращает адрес пустой строки,
 // при успешном выполнении - адрес строки

_MAPIMP const char* _MAPAPI mapSemanticClassificatorName(HOBJ info,long int code,
                                                         long int number);

_MAPIMP long int _MAPAPI mapSemanticClassificatorNameEx(HOBJ info, long int code,
                                                        long int number,
                                                        char *name, long int size);

_MAPIMP long int _MAPAPI mapSemanticClassificatorNameUn(HOBJ info,long int code,
                                                        long int number,
                                                        WCHAR *name, long int size);

 // Запросить код значения характеристики из
 // классификатора семантики по коду и
 // последовательному номеру в классификаторе
 // info   - идентификатор объекта карты в памяти
 // number - последовательный номер в классификаторе(1,2,3...)
 // code   - внешний код семантики
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSemanticClassificatorCode(HOBJ info, long int code,
                                                      long int number);


 // ************************************************************
 //                                                            *
 //      РЕДАКТИРОВАНИЕ СЕМАНТИКИ (АТРИБУТОВ) ОБ'ЕКТА          *
 //                                                            *
 // ************************************************************

 // Запросить имеются ли семантики, которые еще могут быть
 // добавлены для данного объекта
 // Результат запроса изменяется в процессе редактирования семантики объекта !
 // (некоторые характеристики могут присваиваться только один раз)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsAvailableSemantic(HOBJ info);

 // Запросить количество видов семантик, которые еще могут быть
 // добавлены для данного объекта
 // Изменяется в процессе редактирования семантики объекта !
 // (некоторые характеристики могут присваиваться только один раз)
 // info    - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAvailableSemanticCount(HOBJ info);

 // Запросить внешний код доступной семантики на объект
 // по последовательному номеру доступных семантик
 // Изменяется в процессе редактирования семантики объекта !
 // info    - идентификатор объекта карты в памяти
 // number - последовательный номер доступных семантик (1,2,3...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAvailableSemanticCode(HOBJ info, int number);

 // Запросить список внешних кодов доступных семантик на объект
 // Изменяется в процессе редактирования семантики объекта !
 // info    - идентификатор объекта карты в памяти
 // buffer  - указатель на область памяти для размещения списка кодов семантик
 // count   - максимальное число элементов в списке (размер буфера деленный на 4)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAvailableSemanticList(HOBJ info, long int *buffer, long int count);

 // Запросить список внешних кодов доступных обязательных семантик на объект        // 15/10/15
 // Изменяется в процессе редактирования семантики объекта !
 // info    - идентификатор объекта карты в памяти
 // buffer  - указатель на область памяти для размещения списка кодов семантик
 // count   - максимальное число элементов в списке (размер буфера деленный на 4)
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapAvailableMustSemanticList(HOBJ info,long int * buffer, long int count);

 // Добавить новую характеристику в семантику объекта
 // info    - идентификатор объекта карты в памяти
 // code    - внешний код характеристики
 // value   - адрес строки,содержащей новое значение
 //           в символьном виде, числа с плавающей точкой могут иметь
 //           разделителем только символ точка "."
 // size    - максимальная длина строки
 // (числовые значения будут преобразовываться в двоичный вид)
 // Если такая семантика была и она не повторяемая - значение заменяется  // 10/07/06
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер
 // созданной характеристики
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendSemantic(HOBJ info, long int code,
                                           const char *value,
                                           long int size);

 // Добавить новую характеристику в семантику объекта в виде UNICODE-строки  // 24/03/11
 // info    - идентификатор объекта карты в памяти
 // code    - внешний код характеристики
 // value   - адрес строки,содержащей новое значение
 //           в символьном виде UNICODE UTF-16,
 // size    - максимальная длина строки
 // Если текст содержит латинские символы от 0x0001 до 0x007E
 // или кириллицу (0x0400 - 0x045F) и на компьютере
 // установлена русская Windows (OEM 866 или 1251), то текст
 // автоматически запишется в ANSI,
 // иначе новое значение сохранится в UTF-16
 // Числовые значения будут преобразовываться в двоичный вид
 // Если такая семантика была и она не повторяемая - значение заменяется
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер созданной характеристики
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendSemanticUnicode(HOBJ info, long int code,
                                                  const WCHAR *value, long int size);

 // Устаревшая функция - вызывает mapAppendSemanticUnicode

_MAPIMP long int _MAPAPI mapAppendSemanticUn(HOBJ info, long int code,
                                             const WCHAR *value, long int size);

 // Добавить новую характеристику в семантику объекта,
 // info    - идентификатор объекта карты в памяти
 // code    - внешний код характеристики
 // value   - значение в виде числа двойной точности
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер
 // созданной характеристики

_MAPIMP long int _MAPAPI mapAppendSemanticDouble(HOBJ info, long int code,
                                                 double value);

 // Добавить новую характеристику в семантику объекта,   // 12/12/12
 // info    - идентификатор объекта карты в памяти
 // code    - внешний код характеристики
 // value   - значение в виде целого числа
 // При ошибке возвращает ноль,
 // при успешном выполнении - последовательный номер
 // созданной характеристики

_MAPIMP long int _MAPAPI mapAppendSemanticLong(HOBJ info, long int code,
                                               long int value);

 // Удалить семантическую характеристику объекта
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики,
 //           если номер равен "-1", удаляются все характеристики
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteSemantic(HOBJ info, long int number);


 // Изменить значение кода семантической характеристики объекта
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики
 // code    - внешний код характеристики
 // При ошибке возвращает ноль,
 // иначе - внутренний код семантики

_MAPIMP long int _MAPAPI mapSetSemanticCode(HOBJ info, long int number,
                                            long int code);

 // Изменить значение семантической характеристики объекта
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики,
 // place   - адрес строки, содержащей новое значение
 //           в символьном виде; Для семантики типа "классификатор"
 //           передается код значения в виде числа в символьном виде,
 //           то же - для типа "ссылка на объект".
 // maxsize - длина передаваемой строки (для контроля)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetSemanticValue(HOBJ info, long int number,
                                             const char *place, long int maxsize);

 // Изменить значение семантической характеристики объекта в кодировке UNICODE
 // info    - идентификатор объекта карты в памяти
 // number  - последовательный номер характеристики,
 // place   - адрес строки, содержащей новое значение
 //           в UNICODE. Для семантики типа "классификатор"
 //           передается код значения в виде числа в символьном виде,
 //           то же - для типа "ссылка на объект".
 // maxsize - длина передаваемой строки (для контроля)
 // Если текст содержит латинские символы от 0x0001 до 0x007E
 // или кириллицу (0x0400 - 0x045F) и на компьютере
 // установлена русская Windows (OEM 866 или 1251), то текст
 // автоматически запишется в ANSI,
 // иначе новое значение сохранится в UTF-16
 // Числовые значения будут преобразовываться в двоичный вид
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetSemanticValueUn(HOBJ info, long int number,
                                               const WCHAR *place, long int maxsize);

_MAPIMP long int _MAPAPI mapSetSemanticValueUnicode(HOBJ info, long int number,
                                                    const WCHAR *place, long int maxsize);

 // Изменить описание объекта при изменении семантических
 // характеристик
 // info    - идентификатор объекта карты в памяти
 // Если вид объекта не изменился возвращает 0

_MAPIMP long int _MAPAPI mapRedefineObject(HOBJ info);

 // Обновить значения семантик типа формула при обновлении
 // семантики или метрики объекта
 // Данная функция автоматически вызывается при сохранении объекта (mapCommitObject)
 // info    - идентификатор объекта карты в памяти
 // mode - признак обновления семантики и метрики:
 // 0 - обновилась только метрика, 1 - обновилась семантика
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateSemanticByFormula(HOBJ info, long int mode);

 // ***********************************************************************
 //      РЕДАКТИРОВАНИЕ СЕМАНТИКИ (АТРИБУТОВ) ОБ'ЕКТА -                   *
 //  Создание символьной строки по форматированной строке                 *
 //      с учетом значений семантики объекта                              *
 // формат строки: после %# идет номер семантики, за ним в []- значение,  *
 // которое будет вставлено в строку при отсутствии указанной семантики   *
 // остальной текст в произвольной форме.                                 *
 // Пример:входная строка - "дом N %#45[нет] сост. %#3[не заполнено]"     *
 //     результат по значениям семантики для конкретного объекта          *
 //     "дом N 5 сост. не заполнено" или "дом N 7-a сост. ЖИЛОЙ"          *
 //      или  "дом N нет сост. не заполнено"                              *
 // ***********************************************************************

 // Разбор форматированной  строки на части
 // value - входная строка,
 // размеры строки не более 256.
 // Нет символов форматирования или ошибки - возвращает 0,
 // иначе идентификатор формулы в памяти (HFORMULA)
 // Для каждого полученного и больше не используемого
 // идентификатора HFORMULA необходим вызов функции mapFreeFormula()

_MAPIMP HFORMULA _MAPAPI mapTakeStringToPieces(const char *value);

_MAPIMP HFORMULA _MAPAPI mapTakeStringToPiecesUn(const WCHAR *value);

 // Собрать символьную строку по идентификаторe HFORMULA
 // с учетом семантик объекта
 // info - идентификатор объекта карты в памяти
 // string - строка для записи результата
 // size   - размер строки
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapBuildFormulaString(HFORMULA formula, HOBJ info,
                                               char *string, long int size);

_MAPIMP long int _MAPAPI mapBuildFormulaStringUn(HFORMULA formula, HOBJ info,
                                                 WCHAR *string, long int size);

 // Освободить  идентификатор HFORMULA
_MAPIMP void _MAPAPI mapFreeFormula(HFORMULA formula);

 // ************************************************************
 //                                                            *
 //         ЗАПРОС МЕТРИКИ (КООРДИНАТ) ОБ'ЕКТА                 *
 //                                                            *
 // ************************************************************

 // Запросить замкнутость объекта/подобъекта
 // info   - идентификатор объекта карты в памяти
 // number - номер подобъекта (для объекта - равен нулю)
 // Возвращает:  0 - не замкнут, не 0 - замкнут

_MAPIMP long int _MAPAPI mapGetExclusiveSubject(HOBJ info, long int number);

 // Запрос габаритов объекта (в метрах)
 // info   - идентификатор объекта карты в памяти
 // dframe - габариты метрики объекта в метрах в системе документа
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectFrame(HOBJ info, MAPDFRAME *dframe);

 // Запрос/Пересчет габаритов изображения знака объекта (в метрах)
 // info   - идентификатор объекта карты в памяти
 // dframe - габариты изображения объекта в метрах
 // force  - признак принудительного пересчета габаритов (необходимо установить,
 //          если объект редактировался, но не записан на карту)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectViewFrameEx(HOBJ info, MAPDFRAME *dframe,
                                              long int force);
_MAPIMP long int _MAPAPI mapObjectViewFrame(HOBJ info, MAPDFRAME *dframe);

 // Запрос числа точек метрики объекта/подобъекта
 // info    - идентификатор объекта карты в памяти
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPointCount(HOBJ info, long int subject);

 // Запрос числа составных частей (подобъектов + 1)
 // info    - идентификатор объекта карты в памяти
 // Если подобъектов нет - возвращает 1 (только объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPolyCount(HOBJ info);

 // Запросить координаты точки в системе координат документа
 // number - номер точки (начинается с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetPlanePoint(HOBJ info, DOUBLEPOINT *point,
                                          long int number, long int subject);

 // Запрос координат точки в метрах в системе координат карты
 // number - номер точки
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMapPlanePoint(HOBJ info, DOUBLEPOINT *point,
                                             long int number, long int subject);

 // Запрос координаты точки объекта в прямоугольной системе
 // в метрах на местности
 // info    - идентификатор объекта карты в памяти
 // number - номер точки (начинается с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Функции реализованы через вызов mapGetPlanePoint()
 // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapXPlane(HOBJ info, long int number = 1,
                                 long int subject = 0);

_MAPIMP long int _MAPAPI mapXPlaneEx(HOBJ info, long int number,
                                     long int subject, double *x);

_MAPIMP double _MAPAPI mapYPlane(HOBJ info, long int number = 1,
                                 long int subject = 0);

_MAPIMP long int _MAPAPI mapYPlaneEx(HOBJ info, long int number,
                                     long int subject, double *y);

_MAPIMP double _MAPAPI mapHPlane(HOBJ info, long int number = 1,
                                 long int subject = 0);
_MAPIMP long int _MAPAPI mapHPlaneEx(HOBJ info, long int number,
                                     long int subject, double *h);

 // Запросить геодезические координаты точки в радианах в системе документа
 // number - номер точки (начинается с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetGeoPoint(HOBJ info, DOUBLEPOINT *point,
                                        long int number, long int subject);

 // Запросить геодезические координаты точки в радианах на эллипсоиде WGS-84
 // number - номер точки (начинается с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetGeoPointWGS84(HOBJ info, DOUBLEPOINT *point,
                                             long int number, long int subject);

_MAPIMP long int _MAPAPI mapGetGeoPointWGS843D(HOBJ info, DOUBLEPOINT *point,
                                               double *heigth, long int number,
                                               long int subject);
                                          
 // Запрос геодезических координат точки объекта
 // в радианах (BGeo - широта, LGeo - долгота)
 // number - номер точки (начинается с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Функции реализованы через вызов mapGetGeoPoint()
 // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapBGeo(HOBJ info, long int number  = 1,
                                          long int subject = 0);

_MAPIMP double _MAPAPI mapLGeo(HOBJ info, long int number  = 1,
                                          long int subject = 0);



 // Запросить имеет ли объект 3-ехмерную метрику
 // info    - идентификатор объекта карты в памяти
 // Если да, возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsObject3D(HOBJ info);

 // Запросить является ли объект мультиполигоном
 // Мультиполигон - это площадной объект, у которого некоторые подобъекты
 // могут быть вне границ объекта
 // При подсчете площади мультиполигона площадь внешних подобъектов
 // будет добавляться к площади основного объекта, а площади
 // внутренних подобъектов - вычитаться
 // info    - идентификатор объекта карты в памяти
 // Если да, возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsMultiPolygon(HOBJ info);

 // Установить\сбросить признак мультиполигона                       // 31/05/16
 // info  - идентификатор объекта карты в памяти
 // multi - признак мультиполигона
 // isautoset - признак автоматической расстановки флага размещения (входимости)
 // При первой установке признака мультиполигона может выполняться автоматическая
 // расстановка флага размещения подобъекта по первой точке метрики подобъекта -
 // поиск входимости одних подобъектов в другие
 // Если установку флагов размещения выполняет внешняя функция, то
 // признак мультиполигона может устанавливаться после завершения создания
 // объекта и всех подобъектов без автоматической расстановки,
 // затем выполняется установка флагов подобъектов в функции mapSetSubjectMultiFlag
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMultiPolygonEx(HOBJ info, long int multi, long int isautoset = 1);
_MAPIMP long int _MAPAPI mapSetMultiPolygon(HOBJ info, long int multi);

 // Запросить флаг размещения подобъекта вне объекта                 // 31/05/16
 // subject - номер подобъекта, если 0 - запрос для объекта
 // Для внешних подобъектов возвращает отрицательное значение (-1),
 // для внутренних подобъектов возвращает номер внешнего подобъекта (c 0), в который
 // входит данный подобъект
 // При отсутствии описания возвращает ноль

_MAPIMP long int _MAPAPI mapGetSubjectMultiFlag(HOBJ info, long int subject);

 // Установить флаг размещения подобъекта вне объекта                // 31/05/16
 // subject - номер подобъекта, если 0 - запрос для объекта
 // flag    - признак размещения (входимости) подобъекта
 // Для внешних подобъектов устанавливается отрицательное значение (-1),
 // для внутренних подобъектов устанавливается номер внешнего подобъекта (c 0), в который
 // входит данный подобъект
 // При отсутствии описания возвращает ноль

_MAPIMP long int _MAPAPI mapSetSubjectMultiFlag(HOBJ info, long int subject, long int flag);

 // Запросить тип высоты в третьей координате
 // Реально высота может быть и не задана
 // (0 - абсолютная, иначе - относительная)

_MAPIMP long int _MAPAPI mapGetHeightType(HOBJ info);

 // Установить тип высоты в третьей координате
 // Значение высоты может быть установлено позднее
 // (0 - абсолютная, иначе - относительная)
 // Объекты с относительной высотой не влияют на построение матрицы высот
 // Например - трубопроводы наземные и подземные

_MAPIMP long int _MAPAPI mapSetHeightType(HOBJ info, long int type);

 // Определить центр объекта
 // info  - идентификатор объекта карты в памяти
 // x, y  - расчитанные координаты центра объекта в метрах в системе документа
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetObjectCenter(HMAP hmap, HOBJ info, double *x, double *y);

 // Запросить - используется ли общий буфер метрики для работы с векторными картами
 // Общий буфер применяется на 32-разрядной платформе при работе с большим числом
 // одновременно открытых векторных карт
 // В этом случае многопоточный режим использования MAPAPI-функций не применим

_MAPIMP long int _MAPAPI mapIsCommonBufferActive();

 // ************************************************************
 //                                                            *
 //      РЕДАКТИРОВАНИЕ МЕТРИКИ (КООРДИНАТ) ОБ'ЕКТА            *
 //                                                            *
 // ************************************************************

 // Добавить в конец метрики объекта точку
 // info    - идентификатор объекта карты в памяти
 // x,y     - координаты точки в метрах
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат задано в метрах на местности
 // Для изменения координаты Н необходимо далее
 // выполнить функцию SetHPlane(...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendPointPlane(HOBJ info, double x, double y,
                                             long int subject = 0);

 // Добавить в конец метрики объекта точку
 // info    - идентификатор объекта карты в памяти
 // x,y     - координаты точки в метрах
 // h       - высота (абсолютная или относительная) в метрах
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Тип высоты определяется функциями mapGetHeightType и mapSetHeightType
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendPointPlane3D(HOBJ info, double x, double y,
                                               double h, long int subject = 0);

 // Удалить заданную точку метрики
 // info    - идентификатор объекта карты в памяти
 // number  - номер точки (с 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeletePointPlane(HOBJ info, long int number,
                                             long int subject = 0);
_MAPIMP long int _MAPAPI mapDeletePoint(HOBJ info, long int number,
                                        long int subject = 0);

 // Удаление из метрики одинаковых точек
 // precision - величина расхождения значений координат в метрах на местности
 // height    - признак учета трехмерной метрики (в этом случае две одинаковые
 //             точки с разной высотой считаются разными)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDeleteEqualPoint(HOBJ info, double precision, long int height);

 // Удалить из объекта/подобъекта участок с точки number1 по точку number2
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Текущей становится точка следующая за удаленными
 // При удалении всех точек объекта/подобъекта удаляется только метрика,
 // объект/подобъект не удаляется

_MAPIMP long int _MAPAPI mapDeletePartObject(HOBJ info, long int number1,
                                             long int number2, long int subject = 0);

 // Вставить в метрику объекта точку
 // info    - идентификатор объекта карты в памяти
 // x,y     - координаты точки в метрах
 // number  - номер точки за которой будет добавлена новая точка
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат задано в метрах на местности
 // Для изменени координаты Н необходимо далее
 // выполнить функцию mapSetHPlane(...)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapInsertPointPlane(HOBJ info, double x, double y,
                                             long int number, long int subject = 0);

 // Изменить координаты точки метрики
 // Значение координат задано в метрах на местности
 // info    - идентификатор объекта карты в памяти
 // x,y     - координаты точки в метрах
 // number  - номер точки (c 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdatePointPlane(HOBJ info, double x, double y,
                                             long int number, long int subject = 0);

_MAPIMP long int _MAPAPI mapUpdatePointPlane3D(HOBJ info, double x, double y, double h,
                                               long int number, long int subject = 0);

 // Добавить в конец метрики объекта точку
 // info    - идентификатор объекта карты в памяти
 // b,l     - координаты точки в радианах в системе координат документа
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат должно соответствовать системе координат,
 // проекции и эллипсоиду карты
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendPointGeo(HOBJ info, double b, double l,
                                           long int subject = 0);

 // Добавить в конец метрики объекта точку
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат задано в радианах на эллипсоиде WGS-84
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendPointGeoWGS84(HOBJ info, double b, double l,
                                                long int subject);

_MAPIMP long int _MAPAPI mapAppendPointGeoWGS843D(HOBJ info, double b, double l, double h,
                                                  long int subject);

 // Вставить в метрику объекта точку
 // info    - идентификатор объекта карты в памяти
 // b,l     - координаты точки в радианах
 // number - номер точки за которой будет добавлена новая точка
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат должно соответствовать системе координат,
 // проекции и эллипсоиду карты
 // Для изменения координаты Н необходимо далее
 // выполнить функцию HPlane(...)
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapInsertPointGeo(HOBJ info, double b, double l,
                                            long int number, long int subject = 0);

 // Изменить координаты точки метрики
 // info    - идентификатор объекта карты в памяти
 // b,l     - координаты точки в радианах
 // number  - номер обновляемой точки
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // Значение координат должно соответствовать системе координат,
 // проекции и эллипсоиду карты
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdatePointGeo(HOBJ info, double b, double l,
                                           long int number, long int subject = 0);

_MAPIMP long int _MAPAPI mapUpdatePointGeo3D(HOBJ info, double b, double l,
                                             double h, long int number,
                                             long int subject = 0);

 // Изменить координаты общей точки метрики у данного объекта и
 // у всех объектов карты, имеющих такую точку
 // Изменение выполняется после вызова mapCommitObject()
 // или mapCommitWithPlace()
 // Значение координат задано в метрах на местности
 // info    - идентификатор объекта карты в памяти
 // x,y,h   - координаты точки в метрах
 // number  - номер точки (c 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdatePointPlaneInMap(HOBJ info, double x, double y,
                                                  long int number,
                                                  long int subject = 0);

_MAPIMP long int _MAPAPI mapUpdatePointPlane3DInMap(HOBJ info, double x, double y,
                                                    double h, long int number,
                                                    long int subject = 0);

 // Изменить координаты общей точки метрики у данного объекта и
 // у всех объектов общего слоя,  имеющих такую точку
 // Изменение выполняется после вызова mapCommitObject()
 // или mapCommitWithPlace()
 // Значение координат задано в метрах на местности
 // info    - идентификатор объекта карты в памяти
 // x,y,h   - координаты точки в метрах
 // number  - номер точки
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdatePointPlaneInLayer(HOBJ info, double x, double y,
                                                    long int number,
                                                    long int subject = 0);

_MAPIMP long int _MAPAPI mapUpdatePointPlane3DInLayer(HOBJ info, double x, double y,
                                                      double h, long int number,
                                                      long int subject = 0);

 // Редактирование координаты точки объекта/подобъекта
 // в прямоугольной системе в метрах на местности
 // info    - идентификатор объекта карты в памяти
 // x,y,h   - координаты точки в метрах
 // number  - номер точки (c 1)
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetXPlane(HOBJ info, double x, long int number,
                                      long int subject = 0);

_MAPIMP long int _MAPAPI mapSetYPlane(HOBJ info, double y, long int number,
                                      long int subject = 0);

_MAPIMP long int _MAPAPI mapSetHPlane(HOBJ info, double h, long int number,
                                      long int subject = 0);

 // Создать дескриптор подобъекта в записи метрики
 // В конец записи добавляется дескриптор подобъекта с нулевым числом точек
 // info    - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateSubject(HOBJ info);

 // Удалить подобъект в записи метрики
 // info   - идентификатор объекта карты в памяти
 // number - номер удаляемого подобъекта (с 1),
 // если равен (-1), то удаляется вся метрика объекта вместе с подобъектами
 // Текущей становится первая точка объекта
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteSubject(HOBJ info, long int number);

 // Сместить все координаты метрики объекта на заданную
 // величину (delta) в метрах
 // info   - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapRelocateObjectPlane(HOBJ info,
                                                DOUBLEPOINT *delta);

 // Сместить все координаты метрики объекта на заданную
 // величину (delta) в соответствии с параметром place
 // hmap  - идентификатор открытых данных
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapRelocateObject(HMAP hmap, HOBJ info,
                                           DOUBLEPOINT * delta,
                                           long int place/* = PP_MAP*/);

 // Изменить направление цифрования подобъекта
 // info  - идентификатор объекта карты в памяти
 // number - номер подобъекта (с 0),
 // При ошибке возвращает ноль, иначе - новое значение
 // (OD_RIGHT,OD_LEFT,... - см. Maptype.h)

_MAPIMP long int _MAPAPI mapChangeSubjectDirect(HOBJ info, long int number);

 // Изменить направление цифрования объекта
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль, иначе - новое значение
 // (OD_RIGHT,OD_LEFT,... - см. Maptype.h)

_MAPIMP long int _MAPAPI mapChangeObjectDirect(HOBJ info);

 // Переформировать объект (подобъект), установив первой заданную точку
 // number - номер точки
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetFirstPoint(HOBJ info, long int number,
                                          long int subject);

 // Линейная фильтрация метрики
 // info  - идентификатор объекта карты в памяти
 // precision - точность в метрах
 // Удаляет: 1. двойные точки метрики;
 //          2. незамкнутые подобъекты < 2 точек;
 //          3. замкнутые подобъекты < 4 точек;
 //          4. точки метрики, лежащие в середине отрезка прямой
 //             на расстоянии precision от прямой.
 // Объект не удаляет никогда !!!
 // Возвращает общее число точек метрики
 // При ошибках возвращает:
 //    0 - ошибка структуры
 //   -1 - объект состоит из одной точки
 //   -2 - объект состоит из двух одинаковых точек
 //   -3 - число точек замкнутого контура объекта равно 3
 //  -10 - число точек метрики превышает длину записи метрики

_MAPIMP long int _MAPAPI mapLinearFilter(HOBJ info,
                                         double precision/* = DELTANULL*/);

 // Фильтрация объекта с учетом топологических связей с соседними
 // объектами листа карты, которому принадлежит объект
 // (фильтруются и соседние объекты, имеющие общие точки;
 //  концевые общие точки не фильтруются)
 // hMap      - идентификатор открытых данных
 // info      - фильтруемый объект
 // precision - точность в метрах на местности
 //             (минимальное расстояние от точки до прямой,
 //              соединяющей предыдущую и следующую точки)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGeneralFilter(HMAP hMap, HOBJ hobj,
                                          double precision/* = DELTANULL*/);

 // Фильтрация всех объектов одного листа карты с учетом
 // топологических связей с соседними объектами того же листа
 // той же карты (фильтруются и соседние объекты, имеющие общие точки;
 // концевые общие точки не фильтруются)
 // hMap      - идентификатор открытых данных
 // hSite     - идентификатор обрабатываемой карты
 // list      - номер листа
 // precision - точность в метрах на местности
 //             (минимальное расстояние от точки до прямой,
 //              соединяющей предыдущую и следующую точки)
 // hwnd      - идентификатор окна,которое будет извещаться
 //             (для отмены сообщений установить идентификатор в ноль)
 // Процесс посылает сообщение 0x590:
 // wparm : процент обработки,
 // lparm : количество обработанных объектов.
 //   общее количество = mapGetObjectCount(hMap, list)     - для основной карты
 //   общее количество = mapGetSiteObjectCount(hMap, hSit) - для пользовательской
 // Для завершения процесса вернуть число 0x590
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGeneralFilterInMap(HMAP hMap, HSITE hSite,
                                               long int list,
                                               double precision/* = DELTANULL*/,
                                               HWND hwnd);

 // Создание сплайна - сглаживание объекта и всех его подобъектов
 // Это сплайн, который проходит только
 // через первую и последнюю точки объекта(подобъекта) и как бы
 // сглаживает (спиливает) углы ломаной, соединяющей точки объекта
 // (метрику исходного объекта/подобъекта).
 // info  - исходная метрика объекта, по которому строится сплайн
 // cashion   - условный процент спиливания
 //             углов ломаной линии объекта (1<= cashion <= 50)
 //             (метрика исходного объекта/подобъекта).
 //             Чем больше cashion, тем больше спиливается угол
 // smooth    - плавность кривой сплайна
 //             (число точек между узлами объекта smooth >= 3).
 //             Чем больше smooth,тем глаже смотрится линия
 // precision - порог (точность) при фильтрации точек, для автоматического
 //             определения точности установить значение "-1".
 // Если исходный объект имел 3-ю координату (высоту),то у сплайна
 // также есть высота (интерполяция для новых точек).
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCashionSpline(HOBJ info, long int cashion,
                                          long int smooth,
                                          double precision/* = -1*/);

 // Создание сплайна - сглаживание подобъекта
 // Это сплайн, который проходит только
 // через первую и последнюю точки подобъекта и как бы
 // сглаживает (спиливает) углы ломаной, соединяющей точки объекта
 // (метрику исходного подобъекта).
 // info  - исходная метрика объекта, по подобъекту которого строится сплайн
 // subject - номер обрабатываемого подобъекта (0,1,2...)
 // cashion   - условный процент спиливания
 //             углов ломаной линии объекта (1<= cashion <= 50)
 //             (метрика исходного подобъекта).
 //             Чем больше cashion, тем больше спиливается угол
 // smooth    - плавность кривой сплайна
 //             (число точек между узлами подобъекта smooth >= 3).
 //             Чем больше smooth,тем глаже смотрится линия
 // precision - порог (точность) при фильтрации точек, для автоматического
 //             определения точности установить значение "-1".
 // Если исходный подобъект имел 3-ю координату (высоту),то у сплайна
 // также есть высота

_MAPIMP long int _MAPAPI mapCashionSplineSubject(HOBJ info, long int subject,
                                                 long int cashion,
                                                 long int smooth,
                                                 double precision/* = -1*/);

 // Создание сплайна - огибание объекта и всех его подобъектов
 // Это сплайн, который проходит через все точки исходного объекта
 // (метрика исходного объекта) и огибает его. Исходный объект
 // как бы вписан в сплайн.
 // info  - исходная метрика объекта, по которому строится сплайн
 // press     - максимальная амплитуда
 //             отхода кривой сплайна от отрезка
 //             в процентах от длины отрезка ( >= 5 ).
 //             Чем больше press, тем более сплайн может
 //             удаляться от отрезка ломаной (метрики исходного
 //             объекта/подобъекта).
 // smooth    - плавность кривой сплайна
 //             (число точек между узлами объекта smooth >= 3).
 //             Чем больше smooth,тем глаже смотрится линия
 // precision - порог (точность) при фильтрации точек, для автоматического
 //             определения точности установить значение "-1".
 // Если исходный объект имел 3-ю координату (высоту),то у сплайна
 // также есть высота (интерполяция для новых точек).
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapBendSpline(HOBJ info, long int press,
                                       long int smooth,
                                       double precision/* = -1*/);

 // Создание сплайна - огибание подобъекта
 // Это сплайн, который проходит
 // через все точки исходного подобъекта и огибает его.
 // Исходный объект как бы вписан в сплайн.
 // info  - исходная метрика объекта, по которому строится сплайн
 // subject - номер обрабатываемого подобъекта (0,1,2...)
 // press     - максимальная амплитуда
 //             отхода кривой сплайна от отрезка
 //             в процентах от длины отрезка ( >= 5 ).
 //             Чем больше press, тем более сплайн может
 //             удаляться от отрезка ломаной (метрики исходного
 //             объекта/подобъекта).
 // smooth    - плавность кривой сплайна
 //             (число точек между узлами объекта smooth >= 3).
 //             Чем больше smooth,тем глаже смотрится линия
 // precision - порог (точность) при фильтрации точек
 // Если исходный подобъект имел 3-ю координату (высоту),то у сплайна
 // также есть высота

_MAPIMP long int _MAPAPI mapBendSplineSubject(HOBJ info, int subject,
                                              long int press, long int smooth,
                                              double precision/* = -1*/);

// Сглаживание объекта с учетом топологических связей с соседними
// объектами листа карты, которому принадлежит объект
// hMap       - идентификатор открытых данных
// info       - фильтруемый объект
// press      - максимальная амплитуда отхода кривой сплайна от отрезка
//              в процентах от длины отрезка (>= 5), чем больше press,
//              тем больше сплайн может оклоняться от исходного контура
// smooth     - плавность кривой сплайна (>= 3) число точек между узлами объекта,
//              чем больше smooth, тем более гладким будет сплайн
// adjustdist - допуск согласования - максимальное расстояние, при котором две соседние
//              точки считаются расположенными на одном месте
// filterprec - уровень фильтрации (минимальное расстояние от точки до прямой,
//              соединяющей предыдущую и следующую точки)
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGeneralBendSpline(HMAP hMap, HOBJ info, long int press, long int smooth,
                                              double adjustdist, double filterprec);

 // Одномерный сглаживающий сплайн
 // points - массив точек (одна из координат)
 // count  - количество точек
 // smooth - уровень сглаживания (0..1; 0 - прямая линия, 1 - кубический сплайн)
 // При ошибке возвращает ноль
 
_MAPIMP long int _MAPAPI mapSmoothingSpline1(double *points, int count,
                                             double smooth);

 // Двухмерный сглаживающий сплайн
 // points - массив точек (x, y)
 // count  - количество точек
 // smooth - уровень сглаживания (0..1; 0 - прямая линия, 1 - кубический сплайн)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSmoothingSpline2(DOUBLEPOINT *points, int count,
                                             double smooth);

 // Трёхмерный сглаживающий сплайн
 // points - массив точек (x, y, h)
 // count  - количество точек
 // smooth - уровень сглаживания (0..1; 0 - прямая линия, 1 - кубический сплайн)
 // При ошибке возвращает ноль
 
_MAPIMP long int _MAPAPI mapSmoothingSpline3(XYHDOUBLE *points, int count,
                                             double smooth);

 // Cглаживающий сплайн (2-х или 3-х мерный в зависимости от наличия высоты)
 // info    - сглаживаемый объект
 // subject - сглаживаемый подобъект
 // smooth  - уровень сглаживания (0..1; 0 - прямая линия, 1 - кубический сплайн)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSmoothingSplineSubject(HOBJ info, long int subject,
                                                   double smooth);

 // Cглаживающий сплайн (2-х или 3-х мерный в зависимости от наличия высоты)
 // info   - сглаживаемый объект
 // smooth - уровень сглаживания (0..1; 0 - прямая линия, 1 - кубический сплайн)
 // При ошибке возвращает ноль
 
_MAPIMP long int _MAPAPI mapSmoothingSplineObject(HOBJ info, double smooth);

 // Построить дугу заданного радиуса с центром
 // в точке point2 (в метрах на местности)
 // точки point1 и point3 задаются для определения
 // направлений (в метрах на местности)
 // hmap   - идентификатор открытых данных
 // info   - идентификатор объекта карты в памяти
 // radius - в метрах на местности
 // метрика строится в info
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCreateArc(HMAP hmap, HOBJ info, DOUBLEPOINT *point1,
                                      DOUBLEPOINT *point2, DOUBLEPOINT *point3,
                                      double radius);

 // Построение зоны вокруг контура объекта или подобъекта
 // Функция строит зоны с прямыми углами (вокруг зданий и объектов
 // с длинными сторонами и большими углами между ними)
 // Устаревшая функция, сответствует вызову
 //  if (mapGetExclusiveSubject(info, subject))
 //   return mapInsideZoneObjectEx(radius, info, subject, form, 0.001 * Map->Scale(), 3.);
 // return mapZoneObjectEx(radius, info, subject, form, 0.001 * Map->Scale(), 3., 3);
 // radius  - радиус создаваемой зоны (в метрах на местности)
 // info    - идентификатор копии объекта, по метрике которого строится зона.
 //           В этот объект будет записана метрика построенной зоны, которую
 //           можно будет сохранить как новый площадной объект
 // subject - номер подобъекта, вокруг которого строим зону
 //           0 - зона строится для основного контура объекта (без учета подобъектов)
 // form    - форма края зоны
 //           для незамкнутых объектов:
 //           0 - край углом
 //           1 - прямоугольный край
 //           для замкнутых объектов:
 //           0 - "срезанные" углы
 //           1 - прямые углы
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapZoneObject(double radius,
                                       HOBJ info, long int subject/* = 0*/,
                                       long int form/* = 0*/);

 // Построение зоны вокруг подобъекта
 // radius  - радиус создаваемой зоны (в метрах на местности)
 // info    - идентификатор копии объекта, по метрике которого строится зона.
 //           В этот объект будет записана метрика построенной зоны.
 // subject - номер подобъекта, вокруг которого строится зона
 // form    - тип угла 0 - прямой, 1 - закругленный
 // arcdist - расстояние между точками по дуге (в метрах на местности)
 //           рекомендуется radius / 15
 // cornerfactor - коэффициент для расчета максимальной длины угла (рекомендуется 3)
 // Если задан прямой тип угла, то внешний угол обрезается по расстоянию от узла по
 // допуску radius*cornerfactor для устранения длинных углов
 // side    - направление построения зоны (1-справа, 2-слева, 3-с обеих сторон)
 // При ошибке возвращает ноль
 
_MAPIMP long int _MAPAPI mapZoneObjectEx(double radius, HOBJ info, long int subject,
                                         long int form, double arcdist, double cornerfactor, 
                                         long int side);

 // Построение зоны снаружи/внутри подобъекта
 // Функция строит зоны с прямыми углами (вокруг зданий и объектов
 // с длинными сторонами и большими углами между ними)
 // Устаревшая функция, соответствует вызову
 // mapInsideZoneObjectEx(radius, info, subject, 0, 0.001 * Map->Scale(), 3.)
 // radius    - радиус создаваемой зоны
 //    отрицательное значение - внутри объекта
 //    положительное значение - снаружи объекта
 // info      - метрика объекта; зона строится вокруг
 // subject   - номер подобъекта, вокруг которого строим зону
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapInsideZoneObject(double radius,
                                             HOBJ info, long int subject);

 // Построение зоны снаружи/внутри подобъекта
 // radius  - радиус создаваемой зоны (в метрах на местности)
 //           отрицательное значение - внутри объекта
 //           положительное значение - снаружи объекта
 // info    - идентификатор копии объекта, по метрике которого строится зона.
 //           В этот объект будет записана метрика построенной зоны.
 // subject - номер подобъекта, вокруг которого строится зону
 // form    - тип угла 0 - прямой, 1 - закругленный
 // arcdist - расстояние между точками по дуге (в метрах на местности)
 //           рекомендуется radius / 15
 // cornerfactor - коэффициент для расчета максимальной длины угла (рекомендуется 3)
 // Если тип угла прямой, то внешний угол обрезается по расстоянию от узла по
 // допуску radius*cornerfactor для устранения длинных углов
 // При ошибке возвращает ноль
 
_MAPIMP long int _MAPAPI mapInsideZoneObjectEx(double radius, HOBJ info, long int subject,
                                               long int form, double arcdist, double cornerfactor);

 // Построение половины зоны вокруг объекта / подобъекта
 // (справа от объекта по направлению цифрования)
 // radius    - радиус создаваемой зоны (в метрах на местности)
 // info - метрика объекта, по которому строится зона
 // subject - номер подобъекта, вокруг которого строим зону
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapHalfZoneObject(double radius,
                                           HOBJ info, long int subject/* = 0*/);


// Построение зоны вокруг основного контура линейного незамкнутого
// Устаревшая функция, соответствует вызову
// return mapZoneObjectEx(radius, info, 0, 1, 0.001 * mapGetMapScale(hmap), 3., 3);
// объекта вида "змейка" (без учета подобъектов)
// radius - радиус создаваемой зоны (в метрах на местности)
// info   - метрика объекта, по которому строится зона.
//          Исходный объект делится на части, вокруг каждой части строится зона.
//          Отдельные зоны объединяются в одну
// В info записывается площадной объект - зона
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapZoneLineObject(HMAP hmap, HOBJ info, double radius);

 // Замкнуть  метрику объект и все его подобъекты для
 // площадного или линейного объекта
 // info  - идентификатор объекта карты в памяти
 // delta - порог замыкания в мм на карте
 // если расстояние между первой и последней точкой меньше delta, то
 // вместо последней точки пишем первую
 // если расстояние между первой и последней точкой больше delta, то
 // после последней точки добавляем первую
 // возврат - 1 - нормальное завершение
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAbrige(HOBJ info, double delta);

 // Округлить метрику объекта по установленной точности карты (мм, см)
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapRoundObject(HOBJ info);                 // 07/06/16

 // Построение ортодромии
 // first - координаты первой точки в радианах
 // second - координаты второй точки в радианах
 // array - адрес массива координат построенной ортодромии,
 //         размер массива равен count
 // count - количество точек для построения ортодромии
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOrthodrome(DOUBLEPOINT *first, DOUBLEPOINT *second,
                                       DOUBLEPOINT *array, long int count);

 // Построение ортодромии (дуга на поверхности Земли, задающая кратчайшее расстояние)
 // между заданными точками
 // info   - идентификатор объекта карты в памяти
 // first  - координаты первой точки в радианах на текущем эллипсоиде документа (SetDocProjection)
 // second - координаты второй точки в радианах
 // При больших расстояниях точки дуги формируются с шагом не более 0,5 градуса,
 // при малых растояниях - не чаще 10 километров, что обеспечивает определение
 // длин и углов с точностью триангуляции 1 класса
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOrthodromeObject(HOBJ info, DOUBLEPOINT *first, DOUBLEPOINT *second);

 // Построение локсодромии
 // first - координаты первой точки в радианах
 // second - координаты второй точки в радианах
 // array - адрес массива координат построенной локсодромии,
 //         размер массива равен count
 // count - количество точек для построения локсодромии
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapLoxodrome(DOUBLEPOINT *first, DOUBLEPOINT *second,
                                      DOUBLEPOINT *array, long int count);

 // Построение зоны видимости по матрице высот в виде растрового изображения
 // hmap - идентификатор открытой векторной карты
 // namerst - полное имя растра
 // zonevisibility - параметры построения зоны (см.maptype.h)
 // hpaint - контекст поддержки многопоточного вызова (см. mapCreatePaintControl)
 // Построение производится при наличии открытой матрицы высот
 // Результат записывается в файл namerst
 // Возвращает номер растра в цепочке
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapVisibilityZone(HMAP hMap, const char *namerst,
                                           TBUILDZONEVISIBILITY *zonevisibility);

_MAPIMP long int _MAPAPI mapVisibilityZoneUn(HMAP hmap, const WCHAR *namerst,
                                             TBUILDZONEVISIBILITY *zonevisibility);

_MAPIMP long int _MAPAPI mapVisibilityZoneEx(HMAP hmap, const WCHAR *namerst,
                                             TBUILDZONEVISIBILITY *zonevisibility, HPAINT hPaint);


 // Определение видимости из точки point1 (координаты в метрах на местности) точку point2
 // deltaheight  - высота наблюдения (в метрах),
 // добавляется к высоте в точке point1
 // Вычисление производится при наличии открытой матрицы высот
 // Возвращает 0 - point2 не видна из point1
 //            1 - point2 видна из point1

_MAPIMP long int _MAPAPI mapVisibilityFromPoint(HMAP hmap, DOUBLEPOINT *point1,
                                                DOUBLEPOINT *point2, double deltaheight);


// Создание объектов - пустот по выделенным объектам                                    // 12/01/15
// hmap - идентификатор открытой векторной карты с выделенными объектами
// hsite - идентификатор пользовательской карты для записи объектов - пустот
// info   - идентификатор объекта карты в памяти, граница области для создания объектов - пустот
// incode - внутренний код объекта для сохранения объектов - пустот
// Возвращает количество созданных объектов
// При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateObjectVoid(HMAP hmap, HSITE hsite, HOBJ info, long int incode);


 // Удалить петли у объекта
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteLoop(HOBJ info, double precision);

 // Повернуть объект вокруг заданной в прямоугольной системе точки
 // на заданный угол
 // info   - идентификатор объекта карты в памяти
 // center - координаты точки, вокруг которой поворачивается объект (метры)
 // angle  - угол поворота по часовой стрелке (радианы, от -PI до +PI)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapRotateObject(HOBJ info, DOUBLEPOINT *center, double *angle);

 // Оконтуривание объектов, удовлетворяющих условиям обобщенного поиска,
 // по всем открытым векторным картам
 // hmap - идентификатор главной карты
 // hobj - объект в который записывается определенный контур
 // precision - допуск согласования объектов (должен быть >= DELTANULL)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapContourTotalSeekObjects(HMAP hmap, HOBJ info, double precision);

 // ************************************************************
 //                                                            *
 //      РЕДАКТИРОВАНИЕ ТЕКСТА ПОДПИСИ                         *
 //                                                            *
 // ************************************************************

  // Запросить содержание текстовой строки
  // info    - идентификатор объекта карты в памяти
  // text    - адрес для размещения строки (ANSI)
  // size    - длина выделенной области под строку в байтах
  // subject - номер подобъекта
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetText(HOBJ info, char *text,
                                    long int size, long int subject);

_MAPIMP long int _MAPAPI mapGetTextUnicode(HOBJ info, WCHAR *text,
                                           long int size, long int subject);

_MAPIMP long int _MAPAPI mapGetTextUn(HOBJ info, WCHAR *text,
                                      long int size, long int subject);

  // Установить новое содержание текстовой строки
  // info   - идентификатор объекта карты в памяти
  // text   - адрес новой строки (ANSI)
  // subject - номер подобъекта
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutText(HOBJ info, const char *text, long int subject);

  // Установить новое содержание текстовой строки
  // info   - идентификатор объекта карты в памяти
  // text   - адрес новой строки UNICODE UTF-16
  // Если текст содержит латинские символы от 0x0001 до 0x007E
  // или кириллицу (0x0400 - 0x045F) и на компьютере
  // установлена русская Windows (OEM 866 или 1251), то текст
  // автоматически запишется в ANSI,
  // иначе новое значение сохранится в UTF-16
  // subject - номер подобъекта
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutTextUnicode(HOBJ info, const WCHAR *text, long int subject);

  // Устаревшая функция - вызывает mapPutTextUnicode

_MAPIMP long int _MAPAPI mapPutTextUn(HOBJ info, const WCHAR *text, long int subject);

  // Запросить - хранится ли текст в кодировке UNICODE
  // info   - идентификатор объекта карты в памяти
  // Если текст в UNICODE - возвращает ненулевое значение
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsTextUnicode(HOBJ info);

  // Запросить длину текста в микронах на карте
  // info    - идентификатор объекта карты в памяти
  // subject - номер подобъекта
  // При масштабируемой подписи возвращает 0
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTextLengthMkm(HOBJ info, long int subject);

  // Запросить высоту строки текста для объектов типа подпись
  // в микронах на карте
  // info    - идентификатор объекта карты в памяти
  // При масштабируемой подписи возвращает 0
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTextHeightMkm(HOBJ info);

  // Запросить рамку подписи в пикселах для текущих условий отображения
  // hMap    - идентификатор документа
  // hdc     - идентификатор контекста, на котором рассчитывается размер рамки
  //           (может быть равен нулю)
  // rect    - положение области отображения, относительно которой считается рамка,
  //           в пикселах на документе
  // info    - идентификатор объекта типа подпись, параметры отображения должны быть типа IMG_TEXT
  // box     - координаты 4-ех точек наклонной рамки относительно верхнего левого
  //           угла области rect
  // Подобъекты подписи не учитываются при расчете рамки
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetPaintTextBorder(HMAP hMap, HDC hdc, RECT *rect,
                                               HOBJ info, DRAWBOX *box);

  // Запросить способ выравнивания текста по горизонтали
  // info    - идентификатор объекта карты в памяти
  // subject - номер подобъекта
  // (FA_LEFT,FA_RIGHT,FA_CENTER - см. mapgdi.h)

_MAPIMP long int _MAPAPI mapGetTextHorizontalAlign(HOBJ info, long int subject);

  // Запросить способ выравнивания текста по вертикали
  // info    - идентификатор объекта карты в памяти
  // subject - номер подобъекта
  // (FA_BOTTOM,FA_TOP,FA_BASELINE,FA_MIDDLE)

_MAPIMP long int _MAPAPI mapGetTextVerticalAlign(HOBJ info, long int subject);

  // Установить способ выравнивания текста по горизонтали
  // (FA_LEFT,FA_RIGHT,FA_CENTER)
  // info    - идентификатор объекта карты в памяти
  // subject - номер подобъекта (-1 - установить всем)
  // По умолчанию имеет значение FA_LEFT
  // При успешном выполнении возвращает установленное значение

_MAPIMP long int _MAPAPI mapPutTextHorizontalAlign(HOBJ info, long int align,
                                                   long int subject);

  // Установить способ выравнивания текста по вертикали
  // (FA_BOTTOM,FA_TOP,FA_BASELINE,FA_MIDDLE)
  // info    - идентификатор объекта карты в памяти
  // subject - номер подобъекта (-1 - установить всем)
  // По умолчанию имеет значение FA_BASELINE
  // При успешном выполнении возвращает установленное значение

_MAPIMP long int _MAPAPI mapPutTextVerticalAlign(HOBJ info, long int align,
                                                 long int subject);

 // ************************************************************
 //                                                            *
 //     РЕДАКТИРОВАНИЕ ГРАФИЧЕСКОГО ОПИСАНИЯ ОБ'ЕКТА           *
 //                                                            *
 //  (Графическое описание имеется, как правило, у объектов    *
 //   пользовательской карты, не связанных с классификатором)  *
 //                                                            *
 // ************************************************************


  // Запросить - имеет ли объект графическое описание
  // info    - идентификатор объекта карты в памяти

_MAPIMP long int _MAPAPI mapIsDrawObject(HOBJ info);

  // Запросить количество элементов графического описания
  // info    - идентификатор объекта карты в памяти
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDrawCount(HOBJ info);

  // Запросить вид элемента графического описания
  // по его номеру (number -  от 1 до DrawCount())
  // info    - идентификатор объекта карты в памяти
  // Возвращает номер функции типа IMG_XXXXXXX (см. MAPGDI.H)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDrawImage(HOBJ info, int number);

  // Запросить адрес параметров элемента графического описания
  // по его номеру (number -  от 1 до DrawCount())
  // info    - идентификатор объекта карты в памяти
  // Возвращает адрес структуры типа IMGXXXXXX, в соответствии
  // с видом элемента (см. MAPGDI.H)
  // Для запроса с 0 номером возвращает адрес параметров
  // графического описания объекта структуры IMGDRAW
  // При ошибке возвращает 0

_MAPIMP const char * _MAPAPI mapDrawParameters(HOBJ info, int number);

  // Запросить длину параметров элемента графического описания
  // по его номеру ( от 1 до DrawCount())
  // info    - идентификатор объекта карты в памяти
  // Для запроса с 0 номером возвращает длину параметров
  // графического описания объекта
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDrawLength(HOBJ info, int number);

  // Добавить элемент графического описания объектов
  // info  - идентификатор объекта карты в памяти
  // image - номер функции типа IMG_XXXXXXX (см. MAPGDI.H)
  // parm  - адрес структуры типа IMGXXXXXX
  // При ошибке возвращает ноль,иначе - число элементов в записи

_MAPIMP long int _MAPAPI mapAppendDraw(HOBJ info, int image, const char *parm);

  // Удалить все элементы графического описания объекта
  // info  - идентификатор объекта карты в памяти

_MAPIMP long int _MAPAPI mapClearDraw(HOBJ info);

  // Удалить элемент графического описания объекта
  // info   - идентификатор объекта карты в памяти
  // number - номер элемента (начиная с 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteDraw(HOBJ info, int number);


 // ************************************************************
 //                                                            *
 //         СОХРАНИТЬ/ВОССТАНОВИТЬ ДАННЫЕ ОБ'ЕКТА              *
 //                                                            *
 // ************************************************************

 // Запросить - изменились ли какие-либо данные объекта в памяти:
 // метрика, семантика, графика
 // При изменении данных, которые не были сохранены функцией типа
 // mapCommitObject, возвращает ненулевое значение
 // Изменение кода объекта, границ видимости и других свойств
 // эта функция не проверяет
 // При наличии изменений возвращает флаг изменения (2 - метрика,
 // 4 - семантика, 8 - графика).
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsDirtyObject(HOBJ info);

 // Сохранить данные об объекте в файле
 // info  - идентификатор объекта карты в памяти
 // Номер листа в районе должен быть установлен
 // Предыдущее состояние объекта сохраняется в резервных
 // файлах и может быть восстановлено
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommit(HOBJ info);
_MAPIMP long int _MAPAPI mapCommitObject(HOBJ info);

 // Сохранить данные об объекте в файле
 // info  - идентификатор объекта карты в памяти
 // Объекты будут отображаться в порядке записи в файл
 // Применяется для специальной сортировки объектов
 // (например, линейный может быть под площадным)
 // Предыдущее состояние объекта сохраняется в резервных
 // файлах и может быть восстановлено
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitObjectByOrder(HOBJ info);

 // Сохранить данные об объекте в файле
 // info  - идентификатор объекта карты в памяти
 // Если объект новый - выполняется функция mapCommitObject(),
 // если такой объект уже был, то сохраняется копия
 // объекта с новым уникальным номером (предполагается,
 // что предварительно изменены координаты и т.п.)
 // Номер листа в районе должен быть установлен
 // Позволяет ускорить создание серии однотипных объектов
 // (другой способ - mapCopyObjectAsNew()).
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitObjectAsNew(HOBJ info);

 // Сохранить данные об объекте в файле с выбором листа
 // или делением объекта по листам (при необходимости)
 // info  - идентификатор объекта карты в памяти
 // Для объектов пользовательских карт (обстановки)
 // достаточно mapCommitObject() - там один лист и нет границ.
 // Предыдущее состояние объекта сохраняется в резервных
 // файлах и может быть восстановлено
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitWithPlace(HOBJ info);

 // Сохранить данные об объекте в файле с выбором листа
 // или делением объекта по листам (при необходимости)
 // Объект будет сохранен, как новый, с присвоением
 // нового уникального номера
 // info  - идентификатор объекта карты в памяти
 // Для объектов пользовательских карт (обстановки)
 // достаточно mapCommitObject() - там один лист и нет границ.
 // Предыдущее состояние объекта сохраняется в резервных
 // файлах и может быть восстановлено
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitWithPlaceAsNew(HOBJ info);

 // Сохранить данные об объекте в файле с
 // обрезанием объекта по границам заданного листа
 // info  - идентификатор объекта карты в памяти
 // list  - номер листа ( > 0 ), по рамке которого скорректировать
 // метрические данные объекта
 // Для объектов пользовательских карт (обстановки)
 // достаточно mapCommitObject() - там один лист и нет границ.
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCommitWithPlaceForList(HOBJ info, long int list);

 // Запрость/Установить параметры обработки метрики линейных и площадных объектов,
 // используемые при сохранении объектов с делением на листы в функциях:
 // mapCommitWithPlace, mapCommitWithPlaceAsNew, mapCommitWithPlaceForList
 // parm  - параметры, используемые для автоматического удаления малых отрезков,
 //         малых линейных и вырожденных площадных объектов, которые получаются
 //         при разрезания объектов по рамкам листов карты
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetCommitObjectParm(HMAP hmap, COMMITOBJECTPARM* parm);
_MAPIMP long int _MAPAPI mapSetCommitObjectParm(HMAP hmap, COMMITOBJECTPARM* parm);

 // Удалить объект карты
 // Предыдущее состояние объекта сохраняется в резервных
 // файлах и может быть восстановлено
 // info  - идентификатор объекта карты в памяти
 // Признак удаления записывается в памяти и в файле
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteObject(HOBJ info);

 // Удалить объект карты по его номеру (number)
 // hMap   - идентификатор открытых данных
 // list   - последовательный номер листа (с 1)
 // number - последовательный ноиер объекта в листе
 // Связь с номером - см. mapReadObjectByNumber,mapGetObjectNumber
 // После удаления объекта по номеру вызов Сommit не должен выполняться
 // для этого объекта
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteObjectByNumber(HMAP hMap, long int list,
                                                 long int number);

 // Отменить удаление объекта карты
 // info  - идентификатор объекта карты в памяти
 // Признак удаления убирается в памяти и в файле
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUndeleteObject(HOBJ info);

 // Переместить объект в цепочке в конец (рисуется над всеми)      // 19/02/14
 // Объекту присваивается признак "выше всех"
 // info  - идентификатор объекта карты в памяти
 // Возвращает новый последовательный номер объекта на карте - mapGetObjectNumber()
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateObjectUp(HOBJ info);

 // Переместить объект в цепочке в начало (рисуется под всеми)     // 19/02/14
 // Объекту присваивается признак "ниже всех"
 // info  - идентификатор объекта карты в памяти
 // Возвращает новый последовательный номер объекта на карте - mapGetObjectNumber()
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateObjectDown(HOBJ info);

 // Сбросить признаки "выше всех" и "ниже всех" в объекте          // 19/02/14
 // Положение объекта в соответствии с его слоем и локализацией
 // изменится после сортировки карты
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateObjectNormal(HOBJ info);

 // Запросить флаг размещения объекта (2 - над всеми, 3 - под всеми, 1 - не задано)
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectUpDownState(HOBJ info);         // 28/04/16

 // Отменить удаление объекта карты по его номеру (number)
 // hmap   - идентификатор открытых данных
 // hSite  - идентификатор пользовательской карты
 //          (для фоовой карты равен hMap или 0)
 // list   - последовательный номер листа (с 1)
 // number - последовательный ноиер объекта в листе
 // Связь с номером - см. mapReadObjectByNumber,mapGetObjectNumber
 // Признак удаления убирается в файле
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUndeleteObjectByNumber(HMAP hMap, HSITE hSite,
                                                   long int list, long int number);

 // Восстановить (в памяти) данные об объекте из файла
 // info  - идентификатор объекта карты в памяти
 // Номер листа в районе и номер объекта должны быть установлены
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRevert(HOBJ info);
_MAPIMP long int _MAPAPI mapRevertObject(HOBJ info);

  // Восстановить копию объекта, по состоянию до выполнения транзакции
  // info -  идентификатор существующего объекта,
  // number - номер транзакции
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRestoreBackObjectByAction(HOBJ info, long int number);

  // Восстановить копию объекта, по состоянию до выполнения транзакции
  // info -  идентификатор существующего объекта,
  // number - номер транзакции
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRestoreBackObjectByTime(HOBJ info, long int date, long int time);

  // Установить адрес функции, которая будет вызываться
  // при редактировании объекта карты.
  // Для отмены оповещения о редактировании необходимо вызвать
  // данную функцию и передать в параметрах call и parm нулевые значения
  // Вызываемая функция не должна сама редактировать карту !
  // hmap   - идентификатор открытых данных
  // hSite  - идентификатор пользовательской карты
  //          (для фоновой карты равен hMap или 0)
  // call - адрес вызываемой функции (см. maptype.h),
  // parm - параметр, который будет передан вызываемой функции первым,
  // вторым параметром будет адрес структуры CHANGEINFO (см. maptype.h).
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetChangeCallAndParm(HMAP hmap, HSITE hSite,
                                                 CHANGECALL call, void *parm);

 // *************************************************************
 //                                                             *
 //     ОБРАБОТКА РЕЗЕРВНЫХ ФАЙЛОВ ...\LOG\... ^DA,^SE,^HD      *
 //      (СОДЕРЖАТ КОПИИ ВСЕХ ОБНОВЛЯЕМЫХ ОБ ЕКТОВ ЛИСТА)       *
 //                                                             *
 // *************************************************************

  // Последовательный перебор отредактированных копий объекта info
  // info  - идентификатор объекта карты в памяти
  // copynumber - последовательный номер копии данного объекта (1, 2, ...)
  // в порядке от последней операции редактирования к предыдущей
  // Вызывается с последовательным увеличением copynumber,
  // пока не будет найдена нужная копия или копии закончатся
  // Чтобы сделать найденную копию текущим состоянием объекта на карте,
  // нужно удалить лишние копии объекта функцией
  // mapDeleteObjectCopyToNumber, а затем вызвать функцию mapCommit.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReadObjectCopyByNumber(HOBJ info,
                                                   long int copynumber);

  // Удалить отредактированные копии объекта info
  // info  - идентификатор объекта карты в памяти
  // Удаляет копии объекта от первой до copynumber из файлов отката
  // Копия copynumber+1 становится первой
  // Если вызывается после сохранения соответсвующей копии объекта
  // на карте (mapCommit - отмена выполненных операций редактирования),
  // то число удаляемых копий должно быть на 1 больше, чем номер
  // восстанавливаемой копии. (т.к. mapCommit создает еще одну копию
  // объекта и она становиться первой).
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteObjectCopyToNumber(HOBJ info,
                                                     long int copynumber);


 // ************************************************************
 //                                                            *
 //         РАСЧЕТ ХАРАКТЕРИСТИК ОБ'ЕКТА                       *
 //                                                            *
 // ************************************************************

 // Вычисление длины участка объекта (стороны) на местности, 
 // начиная с указанной точки
 // Для последней точки вычисляет расстояние до первой точки
 // info  - идентификатор объекта карты в памяти
 // number - номер точки, начиная с 1
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает 0 (при совпадении точек также)

_MAPIMP double _MAPAPI mapSideLength(HOBJ info,long int number,
                                     long int subject = 0);

_MAPIMP long int _MAPAPI mapSideLengthEx(HOBJ info, long int number,
                                         long int subject, double *length);

 // Вычисление длины участка объекта (стороны) в проекции документа
 // или на местности, в зависимости от текущего
 // условия выполнения вычислений по карте -
 // mapSetCalculationConventional
 // info  - идентификатор объекта карты в памяти
 // number - номер точки, начиная с 1
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapConventionalSideLength(HOBJ info, long int number,
                                                 long int subject = 0);
                                         
 // Вычисление азимута участка объекта (стороны)
 // Возвращает величину угла в радианах
 // Если геодезия не поддерживается, то вычисляется дирекционный угол
 // Для последней точки вычисляет направление на первую точку
 // У замкнутых объектов первая и последняя точки совпадают
 // info  - идентификатор объекта карты в памяти
 // number - номер точки, начиная с 1
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает 0 (при совпадении точек также)

_MAPIMP double _MAPAPI mapSideAzimuth(HOBJ info,long int number,   
                                      long int subject);
_MAPIMP long int _MAPAPI mapSideAzimuthEx(HOBJ info,long int number,
                                          long int subject, double *azimuth);

 // Вычисление дирекционного угла участка объекта (стороны)
 // Возвращает величину угла в радианах
 // Для последней точки вычисляет направление на первую точку
 // У замкнутых объектов первая и последняя точки совпадают
 // info  - идентификатор объекта карты в памяти
 // number - номер точки, начиная с 1
 // subject - номер подобъекта (если = 0, обрабатывается объект)
 // При ошибке возвращает 0 (при совпадении точек также)

_MAPIMP double _MAPAPI mapSideDirection(HOBJ info,long int number,
                                        long int subject/* = 0*/);

 // Вычисление уточненной площади объекта на местности
 // Для вычисления площади объекта его координаты пересчитываюся
 // в проекцию топографической карты ближайшей зоны!
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSquare(HOBJ info);
_MAPIMP long int _MAPAPI mapSquareEx(HOBJ info, double *square);

 // Вычисление площади объекта в проекции карты
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSquareInMap(HOBJ info);

 // Вычисление площади объекта в проекции карты
 // или на местности, в зависимости от текущего
 // условия выполнения вычислений по карте -
 // mapSetCalculationConventional
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapConventionalSquare(HOBJ info);

 // Вычисление площади отдельного подобъекта в проекции документа
 // или на местности, в зависимости от текущего
 // условия выполнения вычислений по карте -
 // mapSetCalculationConventional
 // info    - идентификатор объекта карты в памяти
 // subject - номер подобъекта от 0 до mapPolyCount,
 //           если равно нулю, то вычисляется площадь внешнего контура
 //           без вычитания площади подобъектов
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapConventionalSubjectSquare(HOBJ info, long int subject);

 // Вычисление площади подобъекта в проекции документа
 // info  - идентификатор объекта карты в памяти
 // subject - номер подобъекта от 0 до mapPolyCount,
 //           если равно нулю, то вычисляется площадь внешнего контура
 //           без вычитания площади подобъектов
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSubjectSquareInMap(HOBJ info, long int subject);

 // Установить условие для расчета расстояний и площадей
 // по карте
 // hmap   - идентификатор открытых данных
 // flag   - условие выполнения расчетов: 
 //          0 - вычислять уточненное значение площадей и 
 //          расстояний по ближайшему осевому меридиану;
 //          1 - вычислять площади и расстояния в проекции
 //          карты, которой принадлежит объект
 // Возвращает предыдущее значение условия 

_MAPIMP long int _MAPAPI mapSetCalculationConventional(HMAP hmap, long int flag);

 // Запросить условие для расчета расстояний и площадей
 // по карте
 // hmap   - идентификатор открытых данных
 // Возвращает условие выполнения расчетов: 
 //          0 - вычислять уточненное значение площадей и 
 //          расстояний по ближайшему осевому меридиану;
 //          1 - вычислять площади и расстояния в проекции
 //          карты, которой принадлежит объект

_MAPIMP long int _MAPAPI mapGetCalculationConventional(HMAP hmap);

                                              
 // Вычисление площади объекта c учетом рельефа
 // При отсутствии рельефа(матрицы высот,слоев,Tin-матрицы) возвращает площадь объекта
 // hmap   - идентификатор открытых данных
 // info  - идентификатор объекта карты в памяти
 // step - шаг инетрполяции в метрах, если 0 то вычисляется автоматически
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSquareWithHeight(HMAP hmap, HOBJ info);
_MAPIMP double _MAPAPI mapSquareWithHeightEx(HMAP hmap, HOBJ info, double step = 0.); //05/06/15

 // Вычисление периметра объекта
 // info  - идентификатор объекта карты в памяти
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapPerimeter(HOBJ info);

 // Вычисление длины объекта
 // Для вычисления длины объекта его координаты пересчитываюся
 // в проекцию топографической карты по каждому отрезку отдельно
 // с установкой осевого меридиана в центре отрезка
 // info  - идентификатор объекта карты в памяти
 // Для подобъектов считается суммарная длина
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapLength(HOBJ info);
_MAPIMP long int _MAPAPI mapLengthEx(HOBJ info, double *length);

 // Вычисление длины объекта по карте
 // Координаты объекта не пересчитываются, полученная длина
 // может значительно отличаться от реальной длины объекта на местности
 // info  - идентификатор объекта карты в памяти
 // Для подобъектов считается суммарная длина
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapLengthInMap(HOBJ info);

 // Вычисление длины объекта от начала до заданной точки
 // info  - идентификатор объекта карты в памяти
 // point - координаты точки, расположенной вдоль(вблизи) объекта
 // Если точка не на объекте - ищется ближайшая точка на контуре
 // Координаты точки обновляются!
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapLengthToPoint(HOBJ info, DOUBLEPOINT *point);

 // Вычисление длины подобъекта/объекта
 // info    - идентификатор объекта карты в памяти
 // subject - номер подобъекта (если = 0, обрабатывается объект;
 //           если равен -1, суммируются длины объекта и подобъектов)
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSubjectLength(HOBJ info, long int subject);

 // Вычисление длины объекта\подобъекта в проекции документа
 // info    - идентификатор объекта карты в памяти
 // subject - номер подобъекта (если = 0, обрабатывается объект;
 //           если равен -1, суммируются длины объекта и подобъектов)
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapSubjectLengthInMap(HOBJ info, long int subject);

 // Вычисление длины объекта\подобъекта в проекции документа
 // или на местности, в зависимости от текущего
 // условия выполнения вычислений по карте -
 // mapSetCalculationConventional
 // info    - идентификатор объекта карты в памяти
 // subject - номер подобъекта (если = 0, обрабатывается объект;
 //           если равен -1, суммируются длины объекта и подобъектов)
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapConventionalSubjectLength(HOBJ info, long int subject = -1);

 // Вычисление длины объекта c учетом рельефа
 // При отсутствии рельефа(матрицы высот,слоев,Tin-матрицы) возвращает длину объекта
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapLengthWithHeight(HMAP hmap, HOBJ info);

 // Определение замкнутости контура подобъекта
 // info  - идентификатор объекта карты в памяти
 // subject - номер текущего подобъекта (0 - объекта)
 // Возвращает: 1 - объект замкнут, иначе 0

_MAPIMP long int _MAPAPI mapCircuitousSubject(HOBJ info, long int subject/* = 0*/);

 // Определение кратчайшего расстояния от точки до объекта
 // hmap   - идентификатор открытых данных
 // info  - идентификатор объекта карты в памяти
 // subject - номер текущего подобъекта (0-объекта)
 // координаты точки point заданы в прямоугольной
 // системе координат , в метрах на местности
 // Возвращает вычисленное расстояние в метрах
 // или 0 в случае ошибки

_MAPIMP double _MAPAPI mapDistancePointSubject(HMAP hmap, HOBJ info,
                                               long int subject,
                                               DOUBLEPOINT * point);

 // Определение кратчайшего расстояния от точки до объекта (включая
 // подобъекты)
 // hmap   - идентификатор открытых данных
 // info  - идентификатор объекта карты в памяти
 // координаты точки point заданы в прямоугольной
 // системе координат , в метрах на местности
 // Возвращает вычисленное расстояние  в метрах
 // или 0 в случае ошибки

_MAPIMP double _MAPAPI mapDistancePointObject(HMAP hmap, HOBJ info,
                                              DOUBLEPOINT * point);

 // Определение кратчайшего расстояния между объектами
 // info1  - идентификатор 1-го объекта карты в памяти
 // info2  - идентификатор 2-го объекта карты в памяти
 // Возвращает вычисленное расстояние  в метрах
 // или 0 в случае ошибки

_MAPIMP double _MAPAPI mapDistanceObject(HOBJ info1, HOBJ info2);

 // Определение кратчайшего расстояния между объектами и координат
 // точек на контурах объектов
 // hobj1  - идентификатор первого объекта карты в памяти
 // hobj2  - идентификатор второго объекта карты в памяти
 // point1 - координаты первой точки линии кратчайшего расстояния
 //          между объектами (на объекте hobj1)
 // point2 - координаты второй точки линии кратчайшего расстояния
 //          между объектами (на объекте hobj2)
 // Возвращает вычисленное расстояние  в метрах
 // или большое значение (>= 100000000) в случае ошибки

_MAPIMP double _MAPAPI mapDistanceObjectEx(HOBJ hobj1, HOBJ hobj2,
                                           DOUBLEPOINT *point1,
                                           DOUBLEPOINT *point2);

 // Прямая геодезическая задача на эллипсоиде
 // Для расстояния не более 250 км координаты определяются с ошибкой до 0,0001",
 // а обратный азимут - до 0,001", что соответствует триангуляции 1 класса
 // Способ вспомогательной точки по методу Красовского
 // Метод предназначен для расстояний меньше радиуса Земли
 // Вычисления выполняются на текущем эллипсоиде, установленном
 // в документе - mapSetDocProjection
 // Если hmap равен 0, то вычисления выполняются на эллипсоиде WGS-84
 // hmap     - идентификатор открытых данных
 // b1,l1    - геодезические координаты исходной точки
 // angle1   - азимут на вторую точку
 // distance - расстояние до второй точки
 // b2,l2    - рассчитанные координаты второй точки
 // angle2   - рассчитанный азимут со второй точки на первую
 //            (если angle2 равен 0, то обратный азимут не вычисляется)
 // При ошибке в параметрах возвращает 0

_MAPIMP long int _MAPAPI mapDirectPositionComputation(HMAP hmap,
                                                      double b1, double l1, double angle1,
                                                      double distance,
                                                      double *b2, double *l2, double *angle2);

 // Обратная геодезическая задача на эллипсоиде
 // Для расстояния не более 180 градусов по широте
 // Выполняется построение ортодромии функцией mapOrthodromeObject
 // и запрос длины объекта и азимута первого отрезка
 // Точность порядка точности триангуляции 1 класса
 // Вычисления выполняются на текущем эллипсоиде, установленном
 // в документе - mapSetDocProjection
 // Если hmap равен 0, то вычисления выполняются на эллипсоиде WGS-84
 // hmap     - идентификатор открытых данных
 // b1,l1    - геодезические координаты первой точки
 // b2,l2    - геодезические координаты второй точки
 // angle    - рассчитанный азимут с первой точки на вторую
 // Возвращает расстояние между заданными точками на текущем эллипсоиде
 // При ошибке в параметрах возвращает 0

_MAPIMP double _MAPAPI mapInversePositionComputation(HMAP hmap, double b1, double l1,
                                                     double b2, double l2, double *angle);

 /*********************************************************
 *                                                        *
 *                МАТЕМАТИЧЕСКИЕ ФУНКЦИИ                  *
 *                                                        *
 *********************************************************/

 // Вычисление расстояния между двумя точками на плоскости
 // point1, point2 - координаты точек в метрах
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapDistance(DOUBLEPOINT *point1, DOUBLEPOINT *point2);

 // Вычисление расстояния между двумя точками на местности
 // hmap   - идентификатор открытых данных
 // point1, point2 - координаты точек в метрах на местности
 // Для вычисления расстояния координаты пересчитываются
 // в проекцию топографической карты с установкой осевого
 // меридиана в центре отрезка
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapRealDistance(HMAP hmap, DOUBLEPOINT *point1,
                                       DOUBLEPOINT *point2);

 // Вычисление расстояния между двумя точками в проекции карты
 // или на местности, в зависимости от текущего
 // условия выполнения вычислений по карте -
 // mapSetCalculationConventional
 // hmap   - идентификатор открытых данных
 // point1, point2 - координаты точек в метрах на местности
 // При ошибке возвращает 0

_MAPIMP double _MAPAPI mapConventionalDistance(HMAP hmap, DOUBLEPOINT *point1,
                                               DOUBLEPOINT *point2);

 // Определение угла наклона биссектрисы угла, заданного
 // точками p1,p2,p3 с вершиной в точке p2
 // Определение угла наклона биссектрисы угла, заданного
 // точками p1,p2,p3 с вершиной в точке p2
 // Возвращаемый угол задан относительно оси X, его положительное
 // направление соответствует положительному направлению оси Y

_MAPIMP double _MAPAPI mapBisectorAngle(DOUBLEPOINT *p1, DOUBLEPOINT *p2,
                                        DOUBLEPOINT *p3);


  /*********************************************************
  *                                                        *
  *          ВЕДЕНИЕ СПИСКА ВРЕЗОК КАРТ                    *
  *                                                        *
  *  ВРЕЗКА - ИЗОБРАЖЕНИЕ ФРАГМЕНТА ОДНОЙ КАРТЫ ВНУТРИ     *
  *           ДРУГОЙ ДЛЯ НАГЛЯДНОСТИ. ВОЗМОЖНО ВРЕЗАТЬ     *
  *           ТУ ЖЕ КАРТУ В ДРУГОМ МАСШТАБЕ И СОСТАВЕ,     *
  *           А ТАКЖЕ ВРЕЗАТЬ ПРОЕКТ ДАННЫХ (MPT)          *
  *                                                        *
  *********************************************************/

  // Добавить врезку на карту
  // hMap - идентификатор открытых данных
  // name - имя карты (проекта, алиаса) для отображения врезки
  // Возвращает текущий номер врезки в списке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendInset(HMAP hmap, const WCHAR *name);

  // Добавить врезку на карту
  // hMap - идентификатор открытых данных
  // item - указатель на описание добавляемой врезки (карты, проекта, алиаса)
  // Возвращает текущий номер врезки в списке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapAppendInsetItem(HMAP hmap, INSETDESC *item);

  // Запросить число врезок на карте
  // hMap - идентификатор открытых данных
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapInsetCount(HMAP hmap);

  // Запросить описание врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте с 1
  // item   - указатель на описание врезки,
  //          которое будет заполнено
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetInset(HMAP hmap, long int number, INSETDESC *item);

  // Запросить описание врезки по идентификатору
  // hMap   - идентификатор открытых данных
  // ident  - идентификатор врезки на карте
  // item   - указатель на описание врезки,
  //          которое будет заполнено
  // При ошибке возвращает ноль, иначе - номер врезки в списке

_MAPIMP long int _MAPAPI mapGetInsetByIdent(HMAP hmap, long int ident, INSETDESC *desc);

  // Запросить название врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте
  // name   - буфер для записи имени
  // size   - длина буфера в байтах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetInsetName(HMAP hmap, long int number, WCHAR *name, long int size);

  // Запросить идентификатор врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetInsetIdent(HMAP hmap, long int number);

  // Запросить идентификатор открытых данных врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте
  // Запрошенный идентификатор не должен закрываться (через вызов mapCloseData)
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapGetInsetMapHandle(HMAP hmap, long int number);

  // Удалить описание врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteInset(HMAP hmap, long int number);

  // Обновить описание врезки по номеру с 1
  // hMap   - идентификатор открытых данных
  // number - порядковый номер врезки на карте
  // item   - указатель на новое описание врезки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateInset(HMAP hmap, long int number, INSETDESC * desc);

  // Установить врезке признак схематичного отображения
  // hMap   - идентификатор открытых данных
  // number - номер врезки, для которой устанавливается флаг,
  //          если равен -1, то устанавливается для всех врезок
  // flag   - признак схематичного отображения (0 или 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetInsetSchemeFlag(HMAP hmap, long int number, long int flag);

  // Запросить признак отображения врезки
  // hMap   - идентификатор открытых данных
  // number - номер врезки с 1, для которой запрашивается флаг
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetInsetViewFlag(HMAP hmap, long int number);

  // Установить врезке признак отображения
  // hMap   - идентификатор открытых данных
  // number - номер врезки, для которой устанавливается флаг,
  //          если равен -1, то устанавливается для всех врезок
  // flag   - признак отображения (0 или 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetInsetViewFlag(HMAP hmap, long int number, long int flag);

  // Запросить признак отображения фона врезки
  // hMap   - идентификатор открытых данных
  // number - номер врезки с 1, для которой запрашивается флаг
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetInsetTransparenceFlag(HMAP hmap, long int number);

  // Установить врезке признак отображения фона
  // hMap   - идентификатор открытых данных
  // number - номер врезки, для которой устанавливается флаг,
  //          если равен -1, то устанавливается для всех врезок
  // flag   - признак отображения (0 или 1)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetInsetTransparenceFlag(HMAP hmap, long int number, long int flag);

  // Отобразить врезку в документе
  // hMap     - идентификатор открытых данных
  // number   - номер отображаемой врезки
  //            если равен -1, то отображаются все врезки
  // hdc      - идентификатор контекста устройства вывода,
  // rect     - координаты отображаемого фрагмента карты (Draw) в изображении (Picture)
  // position - положение верхнего левого угла отображаемой области в окне отображения
  //            (если смещение предварительно задано через SetViewportOrgEx, то параметр равен 0)
  // printscale - соответствие размера пиксела печатающего устройства и пиксела экрана
  //            (для печати, для вывода на экран параметр равен 0)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPaintInset(HMAP hMap, long int number, HDC hdc, RECT *rect,
                                       POINT *position, double *printscale);

 /*********************************************************
 *                                                        *
 *                 СЕРВИСНЫЕ ФУНКЦИИ                      *
 *                                                        *
 *********************************************************/

 // Освободить ресурсы ядра перед закрытием приложения

_MAPIMP  long int _MAPAPI mapCloseMapAccess();

  // Определить тип файла по его имени
  // name - имя тестируемого файла (полный путь)
  // Анализируются первые 4 байта, содержащие идентификатор данных.
  // При ошибке возвращает ноль, иначе - идентификатор файла
  // (см. maptype.h : FILE_SXF, FILE_MAP, FILE_MTW,...)

_MAPIMP long int _MAPAPI mapCheckFile(const char *name);

  // Определить тип файла по его имени
  // name - имя тестируемого файла (полный путь)
  // Анализируются первые 4 байта, содержащие идентификатор данных.
  // При ошибке возвращает ноль, иначе - идентификатор файла
  // (см. maptype.h : FILE_SXF, FILE_MAP, FILE_MTW,...)
  // Дополнительно различает MAP (FILE_MAP) и SIT (FILE_MAPSIT)
  // Имя может быть в виде ALIAS#XXXX для карт на ГИС Сервере

_MAPIMP long int _MAPAPI mapCheckFileEx(const char *name);

_MAPIMP long int _MAPAPI mapCheckFileExUn(const WCHAR *name);

  // Запросить текущее время в формате "HH:MM:SS"
  // buffer - адрес памяти для размещения результата запроса
  // size   - размер выделенной памяти в байтах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetTheTime(char *buffer, long int size);

_MAPIMP long int _MAPAPI mapGetTheTimeUn(WCHAR *buff, long int size);

  // Загрузить библиотеку DLL
  // На каждый вызов mapLoadLibrary должен выполняться вызов mapFreeLibrary
  // При поиске DLL проверяется и директория приложения
  //  void (WINAPI * myfunction)(int param);
  // (FARPROC)myfunction = mapLoadLibrary("ABC.DLL",&instance,"MyFunction");
  //  if (myfunction)
  //      { (*myfunction)(123); ::FreeLibrary(*instance); }
  // При ошибке возвращает ноль и выдает сообщение на экран

_MAPIMP FARPROC _MAPAPI mapLoadLibrary(const char *dllname,
                                       HINSTANCE  *libinst,
                                       const char *funcname);
_MAPIMP FARPROC _MAPAPI mapLoadLibraryUn(const WCHAR *dllname,
                                         HINSTANCE   *libinst,
                                         const WCHAR *funcname);

  // Выгрузить библиотеку DLL

_MAPIMP long int _MAPAPI mapFreeLibrary(HINSTANCE libinst);

  // Загрузить функцию библиотеки DLL

_MAPIMP FARPROC _MAPAPI mapGetProcAddress(HINSTANCE libinst,
                                          const char *funcname);

  // Вызов окна справки системы из файла *.CHM                                     // 13/11/14
  // (Вызывает HelpExecLang или HelpExecLangUn)
  // helpName - имя файла справки (полное или короткое)
  // pageName - имя страницы справки (без расширения), может быть равно 0        
  // Пример вызова функции: HelpExec(TEXT("map3d.chm"), TEXT("Setup"));

_MAPIMP void _MAPAPI HelpExec(const TCHAR* helpName, const TCHAR* pageName);

// Вызов окна справки системы с учётом установленного
// функцией mapSetMapAccessLanguage языка сообщений
// helpName - имя файла справки (полное или короткое)
// pageName - имя страницы справки, может быть равно 0
// Пример вызова функции: HelpExecLang("map3d.chm", "Setup");

_MAPIMP void _MAPAPI HelpExecLangUn(const WCHAR* helpName, const WCHAR* pageName);    // 08/08/14
_MAPIMP void _MAPAPI HelpExecLang(const char* helpName, const char* pageName);        // 18/08/14

// Формирование имени файла справки системы с учётом установленного
// функцией mapSetMapAccessLanguage языка сообщений
// helpName - исходное имя файла справки (без учёта языка сообщений)
// helpNameLang - адрес строки для формируемого имени файла справки
//                (с учётом языка сообщений)
// size - размер строки helpNameLang в байтах

_MAPIMP int _MAPAPI HelpNameLang(const char* helpName, char* helpNameLang, int size);  // 09/07/14
_MAPIMP int _MAPAPI HelpNameLangUn(const WCHAR* helpName, WCHAR* helpNameLang, int size);  // 09/07/14

  // Сортировка отдельной карты документа
  //  mapname - сортируемая карта
  //  flags   - Флажки обработки карты :
  //   0 - сортировать все листы,
  //   1 - только несортированные,
  //   2 - сохранять файлы отката,
  //   4 - повысить точность хранения,
  //   8 - нормальная точность хранения (FLOAT).
  //  16 - повысить точность хранения, формат - см
  //  32 - повысить точность хранения, формат - мм
  //  64 - повысить точность хранения, формат - радианы
  // hEvent - адрес функции обратного вызова для уведомлении о процессе
  // eventparam - параметры функции обратного вызова
  // outpath - адрес строки для записи нового пути к отсортированной карте,
  //           если адрес строки не задан, то карта обновляется на месте
  // size    - размер строки для записи пути
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI MapSortingWithEvent(const char *mapname,
                                             long int flags,
                                             EVENTCALL hEvent,
                                             void *eventparam,
                                             char *outpath,
                                             long int size);

  // Сортировка отдельной карты документа
  // mapname - сортируемая карта
  // flags   - Флажки обработки карты :
  //   0 - сортировать все листы,
  //   1 - только несортированные,
  //   2 - сохранять файлы отката,
  //   4 - повысить точность хранения,
  //   8 - нормальная точность хранения (FLOAT).
  //  16 - повысить точность хранения, формат - см
  //  32 - повысить точность хранения, формат - мм
  //  64 - повысить точность хранения, формат - радианы
  // handle     - идентификатор окна, которому посылаются сообщения WM_LIST и WM_OBJECT,
  //              если не задан параметр hEvent
  // hEvent     - адрес функции обратного вызова для уведомления о проценте выполнения,
  //              если параметр не задан, то посылаются сообщения WM_LIST и WM_OBJECT
  // eventparam - параметр, передаваемый функции обратного вызова
  // outpath    - буфер для записи пути к папке, куда сохранили отсортированную карту,
  //              если задан этот параметр, то исходная карта не обновляется,
  //              папка создается автоматически в программе сортировки
  // size       - длина буфера в байтах
  // format - управление форматом карты :
  //  0 -  не менять,
  //  1 -  установить формат SITX (на входе может быть SIT или MAP с одним листом),
  // -1 -  установить формат SIT (на входе может быть SITX или MAP с одним листом),
  // code - управление шифрованием карты :
  //  0 - не менять,
  //  1 - шифровать данные с помощью пароля из параметра password (формат SITX),
  // -1 - снять шифрование данных
  // password - пароль для шифрования данных, когда code = 1, или 0
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI MapSortingWithEventPro(const WCHAR *mapname, long int flags,
                                                HWND handle, EVENTCALL hEvent, void *eventparam,
                                                WCHAR *outpath, long int size,
                                                long int format, long int code,
                                                const WCHAR *password);

_MAPIMP long int _MAPAPI MapSortingSitePro(HMAP hmap, HSITE hsite, long int flags,
                                           HWND handle, long int format, long int code,
                                           const WCHAR *password);

_MAPIMP long int _MAPAPI MapSortingWithEventUn(const WCHAR *mapname, long int flags,
                                               HWND handle, EVENTCALL hEvent, void *eventparam,
                                               WCHAR *outpath, long int size);

  // Копирование cодержимого открытых файлов
  // in     - имя входного файла
  // out    - имя выходного файла
  // mode   - флажки доступа к выходному файлу (GENERIC_READ, GENERIC_WRITE),
  //          ноль соответствует GENERIC_READ,
  // access - флажки открытия выходного файла (FILE_SHARE_READ,FILE_SHARE_WRITE)
  //          ноль соответствует FILE_SHARE_READ|FILE_SHARE_WRITE
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCopyFile(const char *in, const char *out,
                                     long int access = 0, long int mode = 0);

_MAPIMP long int _MAPAPI mapCopyFileUn(const WCHAR *in, const WCHAR *out,
                                       long int access = 0, long int mode = 0);

  // Запросить длину файла
  // name - полный путь к файлу
  // При ошибке возвращает ноль

_MAPIMP double _MAPAPI mapGetFileLength(const char *name);

_MAPIMP double _MAPAPI mapGetFileLengthUn(const WCHAR *name);

  // Разрешить/Запретить выдачу сообщений на экран
  // (серверный режим работы)
  // enable = 0  - запрет выдачи сообщений,
  // Возвращает предыдущее значение флага

_MAPIMP long int _MAPAPI mapMessageEnable(long int enable);

_MAPIMP long int _MAPAPI mapIsMessageEnable();

  // Выдать сообщение об ошибке (на экран)
  // code - код ошибки (см. MAPERR.RH)

_MAPIMP void _MAPAPI ErrorMessageEx(HWND hwnd, int code, const char *filename);
_MAPIMP void _MAPAPI mapErrorMessage(long int code, const char *filename);
_MAPIMP void _MAPAPI mapErrorMessageUn(long int code, const WCHAR *filename);

  // Выдать сообщение об ошибке (на экран)
  // hwnd - идентификатор родительского окна для выдачи сообщения или 0
  // code - код ошибки (см. MAPERR.RH)
  // filename - имя файла (объекта), для которого возникла ошибка
  // message  - адрес строки для размещения текста сообщения (для записи
  //            в протокол и т.п.), область памяти должна быть
  //            не менее длины имени файла + 256 байт; значение может быть 0

_MAPIMP void _MAPAPI mapErrorMessageExUn(HWND hwnd, long int code, 
                                         const WCHAR *filename, WCHAR *message);

  // Установить функцию обратного вызова для перехвата и замены       // 17/04/16
  // сообщений об ошибках и вопросах, выдаваемых из ГИС-ядра
  // через системную функцию MessageBox
  // call - адрес функции обратного вызова (прототип в maptype.h)
  // parm - значение первого параметра в функции обратного вызова (может быть 0)
  // Остальные параметры в функции обратного вызова имеют то же назначение,
  // что и параметры в функции MessageBox для UNICODE

_MAPIMP void _MAPAPI mapSetMessageBoxCall(MESSAGEBOXCALL call, void * parm);

  // Выдать сообщение на экран через системную функцию MessageBox
  // Если mapIsMessageEnable() равно 0,  то сообщение не выдается
  // и функция возвращает ноль
  // Если установлена функция обратного вызова mapSetMessageBoxCall,
  // то выдача сообщения будет через эту функцию

_MAPIMP long int _MAPAPI mapMessageBox(HWND hwnd, const char *message,
                                       const char *title, long int flag);

_MAPIMP long int _MAPAPI mapMessageBoxUn(HWND hwnd, const WCHAR *message,
                                         const WCHAR *title, long int flag);

  // Выдать сообщение в протокол карты
  // HMAP и HSITE определяют карту, в протокол которой
  // записывается сообщение
  // (для карты местности HSITE = HMAP, см. sitapi.h)
  // message - текст сообщения
  // type - тип сообщения (см. maptype.h : MT_INFO,MT_ERROR,...)

_MAPIMP void _MAPAPI mapMessageToLog(HMAP hMap, HSITE hSite,
                                     const char *message,
                                     long int type);

_MAPIMP void _MAPAPI mapMessageToLogUn(HMAP hMap, HSITE hSite,
                                       const WCHAR *message,
                                       long int type);

  // Запросить версию модуля MapAccess.Dll
  // Если полученная версия не равна значению MAPACCESSVERSION
  // в работе программы может произойти сбой

_MAPIMP long int _MAPAPI GetMapAccessVersion();
_MAPIMP long int _MAPAPI mapGetMapAccessVersion();
_MAPIMP long int _MAPAPI mapGetMapApiVersion();

  // Установить/Запросить язык сообщений
  // 1 - английский, 2 - русский, ... (см. Maptype.h)
  // (по-умолчанию - английский)

_MAPIMP void _MAPAPI SetMapAccessLanguage(int code);
_MAPIMP int  _MAPAPI GetMapAccessLanguage();
_MAPIMP void _MAPAPI mapSetMapAccessLanguage(int code);
_MAPIMP int  _MAPAPI mapGetMapAccessLanguage();

  // Установить идентификатор окна для приема сообщений
  // от "затяжных" процессов (перекодировка при открытии
  // карты, нарезка объектов по заданной границе,...)
  // Код сообщения - 0x590,
  // wparm : -1 старт процесса,-2 завершение,0-100 процент выполнения
  // lparm : текст сообщения
  // Для отмены сообщений - установить идентификатор в ноль
  // (Идентификатор закрытого окна может привести к сбою в системе)
  // Для завершения процесса вернуть число 0x590
  // Возвращает предыдущее значение идентификатора

_MAPIMP HMESSAGE _MAPAPI mapSetHandleForMessage(HMESSAGE hwnd);
_MAPIMP HMESSAGE _MAPAPI mapGetHandleForMessage();

#ifdef WIN32API
  // Установить идентификатор окна для приема сообщений о событиях карты
  // hmap   - идентификатор открытых данных
  // hwnd   - идентификатор окна
  // event - флаг типов событий (1 - перерисовка карты)
  // Код сообщения WM_MAPEVENT (0x591)
  // wparm : идентификатор карты, в которой произошло событие (HMAP)
  // lparm : описание события
  // Для отмены сообщений - установить идентификатор в ноль
  // (Идентификатор закрытого окна может привести к сбою в системе)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetHandleForEvent(HMAP hmap, HWND hwnd, int event);

  // Запросить текущее значение идентификатора окна для приема сообщений

_MAPIMP HWND _MAPAPI mapGetHandleForEvent(HMAP hmap);

  // Уточнение положения диалога относительно габаритов рабочего стола
  // Входные параметры:
  //   left, top, width, height -  адреса значений положения и размеров
  //   проверямого окна диалога. Уточненные величины положения и размеров
  //   присваиваются по этим же адресам.
  //
  // При ошибке и отсутсвии изменений положения и размеров окна
  // возвращает ноль.
  // При изменении положения и размеров окна возвращает единицу.

_MAPIMP int _MAPAPI mapAdjustFormBounds(int* left, int* top, int* width,
                                        int* height);                           // 12/01/16

#endif

  // Сравнить две строки без учета регистра
  // Возвращает : (1-CMLESS, 2-CMEQUAL, 3-CMMORE - см. Maptype.h)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCompareString(const char *string,
                                          const char *filter);

_MAPIMP long int _MAPAPI mapCompareStringUn(const WCHAR *value,
                                            const WCHAR *temp);

  // Преобразовать дату из строки date в число ГГГГММДД
  // Строка может иметь вид ДД/ММ/ГГГГ или ДД.ММ.ГГГГ
  // или ГГГГММДД или ГГГГ/ММ/ДД
  // При ошибке возвращает ноль, иначе - значение даты

_MAPIMP long int _MAPAPI mapDateToLong(const char *date);

_MAPIMP long int _MAPAPI mapDateToLongUn(const WCHAR *date);

  // Преобразовать дату из из числа ГГГГММДД в строку
  // ДД/ММ/ГГГГ
  // number - числовое значение даты,
  // date   - адрес строки для результата
  // size   - размер буфера в байтах
  // При ошибке возвращает ноль, иначе - адрес входной строки

_MAPIMP char* _MAPAPI mapLongToDate(long int number, char *date, long int size);

_MAPIMP WCHAR* _MAPAPI mapLongToDateUn(long int number, WCHAR *date, long int size);

  // Преобразовать время из строки time в число ЧЧММСС
  // Строка может иметь вид ЧЧ:ММ:СС
  // При ошибке возвращает ноль, иначе - значение даты

_MAPIMP long int _MAPAPI mapTimeToLong(const char *time);

_MAPIMP long int _MAPAPI mapTimeToLongUn(const WCHAR *time);

  // Преобразовать время из из числа ЧЧММСС в строку
  // ЧЧ:ММ:СС
  // number - числовое значение времени,
  // time   - адрес строки для результата
  // size   - размер строки в байтах
  // При ошибке возвращает ноль, иначе - адрес входной строки

_MAPIMP char* _MAPAPI mapLongToTime(long int number, char *time, long int size);

_MAPIMP WCHAR* _MAPAPI mapLongToTimeUn(long int number, WCHAR *time, long int size);

  // Преобразовать угловую величину в градусах из строки
  // в числовое значение в радианах
  // Строка может иметь вид ГГГ°ММ'CC.CC" или ГГГ.ГГГГГГГГ°
  // Для Linux вместо символа ° (\xB0) д.б. ^
  // При ошибке возвращает ноль, иначе - значение в радианах

_MAPIMP double _MAPAPI mapAngleToRadian(const char *angle);

_MAPIMP double _MAPAPI mapAngleToRadianUn(const WCHAR *angle);

  // Преобразовать числовое значение из радиан в строку вида
  // ГГГ°ММ'CC.CC"
  // Для Linux вместо символа ° (\xB0) д.б. ^
  // angle  - адрес строки для результата
  // size   - размер строки в байтах
  // При ошибке возвращает ноль, иначе - адрес входной строки

_MAPIMP char* _MAPAPI mapRadianToAngle(double radian, char *angle, int size);

_MAPIMP WCHAR* _MAPAPI mapRadianToAngleUn(double radian, WCHAR *angle, int size);

  // Запись номера объекта в виде строки XXXXX/XXXXX или ХХХХХХХХХХХ
  // (например, 16777339 соответствует строка 256/123)
  // format - формат строки: 1 - XXXXX/XXXXX, 2 - XXXXXXXXXXX,
  // 0 - взять значение по умолчанию (mapGetObjectKeyFormat).
  // Минимальная длина выходной строки - 12 байт
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapObjectKeyToString(long int key , char *string,
                                              long int size, long int format);

_MAPIMP long int _MAPAPI mapObjectKeyToStringUn(long int key, WCHAR *string,
                                                long int size, long int format);

 // Преобразование номера объекта из строки XXXXX/XXXXX или ХХХХХХХХХХХ
 // в число (например, 16777339 соответствует строка 256/123)
 // string - входная строка
 // long   - результат
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapStringToObjectKey(const char *string, long int *key);

_MAPIMP long int _MAPAPI mapStringToObjectKeyUn(WCHAR *string, long int *key);

  // Установить/Запросить формат номера объекта
  // 1 - XXXXX/XXXXX, 2 - XXXXXXXXXXX

_MAPIMP long int _MAPAPI mapGetObjectKeyFormat(void);

_MAPIMP long int _MAPAPI mapSetObjectKeyFormat(long int format);

  // Установить/Запросить путь к директории приложения, где
  // располагаются вспомогательные файлы для функционирования
  // ГИС-ядра (библиотеки условных знаков *.iml, файлы базы данных EPSG,
  // wmslist.xml, xml-схемы и пр.)
  // Рекомендуется устанавливать при запуске приложения

_MAPIMP void _MAPAPI mapSetPathShell(const char *path);

_MAPIMP void _MAPAPI mapSetPathShellUn(const WCHAR *path);

_MAPIMP const char* _MAPAPI mapGetPathShell();
_MAPIMP const WCHAR * _MAPAPI mapGetPathShellUnicode();

_MAPIMP void _MAPAPI mapGetPathShellUn(WCHAR *path, long int size);

_MAPIMP void _MAPAPI mapGetPathShellEx(char *path, long int size);

  // Установить новое имя INI-файла приложения

_MAPIMP void _MAPAPI mapSetIniPath(const char *inipath);

_MAPIMP void _MAPAPI mapSetIniPathUn(const WCHAR *inipath);

  // Запросить имя INI-файла приложения
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetIniPath();

_MAPIMP const WCHAR* _MAPAPI mapGetIniPathUn();

  // Установить/Запросить путь к размещению данных
  // path - адрес строки в кодировке UTF-16 c путем к папке для размещения данных
  // size - длина строки (буфера) в байтах

_MAPIMP void _MAPAPI mapSetDataPathUn(const WCHAR *path);

_MAPIMP void _MAPAPI mapGetDataPathUn(WCHAR *path, long int size);

  // Установить новое имя INI-файла документа
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapSetMapIniName(HMAP hMap, const char *mapininame);

_MAPIMP const WCHAR* _MAPAPI mapSetMapIniNameUn(HMAP hMap, const WCHAR *mapininame);

  // Запросить имя INI-файла документа
  // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI mapGetMapIniName(HMAP hMap);
_MAPIMP const WCHAR * _MAPAPI mapGetMapIniNameEx(HMAP hMap);

  // Запросить имя INI-файла документа в кодировке UNICODE
  // name - адрес строки для размещения результата
  // size - размер строки в байтах
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMapIniNameUn(HMAP hMap, WCHAR *name, long int size);

  // Сформировать имя INI-файла по имени открываемых данных
  // name - путь к открываемому файлу данных или алиас данных с ГИС-Сервера или
  // URL геопортала
  // mapininame - адрес строки для размещения результата
  // size - размер строки, зарезервированный для размещения результата (не менее 260 байт)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapBuildIniName(const char *name, char *mapininame, long int size);

_MAPIMP long int _MAPAPI mapBuildIniNameUn(const WCHAR *name, WCHAR *mapininame, long int size);

  // Установить путь к общим файлам классификаторам (RSC)
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapSetCommonRscPath(const char *rscpath);

_MAPIMP long int _MAPAPI mapSetCommonRscPathUn(const WCHAR *rscpath);

  // Запросить путь к общим файлам классификаторам (RSC)
  // При ошибке возвращает пустую строку

_MAPIMP const char* _MAPAPI mapGetCommonRscPath();

_MAPIMP const WCHAR* _MAPAPI mapGetCommonRscPathUn();

  // Проверить, что имя файла не является алиасом сервера или геопортала
  // При отсутствии спецсимволов возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsNormalPath(const char * name);     // 08/06/14
_MAPIMP long int _MAPAPI mapIsNormalPathUn(const WCHAR * name);

  // Преобразовать имя алиаса или соединения с сервисом в имя файла (без пути)        // 04/08/15
  // alias - имя алиаса или соединения с сервисом (WMS, WFS, WCS)
  // name  - адрес строки для размещения результата
  // size  - размер выделенной памяти в строке в байтах
  // При ошибке возвращает ноль
  
_MAPIMP long int _MAPAPI mapAliasToNormalNameUn(const WCHAR * alias, WCHAR * name, long int size);
_MAPIMP long int _MAPAPI mapAliasToNormalName(const char * alias, char * name, long int size);

  // Формирование сокращенного имени файла из полного имени
  // для отображения в различных диалогах
  // source - исходный путь,
  // target - полученная строка (c:\abc..\klm.ext),
  // size   - предельный размер выходной строки в символах !!!
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPathToShort(const char *source,
                                        char *target, long int size);

_MAPIMP long int _MAPAPI mapPathToShortUn(const WCHAR *source,             // 12/09/12
                                          WCHAR *target, long int size);

  // Построить "короткое" имя файла
  // templ - эталонный путь, относительно которого строится короткий путь;
  // например, путь к библиотекам приложения (mapGetPathShell)
  // name  - полное имя файла
  // Возвращает указатель на "короткое" имя файла
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapBuildShortName(const char *templ,
                                              const char *name);

_MAPIMP const WCHAR* _MAPAPI mapBuildShortNameUn(const WCHAR *templ,
                                                 const WCHAR *name);

  // Построить "короткое" имя файла
  // В качестве эталонного пути применяется путь к библиотекам приложения (mapGetPathShell)
  // name  - полное имя файла в UNICODE
  // Возвращает указатель на "короткое" имя файла
  // При ошибке возвращает ноль

_MAPIMP const WCHAR* _MAPAPI mapBuildShellShortNameUnicode(const WCHAR *name);

  // Построить "длинное" имя файла (полный путь к файлу)
  // templ - эталонный путь, относительно которого строится полный путь;
  // templ - путь к библиотекам приложения (mapGetPathShell)
  // name - короткий путь к файлу; например, имя файла
  // path - "длинное" имя файла (указатель на массив для размещения результата, MAX_PATH)
  // При ошибке возвращает ноль

_MAPIMP char* _MAPAPI mapBuildLongName(const char *templ,
                                       const char *name, char *path);

_MAPIMP long int _MAPAPI mapBuildLongNameUn(const WCHAR *templ,
                                            const WCHAR *name, WCHAR *path);

  // Построить усеченный путь к файлу (относительно данных)
  // templ  - путь к данным (например, mapGetMapPath(HMAP hMap))
  // name   - полное имя файла (c:\abc\def...)
  // target - адрес буфера для рамещения усеченного пути (или полного, если нелья усечь)
  // size   - размер буфера
  // Возвращает указатель на усеченный путь к файлу (.\def...)
  // При ошибке возвращает ноль

_MAPIMP char* _MAPAPI mapCutPath(const char *templ, const char *name,
                                 char *target, int size);

_MAPIMP WCHAR* _MAPAPI mapCutPathUn(const WCHAR *templ, const WCHAR *name,
                                    WCHAR *target, int size);

  // Построить полное имя файла по усеченному пути
  // templ  - путь к данным (например, mapGetMapPath(HMAP hMap))
  // name   - усеченный путь (начинается с .\) или полный путь
  // target - адрес буфера для размещения полного имени файла
  // size   - размер буфера
  // Возвращает указатель на полный путь (с:\abc\def... или \\NAME\abc\...)
  // При ошибке возвращает ноль

_MAPIMP char* _MAPAPI mapUnionPath(const char *templ, const char *name,
                                   char *target, long int size);

_MAPIMP WCHAR* _MAPAPI mapUnionPathUn(const WCHAR *templ, const WCHAR *name,
                                      WCHAR *target, long int size);

  // Векторизация битового растра точечного знака
  //  size     - размер знака: size = 16 (16x16), size = 32 (32x32)
  //  bits     - битовая область знака: 32 байта (16x16), 128 байт (32x32)
  //  length   - длина области data (10000 байт - минимум)
  // Результат - метрика, записанная в data
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapMarkVectorization(int size, char *bits,
                                              int length, POLYDATA *data);

  // Преобразовать текстовую строку из кодировки OEM (DOS) в ANSI (WINDOWS)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapDos2Win(char *string, long int size);

  // Преобразовать текстовую строку из кодировки ANSI (WINDOWS) в OEM (DOS)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapWin2Dos(char *string, long int size);

#ifdef LINUXAPI
  // Преобразовать текстовую строку из кодировки OEM (DOS) в KOI8 (UNIX)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapDos2Koi8(char *string, long int size);

  // Преобразовать текстовую строку из кодировки KOI8 (UNIX) в OEM (DOS)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapKoi82Dos(char *string, long int size);

  // Преобразовать текстовую строку из кодировки ANSI (WINDOWS) в KOI8 (UNIX)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapWin2Koi8(char *string, long int size);

  // Преобразовать текстовую строку из кодировки KOI8 (UNIX) в ANSI (WINDOWS)
  // При ошибке возвращает 0

_MAPIMP char* _MAPAPI mapKoi82Win(char *string, long int size);
#endif

  // Запросить глубину Dib
  // Возвращает количество бит, выделяемое на одну точку изображения карты

_MAPIMP long int _MAPAPI mapGetMapScreenDepth();

 /*********************************************************
 *                                                        *
 *   ФУНКЦИИ СИНХРОНИЗАЦИИ ЗАДАЧ ПО КООРДИНАТАМ ТОЧКИ     *
 *                                                        *
 *   Некоторая задача может менять координаты курсора,    *
 * а другая будет получать уведомления о смене координат  *
 *                                                        *
 *********************************************************/

  // Установить текущие координаты курсора для карты
  // hmap     - идентификатор открытой карты (документа)
  // position - координаты курсора в текущей системе координат документа
  // Если position равно нулю, то координаты считаются не установленными

_MAPIMP void _MAPAPI mapSetMarkerPosition(HMAP hmap, DOUBLEPOINT *position);

  // Запросить текущие координаты курсора для карты
  // hmap     - идентификатор открытой карты (документа)
  // position - координаты курсора в текущей системе координат документа
  // Если координаты не установлены, или параметры ошибочные, то возвращает ноль

_MAPIMP long int _MAPAPI mapGetMarkerPosition(HMAP hmap, DOUBLEPOINT *position);

  // Установить адрес функции, которая будет периодически
  // вызываться при смене координат курсора (вызове mapSetMarkerPosition)
  // Для запроса координат необходимо вызвать mapGetMarkerPosition
  // call - адрес вызываемой функции (см. maptype.h),
  // parm - параметр, который будет передан вызываемой функции
  // При завершении задачи, установившей адрес функции оповещения,
  // необходимо еще раз вызвать функцию с нулевыми параметрами
  // call и parm.

_MAPIMP void _MAPAPI mapSetMarkerPositionCallAndParm(HMAP hmap, BREAKCALL call,
                                                     void *parm);

  /*********************************************************
  *                                                        *
  *              ФУНКЦИИ КАЛИБРОВКИ ИЗОБРАЖЕНИЯ            *
  *                                                        *
  *********************************************************/

  // Установить предельные размеры буфера изображения (не влияет на расчет разрешения экрана)  // 03/06/16
  // Функция должна вызываться до открытия данных
  // Чтобы оставить ширину или высоту без изменения соответствующий параметр
  // должен быть равен 0
  // Если экран компьютера, на котором выполняется программа, имеет большие
  // размеры, то установленные значения будут автоматически увеличены до
  // размеров экрана
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetMaxScreenImageSize(long int width, long int height);

  // Установить расчетные размеры буфера изображения (влияет на расчет разрешения экрана)
  // Функция должна вызываться до открытия данных
  // Чтобы оставить ширину или высоту без изменения соответствующий параметр
  // должен быть равен 0
  // Если экран компьютера, на котором выполняется программа, имеет большие
  // размеры, то установленные значения будут автоматически увеличены до
  // размеров экрана
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetScreenImageSize(long int width, long int height);
_MAPIMP long int _MAPAPI SetScreenImageSize(long int width, long int height);

 // Запросить максимальную ширину изображения карты в точках
 // Установка нового значения - mapSetScreenImageSize

_MAPIMP long int _MAPAPI mapGetMaxScreenWidth();
_MAPIMP long int _MAPAPI mapGetScreenWidth();

 // Запросить максимальную высоту изображения карты в точках
 // Установка нового значения - mapSetScreenImageSize

_MAPIMP long int _MAPAPI mapGetMaxScreenHeight();
_MAPIMP long int _MAPAPI mapGetScreenHeight();

  // Запросить/Установить размер диагонали видимого изображения
  // экрана в миллиметрах (50 - 4000). При установке возвращает
  // старое значение
  // hdc - контекст главного окна для запроса текущих размеров
  // экрана в пикселах
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetScreenSize();
_MAPIMP long int _MAPAPI mapSetScreenSize(long int size);
_MAPIMP long int _MAPAPI mapSetScreenSizeEx(long int size, HDC hdc/* = 0*/);
_MAPIMP double _MAPAPI mapGetScreenSizeEx();
_MAPIMP double _MAPAPI mapSetScreenSizePro(double size);

  // Запросить/Установить коэффициент масштабирования изображения
  // экрана в процентах (100 - 2000). При установке возвращает
  // старое значение и пересчитывает точность текущего режима экрана
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetScreenScale();
_MAPIMP long int _MAPAPI mapSetScreenScale(long int scale);
_MAPIMP double _MAPAPI mapGetScreenScaleEx();
_MAPIMP double _MAPAPI mapSetScreenScaleEx(double scale);

  // Запросить/Установить точность текущего режима экрана в точках
  // на метр (1000 - 100000). При установке возвращает старое
  // значение и пересчитывает коэффициент масштабирования экрана
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetScreenPrecision();
_MAPIMP double _MAPAPI mapGetHorizontalScreenPrecision();
_MAPIMP double _MAPAPI mapGetVerticalScreenPrecision();
_MAPIMP long int _MAPAPI mapSetScreenPrecision(long int value);
_MAPIMP void _MAPAPI mapSetScreenPrecisionEx(double valueHor, double valueVer);

  // Пересчет через текущие параметры экрана

_MAPIMP long int _MAPAPI mapScreenMeter2Pixel(double metric);
_MAPIMP double _MAPAPI mapScreenPixel2Meter(long int pixel);

 // Запросить/Установить максимальный размер и количество одновременно отрытых данных
 // для загрузки в оперативную память и ускорения процесса многопоточной обработки
 // данных
 // maxcount - количество одновременно открытых карт(листов) в оперативной памяти
 // maxsize - максимальный размер одного открытого в оперативной памяти файла карты(листа)
 // При превышении заданных значений будет применяться общий буфер для прокачки данных,
 // что приведет к локированию параллельных потоков (падению производительности).
 // При превышении запрошенных значений доступной оперативной памяти будет
 // происходить отказ при открытии данных

 _MAPIMP void _MAPAPI mapSetOpenDataLimits(long int maxcount, BIGINT maxsize);
 _MAPIMP void _MAPAPI mapGetOpenDataLimits(long int* maxcount, BIGINT* maxsize);


 /*********************************************************
 *                                                        *
 *           ФУНКЦИИ КОМПРЕССИИ И ДЕКОМПРЕССИИ            *
 *                                                        *
 *********************************************************/

  // Компрессия данных по алгоритму типа LZW
  //  in      - массив исходных данных
  //  out     - массив выходных (сжатых) данных
  //  sizein  - размер массива исходных данных
  //  sizeout - размер массива выходных (сжатых) данных
  // Возвращает размер сжатых данных в выходном буфере
  // При ошибке или при сжатии менее чем на 20%
  // выходной массив не заполняется и функция возвращает 0.

_MAPIMP long int _MAPAPI mapCompressLZW(const char *in, long int sizein,
                                        char *out, long int sizeout);

  // Декомпрессия данных по алгоритму типа LZW
  //  in      - массив исходных (сжатых) данных
  //  out     - массив выходных данных
  //  sizein  - размер массива исходных (сжатых) данных
  //  sizeout - размер массива выходных данных
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDecompressLZW(const char *in, long int sizein,
                                          char *out, long int sizeout);

  // Компрессия изображения по алгоритму JPEG
  // in       - массив исходных данных
  // width    - ширина изображения (пикселей)
  // height   - высота изображения (пикселей)
  // bit      - количество бит на пиксель (выполняется сжатие изображения
  //            с количеством бит на пиксель, равным 24)
  // compressionValue - степень сжатия картинки (1-100, 1-максимальное сжатие,
  // 100-сжатие без потери качества), рекомендуется значение 60.
  //  out     - массив выходных данных
  //  sizeout - размер массива выходных данных
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCompressJPEG(const char *in, int width, int height,
                                         int bit, int compressionValue,
                                         char *out, long int sizeout);

  // Декомпрессия изображения по алгоритму JPEG
  // in       - массив исходных (сжатых) данных
  // sizein   - размер массива исходных (сжатых) данных
  // width    - ширина изображения (пикселей)
  // height   - высота изображения (пикселей)
  // bit      - количество бит на пиксель (выполняется сжатие изображения
  //            с количеством бит на пиксель, равным 24)
  //  out     - массив выходных данных
  //  sizeout - размер массива выходных данных
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDecompressJPEG(const char *in, long int sizein,
                                           int width, int height, int bit,
                                           char *out, long int sizeout);

 /*********************************************************
 *                                                        *
 *                   ЗАКРЫТЫЕ ФУНКЦИИ                     *
 *                                                        *
 *********************************************************/

#ifdef WIN32API
  // Запросить номер ключа защиты
_MAPIMP unsigned long int _MAPAPI GetKeyNumber();
#endif

 /*********************************************************
 *                                                        *
 *           ФУНКЦИИ ДОСТУПА К ГИС-СЕРВЕРУ                *
 *                                                        *
 *     Для открытия карты на ГИС-сервере в штатные        *
 *      функции открытия карт передается строка вида      *
 *         "HOST#ХОСТ#ПОРТ#ALIAS#условное_имя_карты"      *
 *    (например: "HOST#233.47.123.99#2047#ALIAS#Noginsk") *
 *                                                        *
 *  Условное имя карты не является путем к паспорту карты *
 *     Строки "HOST#" и "ALIAS#" являются ключевыми       *
 *           словами для обращения к ГИС-серверу          *
 *                                                        *
 *********************************************************/

  // Запросить является ли имя идентификатором данных на Сервере
  // Если да, то возвращает ненулевое значение (1 - устаревший
  // формат без имени сервера, 2 - содержит имя сервера)

_MAPIMP long int _MAPAPI mapIsAliasName(const char *name);

_MAPIMP long int _MAPAPI mapIsAliasNameUn(const WCHAR *name);

  // Запросить имя алиаса данных из полной строки имени, включающей имя хоста
  // Возвращает указатель на имя алиаса (первый символ после ALIAS#) или 0

_MAPIMP const char* _MAPAPI mapGetDataNameFromAlias(const char *name);

_MAPIMP const WCHAR* _MAPAPI mapGetDataNameFromAliasUn(const WCHAR *name);

  // Сформировать имя ресурса на Сервере
  // в формате "HOST#ХОСТ#ПОРТ#ALIAS#условное_имя_карты"
  // host  - имя хоста
  // port  - номер порта
  // alias - имя ресурса (условное имя карты)
  // name  - имя строки для размещения результат
  // size  - максимальный размер в байтах строки
  // При ошибке в параметрах возвращает ноль

_MAPIMP long int _MAPAPI mapBuildAliasName(const char *host, long int port,
                                           const char *alias,
                                           char *name, long int size);

_MAPIMP long int _MAPAPI mapBuildAliasNameUn(const WCHAR *host, long int port,
                                             const WCHAR *alias, WCHAR *name,
                                             long int size);

  // Запросить число подключений к ГИС Серверам
  // При отсутствии подключений возвращает ноль

_MAPIMP long int _MAPAPI mapActiveServerCount();

  // Запросить состояние подключения к серверу
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // При потоковом открытии\добавлении данных с ГИС Сервера рекомендуется
  // после первой ошибки открытия данных проверить состояние подключения
  // и при ошибке прервать потоковую обработку
  // Если после ошибки открытия данных с именем "HOST#..." или "ALIAS#..."
  // подключение не установлено, то нужно убедится, что Сервер запущен и
  // введены правильные параметры соединения
  // Если подключение к серверу установлено - возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsServerActiveEx(long int number);

  // Запросить доступ к средствам мониторинга состояния сервера
  // Если мониторинг запрещен - возвращает нулевое значение

_MAPIMP long int _MAPAPI mapIsServerMonitoringEnable(long int number);

  // Запросить версию ГИС Сервера по номеру соединения
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // Возвращает шестнадцатеричный номер версии ГИС Сервер,
  // например: 0x00040503 соответствует версии 4.5.3
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetServerVersion(long int number);

  // Считать информацию о состоянии открытых соединений (мониторинг)
  // versin  - буфер для размещения строки с именем и версией ГИС Сервера
  // size    - размер буфера (не менее 80 байт)
  // state   - состояние ГИС Сервера, полученное в предыдущем запросе,
  //           если состояние не изменилось, то возвращается сокращенный отчет
  // После завершения обработки данных необходимо освободить ресурсы путем
  // вызова mapFreeServerState с указателем, полученным в mapGetServerState
  // При ошибке возвращает ноль

_MAPIMP GSMONITOR* _MAPAPI mapGetServerState(long int number, WCHAR *version,
                                             long int size, long int state);

  // Освободить ресурсы после обработки данных мониторинга состояния сервера

_MAPIMP void _MAPAPI mapFreeServerState(GSMONITOR *buffer);

  // Запросить выполнено ли подключение и регистрация пользователя
  // для заданного имени алиаса данных
  // name - алиас в формате "HOST#ХОСТ:ПОРТ#ALIAS#условное_имя_карты",
  //        или "HOST#ХОСТ" или "HOST#ХОСТ:ПОРТ"
  // При успешной проверке возвращает номер подключения
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckConnectForAlias(const char *name);

_MAPIMP long int _MAPAPI mapCheckConnectForAliasUn(const WCHAR *name);

  // Открыть новое соединение с ГИС-сервером
  // name - имя хоста (до 256 символов),
  //        или строка адреса "XXX.XXX.XXX.XXX"
  // Если параметр равен нулю - сервер ищется на локальном хосте "localhost".
  // port - номер порта от 1024 до 65536, по умолчанию - 2047 (если port = 0)
  // В случае удачно выполненного подключения возвращает его порядковый номер
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenConnect(const char *name, long int port);

_MAPIMP long int _MAPAPI mapOpenConnectUn(const WCHAR *name, long int port);

  // Открыть новое соединение с ГИС-сервером
  // name - имя хоста (до 256 символов),
  //        или строка адреса "XXX.XXX.XXX.XXX"
  // Если параметр равен нулю - сервер ищется на локальном хосте "localhost".
  // port - номер порта от 1024 до 65536, по умолчанию - 2047 (если port = 0)
  // cansleep - разрешение на открытие виртуального (спящего) соединения,
  //        при отсутствии физического доступа к серверу
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenConnectEx(const char *name, long int port, long int cansleep);

_MAPIMP long int _MAPAPI mapOpenConnectExUn(const WCHAR *name, long int port, long int cansleep);

  // Запросить - можно ли закрыть соединение
  // При ошибке (соединение не найдено) возвращает ноль
  // При занятости соединения возвращает "-1"
  // Если соединение может быть закрыто, то возвращает положительное значение

_MAPIMP long int _MAPAPI mapCanCloseConnect(long int number);

  // Закрыть соединение с ГИС-сервером
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // При ошибке (соединение не найдено) возвращает ноль
  // При занятости соединения возвращает "-1"
  // При успешном выполнении возвращает положительное значение

_MAPIMP long int _MAPAPI mapCloseConnect(long int number);

  // Изменить параметры соединения с ГИС-сервером
  // Вызывается до открытия карт на сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // name - имя хоста (до 256 символов),
  //        или строка адреса "XXX.XXX.XXX.XXX"
  // Если параметр равен нулю - сервер ищется на локальном хосте "localhost".
  // port - номер порта от 1024 до 65536, по умолчанию - 2047 (если port = 0)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetConnectParametersEx(long int number,
                                                   const char *name, long int port);

_MAPIMP long int _MAPAPI mapSetConnectParametersExUn(long int number,
                                                     const WCHAR *name, long int port);

  // Запросить номер порта для связи с ГИС-сервером
  // Номер порта от 1024 до 65536, по умолчанию - 2047
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()

_MAPIMP long int _MAPAPI mapGetConnectPortEx(long int number);

  // Запросить имя\адрес хоста
  // Если было установлен адрес хоста - возвращаемое значение 1,
  // если имя хоста - возвращаемое значение 2.
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // Если установлено оба значения - возвращается адрес хоста
  // name - адрес строки для размещения результата
  // size - размер строки (для имени хоста не менее 256)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetConnectHostEx(long int number, char *name, long int size);

_MAPIMP long int _MAPAPI mapGetConnectHostExUn(long int number, WCHAR *name, long int size);

  // Зарегистрировать пользователя
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // Если соединение с сервером не было установлено -
  // пытается соединиться с установленными ранее параметрами
  // Пароль должен передаваться в зашифрованном виде по алгоритму MD5 (в виде хэша)
  // Для получения хэша пароля следует использовать функцию svStringToHash 
  // (описана в gisdlgs.h)  
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterUserEx(long int number, TMCUSERPARM * parm);

  // Зарегистрировать текущего пользователя ОС как пользователя ГИС Сервера
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // Если соединение с сервером не было установлено -
  // пытается соединиться с установленными ранее параметрами
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterSystemUserEx(long int number);

  // Удалить в памяти параметры регистрации пользователя
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // После закрытия последнего документа на Сервере соединение
  // разрывается и для последующего открытия карты нужно повторно
  // выполнить mapRegisterUser()

_MAPIMP void _MAPAPI mapUnRegisterUserEx(long int number);

  // Запросить тип регистрации пользователя
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // Если регистрация пользователя выполнялась через функцию mapRegisterSystemUserEx,
  // то возвращается положительное значение

_MAPIMP long int _MAPAPI mapGetRegisterUserType(long int number);

  // Запросить список доступных пользователю карт на ГИС-сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес памяти для размещения списка карт,
  //          структура TMCMAPLIST описана в maptype.h
  // Если buffer равно нулю, возвращает размер необходимой области памяти
  // length - длина выделенной области памяти
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetMapListforUserEx(long int number, TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю атласов на ГИС-сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес памяти для размещения списка атласов,
  //          структура TMCMAPLIST описана в maptype.h
  // length - длина выделенной области памяти
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetAlsListforUserEx(long int number, TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю матриц на ГИС-сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес памяти для размещения списка матриц,
  //          структура TMCMAPLIST описана в maptype.h
  // Если buffer равно нулю, возвращает размер необходимой области памяти
  // length - длина выделенной области памяти
  // Возвращает общий размер считанной записи или 0

_MAPIMP  long int _MAPAPI mapGetMtwListforUserEx(long int number, TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю растров на ГИС-сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес памяти для размещения списка растров,
  //          структура TMCMAPLIST описана в maptype.h
  // Если buffer равно нулю, возвращает размер необходимой области памяти
  // length - длина выделенной области памяти
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetRswListforUserEx(long int number, TMCMAPLIST *buffer, int length);

  // Запросить имя пользователя, подключившегося к ГИС-серверу
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI mapGetCurrentUserNameEx(long int number);

  // Установить текущий тип передаваемых данных
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // карты - 1, растры - 2, матрицы - 3, атласы  - 4
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetActiveDataType(long int number, long int type);

  // Запросить текущий тип передаваемых данных
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // карты - 1, растры - 2, матрицы - 3, атласы  - 4
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActiveDataType(long int number);

  // Запросить путь к папке для хранения кэшируемых данных с ГИС Сервер

_MAPIMP const char* _MAPAPI mapGetCachePath();
_MAPIMP const WCHAR* _MAPAPI mapGetCachePathUn();

  // Установить путь к папке для хранения кэшируемых данных с ГИС Сервер
  // Если приложение не установило путь к папке для кэширования данных,
  // то она автоматически будет размещена внутри системной папки Temp
  // в папке Panorama.Cache.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetCachePath(const char *path);
_MAPIMP long int _MAPAPI mapSetCachePathUn(const WCHAR *path);

  // Очистить кэш данных для всех карт текущего документа,
  // открытых с ГИС Сервера
  // hMap  -  идентификатор открытых данных
  // Кэш автоматически очищается при сортировке карты на ГИС Сервере
  // или обнаружении большого числа выполненных транзакций,
  // с момента предыдущего обращения к данным
  // Иначе кэш обновляется (реплицируется) в соответствии
  // с журналом транзакций на ГИС Сервере

_MAPIMP void _MAPAPI mapClearDocCache(HMAP hMap);

  // Очистить кэш данных для открытой пользовательской карты,
  // открытой с ГИС Сервера
  // hMap  -  идентификатор открытых данных
  // hSite - идентификатор открытой пользовательской карты
  // Кэш автоматически очищается при сортировке карты на ГИС Сервере
  // или обнаружении большого числа выполненных транзакций,
  // с момента предыдущего обращения к данным
  // Иначе кэш обновляется (реплицируется) в соответствии
  // с журналом транзакций на ГИС Сервере

_MAPIMP void _MAPAPI mapClearSiteCache(HMAP hMap, HSITE hSite);

  // Запросить открыта ли карта на сервере или локально
  // hmap -  идентификатор открытых данных
  // hSite - идентификатор открытой пользовательской карты
  // (для фоновой (основной) карты hSite = hMap)
  // Если карта открыта на сервере возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsMapFromServer(HMAP hMap, HSITE hSite);

  // Запросить список папок на сервере, доступных для записи файлов            // 02/07/14
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // folder - путь к папке, в которой запрашивается список файлов и папок или 0
  // Например, "Data\\Maps"
  // Если folder равно 0, то запрашивается список алиасов всех доступных папок
  // allfiles - признак запроса всех файлов в папке folder, если не установлен,
  // то буден выдан список внутренних папок и файлов MAP,SIT,SITX,RSC,MTW,MTQ,RSW
  // parm   - адрес буфера для размещения списка запрошенных данных или 0
  // Если parm равно 0, то запрашивается размер буфера, требуемый для размещения списка
  // length - размер буфера для размещения списка
  // Список данных заполняется только для файлов и папок, непосредственно расположенных
  // в заданной папке без вложений
  // При успешном выполнении возвращает размер сформированного списка
  // Если размер списка превышает размер буфера, то данные считаны не полностью.
  // Тогда нужно выделить больший буфер и запросить данные повторно
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetFolderList(long int number, const WCHAR * folder, long int allfiles,
                                          TMCDATALIST * parm, long int size);

  // Создать папку на сервере относительно алиаса доступной папки            // 26/06/14
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // folder - путь к создаваемой папке
  // Например, "Data\\Maps" или "Data/Maps",
  // где "Data" - корневая папка в настройках ГИС Сервера, "Maps" - создаваемая папка
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateFolderOnServer(long int number, const WCHAR * folder);

  // Удалить папку на сервере относительно алиаса доступной папки            // 27/06/14
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // folder - путь к удаляемой папке
  // Например, для удаления папки "Maps": "Data\\Maps" или "Data/Maps",
  // где "Data" - корневая папка в настройках ГИС Сервера, "Maps" - удаляемая папка
  // deletefiles - удалить все файлы в папке (если задано ненулевое значение)
  // deletefolders - удалить все подпапки с файлами в них (если задано ненулевое значение)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteFolderOnServer(long int number, const WCHAR * folder,
                                                 long int deletefiles, long int deletefolders);

  // Удалить файл на сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // file - путь к удаляемому файлу
  // Например, "Data\\Maps\\image.rsw",
  // где "Data" - корневая папка в настройках ГИС Сервера, "image.rsw" - удаляемый файл
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteFileOnServer(long int number, const WCHAR * file);

  // Переименовать (переместить) файл или папку на сервере
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // source - путь к изменяемому файлу или папке
  // Например, "Data\\Maps\\image.sitx",
  // где "Data" - корневая папка в настройках ГИС Сервера, "image.sitx" - перемещаемый файл
  // target - новый путь к файлу или папке
  // Например, "Storage\\Roads\\road_M4.sitx",
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRenameFileOnServer(long int number, const WCHAR * source, const WCHAR * target);

  // Сохранить файл на сервере                                               // 26/06/14
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // folder - путь для сохранения файла, путь должен начинаться с алиаса папки,
  //          доступной для записи данных
  // Например, "Data\\Maps" или "Data/Maps",
  // где "Data" - корневая папка в настройках ГИС Сервера
  // file   - путь к файлу, который будет записан в папку на сервере (файл должен
  //          содержать данные, а не выполняемый код).
  // Имя файла, расширение и атрибуты чтения/записи сохраняются.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveFileOnServer(long int number, const WCHAR * folder,
                                             const WCHAR * file);

  // Сохранить карту на сервере                                               // 02/07/14
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // folder - путь для сохранения файла, путь должен начинаться с алиаса папки,
  //          доступной для записи данных
  // Например, "Data\\Maps" или "Data/Maps",
  // где "Data" - корневая папка в настройках ГИС Сервера
  // hMap -  идентификатор открытых данных
  // hSite - идентификатор открытой пользовательской карты
  // (для фоновой (основной) карты hSite = hMap)
  // rscsave - признак необходимости сохранения файла RSC на сервер
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveMapOnServer(long int number, const WCHAR * folder,
                                            HMAP hMap, HSITE hSite, long int rscsave);

  // Прочитать файл на сервере (если есть доступ к виртуальной папке)
  // alias   - алиас файла на сервере, начиная с имени виртуальной папки (кроме выполняемых файлов,
  //           файлов карт, матриц и растров)
  // path    - путь, по которому будет сохранен файл (после записи путь дополняется именем файла)
  // size    - размер поля, содержащего путь
  // error   - поле для записи кода ощибки, если функция вернет ноль
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReadFileOnServer(long int number, const WCHAR * alias, WCHAR * path, long int size,
                                             long int * error);

  // Сохранить данные в Банк данных ЦК и ДЗЗ                   // 24/08/15
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // handle  - иденетификатор окна, которому посылаются сообщения о ходе загрузки данных в Банк данных
  //           (WM_PROGRESSBAR, WPARAM = -1 - старт, LPARAM - текст сообщения в кодировке приложения,
  //                            0-100 - % выполнения, -2 - завершение процесса)
  //           цикл сообщений посылается при передаче каждого набора данных (файла) на сервер и
  //           при загрузке наборов в Банк данных на сервере
  // dslist  - путь к файлу - списку загружаемых наборов данных (111, 222, 333 - имена файлов)
  //           <?xml version="1.0" encoding="UTF-8"?> <dsl> 111, 222, 333 </dsl>
  // logname - буфер строки для получения имени файла с протоколом загрузки наборов в Банк данных
  // size    - размера буфера для записи имени файла протокола
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveToGeoDB(long int number, HMESSAGE handle,
                                        const WCHAR * dslist, WCHAR * logname, long int size);

  // Отправить команду на загрузку наборов данных, которые уже переданы на ГИС Сервер,     // 09/02/16
  // в Банк данных ЦК и ДЗЗ
  // Наборы данных могут быть загружены удаленно другим приложением на ГИС Сервер
  // в специальную папку для Банка Данных
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // handle  - идентификатор окна, которому посылаются сообщения о ходе загрузки данных в Банк данных
  // dslist  - имя файла - списка загружаемых наборов данных
  // callevent - адрес функции оборатного вызова для уведомления о проценте обработанных наборов данных (см. maptype.h)
  // parm    - адрес параметров, которые будут переданы при вызове функции (обычно адрес класса управляющей программы),
  //           вторым параметром в вызываемой функции передается процент от 0 до 100
  // logname - буфер строки для получения имени файла с протоколом загрузки наборов в Банк данных
  // size    - размера буфера для записи имени файла протокола
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCallGeoDBLoader(long int number, HMESSAGE handle, const WCHAR * dslist, WCHAR * logname, long int size);
_MAPIMP long int _MAPAPI mapCallGeoDBLoaderEx(long int number, EVENTSTATE callevent, void * parm,
                                              const WCHAR * dslist, WCHAR * logname, long int size);

  // Отправить команду на формирование геопокрытия из наборов данных Банка данных ЦК и ДЗЗ     // 22/06/16
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // geolist - имя файла - списка загружаемых наборов данных
  // callevent - адрес функции оборатного вызова для уведомления о проценте обработанных наборов данных (см. maptype.h)
  // parm    - адрес параметров, которые будут переданы при вызове функции (обычно адрес класса управляющей программы),
  //           вторым параметром в вызываемой функции передается процент от 0 до 100
  // logname - буфер строки для получения имени файла с протоколом загрузки наборов в Банк данных
  // size    - размера буфера для записи имени файла протокола
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCallGeoLevel(long int number, EVENTSTATE callevent, void * parm,
                                         const WCHAR * geolist, WCHAR * logname, long int size);
_MAPIMP long int _MAPAPI mapCallGeoLevelEx(long int number, EVENTSTATE callevent, void * parm,
                                           const WCHAR * geolist, WCHAR * logname, long int size,
                                           long int * error);

  // Запросить доступность функции сохранения наборов данных в Банк данных ЦК и ДЗЗ
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapIsGeoDBEnable(long int number);
                                        
  // Запросить список обрабатываемых в Банке данных форматов      // 13/11/15
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес буфера для размещения результата запроса
  // size   - размер буфера
  // Список форматов состоит из трех текстовых строк, заканчивающихся символом '\n'
  // В первой строке записывается список форматов векторых карт, во второй - растров (снимков),
  // в третьей - матриц (моделей). Форматы данных разделяются запятыми.
  // Кроме форматов основных файлов указываются форматы вспомогательных файлов
  // (метаданных, уменьшенных изображений данных и других).
  // Например:
  // SXF,RSC,DIR,MIF,MID,KML,SITX,XML,SHP,DBF,PRJ,SHX,preview.png,
  // RSW,TAB,INI,JPG,preview.png,TIF,IMG,TIFF,
  // MTW,TIFF,TIF,preview.png,HGT,
  // При ошибке возвращает ноль, иначе - размер считанных данных

_MAPIMP long int _MAPAPI mapGetGeoDBFormats(long int number, char * buffer, long int size);

  // Запросить список обрабатываемых в Банке данных групп
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // buffer - адрес буфера для размещения результата запроса
  // size   - размер буфера
  // Список групп состоит из текстовых строк, заканчивающихся символом '\n'
  // В начале строки указан идентификатор группы, а через запятую указано название группы в кодировке UTF-8
  // Например: "7,Обзорно-географические карты\n"
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetGeoDBGroups(long int number, char * buffer, long int size);      // 25/04/16

  // Запросить разрешение на копирование файлов карты на клиент       // 05/03/16
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // alias - алиас данных на сервере
  // error - код ошибки доступа (см. maperr.rh)
  // При ошибке возвращает ноль, иначе - тип данных (1 - карта, 2 - растр, 3 - матрица)

_MAPIMP long int _MAPAPI mapIsCopyDataEnabled(long int number, const WCHAR * alias, long int type, long int * error);

  // Скопировать набор данных с ГИС Сервера на клиент                 // 07/03/16
  // number - номер активного подключения к ГИС Серверу от 1 до mapActiveServerCount()
  // alias  - алиас данных на сервере
  // type   - тип данных (1 - карта, 2 - растр, 3 - матрица или 0 - требуется определить)
  // target - имя файла выходного набора данных (если задана только папка, то имя формируется
  //          с учетом алиаса и типа данных
  // error  - код ошибки доступа (см. maperr.rh)
  // При ошибке возвращает ноль, иначе - тип данных (1 - карта, 2 - растр, 3 - матрица)

_MAPIMP long int _MAPAPI mapCopyDataFromServer(long int number, const WCHAR * alias, long int type,
                                               const WCHAR * target, long int * error);

 /*********************************************************
 *                                                        *
 *    Функции кодирования/декодирования данных            *
 *                                                        *
 *********************************************************/

   // Получить идентификатор для системы кодирования
   // key  - строка, содержащая двоичный ключ для кодирования данных
   //        Ключ должен иметь случайное равномерное заполнение,
   //        например, методом преобразования пароля пользователя и текущего
   //        времени по алгоритму MD5
   // size - длина строки (кратно 16 байт)
   // После завершения кодирования/раскодирования нужно освободить ресурсы
   // функцией mapDeleteCoder
   // При ошибке возвращает ноль

_MAPIMP HANDLE WINAPI mapCreateCoder(const char *key, int size);

   // Закодировать область данных заданным ключом (длина области кратна 16)
   // hcoder - идентификатор для системы кодирования
   // memory - адрес области памяти, которую нужно закодировать
   // size   - размер области памяти для кодирования, кратный 16 байтам
   // Для кодирования применяются операции XOR и циклического сдвига данных
   // Состояние ключа меняется при кодировании под управлением кодируемых данных
   // При ошибке возвращает ноль

_MAPIMP long int WINAPI mapCoderOn(HANDLE coder, char *memory, long int size);

   // Раскодировать область данных заданным ключом (длина области кратна 16)
   // hcoder - идентификатор для системы кодирования
   // memory - адрес области памяти, которую нужно закодировать
   // size   - размер области памяти для кодирования, кратный 16 байтам
   // Для кодирования применяются операции XOR и циклического сдвига данных
   // Состояние ключа меняется при кодировании под управлением кодируемых данных
   // При ошибке возвращает ноль

_MAPIMP long int WINAPI mapCoderOff(HANDLE coder, char *memory, long int size);

   // Освободить ресурсы после завершения кодирования/раскодирования

_MAPIMP void WINAPI mapDeleteCoder(HANDLE coder);

}       // extern "C"


 /*********************************************************
 *                                                        *
 *                Устаревшие функции                      *
 *                                                        *
 *********************************************************/

extern "C"
{
  // Открыть данные с автоматическим определением их типа
  // (векторные,растровые,матричные...)
  // name - имя открываемого файла (MAP, SIT, MTW, RSW, MPT)
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение, при этом не открываются
  // файлы \Log\name.log и \Log\name.tac - протокол работы и журнал транзакций
  // error - после выполнения функции переменная содержит код ошибки
  //        (когда HMAP равен 0) или 0; коды ошибок приведены в maperr.rh
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenDataEx(const char *name,
                                   long int mode, long int *error);

  // Открыть данные с автоматическим определением их типа
  // (векторные,растровые,матричные...)
  // name - имя открываемого файла (MAP, SIT, MTW, RSW, MPT)
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение, при этом не открываются
  // файлы \Log\name.log и \Log\name.tac - протокол работы и журнал транзакций
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenData(const char *name, long int mode = 0);

  // Открыть векторные данные (аналогично mapOpenData)
  // Возвращает идентификатор открытой векторной карты
  // mapname - имя файла карты
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE, или 0)
  // GENERIC_READ - все данные только на чтение, при этом не открываются
  // файлы \Log\name.log и \Log\name.tac - протокол работы и журнал транзакций
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenMap(const char *mapname, long int mode = 0);

  // Проверка корректности паспортных данных и, если надо, то заполнение
  // координат по признаку приоритета 0 - расчет прямоугольных координат
  //                                  1 - геодезических
  // Структуры MAPREGISTER, LISTREGISTER описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI CheckAndUpdate(MAPREGISTER *mapreg,
                                        LISTREGISTER *listreg,
                                        long int priority);

  // Создать новый район (новую векторную карту)
  // mapname - полное имя файла карты
  // rscname - полное имя файла ресурсов
  // Возвращает идентификатор открытой векторной карты
  // Структуры MAPREGISTER и LISTREGISTER описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateMap(const char *mapname, const char *rscname,
                                  MAPREGISTER *map, LISTREGISTER *sheet);

  // Создать пользовательскую карту
  // mapname - полное имя файла карты
  // rscname - полное имя файла ресурсов
  // Возвращает идентификатор открытой векторной карты
  // Структура CREATESITE описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateSite(const char *mapname,
                                   const char *rscname,
                                   CREATESITE *createsite);

_MAPIMP HMAP _MAPAPI mapCreateSiteEx(const char *mapname,
                                     const char *rscname,
                                     CREATESITEEX *createsite);

  // Создать пользовательскую карту по открытой карте
  // hmap -  идентификатор открытых данных
  // mapname - полное имя файла карты
  // rscname - полное имя файла ресурсов
  // Структура CREATESITE описана в maptype.h
  // Возвращает идентификатор открытой пользовательской карты
  // При ошибке возвращает ноль

_MAPIMP HSITE _MAPAPI mapCreateAndAppendSite(HMAP hmap,
                                             const char *mapname,
                                             const char *rscname,
                                             CREATESITE *createsite);


  // Создать пользовательскую карту по открытой карте
  // hmap -  идентификатор открытой векторной карты
  // mapname - полное имя файла карты
  // rscname - полное имя файла ресурсов
  // issitx  - признак создания карты типа SITX
  // При ошибке возвращает ноль
  
_MAPIMP HSITE _MAPAPI mapCreateAndAppendSiteForMap(HMAP hmap,
                                             const char * mapname,
                                             const char * rscname);

_MAPIMP HSITE _MAPAPI mapCreateAndAppendSiteForMapUn(HMAP hmap,              // 23/01/15
                                                     const WCHAR * mapname,
                                                     const WCHAR * rscname,
                                                     long int issitx = 0);
                                             
 // Установить пошаговый вывод DIB в окно
 // flag - прорисовка окна за несколько проходов (1) или однократная прорисовка (0)
 // По-умолчанию при скроллинге окна карта подрисовывается несколько
 // раз нарастающими порциями. Для больших карт это убирает скачки изображения
 // (белая полоса - картинка). Площадные объекты почти сразу обновляют фон,
 // затем к ним добавляются другие объекты.
 // Но при отрисовке небольших фрагментов такой способ может создавать мерцание,
 // поэтому его лучше отключать.
 // Переключение режима лучше делать непосредственно перед отображением, а затем
 // восстанавливать предыдущее значение.
 // Возвращает предыдущее значение

_MAPIMP long int _MAPAPI mapSetPaintStep(HMAP hmap, long int flag);

 // Запросить значение пошагового вывода DIB в окно
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetPaintStep(HMAP hmap);

#ifdef WIN32API
  // Запросить доступ к групповым объектам
  // Если доступ запрещен - групповые объекты не создаются
  // и не обрабатываются

_MAPIMP long int _MAPAPI GetGroupObjectsAccess();
_MAPIMP long int _MAPAPI SetGroupObjectsAccess(long int access);
#endif

 // Запрос признака группового объекта (устаревшая функция)
 // info - идентификатор объекта карты в памяти
 // Если объект групповой, возвращает 1, иначе 0

_MAPIMP long int _MAPAPI mapObjectIsGroup(HOBJ info);

 // Запрос первого объекта в группе (устаревшая функция)
 // info - исходный объект
 // infofirst - первый объект в группе
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapFirstObjectInGroup(HOBJ infofirst, HOBJ info);

 // Запрос следующего объекта в группе (устаревшая функция)
 // info - исходный объект
 // infonext - следующий за исходным объект в группе
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapNextObjectInGroup(HOBJ infonext, HOBJ info);

 // Объединение метричных данных групповых объектов (устаревшая функция)
 // info - идентификатор объекта карты в памяти
 // limit - допуск для дотягивания (в метрах)
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUnionGroupObject(HOBJ info, double limit);


 /*********************************************************
 *                                                        *
 *  УСТАРЕВШИЕ ФУНКЦИИ. ОБРАЩАЮТСЯ К ПЕРВОМУ СОЕДИНЕНИЮ.  *
 *                                                        *
 *      Устаревший формат имени ресурса на ГИС Сервере -  *
 *           "ALIAS#условное_имя_карты"                   *
 *                                                        *
 *********************************************************/

  // Установить параметры соединения с ГИС-сервером
  // Вызывается до открытия карт на сервере
  // name - имя хоста (до 256 символов), если ishost не равен 0
  //        или строка адреса "XXX.XXX.XXX.XXX"
  // Если параметр равен нулю - сервер ищется на локальном хосте "localhost".
  // port - номер порта от 1024 до 65536, по умолчанию - 2047 (если port = 0)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetConnectParameters(const char *name,
                                                 long int port,
                                                 long int ishost);

  // Запросить номер порта для связи с ГИС-сервером
  // Номер порта от 1024 до 65536, по умолчанию - 2047

_MAPIMP long int _MAPAPI mapGetConnectPort();

  // Запросить имя\адрес хоста
  // Если было установлен адрес хоста - возвращаемое значение 1,
  // если имя хоста - возвращаемое значение 2.
  // Если установлено оба значения - возвращается адрес хоста
  // name - адрес строки для размещения результата
  // size - размер строки (для имени хоста не менее 256)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetConnectHost(char *name, long int size);

  // Зарегистрировать пользователя
  // Если соединение с сервером не было установлено -
  // пытается соединиться с установленными ранее параметрами
  // Структура USERPARM описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterUser(TMCUSERPARM *parm);

  // Зарегистрировать пользователя операционной системы на ГИС Сервере
  // Если соединение с сервером не было установлено -
  // пытается соединиться с установленными ранее параметрами
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRegisterSystemUser();

  // Удалить в памяти параметры регистрации пользователя
  // После закрытия последнего документа на Сервере соединение
  // разрывается и для последующего открытия карты нужно повторно
  // выполнить mapRegisterUser() или mapRegisterSystemUser()

_MAPIMP void _MAPAPI mapUnRegisterUser();

  // Запросить имя пользователя, подключившегося к ГИС-серверу
  // При ошибке возвращает пустую строку

_MAPIMP const char* _MAPAPI mapGetCurrentUserName();

  // Запросить список доступных пользователю карт на ГИС-сервере
  // До вызова функции необходимо зарегистрировать пользователя
  // функцией mapRegisterUser.
  // buffer - адрес памяти для размещения списка карт,
  //          структура TMCMAPLIST описана в maptype.h
  // length - длина выделенной области памяти
  // Для открытия выбранной карты функциями MAPAPI к ее
  // имени должен быть добавлен префикс "ALIAS#"
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetMapListforUser(TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю матриц на ГИС-сервере
  // buffer - адрес памяти для размещения списка карт,
  //          структура TMCMAPLIST описана в maptype.h
  // length - длина выделенной области памяти
  // Для открытия выбранной матрицы функциями MAPAPI к ее
  // имени должен быть добавлен префикс "ALIAS#"
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetMtwListforUser(TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю растров на ГИС-сервере
  // buffer - адрес памяти для размещения списка карт,
  //          структура TMCMAPLIST описана в maptype.h
  // length - длина выделенной области памяти
  // Для открытия выбранного растра функциями MAPAPI к ее
  // имени должен быть добавлен префикс "ALIAS#"
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetRswListforUser(TMCMAPLIST *buffer, int length);

  // Запросить список доступных пользователю атласов на ГИС-сервере
  // buffer - адрес памяти для размещения списка атласов,
  //          структура TMCMAPLIST описана в maptype.h
  // length - длина выделенной области памяти
  // Возвращает общий размер считанной записи или 0

_MAPIMP long int _MAPAPI mapGetAlsListforUser(TMCMAPLIST *buffer, int length);

  // Запросить состояние подключения к серверу
  // hmap -  идентификатор открытых данных или 0
  // При потоковом открытии\добавлении данных с ГИС Сервера рекомендуется
  // после первой ошибки открытия данных проверить состояние подключения
  // и при ошибке прервать потоковую обработку
  // Если после ошибки открытия данных с именем "ALIAS#..."
  // подключение не установлено, то нужно убедиться, что Сервер запущен и
  // введены правильные параметры соединения
  // Если подключение к серверу установлено - возвращает ненулевое значение

_MAPIMP long int _MAPAPI mapIsServerActive(HMAP hMap);

 /*********************************************************
 *                                                        *
 *    ФУНКЦИИ ЗАПРОСА/УСТАНОВКИ ШРИФТОВ (Устаревшие)      *
 *                                                        *
 *********************************************************/

#ifdef WIN32API
  // Выполнить инициализацию системы gdi+ для ускорения отрисоки        // 14/04/16
  // Функция применяется только в многопоточных приложениях до начала
  // отрисовки для исключения одновременных запросов на подключение gdi+
  // При успешной инициализации возвращает 1,
  // если не найден кодек image/png возвращает -1
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapInitGDIPlus(void);

  // Запросить/Установить шрифт карты
  // Функции определены для ОС Windows
  // technology - тип устройства (DT_RASDISPLAY, DT_PLOTTER)
  //              определяемый функцией GetDeviceCaps(...)
  // index      - номер шрифта (0-4)
  // fontname   - указатель на имя шрифта, размером 32 символа,
  //              включая конец строки 0x00

_MAPIMP char* _MAPAPI mapGetMapFont(long int technology, long int index = 0);
_MAPIMP char* _MAPAPI mapSetMapFont(long int technology, long int index, char* fontname);
_MAPIMP char* _MAPAPI GetMapFont(int technology, int index = 0);
_MAPIMP char* _MAPAPI SetMapFont(int technology, int index, char* fontname);

_MAPIMP long int _MAPAPI mapGetMapFontName(long int technology, long int index,
                                           char *name, long int size);

_MAPIMP long int _MAPAPI mapSetMapFontName(long int technology, long int index,
                                           char *name, long int size);

  // Запросить/Установить кодировку шрифта карты
  // technology - тип устройства (DT_RASDISPLAY, DT_PLOTTER)
  //              определяемый функцией GetDeviceCaps(...)
  // index      - номер шрифта (0-4)
  // code       - (RUSSIAN_CHARSET, DEFAULT_CHARSET, ANSI_CHARSET)

_MAPIMP long int _MAPAPI mapGetMapFontCharSet(long int technology, long int index);
_MAPIMP long int _MAPAPI mapSetMapFontCharSet(long int technology, long int index,
                                              long int code);

  // Запросить номер шрифта (0-4)

_MAPIMP int _MAPAPI mapGetMapFontIndex(long int technology, char* fontname);
_MAPIMP int _MAPAPI GetMapFontIndex(long int technology, char* fontname);

  // Запросить/Установить шрифт строк диалогов

_MAPIMP char* _MAPAPI mapGetDialogStringFont();
_MAPIMP char* _MAPAPI mapSetDialogStringFont(char* fontname);
_MAPIMP char* _MAPAPI GetDialogStringFont();
_MAPIMP char* _MAPAPI SetDialogStringFont(char* fontname);

  // Запросить/Установить шрифт таблиц диалогов

_MAPIMP char* _MAPAPI mapGetDialogTableFont();
_MAPIMP char* _MAPAPI mapSetDialogTableFont(char* fontname);
_MAPIMP char* _MAPAPI GetDialogTableFont();
_MAPIMP char* _MAPAPI SetDialogTableFont(char* fontname);

  // Запросить/Установить размер шрифта строк и таблиц диалогов

_MAPIMP long int _MAPAPI mapGetDialogStringFontSize();

_MAPIMP long int _MAPAPI mapGetDialogTableFontSize();

_MAPIMP long int _MAPAPI mapSetDialogStringFontSize(long int size);

_MAPIMP long int _MAPAPI mapSetDialogTableFontSize(long int size);

  // Запросить/Установить кодовую страницу шрифта строк и таблиц диалогов

_MAPIMP long int _MAPAPI mapGetDialogStringFontCharSet();

_MAPIMP long int _MAPAPI mapGetDialogTableFontCharSet();

_MAPIMP long int _MAPAPI mapSetDialogStringFontCharSet(long int charset);

_MAPIMP long int _MAPAPI mapSetDialogTableFontCharSet(long int charset);

  // Запросить языковую локализацию VCL-диалога (для Windows)
  // Возвращает: DEFAULT_CHARSET, RUSSIAN_CHARSET, ...

_MAPIMP long int _MAPAPI mapGetWinCharSet();  // 23/10/15

#endif  // WIN32API

}       // extern "C"


#if defined(__BORLANDC__) || defined(LINUXAPI)                    // 21/04/14
inline int _ScaleToString(double scale, TCHAR* string, int size)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapScaleToString(scale, string, size);
#else
  return mapScaleToStringUn(scale, string, size);
#endif
}

inline int _LongToString(int value, TCHAR* string, int size)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapLongToString(value, string, size);
#else
  return mapLongToStringUn(value, string, size);
#endif
}

inline int _PathToShort(const TCHAR * source, TCHAR * target, int size)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapPathToShort(source, target, size);
#else
  return mapPathToShortUn(source, target, size);  // Размер в символах
#endif
}

inline long int _BuildLongName(const TCHAR * templ, const TCHAR * name, TCHAR * path)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return (mapBuildLongName(templ, name, path) != 0);
#else
  return mapBuildLongNameUn(templ, name, path);
#endif
}

inline const TCHAR * _BuildShortName(const TCHAR * templ, const TCHAR * name)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapBuildShortName(templ, name);
#else
  return mapBuildShortNameUn(templ, name);
#endif
}

inline void _ErrorMessage(int code, const TCHAR *filename)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  mapErrorMessage(code, filename);
#else
  mapErrorMessageUn(code, filename);
#endif
}

inline const TCHAR * _GetIniPath()
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapGetIniPath();
#else
  return mapGetIniPathUn();
#endif
}

inline const TCHAR * _GetPathShell()
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapGetPathShell();
#else
  return mapGetPathShellUnicode();
#endif
}

inline void _MessageToLog(HMAP hMap, HSITE hSite, 
                          const TCHAR *message, long int type)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  mapMessageToLog(hMap, hSite, message, type);
#else
  mapMessageToLogUn(hMap, hSite, message, type);
#endif
}

inline int _GetMapName(HMAP hmap, TCHAR *name, int size)
{
#if !defined(UNICODE11) && !defined(UNICODE)
   return mapGetMapNameEx(hmap, name, size);
#else
   return mapGetMapNameUn(hmap, name, size);
#endif
}

inline const TCHAR * _GetSiteName(HMAP hmap, HSITE hsite)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapGetSiteName(hmap, hsite);
#else
  return mapGetSiteNameEx(hmap, hsite);
#endif  
}

inline int _GetSiteFileNameEx(HMAP hmap, HSITE hsite, TCHAR *name, int size)
{
#if !defined(UNICODE11) && !defined(UNICODE)
   return mapGetSiteFileNameEx(hmap, hsite, name, size);
#else
   return mapGetSiteFileNameUn(hmap, hsite, name, size);
#endif
}

inline const TCHAR * _GetMapIniName(HMAP hMap)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapGetMapIniName(hMap);
#else
  return mapGetMapIniNameEx(hMap);
#endif
}

inline const TCHAR * _GetMainName(HMAP hMap)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapGetMainName(hMap);
#else
  return mapGetMainNameEx(hMap);
#endif
}

inline long int _IsAliasName(const TCHAR * name)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapIsAliasName(name);
#else
  return mapIsAliasNameUn(name);
#endif
}
#endif

inline long int _IsNormalPath(const TCHAR * name)
{
#if !defined(UNICODE11) && !defined(UNICODE)
  return mapIsNormalPath(name);
#else
  return mapIsNormalPathUn(name);
#endif
}

#endif  // MAPAPI_H

