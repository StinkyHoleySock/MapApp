/****** RSTAPI.H *************** Belenkov O.V.   ****** 19/01/14 ***
******* RSTAPI.H *************** Shabakov D.A.   ****** 11/07/16 ***
******* RSTAPI.H *************** Gorbunov A.V.   ****** 14/07/10 ***
******* RSTAPI.H *************** Kruzhkov A.E.   ****** 31/05/10 ***
******* RSTAPI.H *************** Vitsko D.A.     ****** 23/11/05 ***
******* RSTAPI.H *************** Derevyagina Zh.A.***** 12/10/09 ***
******* RSTAPI.H *************** Borzov A.U.      ***** 06/11/15 ***
*                                                                  *
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*       ОПИСАНИЕ КЛАССА ДОСТУПА К ОБ'ЕКТУ "ЭЛЕКТРОННАЯ КАРТА"      *
*        ИНТЕРФЕЙС ДЛЯ ПРОГРАММ НА ЯЗЫКАХ C, PASCAL, BASIC         *
*                                                                  *
********************************************************************
*                                                                  *
*    Импортирование функций в Windows :                            *
*                                                                  *
*    // Загрузка библиотеки                                        *
*    HINSTANCE libInst = ::LoadLibrary("gisacces.dll");            *
*                                                                  *
*    // Вызов функции                                              *
*    HMAP (WINAPI * lpfn_OpenMap)(const char*, long int);          *
*    (FARPROC)lpfn_OpenMap = GetProcAddress(libInst, "OpenMap");   *
*    HMAP hMap = (*lpfn_OpenMap)("Noginsk.map",0);                 *
*                                                                  *
*    // Выгрузка библиотеки                                        *
*    ::FreeLibrary(libInst);                                       *
*                                                                  *
*******************************************************************/

#ifndef RSTAPI_H
#define RSTAPI_H

#ifndef MAPTYPE_H
  #include "maptype.h"   // Описание структур интерфейса MAPAPI
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К РАСТРОВОЙ КАРТЕ ++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern "C"
{

  // Открыть растровые данные
  // Возвращает идентификатор открытой растровой карты (TMapAccess*)
  // rstname - имя файла растровой карты
  // mode    - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenRst(const char *rstname, long int mode = 0);
_MAPIMP HMAP _MAPAPI mapOpenRstUn(const WCHAR *rstname,long int mode);

  // Открыть растровые данные в заданном районе работ
  // (добавить в цепочку растров)
  // Возвращает номер файла в цепочке растров
  // hMap    - идентификатор открытой векторной карты
  // rstname - имя файла растровой карты
  // mode    - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenRstForMap(HMAP hMap, const char *rstname, long int mode);
_MAPIMP long int _MAPAPI mapOpenRstForMapUn(HMAP hMap, const WCHAR *name, long int mode);

  // Создание файла растрового изображения
  // rstname    - имя создаваемого файла
  // width      - ширина растрового изображения в элементах
  // height     - высота растрового изображения в элементах
  // nbits      - размер элемента (бит на пиксел)
  // palette    - адрес устанавливаемой палитры
  // colorcount - число элементов в новой палитре
  // scale      - масштаб
  // precision  - разрешение растра
  // При успешном завершении функция создает файл rstname с заполненным
  // паспортом и палитрой растра.
  // При ошибке возвращает 0

_MAPIMP HMAP _MAPAPI mapCreateRst(const char *rstname, long int width, long int height,
                                  long int nbits, COLORREF* palette, long int colorcount,
                                  double scale = 0, double precision = 0);
_MAPIMP HMAP _MAPAPI mapCreateRstUn(const WCHAR *rstname, long int width, long int height,
                                    long int nbits, COLORREF *palette, long int colorcount,
                                    double scale, double precision);

  // Создание файла растровой карты
  // name            - полное имя растра
  // width           - ширина изображения в пикселях
  // height          - высота изображения в пикселях
  // nbits           - количество бит на пиксель (1,4,8,24)
  // palette         - указатель на палитру растра (справедливо для 1,4,8 бит на пиксель)
  // scale           - мастаб растра
  // precisionMet    - разрешения растра (точек на метр)
  // meterInElementX - размер пикселя растра в метрах на местности по оси X (по вертикали)
  // meterInElementY - размер пикселя растра в метрах на местности по оси Y (по горизонтали)
  //                   meterInElementX и meterInElementY могут иметь разные значения
  // location        - координаты юго-западного угла растра в метрах, соответствующие СК в mapregister
  // mapregister     - проекции исходного материала
  // Важно!!!
  // Для корректного открытия растров в 10-ой и более ранних версиях необходимо выполнить условие:
  //                            meterInElementX = scale/precisionMet;
  // Иначе масштаб и разрешение будут пересчитаны!!!
  // При ошибке возвращает 0

_MAPIMP HMAP _MAPAPI mapCreateRstEx(char *rstname, long int width, long int height, long int nbits,
                                    COLORREF* palette, long int colorcount, double scale, double precisionMet,
                                    double meterInElementX, double meterInElementY,
                                    DOUBLEPOINT *location, MAPREGISTEREX *mapregister);
_MAPIMP HMAP _MAPAPI mapCreateRstExUn(WCHAR *rstname, long int width, long int height, long int nbits,
                                      COLORREF* palette, long int colorcount, double scale,
                                      double precisionMet, double meterInElementX, double meterInElementY,
                                      DOUBLEPOINT *location, MAPREGISTEREX* mapregister);

  // Создание файла растровой карты
  // name            - полное имя растра
  // width           - ширина изображения в пикселях
  // height          - высота изображения в пикселях
  // nbits           - количество бит на пиксель (1,4,8,24)
  // palette         - указатель на палитру растра (справедливо для 1,4,8 бит на пиксель)
  // meterInElementX - размер пикселя растра в метрах на местности по оси X (по вертикали)
  // meterInElementY - размер пикселя растра в метрах на местности по оси Y (по горизонтали)
  //                   meterInElementX и meterInElementY могут иметь разные значения
  // location        - координаты юго-западного угла растра в метрах, соответствующие СК в mapregister
  // mapregister     - указатель на структуру, содержащую параметры проекции исходного материала
  // datumparam      - указатель на структуру, содержащую коэффициенты трансформирования геодезических координат
  // ellipsoidparam  - указатель на структуру, содержащую параметры эллипсоида
  // При ошибке возвращает 0

_MAPIMP  HMAP _MAPAPI mapCreateRaster(const char *rstname, long int width, long int height, long int nbits,
                                      COLORREF* palette, long int colorcount, double meterInElementX,
                                      double meterInElementY, DOUBLEPOINT *location,
                                      MAPREGISTEREX* mapregister, DATUMPARAM * datum, ELLIPSOIDPARAM * ellipsoid);
_MAPIMP HMAP _MAPAPI mapCreateRasterUn(const WCHAR *rstname, long int width, long int height, long int nbits,
                                       COLORREF *palette, long int colorcount, double meterInElementX,
                                       double meterInElementY, DOUBLEPOINT *location, MAPREGISTEREX *mapregister,
                                       DATUMPARAM *datum, ELLIPSOIDPARAM *ellipsoid);

  // Создание файла растрового изображения
  // hMap       - идентификатор открытой векторной карты
  // rstname    - имя создаваемого файла
  // width      - ширина растрового изображения в элементах
  // height     - высота растрового изображения в элементах
  // nbits      - размер элемента (бит на пиксел)
  // palette    - адрес устанавливаемой палитры
  // colorcount - число элементов в новой палитре
  // scale      - масштаб
  // precision  - разрешение растра
  // location   - привязка юго-западного угла растра в районе(в метрах)
  // При успешном завершении функция создает файл rstname с заполненным
  // паспортом и палитрой растра и добавляет его в цепочку растров
  // открытой векторной карты (hMap).
  // Возвращает  номер файла (начиная с 1) в цепочке растров открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateAndAppendRst(HMAP hMap,const char *rstname, long int width, long int height,
                                               long int nbits, COLORREF* palette, long int colorcount,
                                               double scale, double precision, DOUBLEPOINT location);
_MAPIMP long int _MAPAPI mapCreateAndAppendRstUn(HMAP hMap, const WCHAR *rstname, long int width, long int height,
                                                 long int nbits, COLORREF *palette, long int colorcount,
                                                 double scale, double precision, DOUBLEPOINT location);

  // Создание файла растрового изображения
  // hMap       - идентификатор открытой векторной карты
  // rstname    - имя создаваемого файла
  // width      - ширина растрового изображения в элементах
  // height     - высота растрового изображения в элементах
  // nbits      - размер элемента (бит на пиксел)
  // palette    - адрес устанавливаемой палитры
  // colorcount - число элементов в новой палитре
  // scale      - масштаб
  // precision  - разрешение растра (точек на метр)
  // meterInElementX - размер пикселя растра в метрах на местности по оси X (по вертикали)
  // meterInElementY - размер пикселя растра в метрах на местности по оси Y (по горизонтали)
  //                   meterInElementX и meterInElementY могут иметь разные значения
  // location        - координаты юго-западного угла растра в метрах, соответствующие СК в mapregister
  // mapregister     - проекции исходного материала
  // Важно!!!
  // Для корректного открытия растров в 10-ой и более ранних версиях необходимо выполнить условие:
  //                            meterInElementX = scale/precision;
  // Иначе масштаб и разрешение будут пересчитаны!!!
  // При успешном завершении функция создает файл rstname с заполненным
  // паспортом и палитрой растра и добавляет его в цепочку растров
  // открытой векторной карты (hMap).
  // Возвращает  номер файла (начиная с 1) в цепочке растров открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapCreateAndAppendRstEx(HMAP hMap,const char * rstname,
                                                  long int width, long int height, long int nbits,
                                                  COLORREF* palette, long int colorcount,
                                                  double scale, double precision,
                                                  double meterInElementX, double meterInElementY,
                                                  DOUBLEPOINT* location, MAPREGISTEREX* mapregister);
_MAPIMP long int _MAPAPI mapCreateAndAppendRstExUn(HMAP hMap, const WCHAR *rstname, long int width,
                                                   long int height, long int nbits, COLORREF* palette,
                                                   long int colorcount, double scale, double precision,
                                                   double meterInElementX, double meterInElementY,
                                                   DOUBLEPOINT* location, MAPREGISTEREX* mapregister);

  // Создание файла растрового изображения
  // hMap       - идентификатор открытой векторной карты
  // rstname    - имя создаваемого файла
  // width      - ширина растрового изображения в элементах
  // height     - высота растрового изображения в элементах
  // nbits      - размер элемента (бит на пиксел)
  // palette    - адрес устанавливаемой палитры
  // colorcount - число элементов в новой палитре
  // scale      - масштаб
  // precision  - разрешение растра (точек на метр)
  // meterInElementX - размер пикселя растра в метрах на местности по оси X (по вертикали)
  // meterInElementY - размер пикселя растра в метрах на местности по оси Y (по горизонтали)
  //                   meterInElementX и meterInElementY могут иметь разные значения
  // location        - координаты юго-западного угла растра в метрах, соответствующие СК в mapregister
  // mapregister     - указатель на структуру, содержащую параметры проекции исходного материала
  // datumparam      - указатель на структуру, содержащую коэффициенты трансформирования геодезических координат
  // ellipsoidparam  - указатель на структуру, содержащую параметры эллипсоида
  // Важно!!!
  // Для корректного открытия растров в 10-ой и более ранних версиях необходимо выполнить условие:
  //                            meterInElementX = scale/precision;
  // Иначе масштаб и разрешение будут пересчитаны!!!
  // При успешном завершении функция создает файл rstname с заполненным
  // паспортом и палитрой растра и добавляет его в цепочку растров
  // открытой векторной карты (hMap).
  // Возвращает  номер файла (начиная с 1) в цепочке растров открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCreateAndAppendRaster(HMAP hMap, const char *rstname, long int width,
                                                  long int height, long int nbits, COLORREF *palette,
                                                  long int colorcount, double meterInElementX,
                                                  double meterInElementY,
                                                  DOUBLEPOINT *location, MAPREGISTEREX *mapregister,
                                                  DATUMPARAM *datum, ELLIPSOIDPARAM *ellipsoid);
_MAPIMP long int _MAPAPI mapCreateAndAppendRasterUn(HMAP hMap, const WCHAR *rstname, long int width,
                                                    long int height, long int nbits, COLORREF *palette,
                                                    long int colorcount, double meterInElementX,
                                                    double meterInElementY, DOUBLEPOINT *location,
                                                    MAPREGISTEREX *mapregister, DATUMPARAM *datum,
                                                    ELLIPSOIDPARAM *ellipsoid);

  // Оценка теорeтической длины файла растровой карты до ее создания
  // hMap       - идентификатор открытой векторной карты
  // width      - ширина растрового изображения в элементах
  // height     - высота растрового изображения в элементах
  // nbits      - размер элемента (бит на пиксел)
  // Возвращает  теорeтическую длину файла растровой карты (Байт)
  // При ошибке возвращает ноль

_MAPIMP  double _MAPAPI mapRstFileLengthCalculation(HMAP hMap, long int width,
                                                    long int height, long int nbits);
  // Устаревшая функция
_MAPIMP  double _MAPAPI mapRtsFileLengthCalculation(HMAP hMap, long int width,
                                                    long int height, long int nbits);

  // Запросить длину в байтах файла растровой карты
  // Максимальный размер файла 8 ГБ.
  // Растровая карта размером более 4Gb состоит из 2-х файлов: *.rsw и *.rsw.01
  // hMap       - идентификатор открытой карты
  // number - номер растрового файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP __int64 _MAPAPI mapRstFileLength(HMAP hMap, long int number);

  // Запросить номер растра в цепочке по имени файла
  // name        - имя файла растра
  // В цепочке номера растров начинаются с 1.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstNumberByName(HMAP hMap, const char *name);
_MAPIMP long int _MAPAPI mapGetRstNumberByNameUn(HMAP hMap, const WCHAR* name);

  // Закрыть растровые данные в заданном районе работ
  // hMap   - идентификатор открытой векторной карты
  // number - номер растрового файла в цепочке
  // Если number равен 0, закрываются все растровые данные
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCloseRstForMap(HMAP hMap, long int number);

  // Закрыть растровые данные
  // hMap   - идентификатор открытой векторной карты
  // number - номер закрываемого растра
  // если number == 0, закрываются все растры в окне

_MAPIMP void _MAPAPI mapCloseRst(HMAP hMap, long int number = 0);

  // Закрыть растровые данные и удалить файл
  // hMap   - идентификатор открытой векторной карты
  // number - номер растрового файла в цепочке
  // Если number равен 0, закрываются и удалаются все растровые данные
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteRst(HMAP hMap, long int number);

  // Запросить имя файла растровых данных
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает пустую строку

_MAPIMP const char* _MAPAPI mapGetRstName(HMAP hMap, long int number);

  // Запросить имя файла растровых данных в кодировке UNICODE
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // name - адрес строки для размещения результата
  // size - размер строки
  // При ошибке возвращает пустую строку

_MAPIMP long int _MAPAPI mapGetRstNameUn(HMAP hMap, long int number,
                                         WCHAR *name, long int size);

  // Запросить число открытых файлов растровых данных
  // hMap    - идентификатор открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRstCount(HMAP hMap);

  // Запрос текущего номера растра
  // hMap    - идентификатор открытой векторной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCurrentNumber(HMAP hMap);

  // Установка текущего номера растра
  // hMap    - идентификатор открытой векторной карты
  // number  - номер растра
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstCurrentNumber(HMAP hMap, long int number);

  // Очистить кэш растровых данных, открытых на ГИС Сервере
  // hMap    - идентификатор открытой векторной карты
  // number  - номер растра, для которого нужно очистить кэш, или -1 (все растры)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapClearRstCache(HMAP hMap, long int number);

  // Запросить время крайнего редактирования растра
  // hMap    - идентификатор открытой векторной карты
  // number  - номер растра
  // Возвращает системное время редактирования (создания) по Гринвичу
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstSystemTime(HMAP hMap, long int number, SYSTEMTIME *time);

  // Запросить/Установить степень видимости растра
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  view = 0 - не виден
  //  view = 1 - полная видимость
  //  view = 2 - насыщенная
  //  view = 3 - полупрозрачная
  //  view = 4 - средняя
  //  view = 5 - прозрачная

_MAPIMP  long int _MAPAPI mapGetRstView(HMAP hMap,long int number);
_MAPIMP  long int _MAPAPI mapSetRstView(HMAP hMap,long int number,
                                        long int view);

  // Запросить прозрачность палитры растра                   // 19/01/14
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstTransparent(HMAP hMap, long int number);

  // Установить прозрачность палитры растра                   // 19/01/14
  // hMap   - идентификатор открытых данных
  // transparent - прозрачность в процентах от 0 до 100
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstTransparent(HMAP hMap, long int number,
                                              long int transparent);


  // Установить степень видимости группы растров
  //  userLabel  - пользовательская метка растра
  //                -1             - все растры
  //                RSW_QUALITY    - растры качеств
  //                RSW_VISIBILITY - растры видимости
  //                RSW_WEB        - растровые карты для WEB
  //  view - степень видимости
  //                0 - не виден
  //                1 - полная
  //                2 - насыщенная
  //                3 - полупрозрачная
  //                4 - средняя
  //                5 - прозрачная
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRstGroupView(HMAP hMap, long int userLabel, long int view);

  // Запросить/Установить порядок отображения растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер растрового файла в цепочке
  //  (0 - под картой, 1 - над картой)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRstViewOrder(HMAP hMap, long int number, long int order);
_MAPIMP  long int _MAPAPI mapGetRstViewOrder(HMAP hMap, long int number);

  // Поменять очередность отображения растров (rst) в цепочке
  // Последний растр в цепочке отображается в последнюю очередь
  // Нумерация растров в цепочке начинается с 1 и заканчивается номером mapGetRstCount(..)
  // oldNumber - номер файла в цепочке
  // newNumber - устанавливаемый номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapChangeOrderRstShow(HMAP hMap, long int oldNumber, long int newNumber);
_MAPIMP long int _MAPAPI mapChangOrderRstInChain(HMAP hMap, long int oldNumber, long int newNumber);

  // Последовательный просмотр растров над картой
  // Если все растры отображаются под картой, то
  // первый растр будет отображен над картой. При следующем
  // вызове второй растр будет отображен над картой, остальные -
  // под картой. После последнего растра в списке над картой -
  // все растры под картой. Далее - опять первый растр над картой.
  // Возвращает номер растра отображаемого над картой или ноль.
  // Для получения результата на экране - карту нужно перерисовать
  // hMap    - идентификатор открытой векторной карты

_MAPIMP long int _MAPAPI mapTurnRstViewOrder(HMAP hMap);

  // Запросить количество цветов в палитре растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращается 0

_MAPIMP long int _MAPAPI mapGetRstColorCount (HMAP hMap, long int number);

  // Запросить описание палитры растра
  // hMap    - идентификатор открытой векторной карты
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  //           (если count > 256, то возвращается ноль)
  // number  - номер файла в цепочке
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPalette(HMAP hMap, COLORREF *palette,
                                          long int count, long int number);

  // Установить описание палитры растра
  // hMap    - идентификатор открытой векторной карты
  // palette - адрес устанавливаемой палитры
  // count   - число элементов в новой палитре
  // number  - номер файла в цепочке
  // Если palette равно 0, устанавливается палитра из заголовка
  // count не более 256
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstPalette(HMAP hMap, COLORREF *palette,
                                          long int count, long int number);

  // Запросить описание эталонной палитры растра
  // (без учета яркости и контрасности)
  // hMap    - идентификатор открытой векторной карты
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  //           (если count > 256, то возвращается ноль)
  // number  - номер файла в цепочке
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstStandardPalette(HMAP hMap, COLORREF *palette,
                                                  long int count, long int number);

  // Запросить описание палитры из заголовка растра
  // hMap   - идентификатор открытой векторной карты
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  //           (если count > 256, то возвращается ноль)
  // number  - номер файла в цепочке
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPaletteFromHeader(HMAP hMap, COLORREF *palette,
                                                    long int count, long int number);

  // Запросить яркость палитры растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstBright(HMAP hMap, long int number);

  // Запросить контрастность палитры растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstContrast(HMAP hMap, long int number);

  // Запросить параболическую яркость растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstGamma(HMAP hMap, long int number);

  // Установить яркость палитры растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // bright - яркость
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstBright(HMAP hMap, long int bright, long int number);

  // Установить контрастность палитры растра
  // hMap     - идентификатор открытой векторной карты
  // number   - номер файла в цепочке
  // contrast - контраст
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstContrast(HMAP hMap, long int contrast, long int number);

  // Установить параболическую яркость растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // gamma  - параболическая яркость
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstGamma(HMAP hMap, long int gamma, long int number);

  //  Обновить активную палитру с нулевой яркостью и контрастностью
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapRestoreRstPalette(HMAP hMap, long int number);

  // Запросить значение инверсии растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // Если изображение растра позитивное - возвращает ноль
  // Если изображение растра негативное - возвращает 1
  // При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapCheckInversionRst(HMAP hMap, long int number);

  // Инвертировать растровую карту
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // value:
  // 0 - установить изображение растра позитивным
  // 1 - установить изображение растра негативным
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapInvertRst(HMAP hMap, long int number, long int value);

  // Запросить видимость цвета (для 16- и 256-цветных растров)
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  index  - индекс цвета в палитре растра(начиная с 0)
  //  Возвращает: 1 - цвет с данным индексом отображается
  //              0 - цвет с данным индексом не отображается
  //  При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapCheckVisibilityColor(HMAP hMap, long int number, long int index);

  // Установить видимость цвета (для 16- и 256-цветных растров)
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  index  - индекс цвета в палитре растра(начиная с 0)
  //  value: 1 - включить отображение цвета с данным индексом
  //         0 - отключить отображение цвета с данным индексом
  //  Сохранение видимости цветов в INI-файле (не заносится в заголовк файла растра)
  //  При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetVisibilityColor(HMAP hMap, long int number,
                                               long int index, long int value);

  // Установить видимость цвета (для 16- и 256-цветных растров)
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  index  - индекс цвета в палитре растра(начиная с 0)
  //  value: 1 - включить отображение цвета с данным индексом
  //         0 - отключить отображение цвета с данным индексом
  //  Сохранение видимости цветов в заголовке файла растра(а также в INI-файле)
  //  При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetVisibilityColorInRstFile(HMAP hMap, long int number,
                                                        long int index, long int value);

  // Установить прозрачный цвет растра (для 16-,24-,32-битных растров)
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  //  color  - значение прозрачного цвета в формате RGB (от 0 до 0x00FFFFFF)
  // При установке IMGC_TRANSPARENT (0xFFFFFFFF) прозрачный цвет не используется
  // При ошибке возвращает IMGC_TRANSPARENT

_MAPIMP COLORREF _MAPAPI mapSetRstTransparentColor(HMAP hMap, long int number, COLORREF color);

  // Запросить прозрачный цвет растра (для 16-,24-,32-битных растров)
  //  hMap   - идентификатор открытой векторной карты
  //  number - номер файла в цепочке
  // Возвращает цвет в формате RGB (от 0 до 0x00FFFFFF)
  // При возврате IMGC_TRANSPARENT (0xFFFFFFFF) прозрачный цвет не используется
  // При ошибке возвращает IMGC_TRANSPARENT

_MAPIMP COLORREF _MAPAPI mapGetRstTransparentColor(HMAP hMap, long int number);

  // Запросить типа и шаг маски растра
  // hMap      - идентификатор открытой векторной карты
  // maskType  - типа маски(0 - маска отсутствует)
  // maskStep  - шаг маски
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstMaskType(HMAP hMap, long int number,
                                           long int *maskType, long int *maskStep);

  // Установить типа и шаг маски растра
  // hMap      - идентификатор открытой векторной карты
  // maskType  - типа маски(0 - маска отсутствует)
  // maskStep  - шаг маски
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstMaskType(HMAP hMap,long int number,
                                           long int maskType, long int maskStep);

  // Установка взаимосвязанных параметров растра
  // scale           - знаменатель масштаба
  // precision       - разрешение (точек на метр)
  // meterinelementX - количества метров на элемент  ПО ОСИ X
  // meterinelementY - количества метров на элемент  ПО ОСИ Y
  // meterinelementX и meterinelementY могут отличаться
  // Важно!!!
  // Для правильного отображения растров в 10-ой и более ранних версиях необходимо:
  //                         meterinelementX = scale / precision;
  // Если условие не выполняется, то meterinelementX и meterinelementY игнорируются и расчитываются по формуле.

_MAPIMP long int _MAPAPI mapSetRstParameters(HMAP hMap, long int number,
                                             double scale, double precision,
                                             double meterinelementX,
                                             double meterinelementY);

  // Установить масштаб растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // scale      - знаменатель масштаба
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstScale(HMAP hMap, long int number, double scale);

  // Запросить масштаб растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // scale      - указатель переменной, куда вносится значение масштаба
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstScale(HMAP hMap, long int number, double *scale);


  // Запросить значения масштаба нижней и верхней границ видимости растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // По адресу bottomScale записывается знаменатель масштаба нижней границы видимости растра
  // По адресу topScale записывается знаменатель масштаба верхней границы видимости растра
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstRangeScaleVisible(HMAP hMap, long int number,
                                                    long int *bottomScale, long int *topScale);

  // Установить значения масштаба нижней и верхней границ видимости растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // bottomScale   - знаменатель масштаба нижней границы видимости растра
  // topScale   - знаменатель масштаба верхней границы видимости растра
  //              bottomScale <= topScale, иначе возвращает 0
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstRangeScaleVisible(HMAP hMap, long int number,
                                                    long int bottomScale, long int topScale);

  // Установить разрешение растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // precision  - разрешение растра, полученное при сканировании (точек на метр)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstPrecision(HMAP hMap, long int number, double precision);

  // Запросить разрешение растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // precision  - разрешение растра (точек на метр)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstPrecision(HMAP hMap, long int number, double *precision);

  // Установить рамку растра по метрике замкнутого объекта
  // Замкнутый объект должен иметь не менее 4-х точек
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // info       - замкнутый объект карты
  // После выполнения функции отображение растра ограничится заданной областью
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstBorder(HMAP hMap, long int number, HOBJ info);

  // Установить рамку растра по метрике замкнутого объекта
  // Замкнутый объект должен иметь не менее 4-х точек
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // info       - замкнутый объект карты
  // flagSubject- флаг использования подобъектов объекта при установке рамки растра (0/1)
  //              0 - в качестве рамки растра устанавливается контур объекта
  //              1 - в качестве рамки растра устанавливается контур объекта с подобъектами
  // После выполнения функции отображение растра ограничится заданной областью
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstBorderEx(HMAP hMap, long int number, HOBJ info,
                                           long int flagSubject);

  // Запросить объект рамки растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // info       - идентификатор объекта рамки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBorder(HMAP hMap, long int number, HOBJ info);

  // Удалить рамку растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // После выполнения функции отображение растра будет полным
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDeleteRstBorder(HMAP hMap, long int number);

  // Определение существования маски растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstMask(HMAP hMap, long int number);

  // Определение способа отображения растра(относительно маски)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // Возвращает 1 - при отображении растра по маске
  //            0 - при отображении растра без учета маски
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetShowRstByMask(HMAP hMap, long int number);

  // Установка отображения растра по маске
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // value  = 1 - отобразить растр по маске
  //        = 0 - отобразить растр без учета маски

_MAPIMP long int _MAPAPI mapSetShowRstByMask(HMAP hMap, long int number, long int value);

  // Определение существования рамки растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCheckExistenceRstBorder(HMAP hMap, long int number);

  // Установка отображения растра по рамке
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // value  = 1 - отобразить растр по рамке
  //        = 0 - отобразить растр без учета рамки

_MAPIMP long int _MAPAPI mapShowRstByBorder(HMAP hMap, long int number, long int value);

  // Определение способа отображения растра(относительно рамки)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // Возвращает 1 - при отображении растра по рамке
  //            0 - при отображении растра без учета рамки
  // При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapCheckShowRstByBorder(HMAP hMap, long int number);

  // Определить координаты точки рамки, которая
  // входит в прямоугольник Габариты растра(матрицы) и
  // имеет наименьшее удаление от точки pointIn (координаты в метрах).
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // По адресу pointOut записываются координаты найденной точки в метрах
  // При ошибке или отсутствии рамки возвращает 0

_MAPIMP long int _MAPAPI mapGetImmediatePointOfRstBorder(HMAP hMap, long int number,
                                                         DOUBLEPOINT* pointIn,
                                                         DOUBLEPOINT* pointOut);

  // Запрос - поддерживается ли пересчет к геодезическим
  // координатам из плоских прямоугольных и обратно
  // hMap     - идентификатор открытой основной карты
  // number   - номер растра
  // Если нет - возвращает ноль

  _MAPIMP long int _MAPAPI mapIsRstGeoSupported(HMAP hMap, long int number);

  // Запросить данные о проекции растра
  // number      - номер файла в цепочке
  // mapregister - адрес структуры, в которой будут размещены
  // данные о проекции
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstProjectionData(HMAP hMap, int number,
                                                 MAPREGISTEREX* mapregister);

  // Запросить данные о проекции растра по имени файла
  // name        - имя файла растра
  // mapregister - адрес структуры, в которой будут размещены
  // данные о проекции
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstProjectionDataByName(const char *name,
                                                       MAPREGISTEREX* mapregister);
_MAPIMP long int _MAPAPI mapGetRstProjectionDataByNameUn(const WCHAR *name,
                                                         MAPREGISTEREX* mapregister);

  // Установить данные о проекции растра
  // number - номер файла в цепочке
  // mapregister - адрес структуры
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapSetRstProjectionData(HMAP hMap, int number,
                                                  MAPREGISTEREX* mapregister);

  // Запросить параметры эллипсоида растра
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла растра в цепочке
  // ellipsoidparam - адрес структуры, в которой будут размещены
  // параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstEllipsoidParam(HMAP hMap, long int number,
                                                 ELLIPSOIDPARAM *ellipsoidparam);

  // Установить параметры эллипсоида растра
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла растра в цепочке.
  // ellipsoidparam - адрес структуры, содержащей параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstEllipsoidParam(HMAP hMap, long int number,
                                                 const ELLIPSOIDPARAM *ellipsoidparam);

  // Запросить коэффициенты трансформирования геодезических координат растра
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла растра в цепочке
  // datumparam - адрес структуры, в которой будут размещены
  // коэффициенты трансформирования геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstDatumParam(HMAP hMap, long int number,
                                             DATUMPARAM *datumparam);

  // Установить коэффициенты трансформирования геодезических координат растра
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла растра в цепочке.
  // datumparam - адрес структуры, содержащей коэффициенты трансформирования
  // геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstDatumParam(HMAP hMap, long int number,
                                             const DATUMPARAM *datumparam);

  // Запросить габариты растра в метрах в районе работ
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // frame      - возвращаемые габариты растра
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstFrameMeters(HMAP hMap, DFRAME *frame, long int number);

  // Запросить фактические габариты отображаемого растра в метрах в районе работ
  // При отображение растра по рамке возвращаются габариты рамки
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // frame      - возвращаемые габариты растра
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActualRstFrame(HMAP hMap, DFRAME *frame, long int number);

  // Установить привязку растра в районе работ(в метрах)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // location   - координаты юго-западного угла растра(в метрах)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstLocation(HMAP hMap, long int number, DOUBLEPOINT *location);

  // Запросить привязку растра в районе работ(в метрах)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // location   - координаты юго-западного угла растра(в метрах)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstLocation(HMAP hMap, long int number, DOUBLEPOINT *location);

  // Запросить флаг существования привязки растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCheckExistenceRstLocation(HMAP hMap, long int number);

  // Запросить - может ли растр копироваться или экспортироваться
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // Если нет - возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCopyFlag(HMAP hMap, long int number);

  //**********************************************************************
  // Как подвинуть растр на отрезок (Dx,Dy)?
  //    Запросите привязку растра - mapGetRstLocation(...)
  //    Измените точку привязки на отрезок (Dx,Dy)
  //    Установите новую точку привязки - mapSetRstLocation(...)
  //    Перерисуйте окно.
  //**********************************************************************
  // Как подвинуть растр с изменением масштаба ?
  //    Запросите привязку растра - mapGetRstLocation(...)
  //    Запросите знаменатель масштаба растра - mapGetRstScale(...)
  //    Измените точку привязки и расчитайте знаменатель масштаба
  //    Установите новую точку привязки - mapSetRstLocation(...)
  //    Установите новый знаменатель масштаба - mapSetRstScale(...)
  //    Перерисуйте окно.
  //**********************************************************************

  // Устаревшая функция
  // Запросить размер элемента растра в метрах на местности
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // metinelem  - размер элемента растра в метрах на местности
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstMeterInElement(HMAP hMap, long int number, double *metinelem);

  // Запросить размер элемента растра в метрах по оси X
  // number    - номер файла в цепочке
  // metinelemX - размер элемента растра в метрах на местности по оси X
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstMeterInElementX(HMAP hMap, long int number, double *metinelemX);

  // Запросить размер элемента растра в метрах по оси Y
  // number    - номер файла в цепочке
  // metinelemY - размер элемента растра в метрах на местности по оси Y
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstMeterInElementY(HMAP hMap, long int number, double *metinelemY);

  // Устаревшая функция
  // Запросить размер точки экрана в элементах растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // eleminpix  - размер точки экрана в элементах растра
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetSizeRstElemInPix(HMAP hMap, long int number, double *eleminpix);

  // Запросить размер элемента растра в пикселах экрана по оси X
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetSizeRstElemXInPix(HMAP hMap, long int number, double *eleminpix);

  // Запросить размер элемента растра в пикселах экрана по оси Y
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetSizeRstElemYInPix(HMAP hMap, long int number, double *eleminpix);

  // Запросить ширину растра в элементах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstWidth(HMAP hMap, long int number);

  // Запросить высоту растра в элементах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstHeight(HMAP hMap, long int number);

  // Запросить объем растра в байтах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP unsigned long int _MAPAPI mapGetRstLength(HMAP hMap, long int number);

  // Запросить размер элемента растра в битах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstElementSize(HMAP hMap, long int number);

  // Запросить флаг редактируемости растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstEdit(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapCheckRstEdit(HMAP hMap, long int number);

  // Запросить номер алгоритма сжатия растра (0 - растр не сжат, 1 - LZW, 2 - JPEG)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckRstCompressNumber(HMAP hMap, long int number);

  // Установить в заголовок растра номер алгоритма сжатия(0 - растр не сжат, 1 - LZW, 2 - JPEG)
  // ВНИМАНИЕ: Функция не выполняет сжатие изображения
  // Для сжатия изображения по методу LZW воспользуйтесь функцией mapCompressLZW(), объявленной в mapapi.h
  // Для сжатия изображения по методу JPEG воспользуйтесь функцией mapCompressJPEG(), объявленной в mapapi.h
  // hMap       - идентификатор открытой карты
  // number     - номер файла в цепочке
  // value      - номер алгоритма сжатия(0 - растр не сжат, 1 - LZW, 2 - JPEG)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstCompressNumber(HMAP hMap, long int number, int value);

  // Запросить степень сжатия блока растра по алгоритму JPEG
  // hMap       - идентификатор открытой карты
  // number - номер файла в цепочке
  // Возвращает степень сжатия изображения блока растра по алгоритму JPEG
  //          (1-100, 1-максимальное сжатие, 100-сжатие без потери качества),
  //          рекомендуемое значение 60.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCompressJpegQuality(HMAP hMap, long int number);

  // Установить в заголовок растра степень сжатия блока растра по алгоритму JPEG
  // Используйте для установки в заголовок растра номера алгоритма сжатия функцию mapSetRstCompressNumber()
  // ВНИМАНИЕ: Функция не выполняет сжатие изображения
  // Для сжатия изображения по методу JPEG воспользуйтесь функцией mapCompressJPEG(), объявленной в mapapi.h
  // hMap       - идентификатор открытой карты
  // number - номер файла в цепочке
  // value  - степень сжатия изображения блока растра по алгоритму JPEG
  //          (1-100, 1-максимальное сжатие, 100-сжатие без потери качества),
  //          рекомендуемое значение 60.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstCompressJpegQuality(HMAP hMap, long int number, int value);

  // Чтение элемента по абсолютным индексам
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string, column - строка и столбец элемента
  // value      - значение элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPoint(HMAP hMap, long int number, long int *value,
                                        long int string, long int column);

  // Запись элемента по абсолютным индексам
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string, column - строка и столбец элемента
  // value      - значение элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutRstPoint(HMAP hMap, long int number,long int value,
                                        long int string, long int column);

  // Чтение элемента  по его прямоугольным координатам (в метрах) из буфера
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // x,y        - координаты элемента
  // value      - значение элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPlanePoint(HMAP hMap, long int number,
                                             long int *value, double x, double y);

  // Чтение элемента по его прямоугольным координатам по методу треугольников
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // x,y        - координаты элемента
  // value      - значение элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPlanePointTriangle(HMAP hMap, long int number,
                                                     long int *value, double x, double y);


  // Определение цвета точки растра по прямоугольным координатам точки
  // (в метрах)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // x,y        - координаты элемента
  // color      - цвет элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstPlanePointColor(HMAP hMap, long int number,
                                                  COLORREF *color, double x, double y);

 // Билинейная интерполяция - определение цвета точки по 4 соседним пикселям растра
 // hMap       - идентификатор открытой векторной карты
 // number     - номер файла в цепочке
 // color      - заполняется вычисленным цветом
 // indexColor - заполняется индексом ближайшего цвета к вычисленному из палитры растра
 //              (для 1,4 и 8 бит на пиксель)
 // x, y       - прямоугольные координаты точки в системе координат растра в метрах на местности
 // При попадании в крайние пиксели растра возвращается цвет ближайшего пикселя
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBilinearInterpolationColor(HMAP hMap, long int number, COLORREF* color,
                                                             long int *indexcolor, double x, double y);

 // Бикубическая интерполяция - определение цвета точки по 16 соседним пикселям растра
 // hMap       - идентификатор открытой векторной карты
 // number     - номер файла в цепочке
 // color      - заполняется вычисленным цветом
 // indexColor - заполняется индексом ближайшего цвета к вычисленному из палитры растра
 //              (для 1,4 и 8 бит на пиксель)
 // x, y       - прямоугольные координаты точки в системе координат растра в метрах на местности
 // При попадании в крайних 2 пикселя растра возвращается цвет ближайшего пикселя
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBicubicInterpolationColor(HMAP hMap, long int number, COLORREF* color,
                                                            long int *indexcolor, double x, double y);

  // Запись элемента  по его прямоугольным координатам
  // (в метрах) в буфер
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // x,y        - координаты элемента
  // value      - значение элемента
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutRstPlanePoint(HMAP hMap, long int number, long int value,
                                             double x, double y);

  // Запись отрезка в изображение основного растра по прямоугольным координатам (в метрах)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // color  - цвет отрезка типа COLORREF для растров с 16,24,32 точек на пиксель;
  //          индекс цвета в палитре для растров с 1,4,8 точек на пиксель.
  // point1 - координаты начальной точки отрезка
  // point2 - координаты конечной точки отрезка
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutRstPlaneLine(HMAP hMap, long int number, long color,
                                            DOUBLEPOINT *point1, DOUBLEPOINT *point2);


  // Запросить количество блоков растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockCount(HMAP hMap, long int number);

  // Запросить число строк блоков растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockRow(HMAP hMap, long int number);


  // Запросить число столбцов блоков растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockColumn(HMAP hMap, long int number);

  // Запросить размер неусеченного блока растра в байтах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockSize(HMAP hMap, long int number);

  // Запросить размер текущего блока {string,column} растра в байтах
  // (с учетом усеченных блоков)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string, column - координаты блока
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCurrentBlockSize(HMAP hMap,long int number,
                                                   long int string, long int column);

  // Запросить ширину неусеченного блока растра в элементах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockWidth(HMAP hMap, long int number);

  // Запросить высоту неусеченного блока растра в элементах
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBlockHeight(HMAP hMap, long int number);

  // Запросить ширину текущего блока {string,column} растра в элементах
  // (с учетом усеченных блоков)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // column     - столбец блока
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCurrentBlockWidth(HMAP hMap, long int number, long int column);

  // Запросить высоту текущего блока {string,column} растра в элементах
  // (с учетом усеченных блоков)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstCurrentBlockHeight(HMAP hMap, long int number, long int string);

  // Запросить адрес блока растра по номеру строки и столбца
  // Блоки последнего ряда могут иметь усеченный размер
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // column     - столбец блока
  // При запросе следующего блока может вернуть прежний адрес
  // При ошибке возвращает ноль, иначе - адрес считанного блока

_MAPIMP char * _MAPAPI mapGetRstBlock(HMAP hMap, long int number,
                                      long int string, long int column);

  // Запросить адрес блока растра по номеру строки и столбца
  // При отсутствии в файле - создается
  // При запросе следующего блока может вернуть прежний адрес
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // column     - столбец блока
  // При ошибке возвращает ноль

_MAPIMP char * _MAPAPI mapGetRstBlockAndCreate(HMAP hMap, long int number,
                                               long int string, long int column);

  // Запросить наличие блока растра в файле
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // i          - порядковый номер блока
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckRstBlockExistence(HMAP hMap, long int number, long int i);

  // Возврат флага отображения блока
  // (0 - не отображается, 1- отображается, 2 - разделен рамкой )
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // i          - порядковый номер блока
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckRstBlockVisible(HMAP hMap, long int number, long int i);

  // Запись блока {string,column} в файл растрового изображения  из памяти bits.
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // column     - столбец блока
  // bits       - указатель на начало изображения битовой области
  // sizebits   - размер области bits в байтах
  // Возвращает количество записанных байт.
  // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapWriteRstBlock(HMAP hMap,long int number,
                                          long int string, long int column,
                                          char *bits, long int sizebits);

  // Запись блока {string,column} размером "size" по DIB-маске "mask"
  // индексом "value"
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // column     - столбец блока
  // mask       - указатель на начало маски
  // size       - размер области mask в байтах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPutRstBlockByMask(HMAP hMap, long int number,
                                              long int string, long int column,
                                              char *mask, long int size,
                                              long int width,long int height, long value);

  // Записать изменения растра в файл
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveRst(HMAP hMap,long int number);

  // Запись прямоугольного участка растра
  // hMap    - идентификатор открытой векторной карты
  // number  - номер файла в цепочке
  // bits    - указатель на начало изображения битовой области
  // left    - смещение слева в элементах (выравнено на границу байта)
  // top     - смещение сверху в элементах
  // width   - ширина в элементах (выравнено на границу байта)
  // height  - высота в элементах
  // begining  - начало изображения:
  //   ==  1  - (bits - указатель на первую строку битовой области)
  //   == -1  - (bits - указатель на послелнюю строку битовой области,
  //                          в BMP изображение хранится снизу - вверх)
  // widthinbyte    - ширину прямоугольного участка растра в байтах
  // Принцип выравнивания:
  // при ElementSize() == 1 (бит) - left,width кратны 8,
  //                   == 4 (бит) - left,width кратны 2
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapPutRstFrame(HMAP hMap, long int number, char* bits,
                                        long int left, long int top,
                                        long int width, long int height,
                                        long int begining/* = 1*/,
                                        long int widthinbyte /*= 0*/);

  // Чтение прямоугольного участка растра
  //  hMap       - идентификатор открытой векторной карты
  //  number     - номер файла в цепочке
  //  bits    - указатель на начало изображения битовой области
  //  left    - смещение слева в элементах (выравнено на границу байта)
  //  top     - смещение сверху в элементах
  //  width   - ширина в элементах (выравнено на границу байта)
  //  height  - высота в элементах
  //  widthinbyte    - ширину прямоугольного участка растра в байтах
  // Принцип выравнивания:
  //  при ElementSize() == 1 (бит) - left,width кратны 8,
  //                    == 4 (бит) - left,width кратны 2
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstFrame(HMAP hMap, long int number,
                                        char* bits, long int left,
                                        long int top, long int width,
                                        long int height, long int widthinbyte = 0);

  // Чтение цветовых плоскостей прямоугольного участка растра
  //  hMap       - идентификатор открытой векторной карты
  //  number     - номер файла в цепочке
  //  bitsR,bitsG - указатели на начало изображения байтовых областей
  //  bitsB         красной, зеленой и синей плоскости
  //  left    - смещение слева в элементах
  //  top     - смещение сверху в элементах
  //  width   - ширина в элементах
  //  height  - высота в элементах
  //  Поддерживает только 8-битные растры (пока)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstFrameRGB(HMAP hMap, long int number,
                                           char* bitsR, char* bitsG, char* bitsB,
                                           long int left, long int top,
                                           long int width, long int height);

  // Отображение прямоугольного участка исходного растра
  // в результирующем растре, расположенном в области памяти.
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // bits   - указатель на начало области памяти;
  // width  - ширина области памяти в элементах COLORREF
  //          (количество столбцов результирующего растра);
  // height - высота области памяти в элементах
  //          (количество строк результирующего растра);
  // StrL,ColL,StrR,ColR - координаты левого и правого элементов исходного
  //                       растра, которые определяют верхний граничный
  //                       отрезок прямоугольного участка.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstFrameTurn(HMAP hMap, long int number, COLORREF* bits,
                                            long int width, long int height,
                                            double StrL, double ColL, double StrR, double ColR);

  // Пересчитать элементы растра в пикселы
  // для текущего масштаба отображения
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // element    - элементы растра
  // pixel      - результат в пикселах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapRstElementToPixel(HMAP hMap, long int number, double element, double *pixel);

  // Пересчитать пикселы в элементы растра
  // для текущего масштаба отображения
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // element    - результат в элементах растра
  // pixel      - пикселы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapPixelToRstElement(HMAP hMap, long int number,
                                              double pixel, double *element);

  // Формирование битовой маски текущего блока {string,column} с учетом рамки растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // string     - строка блока
  // column     - столбец блока
  // bits       - область битовой маски
  // size       - размер области битовой маски в байтах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapBuildRstBlockMask(HMAP hMap, long int number,
                                              char *bits, long int size,
                                              long int string, long int column);

  // Установить маску изображения растра по метрике объекта
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // info       - объект карты с подобъектами
  // После выполнения функции отображение растра ограничится заданной областью
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstMask(HMAP hMap, long int number, HOBJ info);

  // Заливка цветом части растра, ограниченной рамкой.
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapFillRstVisiblePart(HMAP hMap, long int number, long int color);

  // Запросить состояние растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // Возвращаемые значения:
  //          0 - нет данных; или создание уменьшенных копий и сжатие не выполнялись
  //          1 - создание всех уровней уменьшенных копий, сжатие JPEG растра
  //          2 - создание всех уровней уменьшенных копий, сжатие LZW растра
  //          4 - создание всех уровней уменьшенных копий

_MAPIMP long int _MAPAPI mapGetRstProcessingState(HMAP hMap, long int number);

  // Установить состояние растра
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // state      - состояние растра.
  // Возможные значения состояния растра state:
  //          0 - нет данных; или создание уменьшенных копий и сжатие не выполнялись
  //          1 - создание всех уровней уменьшенных копий, сжатие JPEG растра
  //          2 - создание всех уровней уменьшенных копий, сжатие LZW растра
  //          4 - создание всех уровней уменьшенных копий
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstProcessingState(HMAP hMap, long int number, long int state);

  // Оптимизировать растр для открытия в ГИС Сервер                             // 17/04/16
  // Функция проверяет состояние растра и при необходимости выполняет для растра
  // оптимизацию со сжатием (JPEG - для 24-х битных растров, LZW - для всех остальных)
  // и создание всех уровней уменьшенной копии.
  // Необходимо закрыть растр из всех документов перед вызовом функции
  // rswName- имя файла растра
  // handle - идентификатор окна, которое будет извещаться
  // о ходе процесса (0x585 - 0x588)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapOptimizationRstByName(const char* rswName, HWND handle);
_MAPIMP long int _MAPAPI mapOptimizationRstByNameUn(const WCHAR* rswName, HWND handle);

 // Запросить количество созданных уменьшенных копий в растре                   // 08/04/16
 // hMap       - идентификатор открытой основной векторной карты
 // number     - номер файла в цепочке
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstDuplicatesCount(HMAP hMap, long int number);

  // Обновить уменьшенную копию
  // Если уменьшенные копии не существуют, создаются ТРИ копии
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapUpdateRstDuplicates(HMAP hMap, long int number);

  // Обновить уменьшенную копию блока (string,column) растра
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUpdateRstDuplicateOfBlock(HMAP hMap, long int number,
                                                      int string, int column);

  // Запросить пользовательский идентификатор растра
  // number    - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstUserLabel(HMAP hMap, long int number);

  // Установить пользовательский идентификатор растра
  // number    - номер файла в цепочке
  // userLabel - идентификатор модели
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstUserLabel(HMAP hMap, long int number, long int userLabel);

  // Запросить координаты Юго-Западного угла растра в метрах
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла в цепочке
  // По адресу x,y записываются координаты найденной точки в метрах
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapWhereSouthWestRstPlane(HMAP hMap,long int number,
                                                   double *x, double *y);

  // Удалить файл RSW
  // Функция предназначена для удаления растра и его составных частей
  // Растровая карта размером более 4Gb состоит из 2-х файлов: *.rsw и *.rsw.01
  // Анолог функции DeleteTheFile()

_MAPIMP long int _MAPAPI mapDeleteRstFile(const char *name);
_MAPIMP long int _MAPAPI mapDeleteRstFileUn(const WCHAR *name);

  // Переименовать имя файла RSW
  // Функция предназначена для переименовывания растра и его составных частей
  // Растровая карта размером более 4Gb состоит из 2-х файлов: *.rsw и *.rsw.01
  // Анолог функции MoveTheFile()

_MAPIMP long int _MAPAPI mapMoveRstFile(const char *oldname, const char *newname);
_MAPIMP long int _MAPAPI mapMoveRstFileUn(const WCHAR *oldname, const WCHAR *newname);

  // Скопировать файл RSW
  // Функция предназначена для копирования растра и его составных частей
  // Растровая карта размером более 4Gb состоит из 2-х файлов: *.rsw и *.rsw.01
  // Аналог функции CopyTheFile()

_MAPIMP long int _MAPAPI mapCopyRstFile(const char *oldname, const char *newname, int exist = 0);
_MAPIMP long int _MAPAPI mapCopyRstFileUn(const WCHAR *oldname, const WCHAR *newname, int exist = 0);

  // Запросить тип растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // Возвращает: 0 - обычный растр
  //             1 - растр-пустышка с прямым доступом к файлу TIFF

_MAPIMP long int _MAPAPI mapRstIsAccessTiff(HMAP hMap, long int number);

  // Запросить имя TIFF-файла для растра с номером  number (макс. длина строки = MAX_PATH)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstFileName_Tiff(HMAP hMap, long int number, char *name);

  // Запросить имя TIFF-файла для растра с номером  number
  // hMap     - идентификатор открытой векторной карты
  // number   - номер файла в цепочке
  // name     - возвращаемое имя
  // namesize - размер строки в БАЙТАХ
_MAPIMP long int _MAPAPI mapGetRstFileName_TiffUn(HMAP hMap, long int number,
                                                  WCHAR *name, int namesize);

  // Запросить матрицу аффинных коэффициентов привязки TIFF-файла
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // affincoef - возвращаемая матрица аффинных коэффициентов привязки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstAffinCoef_Tiff(HMAP hMap, long int number, AFFINCOEF *affincoef);

  // Установить матрицу аффинных коэффициентов привязки TIFF-файла
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // affincoef - устанавливаемая матрица аффинных коэффициентов привязки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstAffinCoef_Tiff(HMAP hMap, long int number, const AFFINCOEF *affincoef);

  // Запросить количество каналов TIFF-растра с номером  number
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstBandCount_Tiff(HMAP hMap, long int number);

  // Запросить номер канала TIFF-растра с номером  number, отображаемого красным
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstRedBand_Tiff(HMAP hMap, long int number);

  // Запросить номер канала TIFF-растра с номером  number, отображаемого зеленым
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstGreenBand_Tiff(HMAP hMap, long int number);

  // Запросить номер канала TIFF-растра с номером  number, отображаемого синим
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstBlueBand_Tiff(HMAP hMap, long int number);

  // Установить номер канала TIFF-растра с номером  number, отображаемого красным
  // (если установить -1, то такой канал не используется)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstRedBand_Tiff(HMAP hMap, long int number, int redband);

  // Установить номер канала TIFF-растра с номером  number, отображаемого зеленым
  // (если установить -1, то такой канал не используется)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstGreenBand_Tiff(HMAP hMap, long int number, int greenband);

  // Установить номер канала TIFF-растра с номером  number, отображаемого синим
  // (если установить -1, то такой канал не используется)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstBlueBand_Tiff(HMAP hMap, long int number, int blueband);

  // Установить отображение мультиспектрального растра по вегетационному индексу
  // Перед вызовом необходимо убедиться при помощи функции mapRstIsAccessTiff, что
  // для растра с номером number осуществляется прямой доступ к файлу TIFF
  // Функция справедлива для мультиспектральных изображений (mapGetRstBandCount_Tiff() >= 3)

_MAPIMP long int _MAPAPI mapSetRstVegIndex_Tiff(HMAP hMap, long int number, VEGINDEX *vegindex);

  // Возвращает параметры отображения вегетационного индекса
  // Если отображение по вегетационному индексу не установлено возвращает 0
  // Перед вызовом необходимо убедиться при помощи функции mapRstIsAccessTiff, что
  // для растра с номером number осуществляется прямой доступ к файлу TIFF
  // Функция справедлива для мультиспектральных изображений (mapGetRstBandCount_Tiff() >= 3)

_MAPIMP long int _MAPAPI mapGetRstVegIndex_Tiff(HMAP hMap, long int number, VEGINDEX *vegindex);

  // Возвращает яркость пиксела изображения на канал bandnum
  // x, y  - координаты пикселя в системе координат растра (в пикселях)
  // color - возвращаемое значение реально записанное в растре (может быть 1,4,8,16 бит)
  // bandnum - номер канала (от 0 до mapGetRstBandCount_Tiff - 1)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBandPixel_Tiff(HMAP hMap, long int number,
                                                 long int x, long int y,
                                                 long int bandnum, long int *color);

  // Устанавливает радиус клетки в узлах которой пересчет координат выполняется
  // по строгим формулам при отрисовке растра в системе координат, отличной от
  // системы координат растра.
  // Между узлами пересчет координат выполняется линейной интерполяцией. Коэффициенты
  // линейного пересчета внутри клетки вычисляются по двум верхним узлам клетки.
  // При увеличении радиуса увеличивается скорость отрисовки, но ухудшается качество
  // изображения при значительной деформации системы координат отрисовки относительно
  // системы координат растра (изображение сегментируется по размеру клетки).
  // Этот параметр является глобальным, т.е. с момента установки все растры
  // отрисовываются с использовнаием этого параметра
  // radius - устанавливаемый радиус клетки в пикселях (не может быть меньше 0).
  //          Если равен 0, то все пикселы вычисляются по строгим формулам.
  //          Значение по умолчанию 3.
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstPaintCellRadius_Tiff(long int radius);

  // Возвращает радиус клетки в узлах которой пересчет координат выполняется
  // по строгим формулам при отрисовке растра в системе координат, отличной от
  // системы координат растра.

_MAPIMP long int _MAPAPI mapGetRstPaintCellRadius_Tiff();

  // Возвращает глубину цвета на канал (1, 4, 8, 16)
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBitInBand_Tiff(HMAP hMap, long int number);

  // Возвращает гистограмму
  // Гистограмма - поканальный массив количества пикселей, присутствующих в растре
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // count     - количество элементов в массиве histogram
  //             количество элементов вычисляется по формуле
  //             count = BandCount * (1 << BitInBand)
  //             Для палитровых растров BandCount = 1
  //             для 1 битных растров (палитровых) count = 2
  //             для 4 битных растров (палитровых) count = 16
  //             для 8 битных растров (палитровых) count = 256
  //             для RGB                           count = 3 * 256 = 768
  //             для 8  битных мультиспектральных  count = BandCount * 256
  //             для 16 битных мультиспектральных  count = BandCount * 65536
  // histogram - возвращаемая гистограмма
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstHistogram_Tiff(HMAP hMap, long int number,
                                                 long int count, DWORD *histogram);

  // Возвращает таблицу преобразования цвета для отображения панхроматических,
  // RGB и мультиспектральных растров с глубиной цвета 8 или 16 бит
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // bandnum   - номер канала (от 0 до mapGetRstBandCount_Tiff - 1)
  // table     - возвращаемая таблица преобразования
  // tablesize - размер таблицы table (для 8 бит должно быть 256, для 16 бит - 65536)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstLookupTable_Tiff(HMAP hMap, long int number,
                                                   long int bandnum, BYTE *table,
                                                   long int tablesize);

  // Устанавливает таблицу преобразования цвета для отображения панхроматических,
  // RGB и мультиспектральных растров с глубиной цвета 8 или 16 бит
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // bandnum   - номер канала (от 0 до mapGetRstBandCount_Tiff - 1)
  // table     - возвращаемая таблица преобразования
  // tablesize - размер таблицы table (для 8 бит должно быть не меньше 256,
  //                                   для 16 бит - не меньше 65536)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstLookupTable_Tiff(HMAP hMap, long int number,
                                                   long int bandnum, BYTE *table,
                                                   long int tablesize);

  // Возвращает ширину блока в пикселях
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockWidth_Tiff(HMAP hMap, long int number);

  // Возвращает высоту блока в пикселях
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockHeight_Tiff(HMAP hMap, long int number);

  // Возвращает способ расположения цветовых составляющих пикселя в блоке
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // Возвращаемые значения:
  // 0 - ошибка выполнения
  // 1 - последовательно RGB RGB ...
  // 2 - по цветовым плоскостям  RRR... GGG... BBB...

_MAPIMP long int _MAPAPI mapGetRstBlockPixelType_Tiff(HMAP hMap, long int number);

  // Возвращает количество строк блоков
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockRowCount_Tiff(HMAP hMap, long int number);

  // Возвращает количество столбцов блоков
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockColCount_Tiff(HMAP hMap, long int number);

  // Возвращает размер блока в байтах
  // Для 1 и 4 битных растров в блок записывается 1 байт на пиксель
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockSize_Tiff(HMAP hMap, long int number);

  // Считывает блок из растра
  // Для 1 и 4 битных растров в блок записывается 1 байт на пиксель
  // hMap     - идентификатор открытой векторной карты
  // number   - номер файла в цепочке
  // blockrow - номер строки блоков
  // blockcol - номер столбца блоков
  // buf      - буфер, в который записывается изображение блока
  // bifsize  - размер блока (должен быть равен mapGetRstBlockSize_Tiff)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlock_Tiff(HMAP hMap, long int number, int blockrow,
                                             int blockcol, char *buf, int bufsize);

  // Проверяет файл TIF на возможность открытия без преобразования в формат RSW
_MAPIMP long int _MAPAPI mapIsTiffOpenWithoutConvert(const char *name);
_MAPIMP long int _MAPAPI mapIsTiffOpenWithoutConvertUn(const WCHAR *name);      // 19/06/14


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ СОЗДАНИЯ TIFF ФАЙЛОВ +++++++++++++ 18/09/13
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// Создает TIFF файл
// filename - имя создаваемого TIFF файла
// width    - ширина растра в пикселях
// height   - высота растра в пикселях
// parm     - параметры создания растра
// Возвращает идентификатор созданного растра
// При ошибке возвращает 0

_MAPIMP HANDLE _MAPAPI mapCreateTiff(const char *filename, long int width,
                                     long int height, CREATETIFPARM *parm);
_MAPIMP HANDLE _MAPAPI mapCreateTiffUn(const WCHAR *filename, long int width,
                                       long int height, CREATETIFPARM *parm);

// Освобождает идентификатор создания TIFF файла
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff

_MAPIMP void _MAPAPI mapFreeTiff(HANDLE tiff);

// Возвращает ширину блока в пикселях
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTiffBlockWidth(HANDLE tiff);

// Возвращает высоту блока в пикселях
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTiffBlockHeight(HANDLE tiff);

// Возвращает количество строк блоков
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTiffBlockRowCount(HANDLE tiff);

// Возвращает количество столбцов блоков
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTiffBlockColCount(HANDLE tiff);

// Возвращает тип пикселя на канал (см. PT_BYTE и т.д.)
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetTiffBandPixelType(HANDLE tiff);

// Пишет блок в файл
// tiff    - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// bandnum - номер канала
// col     - номер столбца тайла
// row     - номер строки тайла
// image   - изображение тайла
//           Ширина, высота тайла запрашивается через mapGetTiffBlockWidth, mapGetTiffBlockHeight
//           Размер пикселя в байтах зависит от типа пикселя, определяемый через GetTiffBandPixelType
//           В последних строках и столбцах размер тайла не усеченный
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapWriteTiffBlock(HANDLE tiff, long int bandnum,
                                           long int col, long int row, void *image);

// Устанавливает матрицу привязки растра
// tiff - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// coef - матрица, связывающая систему координат растра в пикселях с системой координат местности в метрах
//        Xmeter = coef->A0 + coef->A1 * Xpix + coef->A2 * Ypix
//        Ymeter = coef->B0 + coef->B1 * Xpix + coef->B2 * Ypix
//        Направление осей
//        -------------Xrst
//        |    Ymeter
//        |      |
//        Yrst   |
//               ---------- Xmeter
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetTiffLocation(HANDLE tiff, AFFINCOEF *coef);

// Устанавливает параметры системы координат растра
// tiff      - идентификатор, полученный при создании TIFF файла функцией mapCreateTiff
// mapreg    - параметры проекции
// ellipsoid - параметры эллипсоида
// datum     - датум
// При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetTiffProjection(HANDLE tiff, MAPREGISTEREX *mapreg,
                                              ELLIPSOIDPARAM *ellipsoid, DATUMPARAM *datum);


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К ГРАФИЧЕСКИМ ФАЙЛАМ +++++++++++++ 08/02/16
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // Запросить тип растра
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // Возвращает: 0 - обычный растр
  //             1 - растр-пустышка с прямым доступом к графическому файлу
  //                 (TIFF, GeoTIFF, IMG, JPEG, PNG, GIF, BMP)

_MAPIMP long int _MAPAPI mapRstIsAccessGraphicFile(HMAP hMap, long int number);

  // Запросить имя графического файла для растра с номером  number
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap     - идентификатор открытой векторной карты
  // number   - номер файла в цепочке
  // name     - возвращаемое имя
  // namesize - размер строки в БАЙТАХ
  // При ошибке возвращает 0
_MAPIMP long int _MAPAPI mapGetRstGraphicFileNameUn(HMAP hMap, long int number,   
                                                    WCHAR *name, int namesize);

  // Запросить матрицу аффинных коэффициентов привязки графического файла
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // affincoef - возвращаемая матрица аффинных коэффициентов привязки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstAffinCoef_GraphicFile(HMAP hMap, long int number,
                                                    AFFINCOEF *affincoef);

  // Установить матрицу аффинных коэффициентов привязки графического файла
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // affincoef - устанавливаемая матрица аффинных коэффициентов привязки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRstAffinCoef_GraphicFile(HMAP hMap, long int number,
                                                    const AFFINCOEF *affincoef);

  // Запросить количество каналов графического файла с номером  number
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRstBandCount_GraphicFile(HMAP hMap, long int number);

  // Запросить номер канала графического файла с номером  number, отображаемого красным
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstRedBand_GraphicFile(HMAP hMap, long int number);

  // Запросить номер канала графического файла с номером  number, отображаемого зеленым
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstGreenBand_GraphicFile(HMAP hMap, long int number);

  // Запросить номер канала графического файла с номером  number, отображаемого синим
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapGetRstBlueBand_GraphicFile(HMAP hMap, long int number);

  // Установить номер канала графического файла с номером  number, отображаемого красным
  // (если установить -1, то такой канал не используется)
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstRedBand_GraphicFile(HMAP hMap, long int number,
                                                      int redband);

  // Установить номер канала графического файла с номером  number, отображаемого зеленым
  // (если установить -1, то такой канал не используется)
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstGreenBand_GraphicFile(HMAP hMap, long int number,
                                                      int greenband);

  // Установить номер канала графического файла с номером  number, отображаемого синим
  // (если установить -1, то такой канал не используется)
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке

_MAPIMP long int _MAPAPI mapSetRstBlueBand_GraphicFile(HMAP hMap, long int number,
                                                      int blueband);

  // Установить отображение мультиспектрального растра по вегетационному индексу
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // Функция справедлива для мультиспектральных изображений
  //                                     (mapGetRstBandCount_GraphicFile() >= 3)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstVegIndex_GraphicFile(HMAP hMap, long int number,
                                                      VEGINDEX *vegindex);

  // Возвращает параметры отображения вегетационного индекса
  // Если отображение по вегетационному индексу не установлено возвращает 0
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // Функция справедлива для мультиспектральных изображений
  //                                     (mapGetRstBandCount_GraphicFile() >= 3)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstVegIndex_GraphicFile(HMAP hMap, long int number,
                                                      VEGINDEX *vegindex);

  // Возвращает яркость пиксела изображения на канал bandnum
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // x, y  - координаты пикселя в системе координат растра (в пикселях)
  // color - возвращаемое значение реально записанное в растре (может быть 1,4,8,16 бит)
  // bandnum - номер канала (от 0 до mapGetRstBandCount_GraphicFile - 1)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBandPixel_GraphicFile(HMAP hMap, long int number,
                                                      long int x, long int y,
                                                      long int bandnum,
                                                      long int *color);

  // Устанавливает радиус клетки в узлах которой пересчет координат выполняется
  // по строгим формулам при отрисовке растра в системе координат, отличной от
  // системы координат растра.
  // Между узлами пересчет координат выполняется линейной интерполяцией. Коэффициенты
  // линейного пересчета внутри клетки вычисляются по двум верхним узлам клетки.
  // При увеличении радиуса увеличивается скорость отрисовки, но ухудшается качество
  // изображения при значительной деформации системы координат отрисовки относительно
  // системы координат растра (изображение сегментируется по размеру клетки).
  // Этот параметр является глобальным, т.е. с момента установки все растры
  // отрисовываются с использовнаием этого параметра
  // radius - устанавливаемый радиус клетки в пикселях (не может быть меньше 0).
  //          Если равен 0, то все пикселы вычисляются по строгим формулам.
  //          Значение по умолчанию 3.

_MAPIMP long int _MAPAPI mapSetRstPaintCellRadius_GraphicFile(long int radius);

  // Возвращает радиус клетки в узлах которой пересчет координат выполняется
  // по строгим формулам при отрисовке растра в системе координат, отличной от
  // системы координат растра.

_MAPIMP long int _MAPAPI mapGetRstPaintCellRadius_GraphicFile();

  // Возвращает глубину цвета на канал (1, 4, 8, 16)
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBitInBand_GraphicFile(HMAP hMap, long int number);

  // Возвращает гистограмму
  // Гистограмма - поканальный массив количества пикселей, присутствующих в растре
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // count     - количество элементов в массиве histogram
  //             количество элементов вычисляется по формуле
  //             count = BandCount * (1 << BitInBand)
  //             Для палитровых растров BandCount = 1
  //             для 1 битных растров (палитровых) count = 2
  //             для 4 битных растров (палитровых) count = 16
  //             для 8 битных растров (палитровых) count = 256
  //             для RGB                           count = 3 * 256 = 768
  //             для 8  битных мультиспектральных  count = BandCount * 256
  //             для 16 битных мультиспектральных  count = BandCount * 65536
  // histogram - возвращаемая гистограмма
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstHistogram_GraphicFile(HMAP hMap, long int number,
                                                 long int count, DWORD *histogram);

  // Возвращает таблицу преобразования цвета для отображения панхроматических,
  // RGB и мультиспектральных растров с глубиной цвета 8 или 16 бит
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // bandnum   - номер канала (от 0 до mapGetRstBandCount_GraphicFile - 1)
  // table     - возвращаемая таблица преобразования
  // tablesize - размер таблицы table (для 8 бит должно быть 256, для 16 бит - 65536)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstLookupTable_GraphicFile(HMAP hMap, long int number,
                                                   long int bandnum, BYTE *table,
                                                   long int tablesize);

  // Устанавливает таблицу преобразования цвета для отображения панхроматических,
  // RGB и мультиспектральных растров с глубиной цвета 8 или 16 бит
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // bandnum   - номер канала (от 0 до mapGetRstBandCount_GraphicFile - 1)
  // table     - возвращаемая таблица преобразования
  // tablesize - размер таблицы table (для 8 бит должно быть не меньше 256,
  //                                   для 16 бит - не меньше 65536)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRstLookupTable_GraphicFile(HMAP hMap, long int number,
                                                   long int bandnum, BYTE *table,
                                                   long int tablesize);

  // Возвращает ширину блока в пикселях
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockWidth_GraphicFile(HMAP hMap, long int number);

  // Возвращает высоту блока в пикселях
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockHeight_GraphicFile(HMAP hMap, long int number);

  // Возвращает способ расположения цветовых составляющих пикселя в блоке
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // Возвращаемые значения:
  // 0 - ошибка выполнения
  // 1 - последовательно RGB RGB ...
  // 2 - по цветовым плоскостям  RRR... GGG... BBB...

_MAPIMP long int _MAPAPI mapGetRstBlockPixelType_GraphicFile(HMAP hMap,
                                                             long int number);

  // Возвращает количество строк блоков
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockRowCount_GraphicFile(HMAP hMap,
                                                            long int number);

  // Возвращает количество столбцов блоков
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockColCount_GraphicFile(HMAP hMap,
                                                            long int number);

  // Возвращает размер блока в байтах
  // Для 1 и 4 битных растров в блок записывается 1 байт на пиксель
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap   - идентификатор открытой векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlockSize_GraphicFile(HMAP hMap, long int number);

  // Считывает блок из растра
  // Для 1 и 4 битных растров в блок записывается 1 байт на пиксель
  // Перед вызовом необходимо убедиться, используя функцию mapRstIsAccessGraphicFile,
  // что для растра с номером number осуществляется прямой доступ к графическому файлу
  // hMap     - идентификатор открытой векторной карты
  // number   - номер файла в цепочке
  // blockrow - номер строки блоков
  // blockcol - номер столбца блоков
  // buf      - буфер, в который записывается изображение блока
  // bifsize  - размер блока (должен быть равен mapGetRstBlockSize_GraphicFile)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRstBlock_GraphicFile(HMAP hMap, long int number,
                                                    int blockrow, int blockcol,
                                                    char *buf, int bufsize);

  // Проверяет графический файл на возможность открытия без преобразования в формат RSW
_MAPIMP long int _MAPAPI mapIsGraphicFileOpenWithoutConvert(const char *name);
_MAPIMP long int _MAPAPI mapIsGraphicFileOpenWithoutConvertUn(const WCHAR *name);   



}  // extern "C"


#endif  // RSTAPI_H