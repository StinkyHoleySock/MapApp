/****** MTRAPI.H *************** Belenkov O.V.   ****** 26/05/16 ***
******* MTRAPI.H *************** Gorbunov A.V.   ****** 17/02/14 ***
******* MTRAPI.H *************** Kruzhkov A.E.   ****** 27/02/14 ***
******* MTRAPI.H *************** Shabakov D.A.   ****** 11/07/16 ***
******* MTRAPI.H *************** Derevyagina Zh.A.***** 16/12/10 ***
******* MTRAPI.H *************** Borzov   A.U.   ****** 14/03/14 ***
******* MTRAPI.H *************** Konon    V.N.   ****** 28/01/16 ***
******* MTRAPI.H *************** Zheleznyakov V.A. **** 06/06/16 ***
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
*    HMAP (WINAPI * lpfn_OpenMtr)(const char*, long int);          *
*    (FARPROC)lpfn_OpenMtr = GetProcAddress(libInst, "OpenMtr");   *
*    HMAP hMap = (*lpfn_OpenMtr)("Noginsk.mtw",0);                 *
*                                                                  *
*    // Выгрузка библиотеки                                        *
*    ::FreeLibrary(libInst);                                       *
*                                                                  *
*******************************************************************/

#ifndef MTRAPI_H
#define MTRAPI_H

#ifndef MAPCREAT_H
  #include "mapcreat.h"
#endif

#ifndef MAPOLD_H
  #include "mapold.h"
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К МАТРИЦАМ +++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern "C"
{

  // Открыть матричные данные
  // Возвращает идентификатор открытой матричной карты (TMapAccess*)
  // mtrname - имя открываемого файла
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenMtr(const char *mtrname, long int mode = 0);
_MAPIMP HMAP _MAPAPI mapOpenMtrUn(const WCHAR *mtrname, long int mode = 0);

  // Открыть матричные данные в заданном районе работ
  // (добавить в цепочку матриц)
  // Возвращает номер файла в цепочке матриц
  // hMap - идентификатор открытой основной векторной карты
  // mtrname - имя открываемого файла
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenMtrForMap(HMAP hMap, const char *mtrname, long int mode);
_MAPIMP long int _MAPAPI mapOpenMtrForMapUn(HMAP hMap, const WCHAR *name, long int mode);

  // Закрыть матричные данные
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер закрываемой матрицы
  // если number == 0, закрываются все матрицы в окне
  // ЧТОБЫ ОСВОБОДИТЬ ВСЕ РЕСУРСЫ - НУЖНО ВЫЗВАТЬ mapCloseData(hMap)

_MAPIMP void _MAPAPI mapCloseMtr(HMAP hMap, long int number);

  // Закрыть матричные данные в заданном районе работ
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер матричного файла в цепочке
  // Если number == 0, закрываются все матричные данные
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCloseMtrForMap(HMAP hMap, long int number);

  // Запросить время крайнего редактирования матрицы
  // hMap    - идентификатор открытой векторной карты
  // number  - номер матрицы
  // Возвращает системное время редактирования (создания) по Гринвичу
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrSystemTime(HMAP hMap, long int number, SYSTEMTIME *time);

  // Построение матрицы по векторной карте на заданный участок района работ
  // При ошибке возвращает ноль
  // hMap    - исходная карта для построения матрицы
  // mtrname - полное имя создаваемой матрицы
  // filtername - полное имя фильтра объектов
  //   Вместе с картой может располагаться фильтр объектов -
  //   текстовый файл MTRCREA.IMH, содержащий перечень кодов
  //   объектов, используемых при построении матрицы (см. MAPAPI.DOC)
  // Если filtername равно нулю - фильтр объектов не используется
  // mtrparm - параметры создаваемой матрицы,
  // handle   - идентификатор окна диалога, которому посылаются
  // сообщения о ходе процесса :
  //   0x0581 - сообщение о проценте выполненных работ (в WPARAM),
  //   если процесс должен быть принудительно завершен, в ответ
  //   должно вернуться значение 0x0581.
  // Если handle равно нулю - сообщения не посылаются.
  // Параметр LPARAM (не равный 0) сообщения о ходе процесса содержит номер
  // этапа построения матрицы :
  //   1 - Заполнение матрицы абсолютными высотами объектов
  //   2 - Обработка объектов гидрографии с постоянной высотой
  //   3 - Обработка объектов гидрографии с переменной высотой
  //   4 - Обработка точечных объектов с абсолютной высотой
  //   5 - Определение минимальной и максимальной высоты
  //   6 - Вычисление незаполненных элементов матрицы
  //   9 - Заполнение матрицы относительными высотами объектов
  //   10 - Создание матрицы выполнено
  //   11 - Создание матрицы не выполнено
  //   13 - Обработка пустых замкнутых горизонталей
  //   14 - Вычисление элементов матрицы по сетке высотных точек
  //   15 - Построение сетки треугольников по высотным точкам
  //   16 - Сжатие матрицы
  //   17 - Вычисление высот по трехмерной метрике площадных объектов

_MAPIMP long int _MAPAPI mapBuildMtw(HMAP hMap, const char *mtrname, const char *filtername,
                                     BUILDMTW *mtrparm, HWND handle);
_MAPIMP long int _MAPAPI mapBuildMtwUn(HMAP hMap, const WCHAR *mtrname, const WCHAR *txtname,
                                       BUILDMTW *mtrparm, HWND handle);

 // Построение растра качеств по векторной карте на заданный
 // участок района работ
 // При ошибке возвращает ноль
 // hMap    - исходная карта для построения растра,
 // rstname - полное имя создаваемого растра,
 // filtername - полное имя служебного текстового файла
 //   Вместе с картой должен располагаться фильтр объектов -
 //   служебный текстовый файл MАP2RSW.INI, содержащий перечень кодов
 //   объектов, используемых при построении растра
 // mtrparm - параметры создаваемого растра,
 // handle   - идентификатор окна диалога, которому посылаются
 // сообщения о ходе процесса :
 //   0x0581 - сообщение о проценте выполненных работ (в WPARAM),
 //   если процесс должен быть принудительно завершен, в ответ
 //   должно вернуться значение 0x0581.
 // Если handle равно нулю - сообщения не посылаются

_MAPIMP long int _MAPAPI mapBuildRsw(HMAP hMap, const char *rstname, const char *filtername,
                                     BUILDMTW *mtrparm, HWND handle);
_MAPIMP long int _MAPAPI mapBuildRswUn(HMAP hMap, const WCHAR *rstname, const WCHAR *filtername,
                                       BUILDMTW *mtrparm, HWND handle);

  // Очистить кэш матричных данных, открытых на ГИС Сервере
  // hMap    - идентификатор открытой векторной карты
  // number  - номер матрицы, для которой нужно очистить кэш, или -1 (все матрицы)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapClearMtrCache(HMAP hMap, long int number);


  /*********************************************************
  *                                                        *
  *           ЗАПРОС ДАННЫХ ИЗ МАТРИЦЫ ВЫСОТ               *
  *                                                        *
  *     КОД ОШИБКИ "ERRORHEIGHT" РАВЕН (-111111.0)         *
  *  (т.е. если значение высоты меньше (-111 км) - ошибка) *
  *                                                        *
  *********************************************************/

  // Запросить описание файла матричных данных
  // hMap - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // describe - адрес структуры, в которой будет размещено
  // описание матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrDescribe(HMAP hMap, long int number, MTRDESCRIBE *describe);
_MAPIMP long int _MAPAPI mapGetMtrDescribeUn(HMAP hMap, long int number, MTRDESCRIBEUN *describe);

  // Запросить описание диапазона высот матрицы с номером
  // hMap - идентификатор открытой основной векторной карты
  // number в цепочке
  // colornumber - номер диапазона высот
  // colordesc - адрес структуры, в которой будет размещено
  // описание диапазона высот
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrColorDescEx(HMAP hMap, long int number,
                                              long int colornumber,
                                              MTRCOLORDESCEX *colordesc);

  // Запросить имя файла матричных данных
  // hMap - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetMtrName(HMAP hMap, long int number);

  // Запросить имя файла матричных данных
  // hMap - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // name - адрес строки для размещения результата
  // size - размер строки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrNameUn(HMAP hMap, long int number,
                                         WCHAR *name, long int size);

  // Запросить число открытых файлов матричных данных
  // hMap - идентификатор открытой основной векторной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrCount(HMAP hMap);

  // Запросить номер матрицы в цепочке по имени файла
  // name - имя файла матрицы
  // В цепочке номера растров начинаются с 1.
  // При ошибке возвращает ноль
_MAPIMP long int _MAPAPI mapGetMtrNumberByName(HMAP hMap, const char* name);
_MAPIMP long int _MAPAPI mapGetMtrNumberByNameUn(HMAP hMap, const WCHAR *name);

  // Запросить фактические габариты отображаемой матрицы в метрах в районе работ
  // При отображении матрицы по рамке возвращаются габариты рамки
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // frame  - адрес структуры, в которой будут размещены габариты матрицы в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActualMtrFrame(HMAP hMap, DFRAME *frame, long int number);

  // Запросить привязку матрицы  в метрах в районе работ
  // number     - номер файла в цепочке
  // location   - координаты юго-западного угла матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrLocation(HMAP hMap, long int number, DOUBLEPOINT *location);

  // Установить привязку матрицы  в метрах в районе работ
  // number     - номер файла в цепочке
  // location   - координаты юго-западного угла матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrLocation(HMAP hMap, long int number, DOUBLEPOINT *location);

  // Запрос наличия высот рельефа на заданном участке
  // hMap  - идентификатор открытой основной векторной карты
  // frame - адрес структуры, содержащей габариты заданного участка в метрах
  // Если frame равно нулю, то заданный участок определяется габаритами карты
  // При наличии высот рельефа возвращает 1, при отсутствии возвращает 0

_MAPIMP long int _MAPAPI mapHeightValuePresence(HMAP hMap, DFRAME *frame);

  // Выбор значения абсолютной высоты в заданной точке.
  // hMap   - идентификатор открытой основной векторной карты
  // Координаты точки (x,y) задаются в метрах в системе координат
  // векторной карты. Возвращает значение высоты в метрах.
  // В случае ошибки при выборе высоты и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetHeightValue(HMAP hMap,double x, double y);

  // Выбор значения абсолютной высоты в заданной точке из матрицы
  // с номером number в цепочке.
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер матрицы в цепочке.
  // Координаты точки (x,y) задаются в метрах в системе координат
  // векторной карты. Возвращает значение высоты в метрах.
  // hpaint - контекст поддержки многопоточного вызова (см. mapCreatePaintControl)
  // В случае ошибки при выборе высоты и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetHeightValueOfMtr(HMAP hMap, long int number,
                                              double x, double y);

_MAPIMP double _MAPAPI mapGetHeightValueOfMtrControl(HMAP hMap, long int number, double x,
                                                double y, HPAINT hPaint);

  // Возвращает интерполированную высоту в заданной точке
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы высот в цепочке
  // interptype - тип интерполяции
  //              1 - ближайший сосед
  //              2 - интерполяция по ближайшим 3 элементам
  //              3 - билинейная интерполяция по 4 ближайшим элементам
  //              4 - бикубическая интерполяция по 16 ближайшим элементам
  // x, y - координаты точки в метрах
  // h    - возвращаемое значение в метрах (при ошибке устанавливается ERRORHEIGHT)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetHeightValueOfMtrEx(HMAP hMap, long int number,
                                                  long int interptype,
                                                  double x, double y, double *h);

  // Чтение элемента матрицы высот по абсолютным индексам
  // hMap   - идентификатор открытой карты
  // number - номер файла в цепочке
  // value  - полученное значение элемента в метрах
  // string - индекс строки матрицы (значение от 0 до height-1, где height - высота
  //          матрицы элементах, запрашиваемая функцией mapGetMtrHeightInElement)
  // column - индекс колонки матрицы (значение от 0 до width-1, где width - ширина
  //          матрицы элементах, запрашиваемая функцией mapGetMtrWidthInElement)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrPoint(HMAP hMap, long int number, double *value,
                                        long int string, long int column);

  // Запросить размер элемента матрицы в метрах по оси X
  // number    - номер файла в цепочке
  // metinelemX - размер элемента матрицыв метрах на местности по оси X
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrMeterInElementX(HMAP hMap, long int number, double *metinelemX);

  // Запросить размер элемента матрицы в метрах по оси Y
  // number    - номер файла в цепочке
  // metinelemY - размер элемента матрицы в метрах на местности по оси Y
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrMeterInElementY(HMAP hMap, long int number, double *metinelemY);

  // Занесение значения абсолютной высоты в элемент матрицы,
  // соответствующий заданной точке.
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер матрицы в цепочке.
  // Координаты точки (x,y) и значение высоты (h) задаются в метрах
  // в системе координат векторной карты.
  // В случае ошибки возвращает ноль

_MAPIMP long int _MAPAPI mapPutHeightValue(HMAP hMap, long int number,
                                           double x, double y, double h);


  // Выбор значения абсолютной высоты в заданной точке из
  // матрицы с наименьшим размером элемента (более точной)
  // hMap   - идентификатор открытой основной векторной карты 
  // Координаты точки (x,y) задаются в метрах в системе координат
  // векторной карты. Возвращает значение высоты в метрах.
  // hPaint - идентификатор контекста отображения для многопоточного вызова функций,
  //          создается функцией mapCreatePaintControl, освобождается - mapFreePaintControl
  // В случае ошибки при выборе высоты и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetPrecisionHeightValue(HMAP hMap, double x, double y);
_MAPIMP double _MAPAPI mapGetPrecisionHeightValueEx(HMAP hMap, double x, double y, HPAINT hPaint);


  // Расчет абсолютной высоты методом треугольников в заданной точке
  // по матрице с наименьшим размером элемента (более точной).
  // Высота вычисляется по самой точной матрице высот,а в случае
  // необеспеченности заданной точки данными матриц высот -
  // по самой точной матрице слоев.
  // hMap   - идентификатор открытой основной векторной карты
  // Координаты точки (x,y) задаются в метрах в системе координат
  // векторной карты. Возвращает значение высоты в метрах.
  // hPaint - идентификатор контекста отображения для многопоточного вызова функций,
  //          создается функцией mapCreatePaintControl, освобождается - mapFreePaintControl
  // В случае ошибки при выборе высоты и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetPrecisionHeightTriangle(HMAP hMap, double x, double y);
_MAPIMP double _MAPAPI mapGetPrecisionHeightTriangleEx(HMAP hMap,double x, double y, HPAINT hPaint);

  // Расчет абсолютной высоты методом треугольников в заданной точке
  // по матрице с номером number в цепочке
  // Координаты точки (x,y) задаются в метрах в системе координат
  // векторной карты. Возвращает значение высоты в метрах.
  // В случае ошибки при выборе высоты и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetHeightTriangleOfMtr(HMAP hMap, long int number,
                                                 double x, double y);

  // Расчет среднего значения абсолютной высоты по высотам квадратной области
  // Функция может использоваться для создания матрицы обобщенного рельефа
  // xcenter, ycenter - координаты центра области в метрах
  // size - размер стороны области в метрах (размер элемента матрицы обобщенного рельефа)
  // Возвращает среднее значение высоты в метрах
  // В случае ошибки при выборе высот и в случае необеспеченности
  // заданной области матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetGeneralHeight(HMAP hMap, double xcenter,
                                           double ycenter, double size);

  // Расчет среднего значения абсолютной высоты по высотам квадратной области
  // матрицы с номером number в цепочке
  // Функция может использоваться для создания матрицы обобщенного рельефа
  // xcenter, ycenter - координаты центра области в метрах
  // size - размер стороны области в метрах (размер элемента матрицы обобщенного рельефа)
  // Возвращает среднее значение высоты в метрах
  // В случае ошибки при выборе высот и в случае необеспеченности
  // заданной области матричными данными возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapGetGeneralHeightOfMtr(HMAP hMap, long int number, double xcenter,
                                                double ycenter, double size);

  // Запросить номер в цепочке для матрицы, расположенной
  // в заданной точке
  // hMap   - идентификатор открытой основной векторной карты
  // number - порядковый номер, найденной матрицы в точке
  // (1 - первая в данной точке, 2 - вторая ...)
  // При ошибке возвращается ноль

_MAPIMP long int _MAPAPI mapGetMtrNumberInPoint(HMAP hMap,double x,
                                                double y, long int number);

 // Вычисление значения массива элементов с применением метода треугольников     // 15/04/16
 // по матрицам высот (применяется для обработки OGC WCS-запросов)
 // hmap   - идентификатор документа, содержащего открытые матрицы высот (MTW)
 // matrix - указатель на буфер выходной матрицы 4-ех байтовых целочисленных элементов
 // width  - ширина выходной матрицы (число элементов в строке)
 // hight  - высота выходной матрицы (число строк)
 // unit   - единица измерения высоты в матрице (0 - метры, 1 - дециметры, 2 - сантиметры, 3 - миллиметры)
 // dframe - габариты матрицы на местности в системе координат матрицы (и документа)
 //          от юго-западного элемента матрицы до северо-восточного
 // minvalue - поле для записи минимального значения элемента в выходной матрице
 // maxvalue - поле для записи максимального значения элемента в выходной матрице
 // hpaint - контекст поддержки многопоточного вызова (см. mapCreatePaintControl)
 // В случае необеспеченности заданной точки матричными данными в элемент записывается ERRORHEIGHT (-111111)
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetPrecisionHeightFrame(HMAP hmap, int * matrix, long int width, long int hight,
                                                    long int unit, DFRAME * dframe,
                                                    long int * minvalue, long int * maxvalue, HPAINT hpaint);

  // Выбор массива значений абсолютных высот, соответствующих
  // логическим элементам, лежащим на заданном отрезке.
  // hMap   - идентификатор открытой основной векторной карты 
  // Координаты точек, задающих начало и конец отрезка
  // (FirstPoint,SecondPoint) задаются в метрах в системе
  // координат векторной карты.
  // Размер массива высот, заданного адресом HeightArray,
  // должен соответствовать запрашиваемому количеству высот
  // (HeightCount), в противном случае возможны ошибки работы
  // с памятью.
  // В случае необеспеченности логического элемента матричными
  // данными его значение равно ERRORHEIGHT (-111111.0 м)
  // В случае ошибки при выборе высот возвращает ноль

_MAPIMP long int _MAPAPI mapGetHeightArray(HMAP hMap, double *HeightArray,
                                           long int HeightCount,
                                           const DOUBLEPOINT *FirstPoint,
                                           const DOUBLEPOINT *SecondPoint);


  // Запросить размер полного блока матрицы в байтах
  // hMap   - идентификатор открытой основной векторной карты 
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBlockSize(HMAP hMap, long int number);

  // Запросить размер(тип) элемента матрицы в байтах
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке.
  // Возвращаемое значение 1 соответствует типу "unsigned char".
  // Возвращаемое значение 2 соответствует типу "short int".
  // Возвращаемое значение 4 соответствует типу "long int".
  // Возвращаемое значение 8 соответствует типу "double".
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrElementSize(HMAP hMap, long int number);


  // Запросить вертикальный размер блока матрицы в элементах
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBlockSide(HMAP hMap, long int number);


  // Запросить горизонтальный размер блока матрицы в элементах
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBlockWidth(HMAP hMap, long int number);

  // Запросить ширину текущего блока матрицы в элементах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrCurrentBlockWidth(HMAP hMap, long int number,
                                                    long int column);

  // Запросить высоту текущего блока матрицы в элементах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP   long int _MAPAPI mapGetMtrCurrentBlockHeight(HMAP hMap, long int number,
                                                       long int string);

  // Запросить размер текущего блока матрицы в байтах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // string, column - строка и столбец блока соотвественно
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrCurrentBlockSize(HMAP hMap, long int number,
                                                   long int string, long int column);

  // Запросить адрес блока матрицы по номеру строки и столбца
  // Блоки последнего ряда могут иметь усеченный размер
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // string, column - строка и столбец блока соотвественно
  // При запросе следующего блока может вернуть прежний адрес
  // При ошибке возвращает ноль, иначе - адрес считанного блока

_MAPIMP char* _MAPAPI mapGetMtrBlockAddress(HMAP hMap, long int number,
                                            long int string, long int column);

  // Запросить адрес блока матрицы по номеру строки и столбца
  // string, column - строка и столбец блока соотвественно
  // При отсутствии в файле - создается
  // При запросе следующего блока может вернуть прежний адрес
  // При ошибке возвращает ноль

_MAPIMP char* _MAPAPI mapGetMtrBlockAndCreate(HMAP hMap, long int number,
                                              long int string, long int column);

  // Запись блока {string,column} в файл матрицы из памяти bits
  // number   - номер файла в цепочке
  // bits     - указатель на начало изображения битовой области
  // sizebits - размер области bits в байтах
  // Возвращает количество записанных байт.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapWriteMtrBlock(HMAP hMap, long int number, long int string,
                                          long int column, char *bits, long int sizebits);

  // Возврат флага отображения блока матрицы
  // (0 - не отображается, 1- отображается, 2 - разделен рамкой )
  // number - номер файла в цепочке
  // i - порядковый номер (индекс) блока, i = str * blockColumnCount + col, где:
  //     str - индекс строки блоков,
  //     blockColumnCount - число столбцов блоков матрицы (функция mapGetMtqBlockColumn)
  //     col - индекс столбца блоков
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckMtrBlockVisible(HMAP hMap, long int number, long int i);

  // Запросить число строк блоков матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBlockRow(HMAP hMap, long int number);

  // Запросить число столбцов блоков матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBlockColumn(HMAP hMap, long int number);


  // Запросить число строк элементов в матрице
  // hMap   - идентификатор открытой основной векторной карты 
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrElementRow(HMAP hMap, long int number);


  // Запросить число столбцов элементов в матрице
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrElementColumn(HMAP hMap, long int number);

  // Запросить масштаб матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrScale(HMAP hMap, long int number);

  // Запросить значения масштаба нижней и верхней границ видимости матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // По адресу bottomScale записывается знаменатель масштаба нижней границы видимости матрицы
  // По адресу topScale записывается знаменатель масштаба верхней границы видимости матрицы
  // При ошибке возвращает ноль
_MAPIMP long int _MAPAPI mapGetMtrRangeScaleVisible(HMAP hMap, long int number,
                                                    long int *bottomScale,
                                                    long int *topScale);

  // Установить значения масштаба нижней и верхней границ видимости матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // bottomScale - знаменатель масштаба нижней границы видимости матрицы
  // topScale    - знаменатель масштаба верхней границы видимости матрицы
  //               bottomScale <= topScale, иначе возвращает 0
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrRangeScaleVisible(HMAP hMap, long int number,
                                                   long int bottomScale,
                                                   long int topScale);

  // Запросить тип исходной карты
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrMapType(HMAP hMap, long int number);

  // Запросить единицу измерения значений высот матрицы
  // с номером number в цепочке.
  // Возвращает значение поля Unit структуры параметров создания
  // матрицы BUILDMTW
  // Возвращаемые значения :
  //   0-метры, 1-дециметры, 2-сантиметры, 3-миллиметры
  // hMap - идентификатор открытой основной векторной карты
  // При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapGetMtrMeasure(HMAP hMap, long int number);

  // Запрос - поддерживается ли пересчет к геодезическим
  // координатам из плоских прямоугольных и обратно
  // hMap     - идентификатор открытой основной карты
  // number   - номер файла в цепочке
  // Если нет - возвращает ноль

  _MAPIMP long int _MAPAPI mapIsMtrGeoSupported(HMAP hMap, long int number);

  // Запросить данные о проекции матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // projectiondata - адрес структуры, в которой будут размещены
  // данные о проекции
  // Структурa MTRPROJECTIONDATA описанa в maptype.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrProjectionData(HMAP hMap, long int number,
                                                 MTRPROJECTIONDATA *projectiondata);

  // Запросить данные о проекции матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // mapregister - адрес структуры, в которой будут размещены
  // данные о проекции
  // Структурa MAPREGISTEREX описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrProjectionDataEx(HMAP hMap, long int number,
                                                   MAPREGISTEREX *mapregister);

  // Запросить данные о проекции матрицы по имени файла
  // name        - имя файла матрицы
  // mapregister - адрес структуры, в которой будут размещены
  // данные о проекции
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrProjectionDataByName(const char *name,
                                                       MAPREGISTEREX *mapregister);
_MAPIMP  long int _MAPAPI mapGetMtrProjectionDataByNameUn(const WCHAR *name,
                                                          MAPREGISTEREX *mapregister);

  // Установить данные о проекции матрицы
  // ВНИМАНИЕ: помимо данных о проекции функция переносит маштаб mapregister->Scale
  // в заголовок матрицы.
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер матрицы в цепочке.
  // mapregister - адрес структуры, содержащей данные о проекции
  // Структурa MAPREGISTEREX описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrProjectionData(HMAP hMap, long int number,
                                                 MAPREGISTEREX *mapregister);

  // Установить данные о проекции матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер матрицы в цепочке.
  // mapregister, datumparam, ellipsoidparam - адреса структур, содержащих данные о проекции
  // Структуры MAPREGISTEREX, DATUMPARAM, ELLIPSOIDPARAM описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrProjectionDataEx(HMAP hMap, long int number,
                                                   MAPREGISTEREX  *mapregister,
                                                   DATUMPARAM     *datumparam,
                                                   ELLIPSOIDPARAM *ellipsoidparam);

  // Записать изменения матрицы в файл
  // hMap       - идентификатор открытой карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveMtr(HMAP hMap, long int number);

  // Запросить параметры эллипсоида матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла матрицы в цепочке
  // ellipsoidparam - адрес структуры, в которой будут размещены
  // параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrEllipsoidParam(HMAP hMap, long int number,
                                                 ELLIPSOIDPARAM *ellipsoidparam);

  // Установить параметры эллипсоида матрицы
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла матрицы в цепочке.
  // ellipsoidparam - адрес структуры, содержащей параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrEllipsoidParam(HMAP hMap, long int number,
                                                 const ELLIPSOIDPARAM *ellipsoidparam);

  // Запросить коэффициенты трансформирования геодезических координат матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла матрицы в цепочке
  // datumparam - адрес структуры, в которой будут размещены
  // коэффициенты трансформирования геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrDatumParam(HMAP hMap, long int number,
                                             DATUMPARAM *datumparam);

  // Установить коэффициенты трансформирования геодезических координат матрицы
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла матрицы в цепочке.
  // datumparam - адрес структуры, содержащей коэффициенты трансформирования
  // геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrDatumParam(HMAP hMap, long int number,
                                             const DATUMPARAM *datumparam);

 // Вычисление значения абсолютной высоты (point->H) в заданной       // 22/04/16
 // точке (point->X,point->Y) по данным векторной карты
 // Координаты точки задаются в метрах в системе координат
 // векторной карты
 // В случае ошибки при вычислении высоты возвращает 0

_MAPIMP long int _MAPAPI mapCalcAbsoluteHeight(HMAP hMap, XYHDOUBLE* point);

 // Вычисление значения абсолютной высоты в заданной                  // 22/04/16
 // точке (point->X,point->Y) по данным векторной карты
 // Координаты точки задаются в метрах в системе координат
 // векторной карты
 // sectorcount - количество направлений для поиска окружающих высот
 // (должно быть кратно 4, минимальное количество направлений = 4,
 //  максимальное = 256)
 // Возвращает значение высоты в метрах
 // В случае ошибки при вычислении высоты возвращает ERRORHEIGHT

_MAPIMP double _MAPAPI mapCalcAbsoluteHeightBySectors(HMAP hMap,
                                                      DOUBLEPOINT* point,
                                                      long int sectorcount);

  /*********************************************************
  *                                                        *
  *          УПРАВЛЕНИЕ ОТОБРАЖЕНИЕМ МАТРИЦЫ               *
  *                                                        *
  *********************************************************/

  // Запросить/Установить степень видимости матрицы
  //  hMap   - идентификатор открытой основной векторной карты
  //  number - номер файла в цепочке
  //  view = 0 - не виден
  //  view = 1 - полная
  //  view = 2 - насыщенная
  //  view = 3 - полупрозрачная
  //  view = 4 - средняя
  //  view = 5 - прозрачная

_MAPIMP long int _MAPAPI mapGetMtrView(HMAP hMap,long int number);
_MAPIMP long int _MAPAPI mapSetMtrView(HMAP hMap,long int number,
                                       long int view);

  // Запросить прозрачность палитры матрицы                   // 19/01/14
  // number - номер файла в цепочке
  // Возвращает степень прозрачности в процентах от 0 до 100

_MAPIMP long int _MAPAPI mapGetMtrTransparent(HMAP hMap, long int number);

  // Установить прозрачность палитры матрицы                  // 19/01/14
  // hMap   - идентификатор открытых данных
  // number - номер файла в цепочке
  // transparent - прозрачность в процентах от 0 до 100
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrTransparent(HMAP hMap, long int number,
                                              long int transparent);

  // Запросить/Установить порядок отображения матрицы
  //  hMap   - идентификатор открытой основной векторной карты 
  //  number - номер файла в цепочке
  //  order  - порядок (0 - под картой, 1 - над картой)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrViewOrder(HMAP hMap, long int number, long int order);
_MAPIMP long int _MAPAPI mapGetMtrViewOrder(HMAP hMap, long int number);

  // Поменять очередность отображения матриц (mtr) в цепочке
  //   oldNumber - номер файла в цепочке
  //   newNumber - устанавливаемый номер файла в цепочке
  //  При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapChangeOrderMtrShow(HMAP hMap, long int oldNumber,
                                                          long int newNumber);

  // Установить/Запросить глубину тени матрицы высот
  //  hMap  - идентификатор открытой основной векторной карты
  //  value - флаг наложения тени (от 0 до 16)
  //   MTRSHADOW_NONE   =  0,   // Тень отсутствует
  //   MTRSHADOW_PALE   =  1,   // Бледная
  //   MTRSHADOW_WEAK   =  2,   // Слабая
  //   MTRSHADOW_MIDDLE =  4,   // Средняя
  //   MTRSHADOW_HEAVY  =  8,   // Сильная
  //   MTRSHADOW_DEEP   = 16,   // Глубокая

_MAPIMP long int _MAPAPI mapGetMtrShadow(HMAP hMap);
_MAPIMP long int _MAPAPI mapSetMtrShadow(HMAP hMap, long int value);


  // Установить/Запросить интенсивность тени матрицы высот
  //  hMap  - идентификатор открытой основной векторной карты
  //  value - интенсивность тени (от 0 до 100)

_MAPIMP long int _MAPAPI mapGetMtrShadowIntensity(HMAP hMap);
_MAPIMP long int _MAPAPI mapSetMtrShadowIntensity(HMAP hMap, long int value);

  // Запросить число цветов в палитре матриц высот
  // ВСЕ МАТРИЦЫ ВЫСОТ РАБОТАЮТ С ОДНОЙ ПАЛИТРОЙ
  //  hMap   - идентификатор открытой основной векторной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrPaletteCount(HMAP hMap);

  // Запросить текущую палитру матрицы высот
  // (с учетом яркости/контрастности)
  // hMap    - идентификатор открытой основной векторной карты 
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrPalette(HMAP hMap, COLORREF *palette, long int count);

  // Запросить эталонную палитру матрицы высот
  // (без учета яркости/контрасности)
  // hMap    - идентификатор открытой основной векторной карты 
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrStandardPalette(HMAP hMap, COLORREF *palette, long int count);

  // Установить описание палитры матрицы высот
  // hMap    - идентификатор открытой основной векторной карты
  // palette - адрес устанавливаемой палитры
  // count   - число элементов в новой палитре
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrPalette(HMAP hMap, COLORREF *palette, long int count);


  // Запросить яркость палитры матрицы высот
  // hMap   - идентификатор открытой основной векторной карты

_MAPIMP long int _MAPAPI mapGetMtrBright (HMAP hMap);

  // Установить яркость палитры матрицы высот
  // hMap   - идентификатор открытой основной векторной карты 
  // bright - значение яркости (-16..+16) 
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrBright (HMAP hMap, long int bright);


  // Запросить контрастность палитры матрицы высот
  // hMap     - идентификатор открытой основной векторной карты 
  // значение контраста (-16..+16) 
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrContrast (HMAP hMap);

  // Установить контрастность палитры матрицы высот
  // hMap     - идентификатор открытой основной векторной карты 
  // contrast - значение контраста (-16..+16) 
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrContrast (HMAP hMap, long int contrast);

  // Запросить стиль палитры матрицы высот
  // hMap     - идентификатор открытой основной векторной карты 
  // Возвращаемое значение:
  //  0  - полутоновая палитра
  //  1  - цветная палитра
  // -1  - отображаются только тени от рельефа     

_MAPIMP long int _MAPAPI mapGetMtrColorStyle(HMAP hMap);

  // Установить стиль палитры матрицы высот
  // hMap       - идентификатор открытой основной векторной карты 
  // colorstyle - стиль отображения матрицы:
  //  0  - полутоновая палитра
  //  1  - цветная палитра
  // -1  - отображаются только тени от рельефа     
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrColorStyle (HMAP hMap, long int colorstyle);

  // Установить цвет диапазона высот матрицы с номером
  // number в цепочке
  // hMap        - идентификатор открытой основной векторной карты 
  // colornumber - номер диапазона высот
  // color       - цвет диапазона
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrColor(HMAP hMap, long int number,
                                        long int colornumber, COLORREF color);

  // Установить рамку матрицы по метрике замкнутого объекта
  // Замкнутый объект должен иметь не менее 4-х точек
  // hMap       - идентификатор открытой основной векторной карты 
  // number     - номер файла в цепочке
  // info       - замкнутый объект карты
  // После выполнения функции отображение матрицы ограничится заданной областью
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrBorder(HMAP hMap, long int number, HOBJ info);


  // Установить рамку матрицы по метрике замкнутого объекта
  // Замкнутый объект должен иметь не менее 4-х точек
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // info       - замкнутый объект карты
  // flagSubject- флаг использования подобъектов объекта при установке рамки матрицы  (0/1)
  //              0 - в качестве рамки матрицы устанавливается контур объекта
  //              1 - в качестве рамки матрицы устанавливается контур объекта с подобъектами
  // После выполнения функции отображение растра ограничится заданной областью
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetMtrBorderEx(HMAP hMap, long int number,
                                            HOBJ info, long int flagSubject);

  // Удалить рамку матрицы
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // После выполнения функции отображение матрицы будет полным
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMtrBorder(HMAP hMap, long int number);

  // Определение существования рамки матрицы
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckExistenceMtrBorder(HMAP hMap, long int number);

  // Определение способа отображения матрицы(относительно рамки)
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // Возвращает 1 - при отображении матрицы по рамке
  //            0 - при отображении матрицы без учета рамки
  // При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapCheckShowMtrByBorder(HMAP hMap, long int number);

  // Установка отображения матрицы по рамке
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // value = 1 - отобразить матрицу  по рамке
  //       = 0 - отобразить матрицыу  без учета рамки

_MAPIMP long int _MAPAPI mapShowMtrByBorder(HMAP hMap, long int number, long int value);

  // Запросить объект рамки матрицы
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // info       - идентификатор объекта рамки матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrBorder(HMAP hMap, long int number,HOBJ info);

  // Определить координаты и порядковый номер точки рамки, которая
  // входит в прямоугольник Габариты растра(матрицы) и
  // имеет наименьшее удаление от точки pointIn (координаты в метрах).
  // По адресу pointOut записываются координаты найденной точки в метрах
  // hMap       - идентификатор открытой основной векторной карты 
  // number     - номер файла в цепочке
  // При ошибке или отсутствии рамки возвращает 0.

_MAPIMP long int _MAPAPI mapGetImmediatePointOfMtrBorder(HMAP hMap, long int number,
                                                         DOUBLEPOINT *pointIn,
                                                         DOUBLEPOINT *pointOut);

  // Запросить состояние матрицы
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // Возвращаемые значения:
  //          0 - нет данных; или создание уменьшенных копий и сжатие не выполнялись
  //          3 - создание всех уровней уменьшенных копий, сжатие RMF_COMPR_32 матрицы
  //          4 - создание всех уровней уменьшенных копий матрицы

_MAPIMP long int _MAPAPI mapGetMtrProcessingState(HMAP hMap, long int number);

  // Установить состояние матрицы
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // state      - состояние матрицы.
  // Возможные значения состояния матрицы state:
  //          0 - нет данных; или создание уменьшенных копий и сжатие не выполнялись
  //          3 - создание всех уровней уменьшенных копий, сжатие RMF_COMPR_32 матрицы
  //          4 - создание всех уровней уменьшенных копий матрицы
  // При ошибке возвращает 0.

_MAPIMP long int _MAPAPI mapSetMtrProcessingState(HMAP hMap, long int number, long int state);

  // Оптимизировать матрицу для открытия в ГИС Сервер                           
  // Функция проверяет состояние матрицы и при необходимости выполняет для неё
  // оптимизацию со сжатием и создание всех уровней уменьшенной копии.
  // Необходимо закрыть матрицу из всех документов перед вызовом функции
  // mtrName- имя файла матрицы
  // handle - идентификатор окна, которое будет извещаться
  // о ходе процесса (0x585 - 0x588)
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapOptimizationMtrByName(const char* mtrName, HWND handle);
_MAPIMP long int _MAPAPI mapOptimizationMtrByNameUn(const WCHAR* mtrName, HWND handle);


  // Запросить количество созданных уменьшенных копий в матрице
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMtrDuplicatesCount(HMAP hMap, long int number);

  // Обновить уменьшенную копию
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapUpdateMtrDuplicates(HMAP hMap, long int number);

  /*********************************************************
  *                                                        *
  *          ТРЕХМЕРНОЕ ОТОБРАЖЕНИЕ МАТРИЦЫ ВЫСОТ          *
  *                                                        *
  *********************************************************/

  // Открыть сеанс трехмерной визуализации местности,
  // обеспеченной открытыми матрицами высот
  // hMap   - идентификатор открытой основной векторной карты 
  // number - номер файла в цепочке
  // width  - ширина изображения,
  // height - высота изображения.
  // Возвращает идентификатор открытого сеанса (TMtr3D*)
  // При ошибке возвращает ноль

_MAPIMP HMTR3D _MAPAPI mapOpenMtr3D(HMAP hMap, long int width, long int height);


  // Закрыть сеанс трехмерной визуализации местности
  // hMtr3D - идентификатор открытого сеанса 3D визуализации

_MAPIMP void _MAPAPI mapCloseMtr3D(HMTR3D hMtr3D);


  // Отобразить фрагмент местности в трехмерном виде
  // hMtr3D - идентификатор открытого сеанса 3D визуализации
  // parm   - параметры отображения (см. MAPTYPE.H)
  // hDC    - контекст отображения
  // Устаревшая функция (рекомендуется пользоваться mapPaintMtr3Dx)

_MAPIMP void _MAPAPI mapPaintMtr3D(HMTR3D hMtr3D, HDC hDC, MTR3DVIEW* parm);


  // Отобразить фрагмент местности в трехмерном виде
  // hMtr3D - идентификатор открытого сеанса 3D визуализации
  // parm   - параметры отображения (см. MAPTYPE.H)
  // hDC    - контекст отображения

_MAPIMP void _MAPAPI mapPaintMtr3Dx(HMTR3D hMtr3D, HDC hDC, MTR3DVIEWEX* parm);

  // Построить BITMAP с изображением фрагмента местности в трехмерном виде
  // hMtr3D - идентификатор открытого сеанса 3D визуализации
  // parm   - параметры отображения (см. MAPTYPE.H)
  // При ошибке возвращает ноль

_MAPIMP HBITMAP _MAPAPI mapBuildMtr3D(HMTR3D hMtr3D, MTR3DVIEW* parm);

  // Построить BITMAP с изображением фрагмента местности в трехмерном виде
  // hMtr3D - идентификатор открытого сеанса 3D визуализации
  // parm   - параметры отображения (см. MAPTYPE.H)
  // При ошибке возвращает ноль

_MAPIMP HBITMAP _MAPAPI mapBuildMtr3Dx(HMTR3D hMtr3D, MTR3DVIEWEX* parm);

  /*********************************************************
  *                                                        *
  *          ОПИСАНИЕ ФУНКЦИЙ ДЛЯ СОЗДАНИЯ МАТРИЦ          *
  *                                                        *
  *********************************************************/

  // Создать матричную карту
  // mtrname - имя файла создаваемой матрицы
  // Возвращает идентификатор открытой матричной карты
  // Структуры BUILDMTW,MTRPROJECTIONDATA описаны в maptype.h
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapCreateMtw(const char *mtrname, BUILDMTW *mtrparm,
                                  MTRPROJECTIONDATA *mtrprojectiondata);
_MAPIMP HMAP _MAPAPI mapCreateMtwUn(const WCHAR *mtrname, BUILDMTW *mtrparm,
                                    MTRPROJECTIONDATA *mtrprojectiondata);

  // Вывод прямоугольного участка матрицы
  //   hMap - идентификатор открытой матричной карты (TMapAccess *)
  //   number - номер файла в цепочке
  //   bits - адрес логического начала выводимого участка
  //          (см. beginning)
  //   left - смещение участка матрицы слева (в элементах)
  //   top - смещение участка матрицы сверху (в элементах)
  //   width - ширина участка матрицы (в элементах)
  //   height - высота участка матрицы (в элементах)
  //   beginning - определяет, на какую строку указывает bits :
  //     если beginning == 0, то bits указывает
  //       на начало верхней строки выводимого участка
  //     если beginning == 1, то bits указывает
  //       на начало нижней строки выводимого участка.
  //   Размер участка, заданного адресом bits, должен быть не менее
  //   (width * height * размер элемента матрицы в байтах),
  //   в противном случае возможны ошибки работы с памятью.
  //   Запрос размера элемента матрицы в байтах
  //   - функция mapGetMtrElementSize.
  //   Высоты выводимого участка должны быть записаны
  //   в области bits в единицах измерения высот данной матрицы.
  //   Запрос единицы измерения высоты матрицы
  //   - функция mapGetMtrMeasure.
  // При ошибке возвращает ноль

_MAPIMP long int  _MAPAPI mapPutMtrFrame(HMAP  hMap, long int number, char *bits,
                                         long int left, long int top,
                                         long int width, long int height,
                                         long int beginning/* = 0*/);

  // Чтение прямоугольного участка матрицы в заданную область памяти
  // hMap - идентификатор открытой матричной карты (TMapAccess *)
  // number - номер файла в цепочке
  // bits - указатель на начало области памяти
  // left - смещение участка матрицы слева (в элементах)
  // top - смещение участка матрицы сверху (в элементах)
  // width - ширина участка матрицы (в элементах)
  // height - высота участка матрицы (в элементах)
  // widthinbyte - ширинa участка матрицы в байтах
  // Размер участка, заданного адресом bits, должен быть не менее
  // (width * height * размер элемента матрицы в байтах),
  // в противном случае возможны ошибки работы с памятью.
  // Запрос размера элемента матрицы в байтах - функция mapGetMtrElementSize.
  // Высоты участка записываются в область bits в единицах измерения
  // значений высот данной матрицы.
  // Запрос единицы измерения значений высот матрицы - функция mapGetMtrMeasure.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrFrame(HMAP hMap, long int number, char *bits,
                                        long int left, long int top,
                                        long int width, long int height,
                                        long int widthinbyte = 0);

  // Запросить диапазон высот рельефа (суммарный диапазон всех матриц
  // высот, слоев, TIN-моделей)
  // hMap     - идентификатор открытой карты (TMapAccess*)
  // minvalue - минимальная высота диапазона в метрах
  // maxvalue - максимальная высота диапазона в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetReliefRange(HMAP hMap, double *minvalue,
                                           double *maxvalue);

  // Занести в матрицу диапазон значений высот
  // hMap     - идентификатор открытой основной карты (TMapAccess *)
  // number   - номер матрицы в цепочке
  // minvalue - минимальная высота диапазона в метрах
  // maxvalue - максимальная высота диапазона в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrShowRange(HMAP hMap, long int number,
                                            double minvalue, double maxvalue);


  // Запросить диапазон значений высот матрицы
  // hMap     - идентификатор открытой основной карты (TMapAccess *)
  // number   - номер матрицы в цепочке
  // minvalue - минимальная высота диапазона в метрах
  // maxvalue - максимальная высота диапазона в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrShowRange(HMAP hMap, long int number,
                                            double *minvalue, double *maxvalue);

  // Установить диапазон отображаемых высот всей цепочки матриц.
  // Установленный диапазон в матрицу не заносится (сохраняется
  // в INI-файл документа)
  //   hMap - идентификатор открытой основной карты
  //   minvalue - минимальная высота диапазона в метрах
  //   maxvalue - максимальная высота диапазона в метрах
  //   viewOutRange - отображать элементы матрицы, значения которых
  //                  вне диапазона (0 или 1). Если viewOutRange == 0,
  //                  то значения вне диапазона не отображаются.
  //                  Если viewOutRange == 1, то:
  //                - элементы, имеющие значения высот < minvalue,
  //                  отображаются первым цветом палитры матрицы;
  //                - элементы, имеющие значения высот > maxvalue,
  //                  отображаются последним цветом палитры матрицы.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrHeightRangeEx(HMAP hMap,
                                        double minvalue,
                                        double maxvalue,
                                        long int viewOutRange);  // 27/02/14

  // Установить диапазон отображаемых высот всей цепочки матриц.
  // Элемент матрицы не отображается, если он содержит высоту,
  // не входящую в заданный диапазон.
  // Установленный диапазон в матрицу не заносится (сохраняется
  // в INI-файл документа)
  //   hMap - идентификатор открытой основной карты
  //   minvalue - минимальная высота диапазона в метрах
  //   maxvalue - максимальная высота диапазона в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrHeightRange(HMAP hMap, double minvalue,
                                              double maxvalue);


  // Запросить диапазон высот отображаемых элементов
  // цепочки матриц.
  //   hMap - идентификатор открытой основной карты (TMapAccess *)
  //   minvalue - минимальная высота диапазона в метрах
  //   maxvalue - максимальная высота диапазона в метрах
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrHeightRange(HMAP hMap, double* minvalue,
                                              double* maxvalue);

  // Установить суммарный диапазон высот отображаемых элементов
  // цепочки матриц. Суммарный диапазон включает в себя диапазоны
  // всех матриц цепочки.
  // hMap - идентификатор открытой основной карты (TMapAccess *)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapResetMtrHeightRange(HMAP hMap);


  // Настроить параметры отображения цепочки матриц при изменении
  // числа цветов палитры и диапазона отображаемых высот.
  // hMap - идентификатор открытой основной карты (TMapAccess *)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrShowVariables(HMAP hMap);


  // Установить условное имя матрицы
  // hMap   - идентификатор открытой основной карты (TMapAccess *)
  // number - номер матрицы в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtrUserName(HMAP hMap, long int number,
                                           char *username);
_MAPIMP long int _MAPAPI mapSetMtrUserNameUn(HMAP hMap, long int number,
                                             const WCHAR *username);

  // Запросить условное имя матрицы
  //   hMap - идентификатор открытой основной карты (TMapAccess *)
  //   number - номер матрицы в цепочке
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetMtrUserName(HMAP hMap, long int number);

  // Запросить условное имя матрицы
  //   hMap   - идентификатор открытой основной карты (TMapAccess *)
  //   number - номер матрицы в цепочке
  //   name   - адрес строки в которую записывается условное имя матрицы
  //   namesize - размер строки в БАЙТАХ
  // При ошибке возвращает ноль
_MAPIMP long int _MAPAPI mapGetMtrUserNameUn(HMAP hMap, long int number,
                                             WCHAR *name, long int namesize);

  // Запросить координаты Юго-Западного угла матрицы в метрах
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла в цепочке
  // По адресу x,y записываются координаты найденной точки в метрах
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetSouthWestMtrPlane(HMAP hMap, long int number,
                                                 double *x, double *y);

  // Запросить активную матрицу
  // (устанавливается приложением по своему усмотрению)
  // hMap - идентификатор открытой карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActiveMtr(HMAP hMap);

  // Установить активную матрицу
  // (устанавливается приложением по своему усмотрению)
  // hMap - идентификатор открытой карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetActiveMtr(HMAP hMap, long int number);

  // Запросить флаг редактируемости матрицы
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrEdit(HMAP hMap, long int number);

  // Запросить размер файла
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // По адресу fileSize записывается размер файла
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrFileSize(HMAP hMap, long int number, __int64 *fileSize);

  // Запросить ширину матрицы (элементы)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrWidthInElement(HMAP hMap, long int number);

  // Запросить высоту матрицы (элементы)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrHeightInElement(HMAP hMap, long int number);

  // Запросить точность (метр/элем) матрицы
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrAccuracy(HMAP hMap, long int number);

  // Запросить флаг изменения привязки (метры) матрицы
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrFlagLocationChanged(HMAP hMap, long int number);

  // Запросить тип матрицы             /
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrType(HMAP hMap, long int number);

  // Запросить флаг сжатия матрицы
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtrCompress(HMAP hMap, long int number);

  // Занести в матрицу высот номер алгоритма сжатия блоков
  //   hMap - идентификатор открытой матричной карты (TMapAccess *)
  //   number - номер матрицы в цепочке
  //   compressnumber - номер алгоритма сжатия блоков
  // При ошибке возвращает 0

 _MAPIMP long int _MAPAPI mapSetMtrCompressNumber(HMAP hMap, long int number,
                                                  long int compressnumber);

 _MAPIMP long int _MAPAPI mapSetMtqCompressNumber(HMAP hMap, long int number,
                                                  long int compressnumber);

  // Удалить файл матрицы высот
  // Функция предназначена для удаления матрицы высот и еe составных частей
  // Матрица высот размером более 4Gb состоит из 2-х файлов: *.mtw и *.mtw.01
  // Аналог функции DeleteTheFile()

_MAPIMP long int _MAPAPI mapDeleteMtrFile(const char *name);
_MAPIMP long int _MAPAPI mapDeleteMtrFileUn(const WCHAR *filename);

  // Переименовать имя файла матрицы высот
  // Функция предназначена для переименовывания матрицы высот и её составных частей
  // Матрица высот размером более 4Gb состоит из 2-х файлов: *.mtw и *.mtw.01
  // Аналог функции MoveTheFile()

_MAPIMP long int _MAPAPI mapMoveMtrFile(const char *oldname, const char *newname);
_MAPIMP long int _MAPAPI mapMoveMtrFileUn(const WCHAR *oldname, const WCHAR *newname);

  // Скопировать файл матрицы высот
  // Функция предназначена для копирования матрицы высот и её составных частей
  // Матрица высот размером более 4Gb состоит из 2-х файлов: *.mtw и *.mtw.01
  // Аналог функции CopyTheFile()

_MAPIMP long int _MAPAPI mapCopyMtrFile(const char *oldname, const char *newname, int exist = 0);
_MAPIMP long int _MAPAPI mapCopyMtrFileUn(const WCHAR *oldname, const WCHAR *newname, int exist = 0);

  // Функция подготовки легенды матрицы высот
  // При необходимости создания изображения нестандартного размера (отличного от 16x16, 24x24 и 32x32)
  // указать размер в параметре imgsize
  // hmap      - идентификатор открытой векторной карты    
  // xmlname   - имя выходного xml-файла
  // imgpath   - путь к изображениям формата png
  // imgsize   - нестандартный размер изображения (сторона квадрата),
  //             если равен нулю, то будут созданы только изображения размеров 16x16, 24x24 и 32x32)
  // Максимальный размер изображения 1024x1024
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCreateMtrLegendToXML(HMAP hmap,                                                
                                                 const WCHAR * xmlname,
                                                 const WCHAR * imgpath,                                                 
                                                 long int imgsize); 

#ifndef HIDEMTQ

  /*********************************************************
  *                                                        *
  *                    МАТРИЦЫ КАЧЕСТВ                     *
  *                                                        *
  *            Вместо высоты - любое значение,             *
  * для отображения задаются цвета по диапазонам значений  *
  *                                                        *
  *********************************************************/

  // Открыть матрицу качеств
  // Возвращает идентификатор открытой матричной карты
  // mtqname - имя открываемого файла
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // При ошибке возвращает ноль

_MAPIMP HMAP _MAPAPI mapOpenMtq(const char *mtqname, long int mode = 0);
_MAPIMP HMAP _MAPAPI mapOpenMtqUn(const WCHAR *mtqname, long int mode = 0);

  // Закрыть матрицу качеств
  // hMap - идентификатор открытой основной карты
  // number - номер закрываемой матрицы
  // если number = 0, закрываются все матрицы в окне
  // ЧТОБЫ ОСВОБОДИТЬ ВСЕ РЕСУРСЫ - НУЖНО ВЫЗВАТЬ mapCloseData(hMap)

_MAPIMP void _MAPAPI mapCloseMtq(HMAP hMap, long int number = 0);

  // Открыть данные матрицы качеств в заданном районе работ
  // (добавить в цепочку матриц качеств)
  // hMap - идентификатор открытой основной карты
  // mtqname - имя открываемого файла 
  // mode - режим чтения/записи (GENERIC_READ, GENERIC_WRITE или 0)
  // GENERIC_READ - все данные только на чтение
  // Возвращает номер файла в цепочке матриц
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenMtqForMap(HMAP hMap, const char *mtqname, long int mode);
_MAPIMP long int _MAPAPI mapOpenMtqForMapUn(HMAP hMap, const WCHAR *name, long int mode);

  // Закрыть данные матрицы качеств в заданном районе работ
  // hMap   - идентификатор открытой основной карты
  // number - номер матричного файла в цепочке
  // Если number = 0, закрываются все матричные данные
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCloseMtqForMap(HMAP hMap, long int number);

  // Запросить имя файла данных матрицы качеств
  // hMap - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP const char* _MAPAPI mapGetMtqName(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapGetMtqNameUn(HMAP hMap, long int number, WCHAR *name, long int namesize);

  // Запросить номер файла матрицы качеств в цепочке по имени файла
  // В цепочке номера матриц качеств начинаются с 1
  // name - имя файла матрицы качеств
  // При отсутствии файла матрицы качеств в цепочке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqNumber(HMAP hMap, const char *name);
_MAPIMP long int _MAPAPI mapGetMtqNumberByName(HMAP hMap, const char* name);
_MAPIMP long int _MAPAPI mapGetMtqNumberUn(HMAP hMap, const WCHAR *name);

  // Запросить число открытых файлов матриц качеств
  // hMap - идентификатор открытой основной карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqCount(HMAP hMap);

  // Запросить описание файла матрицы качеств
  // hMap   - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // describe - адрес структуры, в которой будет размещено
  // описание матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqDescribe(HMAP hMap, long int number, MTRDESCRIBE *describe);
_MAPIMP long int _MAPAPI mapGetMtqDescribeUn(HMAP hMap, long int number, MTRDESCRIBEUN *describe);

  // Запросить размер элемента матрицы качеств в байтах
  // hMap   - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // Возвращаемое значение 1 соответствует типу "unsigned char".
  // Возвращаемое значение 2 соответствует типу "short int".
  // Возвращаемое значение 4 соответствует типу "long int".
  // Возвращаемое значение 8 соответствует типу "double".
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqElementSize(HMAP hMap, long int number);


  // Запросить/Установить отображение матрицы качеств
  // hMap - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // view : 0 - матрицу не отображать, 1 - отображать

_MAPIMP long int _MAPAPI mapGetMtqView(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapSetMtqView(HMAP hMap, long int number, long int view);

  // Запросить/Установить порядок отображения матрицы качеств
  // hMap   - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // order  - порядок отображения (0 - под картой, 1 - над картой)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqViewOrder(HMAP hMap, long int number, long int order);
_MAPIMP long int _MAPAPI mapGetMtqViewOrder(HMAP hMap, long int number);

  // Поменять очередность отображения матриц (mtq) в цепочке
  // hMap   - идентификатор открытой основной карты
  // oldNumber - номер файла в цепочке
  // newNumber - устанавливаемый номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapChangeOrderMtqShow(HMAP hMap, long int oldNumber,
                                                          long int newNumber);

  // Запросить/Установить тень матрицы качества
  // hMap   - идентификатор открытой основной карты
  // number - номер файла в цепочке
  // value  - флаг наложения тени (1 - тень есть, 0 - нет тени)

_MAPIMP long int _MAPAPI mapGetMtqShadow(HMAP hMap,long int number);
_MAPIMP long int _MAPAPI mapSetMtqShadow(HMAP hMap,long int number, long int value);

  // Создать матрицу качеств
  // Возвращает идентификатор открытой матричной карты (TMapAccess*)
  // Структуры BUILDMTW,MTRPROJECTIONDATA описаны в maptype.h
  // palette - указатель на палитру
  // countpalette - количество цветов в палитре
  // name - имя файла создаваемой матрицы

_MAPIMP HMAP _MAPAPI mapCreateMtq(const char *name, BUILDMTW *parm,
                                  MTRPROJECTIONDATA *projectiondata,
                                  COLORREF* palette, int countpalette);
_MAPIMP HMAP _MAPAPI mapCreateMtqUn(const WCHAR *name, BUILDMTW *parm,
                                    MTRPROJECTIONDATA *projectiondata,
                                    COLORREF *palette, int countpalette);

  // Записать прямоугольный участок матрицы качеств из памяти
  //   hMap - идентификатор открытой матричной карты
  //   number - номер матрицы в цепочке.
  //   bits - адрес логического начала записываемого участка
  //          (см. beginning)
  //   left - смещение участка матрицы слева (в элементах)
  //   top - смещение участка матрицы сверху (в элементах)
  //   width - ширина участка матрицы (в элементах)
  //   height - высота участка матрицы (в элементах)
  //   beginning - определяет, на какую строку указывает bits :
  //     если beginning == 0, то bits указывает
  //       на начало верхней строки выводимого участка
  //     если beginning == 1, то bits указывает
  //       на начало нижней строки выводимого участка
  //   Размер участка, заданного адресом bits, должен быть не менее
  //   (width * height * размер элемента матрицы в байтах),
  //   в противном случае возможны ошибки работы с памятью.
  //   Запрос размера элемента матрицы качеств в байтах
  //   - функция mapGetMtqElementSize.
  //   Значения элементов участка матрицы в области bits должны быть
  //   записаны в единицах измерения значений данной матрицы качеств.
  //   Запрос единицы измерения значений матрицы качеств
  //   - функция mapGetMtqMeasure.
  // При ошибке возвращает ноль

_MAPIMP long int  _MAPAPI mapPutMtqFrame(HMAP hMap, long int number, char *bits,
                                         long int left, long int top,
                                         long int width, long int height,
                                         long int beginning);

  // Установить диапазон отображаемых элементов матрицы качеств
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы в цепочке.
  // minvalue,maxvalue - границы диапазона
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqShowRange(HMAP hMap, long int number,
                                            double minvalue, double maxvalue);

  // Установить флаги отображения элементов матрицы качеств
  // вне границ диапазона, заданного в функции mapSetMtqShowRange
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы в цепочке.
  // viewUp - отображать элементы, значения которых больше верхней
  //          границы диапазона (параметр maxvalue функции mapSetMtqShowRange)
  // viewDown - отображать элементы, значения которых меньше нижней
  //          границы диапазона (параметр minvalue функции mapSetMtqShowRange)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqViewOutRange(HMAP hMap, long int number,
                                               long int viewUp, long int viewDown);

  // Запросить описание палитры матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqPalette(HMAP hMap, COLORREF *palette,
                                          long int count, long int number);


  // Запросить эталонную палитру матрицы качеств
  // (без учета яркости и контрастности)
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // palette - адрес области для размещения палитры
  // count   - число считываемых элементов палитры
  // (размер области в байтах / 4)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqStandardPalette(HMAP hMap, COLORREF *palette,
                                                  long int count, long int number);

  // Установить описание палитры матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // palette - адрес устанавливаемой палитры
  // count   - число элементов в новой палитре
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqPalette(HMAP hMap, COLORREF *palette,
                                          long int count, long int number);

  // Установить верхние значения диапазонов неравномерной палитры матрицы качеств
  // hMap     - идентификатор открытой основной карты
  // number   - номер матрицы в цепочке.
  // diapason - адрес массива устанавливаемых значений диапазонов
  // count    - число элементов в массиве значений диапазонов
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqPaletteDiapason(HMAP hMap, double *diapason,
                                                  long int count, long int number);

  // Удалить верхние значения диапазонов неравномерной палитры матрицы качеств
  // (палитра становится равномерной)
  // hMap     - идентификатор открытой матричной карты
  // number   - номер матрицы в цепочке.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapUnsetMtqPaletteDiapason(HMAP hMap, long int number);

  // Запросить верхние значения диапазонов неравномерной палитры и минимальное
  // значение элемента матрицы качеств
  // hMap     - идентификатор открытой матричной карты
  // minimum  - минимальное значение элемента матрицы (результат запроса)
  // diapason - адрес массива верхних значений диапазонов (результат запроса)
  // count    - число элементов в массиве значений диапазонов
  // number   - номер матрицы в цепочке
  // При ошибке и при неустановленных верхних значениях диапазонов возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqPaletteDiapason(HMAP hMap, double *minimum,
                                                  double *diapason, long int count,
                                                  long int number);

  // Сохранить в файле описание палитры матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // palette - адрес сохраняемой палитры
  //           (если palette = 0, то в файл записывается текущая
  //            палитра матрицы, установленная функцией mapSetMtqPalette)
  // count   - число элементов сохраняемой палитры
  // number  - номер матрицы в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveMtqPalette(HMAP hMap, COLORREF *palette,
                                           long int count, long int number);

  // Установить описание двухинтервальной палитры матрицы качеств
  // Двухинтервальная палитра формируется с использованием трёх
  // цветов (начального, промежуточного, конечного), задающих границы
  // двух интервалов. Составляющие интенсивности цветов внутри интервала
  // равномерно изменяются от начального цвета интервала к конечному.
  // hMap - идентификатор открытой основной карты
  // firstColor - адрес начального цвета первого интервала
  // mediumColor - адрес промежуточного цвета (конечного первого
  //               интервала, начального второго интервала)
  // lastColor - адрес конечного цвета второго интервала
  // count  - число элементов в палитре
  // mediumPosition - номер промежуточного цвета в палитре,
  //                  (число от 0 до count-1)
  // number - номер матрицы в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqTwoIntervalPalette(HMAP hMap,
                                                     COLORREF firstColor,
                                                     COLORREF mediumColor,
                                                     COLORREF lastColor,
                                                     long int count,
                                                     long int mediumPosition,
                                                     long int number);

  // Запросить/Установить яркость палитры матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // bright  - яркость (-16..+16)

_MAPIMP long int _MAPAPI mapGetMtqBright(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapSetMtqBright(HMAP hMap, long int number,
                                         long int bright);

  // Запросить/Установить контрастность палитры матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // contrast- контраст (-16..+16)

_MAPIMP long int _MAPAPI mapGetMtqContrast(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapSetMtqContrast(HMAP hMap, long int number,
                                           long int contrast);

  // Запросить число цветов в палитре матрицы качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // При ошибке возвращается 0

_MAPIMP long int _MAPAPI mapGetMtqPaletteCount(HMAP hMap, long int number);

  // Запросить описание диапазона значений матрицы с номером
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // colornumber - номер диапазона значений
  // colordesc - адрес структуры, в которой будет размещено
  // описание диапазона значений
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqColorDescEx(HMAP hMap, long int number,
                                              long int colornumber,
                                              MTRCOLORDESCEX *colordesc);

  // Установить цвет диапазона значений элементов матрицы с номером
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // colornumber - номер диапазона значений
  // color - цвет для диапазона
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqColor(HMAP hMap, long int number,
                                        long int colornumber,
                                        COLORREF color);

  // Выбор значения в заданной точке из матрицы
  // с номером number в цепочке.
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы в цепочке.
  // Координаты точки (x,y) задаются в метрах.
  // Возвращает значение элемента с учётом единицы измерения.
  // Возвращаемое значение равно значению элемента из файла матрицы,
  // делённому на 10 в степени n, где n = mapGetMtqMeasure().
  // В случае ошибки при выборе значения и в случае необеспеченности
  // заданной точки матричными данными возвращает ERRORHEIGHT.

_MAPIMP double _MAPAPI mapGetMtqValue(HMAP hMap, long int number,
                                      double x, double y);

  // Возвращает интерполированное значение из матрицы качеств
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы качеств в цепочке
  // interptype - тип интерполяции
  //          1 - ближайший сосед
  //          2 - интерполяция по ближайшим 3 элементам
  //          3 - билинейная интерполяция по 4 ближайшим элементам
  //          4 - бикубическая интерполяция по 16 ближайшим элементам
  // x, y  - координаты точки в метрах
  // value - возвращаемое значение (при ошибке устанавливается ERRORHEIGHT)
  // hPaint - идентификатор контекста отображения для многопоточного вызова функций,
  //          создается функцией mapCreatePaintControl, освобождается - mapFreePaintControl
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMtqValueEx(HMAP hMap, long int number, long int interptype,
                                          double x, double y, double *value);
_MAPIMP double _MAPAPI mapGetMtqValuePro(HMAP hMap, long int number, long int interptype,
                               double x, double y, double *value, HPAINT hPaint);

  // Чтение элемента матрицы качеств по абсолютным индексам
  // hMap   - идентификатор открытой карты
  // number - номер файла в цепочке
  // value  - полученное значение элемента в метрах
  // string - индекс строки матрицы (значение от 0 до height-1, где height - высота
  //          матрицы элементах, запрашиваемая функцией mapGetMtqHeightInElement)
  // column - индекс колонки матрицы (значение от 0 до width-1, где width - ширина
  //          матрицы элементах, запрашиваемая функцией mapGetMtqWidthInElement)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqPoint(HMAP hMap, long int number, double *value,
                                        long int string, long int column);

  // Занесение значения в элемент матрицы,
  // соответствующий заданной точке.
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // Координаты точки (x,y) задаются в метрах
  // В матрицу заносится значение элемента с учётом единицы измерения.
  // Заносимое значение равно h, умноженному на 10 в степени n,
  // где n = mapGetMtqMeasure().
  // В случае ошибки возвращает ноль.

_MAPIMP long int _MAPAPI mapPutMtqValue(HMAP hMap, long int number,
                                        double x, double y, double h);

  // Запросить фактические габариты отображаемой матрицы в метрах в районе работ
  // При отображение матрицы по рамке возвращаются габариты рамки
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // frame   - адрес для размещения результата
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActualMtqFrame(HMAP hMap, DFRAME *frame,
                                              long int number);

  // Запросить масштаб матрицы
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqScale(HMAP hMap, long int number);

  // Запрос - поддерживается ли пересчет к геодезическим
  // координатам из плоских прямоугольных и обратно
  // hMap     - идентификатор открытой основной карты
  // number   - номер файла в цепочке
  // Если нет - возвращает ноль

  _MAPIMP long int _MAPAPI mapIsMtqGeoSupported(HMAP hMap, long int number);

  // Запросить данные о проекции матричных данных
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // projectiondata - адрес структуры, в которой будут размещены
  // данные о проекции
  // Структурa MTRPROJECTIONDATA описанa в maptype.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqProjectionData(HMAP hMap, long int number,
                                                 MTRPROJECTIONDATA *projectiondata);

  // Установить данные о проекции матричных данных
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке.
  // mapregister - адрес структуры, содержащей данные о проекции
  // Структурa MAPREGISTEREX описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqProjectionData(HMAP hMap, long int number,
                                                 MAPREGISTEREX *mapregister);

  // Установить данные о проекции матрицы качеств
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер матрицы в цепочке.
  // mapregister, datumparam, ellipsoidparam - адреса структур, содержащих данные о проекции
  // Структуры MAPREGISTEREX, DATUMPARAM, ELLIPSOIDPARAM описаны в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqProjectionDataEx(HMAP hMap, long int number,
                                                   MAPREGISTEREX  *mapregister,
                                                   DATUMPARAM     *datumparam,
                                                   ELLIPSOIDPARAM *ellipsoidparam);

  // Запросить данные о проекции матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // mapregister - адрес структуры, в которой будут размещены
  // данные о проекции
  // Структурa MAPREGISTEREX описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqProjectionDataEx(HMAP hMap, long int number,
                                                   MAPREGISTEREX *mapregister);

  // Запросить параметры эллипсоида матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла матрицы в цепочке
  // ellipsoidparam - адрес структуры, в которой будут размещены
  // параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqEllipsoidParam(HMAP hMap, long int number,
                                                 ELLIPSOIDPARAM *ellipsoidparam);

  // Установить параметры эллипсоида матрицы
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла матрицы в цепочке.
  // ellipsoidparam - адрес структуры, содержащей параметры эллипсоида
  // Структурa ELLIPSOIDPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqEllipsoidParam(HMAP hMap, long int number,
                                                 const ELLIPSOIDPARAM *ellipsoidparam);

  // Запросить коэффициенты трансформирования геодезических координат матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла матрицы в цепочке
  // datumparam - адрес структуры, в которой будут размещены
  // коэффициенты трансформирования геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqDatumParam(HMAP hMap, long int number,
                                             DATUMPARAM *datumparam);

  // Установить коэффициенты трансформирования геодезических координат матрицы
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла матрицы в цепочке.
  // datumparam - адрес структуры, содержащей коэффициенты трансформирования
  // геодезических координат
  // Структурa DATUMPARAM описанa в mapcreat.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqDatumParam(HMAP hMap, long int number,
                                             const DATUMPARAM *datumparam);

  // Чтение прямоугольного участка матрицы качеств в заданную область памяти
  //   hMap - идентификатор открытой основной карты (TMapAccess *)
  //   number - номер матрицы в цепочке.
  //   bits - указатель на начало области памяти
  //   left - смещение участка матрицы слева (в элементах)
  //   top - смещение участка матрицы сверху (в элементах)
  //   width - ширина участка матрицы (в элементах)
  //   height - высота участка матрицы (в элементах)
  //   widthinbyte - ширинa участка матрицы в байтах
  //   Размер участка, заданного адресом bits, должен быть не менее
  //   (width * height * размер элемента матрицы в байтах),
  //   в противном случае возможны ошибки работы с памятью.
  //   Запрос размера элемента матрицы качеств в байтах
  //   - функция mapGetMtqElementSize.
  //   Значения элементов участка матрицы записываются в область bits
  //   в единицах измерения значений данной матрицы качеств.
  //   Запрос единицы измерения значений матрицы качеств
  //   - функция mapGetMtqMeasure.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqFrame(HMAP hMap, long int number, char *bits,
                                        long int left,  long int top,
                                        long int width, long int height,
                                        long int widthinbyte = 0);

  // Запросить единицу измерения значений матрицы качеств
  // hMap - идентификатор открытой основной карты (TMapAccess *)
  // number - номер матрицы в цепочке.
  // Значение элемента в файле матрицы равно значению качества,
  // умноженному на 10 в степени n, где n - единица измерения.
  // Функция возвращает значение поля Unit структуры параметров создания
  // матрицы BUILDMTW
  // Возвращаемые значения :
  //   0-"метры", 1-"дециметры", 2-"сантиметры", 3-"миллиметры"
  // При ошибке возвращает -1

_MAPIMP long int _MAPAPI mapGetMtqMeasure(HMAP hMap, long int number);

 // Построение зоны затопления по набору отметок уровня воды
 // Уровень воды задаётся величиной относительно поверхности рельефа
 // В результате построения формируется матрица качеств, элементы
 // которой содержат глубины в зоне затопления.
 // Габариты матрицы качеств определяются координатами точек с
 // отметками уровня воды (массив pointArray) и величиной расширения
 // габаритов области (areaExtension).
 // hMap    - исходная карта с матрицей высот для построения зоны затопления,
 // mtqName - полное имя создаваемой матрицы качеств,
 // pointArray - адрес массива точек с отметками уровня воды
 //   Координаты точек (pointArray->X,pointArray->Y) и значения уровня
 //   (pointArray->H) задаются в метрах в системе координат векторной
 //   карты,
 // pointCount - число точек в массиве pointArray
 //   Размер в байтах массива, заданного адресом pointArray, должен
 //   быть не менее pointCount * sizeof(XYHDOUBLE), в противном случае
 //   возможны ошибки работы с памятью,
 // areaExtension - положительное число, задающее величину
 //   расширения габаритов области в метрах,
 // minDepth - положительное число, задающее минимальную глубину
 //   зоны затопления в метрах (глубины, меньшие minDepth в матрицу
 //   качеств не заносятся),
 // handle - идентификатор окна диалога, которому посылаются
 // сообщения о ходе процесса :
 //   0x0581 - сообщение о проценте выполненных работ (в WPARAM),
 //   если процесс должен быть принудительно завершен, в ответ
 //   должно вернуться значение 0x0581.
 //   Если handle равно нулю - сообщения не посылаются.
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapBuildFloodZone(HMAP hMap, const char *mtqName,
                                           XYHDOUBLE *pointArray, long int pointCount,
                                           double areaExtension, double minDepth, HWND handle);
_MAPIMP long int _MAPAPI mapBuildFloodZoneUn(HMAP hMap, const WCHAR *mtqName,
                                             XYHDOUBLE *pointArray, long int pointCount,
                                             double areaExtension, double minDepth, HWND handle);

 // Построение зоны затопления по набору отметок уровня воды
 // Уровень воды задаётся абсолютным значением высоты
 // В результате построения формируется матрица качеств, элементы
 // которой содержат глубины в зоне затопления.
 // Габариты матрицы качеств определяются координатами точек с
 // отметками уровня воды (массив pointArray) и величиной расширения
 // габаритов области (areaExtension).
 // hMap    - исходная карта с матрицей высот для построения зоны затопления,
 // mtqName - полное имя создаваемой матрицы качеств,
 // pointArray - адрес массива точек с отметками уровня воды
 //   Координаты точек (pointArray->X,pointArray->Y) и значения уровня
 //   (pointArray->H) задаются в метрах в системе координат векторной
 //   карты,
 // pointCount - число точек в массиве pointArray
 //   Размер в байтах массива, заданного адресом pointArray, должен
 //   быть не менее pointCount * sizeof(XYHDOUBLE), в противном случае
 //   возможны ошибки работы с памятью,
 // areaExtension - положительное число, задающее величину
 //   расширения габаритов области в метрах,
 // minDepth - положительное число, задающее минимальную глубину
 //   зоны затопления в метрах (глубины, меньшие minDepth в матрицу
 //   качеств не заносятся),
 // handle - идентификатор окна диалога, которому посылаются
 // сообщения о ходе процесса :
 //   0x0581 - сообщение о проценте выполненных работ (в WPARAM),
 //   если процесс должен быть принудительно завершен, в ответ
 //   должно вернуться значение 0x0581.
 //   Если handle равно нулю - сообщения не посылаются.
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapBuildFloodZoneAbs(HMAP hMap, const char *mtqName,
                                              XYHDOUBLE *pointArray, long int pointCount,
                                              double areaExtension, double minDepth,
                                              HWND handle);
_MAPIMP long int _MAPAPI mapBuildFloodZoneAbsUn(HMAP hMap, const WCHAR *mtqName,
                                                XYHDOUBLE *pointArray, long int pointCount,
                                                double areaExtension, double minDepth,
                                                HWND handle);

 // Построение матрицы качеств по массиву значений характеристики качества.
 // hMap - идентификатор открытой исходной карты для построения матрицы качеств
 // mtqName - полное имя создаваемой матрицы качеств
 // palette - адрес палитры создаваемой матрицы качеств,
 //           если palette равно нулю - используется палитра по умолчанию
 // countpalette - количество цветов в палитре
 // pointArray - адрес массива значений характеристики качества
 //              Координаты точек (pointArray->X,pointArray->Y) задаются в метрах
 //              в системе координат векторной карты
 // pointCount - число точек в массиве pointArray
 //              Размер в байтах массива, заданного адресом pointArray, должен
 //              быть не менее pointCount * sizeof(XYHDOUBLE), в противном случае
 //              возможны ошибки работы с памятью
 // elemSizeMeters - размер стороны элементарного участка в метрах на местности
 //                  (дискрет матрицы)
 // minValue,maxValue - диапазон значений характеристики качества создаваемой матрицы
 //                     качеств, если minValue >= maxValue - в матрицу заносится
 //                     фактический диапазон значений из массива pointArray
 // handle - идентификатор окна диалога, которому посылаются
 //          сообщения о ходе процесса :
 //          0x0581 - сообщение о проценте выполненных работ (в WPARAM),
 //          если процесс должен быть принудительно завершен, в ответ
 //          должно вернуться значение 0x0581.
 //          Если handle равно нулю - сообщения не посылаются.
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapBuildMtq(HMAP hMap, const char *mtqName,
                                     const COLORREF *palette, long int countpalette,
                                     const XYHDOUBLE *pointArray, long int pointCount,
                                     double elemSizeMeters, double minValue,
                                     double maxValue, HWND handle);
_MAPIMP long int _MAPAPI mapBuildMtqUn(HMAP hMap, const WCHAR *mtqName,
                                       const COLORREF *palette, long int countpalette,
                                       const XYHDOUBLE *pointArray, long int pointCount,
                                       double elemSizeMeters, double minValue,
                                       double maxValue, HWND handle);

 // Формирование палитры матрицы качеств
 // Для формирования результирующей палитры (resultPalette) используется исходная
 // палитра (skeletPalette), количество цветов результирующей палитры
 // (resultColorCount) и флаг плавного изменения цветов (smoothColorModification).
 // skeletPalette - исходная (скелетная) палитра, массив размером sizeof(COLORREF)*256,
 //                 содержащий граничные цвета интервалов, разделённые пустыми
 //                 элементами (значение 0xFFFFFFFF);
 // resultPalette - результирующая палитра, массив размером sizeof(COLORREF)*256;
 // resultColorCount - количество формируемых цветов результирующей палитры
 //                    (не более 256);
 // smoothColorModification - флаг плавного изменения цветов результирующей палитры :
 //   0 - внутренние цвета интервала повторяют начальный цвет интервала,
 //   1 - составляющие интенсивности внутренних цветов интервала равномерно
 //       изменяются от начального цвета интервала к конечному.
 // Цвета исходной палитры переносятся в соответствующие позиции результирующей
 // палитры, остальные цвета результирующей палитры заполняются поинтервально в
 // зависимости от значения флага smoothColorModification.
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapMakeMtqPalette(COLORREF* skeletPalette,
                                           COLORREF* resultPalette,
                                           long int resultColorCount,
                                           long int smoothColorModification);

 // Построение матрицы поверхности (матрицы качеств или матрицы высот)
 // по данным векторной карты. Если mtrparm->FileMtw равно 1, то строится
 // матрица высот (*.mtw), иначе строится матрица качеств (*.mtq).
 // hMap - идентификатор открытой исходной карты для построения матрицы
 // mtrname - полное имя создаваемой матрицы
 // mtrparm - параметры создаваемой матрицы (структура BUILDSURFACE описана в maptype.h)
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI mapBuildMatrixSurface(HMAP hMap, const char *mtrname,
                                               BUILDSURFACE *mtrparm);
_MAPIMP long int _MAPAPI mapBuildMatrixSurfaceUn(HMAP hMap, const WCHAR *mtrname,
                                                 BUILDSURFACE *mtrparm);

  // Удалить матрицу качеств
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMtq(HMAP hMap, long int number);

  // Определение способа отображения матрицы (относительно рамки)
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке
  // Возвращает 1 - при отображении матрицы по рамке
  //            0 - при отображении матрицы без учета рамки

_MAPIMP long int _MAPAPI mapGetShowMtqByBorder(HMAP hMap, long int number);

  // Определение существования рамки матрицы
  // hMap    - идентификатор открытой основной карты
  // number  - номер матрицы в цепочке
  // Если рамка матрицы существует возвращает 1, иначе возвращает 0.

_MAPIMP long int _MAPAPI mapGetExistenceMtqBorder(HMAP hMap, long int number);

  // Установка отображения матрицы по рамке
  // hMap      - идентификатор открытой основной карты
  // number    - номер матрицы в цепочке
  // value = 1 - отобразить матрицу  по рамке
  //       = 0 - отобразить матрицу  без учета рамки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetShowMtqByBorder(HMAP hMap, long int number, long int value);
_MAPIMP long int _MAPAPI mapShowMtqByBorder(HMAP hMap, long int number, long int value);

  // Запросить значения масштаба нижней и верхней границ видимости матрицы
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы в цепочке
  // По адресу bottomScale записывается знаменатель масштаба нижней границы видимости матрицы
  // По адресу topScale записывается знаменатель масштаба верхней границы видимости матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqRangeScaleVisible(HMAP hMap, long int number,
                                                    long int *bottomScale,
                                                    long int *topScale);

  // Установить значения масштаба нижней и верхней границ видимости матрицы
  // hMap   - идентификатор открытой основной карты
  // number - номер матрицы в цепочке
  // bottomScale - знаменатель масштаба нижней границы видимости матрицы
  // topScale    - знаменатель масштаба верхней границы видимости матрицы
  //               bottomScale <= topScale, иначе возвращает 0
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqRangeScaleVisible(HMAP hMap, long int number,
                                                    long int bottomScale,
                                                    long int topScale);

  // Определить координаты и порядковый номер точки рамки, которая
  // входит в прямоугольник Габариты растра(матрицы) и
  // имеет наименьшее удаление от точки pointIn (координаты в метрах).
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла в цепочке
  // По адресу pointOut записываются координаты найденной точки в метрах
  // При ошибке или отсутствии рамки возвращает 0.

_MAPIMP long int _MAPAPI mapGetImmediatePointOfMtqBorder(HMAP hMap, long int number,
                                                         DOUBLEPOINT *pointIn,
                                                         DOUBLEPOINT *pointOut);

  // Запросить координаты Юго-Западного угла матрицы в метрах
  // hMap    - идентификатор открытой основной векторной карты
  // number  - номер файла в цепочке
  // По адресу x,y записываются координаты найденной точки в метрах
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetSouthWestMtqPlane(HMAP hMap,long int number,
                                                 double *x, double *y);

  // Запросить активную матрицу
  // (устанавливается приложением по своему усмотрению)
  // hMap - идентификатор открытой карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetActiveMtq(HMAP hMap);

  // Установить активную матрицу
  // (устанавливается приложением по своему усмотрению)
  // hMap - идентификатор открытой карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetActiveMtq(HMAP hMap, long int number);

  // Запросить флаг редактируемости матрицы
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetMtqEdit(HMAP hMap, long int number);

  // Запросить размер файла
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // По адресу fileSize записывается размер файла
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqFileSize(HMAP hMap, long int number, __int64 *fileSize);

  // Запросить ширину матрицы (элементы)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqWidthInElement(HMAP hMap, long int number);

  // Запросить высоту матрицы (элементы)
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqHeightInElement(HMAP hMap, long int number);

  // Запросить точность (метр/элем) матрицы             /
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqAccuracy(HMAP hMap, long int number);

  // Запросить флаг изменения привязки (метры) матрицы             /
  // hMap       - идентификатор открытой векторной карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqFlagLocationChanged(HMAP hMap, long int number);

  // Установить условное имя матрицы качеств (имя пользователя)
  // hMap   - идентификатор открытой основной карты (TMapAccess *)
  // number - номер матрицы в цепочке
  // username - имя пользователя
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqUserName(HMAP hMap, long int number, const char *username);
_MAPIMP long int _MAPAPI mapSetMtqUserNameUn(HMAP hMap, long int number, const WCHAR *username);

  // Запросить условное имя матрицы качеств (имя пользователя)
  // hMap   - идентификатор открытой основной карты (TMapAccess *)
  // number - номер матрицы в цепочке
  // namesize - размер строки в БАЙТАХ
  // При ошибке возвращает 0
_MAPIMP const char* _MAPAPI mapGetMtqUserName(HMAP hMap, long int number);
_MAPIMP long int _MAPAPI mapGetMtqUserNameUn(HMAP hMap, long int number, WCHAR *name, long int namesize);

  // Запросить число строк блоков матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqBlockRow(HMAP hMap, long int number);


  // Запросить число столбцов блоков матрицы
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqBlockColumn(HMAP hMap, long int number);

  // Запросить вертикальный размер блока матрицы в элементах
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqBlockSide(HMAP hMap, long int number);

  // Запросить ширину блока матрицы в элементах
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // При ошибке возвращает ноль
_MAPIMP long int _MAPAPI mapGetMtqBlockWidth(HMAP hMap, long int number);

  // Возврат флага отображения блока матрицы
  // (0 - не отображается, 1- отображается, 2 - разделен рамкой )
  // number - номер файла в цепочке
  // i - порядковый номер (индекс) блока, i = str * blockColumnCount + col, где:
  //     str - индекс строки блоков,
  //     blockColumnCount - число столбцов блоков матрицы (функция mapGetMtqBlockColumn)
  //     col - индекс столбца блоков
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCheckMtqBlockVisible(HMAP hMap, long int number, long int i);

  // Установить рамку матрицы по метрике замкнутого объекта
  // Замкнутый объект должен иметь не менее 4-х точек
  // hMap   - идентификатор открытой основной векторной карты
  // number - номер файла в цепочке
  // info   - замкнутый объект карты
  // После выполнения функции отображение матрицы ограничится заданной областью
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetMtqBorder(HMAP hMap, long int number, HOBJ info);

  // Запросить объект рамки матрицы качеств
  // hMap       - идентификатор открытой основной векторной карты
  // number     - номер файла в цепочке
  // info       - идентификатор объекта рамки матрицы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqBorder(HMAP hMap, long int number, HOBJ info);

  // Запросить ширину текущего блока column матрицы в элементах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqCurrentBlockWidth(HMAP hMap, long int number,
                                                    long int column);

  // Запросить высоту текущего блока string матрицы в элементах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqCurrentBlockHeight(HMAP hMap, long int number,
                                                     long int string);

  // Запросить блок матрицы по номеру строки и столбца
  // Блоки последнего ряда могут иметь усеченный размер
  // number - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP char* _MAPAPI mapGetMtqBlockAddress(HMAP hMap, int number, int row, int column);

  // Записать изменения матрицы в файл
  // hMap       - идентификатор открытой карты
  // number     - номер файла в цепочке
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveMtq(HMAP hMap, long int number);

  // Удалить файл матрицы качеств
  // Функция предназначена для удаления матрицы и еe составных частей
  // Матрица размером более 4Gb состоит из 2-х файлов: *.mtq и *.mtq.01
  // Аналог функции DeleteTheFile()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapDeleteMtqFile(const char *name);
_MAPIMP long int _MAPAPI mapDeleteMtqFileUn(const WCHAR *name);

  // Переименовать имя файла матрицы качеств
  // Функция предназначена для переименовывания матрицы и её составных частей
  // Матрица размером более 4Gb состоит из 2-х файлов: *.mtq и *.mtq.01
  // Аналог функции MoveTheFile()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapMoveMtqFile(const char *oldname, const char *newname);
_MAPIMP long int _MAPAPI mapMoveMtqFileUn(const WCHAR *oldname, const WCHAR *newname);

  // Скопировать файл матрицы качеств
  // Функция предназначена для копирования матрицы и её составных частей
  // Матрица размером более 4Gb состоит из 2-х файлов: *.mtq и *.mtq.01
  // Аналог функции CopyTheFile()
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCopyMtqFile(const char *oldname, const char *newname, int exist = 0);
_MAPIMP long int _MAPAPI mapCopyMtqFileUn(const WCHAR *oldname, const WCHAR *newname, int exist = 0);

  // Запись блока {string,column} в файл матрицы из памяти bits.
  // number    - номер файла в цепочке
  //  bits     - указатель на начало изображения битовой области
  //  sizebits - размер области bits в байтах
  // Возвращает количество записанных байт.
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapWriteMtqBlock(HMAP hMap, long int number,
                                          long int string, long int column,
                                          char *bits, long int sizebits);

  // Запросить размер текущего блока {string,column} матрицы в байтах
  // (с учетом усеченных блоков)
  // number - номер файла в цепочке
  // string, column - строка и столбец блока соотвественно
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetMtqCurrentBlockSize(HMAP hMap, long int number,
                                                   long int string, long int column);


  // Функция подготовки легенды матрицы качеств
  // При необходимости создания изображения нестандартного размера (отличного от 16x16, 24x24 и 32x32)
  // указать размер в параметре imgsize
  // hmap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // xmlname   - имя выходного xml-файла
  // imgpath   - путь к изображениям формата png
  // imgsize   - нестандартный размер изображения (сторона квадрата),
  //             если равен нулю, то будут созданы только изображения размеров 16x16, 24x24 и 32x32)
  // Максимальный размер изображения 1024x1024
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapCreateMtqLegendToXML(HMAP hmap, long int number,
                                                 const WCHAR * xmlname,
                                                 const WCHAR * imgpath,
                                                 long int imgsize);

  // Чтение названия характеристики матрицы качеств      // 25/05/16
  // hmap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMtqValueName(HMAP hMap, long int number, WCHAR * value, long int size);

  // Запись названия характеристики матрицы качеств      // 25/05/16
  // hmap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetMtqValueName(HMAP hMap, long int number, const WCHAR * value);

  // Чтение названия единицы измерения матрицы качеств      // 25/05/16
  // hmap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetMtqUnitName(HMAP hMap, long int number, WCHAR * unit, long int size);

  // Запись названия единицы измерения матрицы качеств      // 25/05/16
  // hmap      - идентификатор открытой векторной карты
  // number    - номер файла в цепочке
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetMtqUnitName(HMAP hMap, long int number, const WCHAR * value);


#endif  // !HIDEMTQ


  /************************************************************************************* 
  *                                                                                    *
  *  ЗАПРОС ДАННЫХ ИЗ МАТРИЦЫ ВЫСОТ ГЕОИДА НАД ПОВЕРХНОСТЬЮ ЭЛЛИПСОИДА WGS84 (EGM2008) *
  *                                                                                    *
  *************************************************************************************/

  // Открытие матрицы Egm2008
  // Доступны матрицы egm8_25.mtw (размер элемента 2.5 минуты) и
  //                  egm8_1.mtw  (размер элемента 1 минута)
  // Возвращает идентификатор открытой матрицы Egm2008
  // При ошибке возвращает 0

_MAPIMP HANDLE _MAPAPI mapOpenEgm2008(const char *mtrname);
_MAPIMP HANDLE _MAPAPI mapOpenEgm2008Un(const WCHAR *mtrname);

  // Закрытие матрицы Egm2008
  // mtr - идентификатор открытой матрицы Egm2008

_MAPIMP void _MAPAPI mapCloseEgm2008(HANDLE mtr);

  // Чтение высоты геоида над поверхностью эллипсоида wgs84
  // по геодезическим координатам на эллипсоиде WGS84
  // mtr - идентификатор открытой матрицы Egm2008
  // interptype - тип интерполяциия
  //          1 - ближайший сосед
  //          2 - интерполяция по ближайшим 3 элементам
  //          3 - билинейная интерполяция по 4 ближайшим элементам
  //          4 - бикубическая интерполяция по 16 ближайшим элементам
  // b - широта точки на эллипсоиде WGS84 в радианах
  // l - долгота точки на эллипсоиде WGS84 в радианах
  // h - возвращаемая высота геоида в метрах
  // При ошибке возвращает 0  

_MAPIMP long int _MAPAPI mapReadEgm2008(HANDLE mtr, long int interptype, double b, double l, double *h);




}  // extern "C"

#endif // MTRAPI_H
