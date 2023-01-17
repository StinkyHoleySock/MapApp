/****  GMLAPI.H  *********** Нефедьева О.А. ******* 20/09/2011 *****
*****  GMLAPI.H  *********** Belenkov  O.V. ******* 15/02/2016 *****
*****  GMLAPI.H  *********** Konon     V.N. ******* 28/09/2015 *****
*****  GMLAPI.H  *********** Shabakov D.A.  ******* 20/11/2014 *****
*****  GMLAPI.H  *********** Zheleznyakov V.A. **** 16/06/2016 *****
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*                     FOR  LINUX & MS WINDOWS                      *
*                                                                  *
********************************************************************
*                                                                  *
*       Интерфейс обработки запросов OGC WFS-T (gisu64acces.dll)   *
*                                                                  *
*******************************************************************/

#ifndef GMLAPI_H
#define GMLAPI_H

#ifndef MAPTYPE_H
 #include "maptype.h"
#endif

enum OGCSERVICETYPE       // Тип разметки или формат файла ответа
{
  OST_GML  = 0,           // Набор данных GML
  OST_WFS  = 1,           // GML для передачи по WFS
  OST_JSON = 2,           // JSON  
};

enum OGCSERVICEFLAG       // Вывод расширенных метаданных об объекте 
{
  OSF_METRIC      =   1,  // Вывод метрики  в описании объекта
  OSF_SEMANTIC    =   2,  // Вывод семантики в описании объекта
  OSF_MEASURE     =   4,  // Вывод площади и длины (периметра) объекта
  OSF_MEASURE3D   =   8,  // Вывод площади и длины (периметра) объекта с учетом рельефа местности
  OSF_CENTER      =  16,  // Вывод центра объекта
  OSF_SCALEBORDER =  32,  // Вывод верхней и нижней границы видимости объекта (масштабы отображения)
  OSF_POINT       =  64,  // Вывод первой точки объекта
  OSF_SEMANTICNAME=  128, // Вывод имени семантики при включенном флаге семантика
  OSF_CUTMETRIC   =  256, // Разрезание метрики объектов по области отбора объектов карты
  OSF_BBOX        =  512, // Вывод габаритов объекта
  OSF_KEY         = 1024  // Вывод ключа объекта
};
 
// Идентификатор GML данных
typedef HANDLE HGML;

// Идентификатор для загрузки GML данных
typedef HANDLE HGMLCLASS;

extern "C"
{
 // Открыть доступ к схеме для потокового формирования GML\JSON
 // Если классификатор размещен на ГИС Сервере, то схема открывается через
 // идентификатор карты (gmlOpen, gmlOpenEx, gmlOpenUn)
 // schemafilename - локальный путь к файлу XSD-схемы  
 // rscname        - локальный путь к файлу классификатору RSC,
 //                  с которым связаны карты, планируемые к обработке
 // hrsc - идентификатор классификатора (может быть получен в mapGetRscIdent, mapGetRscIdentByObject)
 // Возвращает идентификатор доступа к GML данным
 // При ошибке возвращает ноль

_MAPIMP HGML _MAPAPI gmlOpenPro(const WCHAR* schemafilename, const WCHAR * rscname);
_MAPIMP HGML _MAPAPI gmlOpenProEx(const WCHAR* schemafilename, HRSC hrsc);

 // Открыть доступ к прикладной схеме для заданной карты
 // hMap - идентификатор набора данных
 // schemafilename - локальный путь к файлу XSD-схемы GML данных
 // schemaURL      - URL к файлу XSD-схемы GML данных
 //                  например: "http://www.gisinfo.net/bsd/topomap.xsd"
 // Возвращает идентификатор доступа к прикладной схеме
 // Для записи набора (dataset) функцией gmlGetFeaturiesDataset
 // доступ к схеме открывается gmlOpenEx
 // По завершении работы необходимо освободить ресурсы вызовом gmlClose
 // При ошибке возвращает ноль

_MAPIMP HGML _MAPAPI gmlOpen(HMAP hMap, const char *schemafilename);
_MAPIMP HGML _MAPAPI gmlOpenEx(HMAP hMap, HSITE hSite,
                               const char * schemafilename,
                               const char * schemaURL);
_MAPIMP HGML _MAPAPI gmlOpenUn(HMAP hMap, HSITE hSite, 
                               const WCHAR * schemafilename,
                               const WCHAR * schemaURL);

 // Закрыть доступ к схеме и освободить ресурсы
 // hGml - идентификатор GML данных

_MAPIMP void _MAPAPI gmlClose(HGML hGml);

 // Запросить число типов объектов GML данных ("слоев")
 // Возвращает число типов объектов GML, содержащихся в схеме
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlFeatureTypeCount(HGML hGml);

 // Запросить имя типа объекта GML данных по порядковому номеру
 // Возвращает имя типа объекта
 // При ошибке возвращает пустую строку

_MAPIMP const char * _MAPAPI gmlFeatureTypeName(HGML hGml, long int number);

 // Запросить порядковый номер типа объекта по имени типа
 // hGml - идентификатор открытой схемы для записи GML
 // featuretypename - имя типа объекта GML данных
 // Возвращает порядковый номер типа объекта в схеме
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlFeatureTypeNameNumber(HGML hGml, const char *featuretypename);

 // Запросить габариты объектов карты по списку номеров типов объектов
 // hGml  - идентификатор GML данных
 // hMap  - идентификатор набора данных
 // hSite - идентификатор карты (для основной карты может быть равен hMap)
 // list  - номер обрабатываемого листа на многолистовой карте или 0 (все листы)
 // hSelect - идентификатор условий отбора объектов или 0
 // epsgcode - код геодезической системы координат в базе данных EPSG, для GML по умолчанию - 4326
 // dframe - габариты объектов карты в указанной системе координат
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI gmlFeaturiesBoundsPro(HGML hGml, HMAP hMap, HSITE hSite, long int list,
                                               HSELECT hSelect, long int epsgcode, DFRAME *dframe);

 // Запросить данные объектов карты по заданным условиям в файле формата GML или JSON
 // hGml - идентификатор открытой схемы для записи GML
 // hMap  - идентификатор набора данных
 // hSite - идентификатор карты
 // list  - номер листа для многолистовой карты или 1, для конвертирования
 //         в один GML-файл сразу всех листов необходимо указать "-1" или 0
 // hSelect - идентификатор условий отбора объектов или 0
 // number - порядковый номер объекта, с которого начинать вывод в файл (с 1)
 // count - число объектов, выводимых в файл, если равно 0, то выводятся все объекты  
 // epsgcode - код геодезической системы координат в базе данных EPSG, 
 // для GML по умолчанию - 4326
 // dframe - бласть отбора объектов карты в указанной системе координат или 0 (левый нижний и правый верхний угол области)
 // format - тип разметки или формат файла: GML, GML/WFS, JSON  (см. OGCSERVICETYPE)
 // targetfilename - имя выходного файла
 // flags - флажки вывода расширенных метаданных об объекте (см. OGCSERVICEFLAG)
 // format - формат вывода данных: GML, JSON (см. OGCSERVICETYPE) 
 // mapid  - указатель на идентификатор карты, который записывается в каждый объект карты или 0
 // completed - признак необходимости записи элементов начала и конца файла
 //             (0 - не формировать, 1 - только закрывающий, 2 - только начальные теги, -1 - начало и конец данных)
 // JSON - { "type": "FeatureCollection",    ... }
 // GML  - <gml:FeatureCollection> ... </gml:FeatureCollection>
 // WFS  - <wfs:FeatureCollection> ... </wfs:FeatureCollection>
 // force - признак принудительной записи объектов, которые не описаны в прикладной схеме,
 //         если равен нулю, то записываются только объекты, виды (коды) которых описаны в схеме
 // hwnd - идентификатор окна для приема сообщений или ноль, посылаются сообщения WM_PROGRESSBAR
 // requestId - идентификатор запроса
 // xmlns - признак записи пространства имен в тег member для gml/wfs
 // secondframe - дополнительные точки для области отбора объектов карты в указанной системе координат 
 // (подается дополнительно к dframe, левый верхний и правый нижний угол области)
 // Возвращает число записанных объектов
 // Если заданы слои, содержащие объекты, которых нет на карте, то возвращает -1
 // Если условиям поиска не соответствует ни один объект, то возвращает -2
 // Если выходной файл не может быть открыт, то возвращает -3
 // Если произошел сбой при работе программы, то возвращает -4
 // Если для карты запрещено копирование, то возвращает -5

_MAPIMP long int _MAPAPI gmlGetFeaturiesPro(HGML hGml, HMAP hMap, HSITE hSite, long int list,
                                            HSELECT hSelect, const WCHAR * metadata,
                                            long int number, long int count,
                                            long int epsgcode, DFRAME* dframe, long int flags,
                                            long int format, const WCHAR *targetfilename,
                                            const WCHAR * mapid, long int completed,
                                            long int force, HWND hwnd, const WCHAR* requestId = 0,
                                            long int xmlns = 0);

_MAPIMP long int _MAPAPI gmlGetFeaturiesProEx(HGML hGml, HMAP hMap, HSITE hSite, long int list,
                                            HSELECT hSelect, const WCHAR * metadata,
                                            long int number, long int count,
                                            long int epsgcode, DFRAME* dframe, long int flags,
                                            long int format, const WCHAR *targetfilename,
                                            const WCHAR * mapid, long int completed,
                                            long int force, HWND hwnd, const WCHAR* requestId = 0,
                                            long int xmlns = 0, DFRAME* secondframe = 0);

 // Запросить данные объектов карты по заданным условиям в памяти в формате GML или JSON
 // Для чтения из памяти результата необходимо вызвать gmlGetFeaturiesPoint
 // После завершения чтения необходимо освободить память вызовом gmlFreeFeaturiesPoint
 // Если объем данных слишком большой, то функция может занять всю память в системе
 // Назначение параметров аналогично функции gmlGetFeaturiesPro
 // xmlns - признак записи пространства имен в тег member для gml/wfs
 // Возвращает в параметре retcode число записанных объектов или код ошибки
 // Если заданы слои, содержащие объекты, которых нет на карте, то записывает -1
 // Если условиям поиска не соответствует ни один объект, то записывает -2
 // Если произошел сбой при работе программы, то записывает -4
 // Если для карты запрещено копирование, то записывает -5
 // При ошибке возвращает ноль иначе идентификатор данных в памяти
 
 _MAPIMP HANDLE _MAPAPI gmlGetFeaturiesProInMemory(HGML hGml, HMAP hMap, HSITE hSite, long int list,
                                                  HSELECT hSelect, const WCHAR * metadata,
                                                  long int number, long int count,
                                                  long int epsgcode, DFRAME* dframe, long int flags,
                                                  long int format, const WCHAR * mapid, long int completed, 
                                                  long int force, long int * retcode, const WCHAR* requestId = 0, long int xmlns = 0);

 // Запросить указатель на данные в памяти по идентификатору данных, 
 // полученному из функции gmlGetFeaturiesProInMemory или gmlGetObjectFeatureProInMemory
 // После завершения чтения необходимо освободить память вызовом gmlFreeFeaturiesPoint
 // При ошибке возвращает ноль иначе указатель на данные в памяти
 
_MAPIMP const char * _MAPAPI gmlGetFeaturiesPoint(HANDLE handle, long int * size);

 // Освободить память под данные по идентификатору данных, 
 // полученному из функции gmlGetFeaturiesProInMemory или gmlGetObjectFeatureProInMemory

_MAPIMP void _MAPAPI gmlFreeFeaturiesPoint(HANDLE handle);

 // Запросить данные объекта карты в файле формата GML или JSON
 // hGml - идентификатор открытой схемы для записи GML
 // hObj - идентификатор объекта карты в памяти
 // epsgcode - код геодезической системы координат в базе данных EPSG
 // flags - флажки вывода расширенных метаданных об объекте (см. OGCSERVICEFLAG)
 // format - формат вывода данных: GML, JSON (см. OGCSERVICETYPE) 
 // targetfilename - имя выходного GML или JSON файла
 // mapid  - указатель на идентификатор карты, который записывается в каждый объект карты или 0
 // requestId - идентификатор запроса
 // xmlns - признак записи пространства имен в тег member для gml/wfs
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlGetObjectFeaturePro(HGML hGml, HOBJ hObj,
                                                long int epsgcode, 
                                                long int flags, long int format,
                                                const WCHAR* targetfilename,
                                                const WCHAR * mapid,
                                                long int completed, const WCHAR* requestId = 0, long int xmlns = 0);

 // Запросить данные объекта карты в памяти в формате GML или JSON
 // Для чтения из памяти результата необходимо вызвать gmlGetFeaturiesPoint
 // После завершения чтения необходимо освободить память вызовом gmlFreeFeaturiesPoint
 // Для дозаписи в память новых объектов параметр handle должен содержать
 // значение, которая вернула функция при первом вызове, когда параметр handle был равен 0
 // Возвращает в параметре retcode число записанных объектов или код ошибки
 // Если заданы слои, содержащие объекты, которых нет на карте, то записывает -1
 // Если условиям поиска не соответствует ни один объект, то записывает -2
 // Если произошел сбой при работе программы, то записывает -4
 // Если для карты запрещено копирование, то записывает -5
 // При ошибке возвращает ноль иначе идентификатор данных в памяти
                                                
_MAPIMP HANDLE _MAPAPI gmlGetObjectFeatureProInMemory(HGML hGml, HOBJ hObj,
                                                      long int epsgcode, 
                                                      long int flags, long int format,
                                                      HANDLE handle,
                                                      const WCHAR * mapid,
                                                      long int completed, long int * retcode,
                                                      const WCHAR* requestId = 0, long int xmlns = 0);
                                        
 // Запросить данные объекта карты по GML-идентификатору
 // hGml - идентификатор открытой схемы для записи GML
 // epsgcode - код геодезической системы координат в базе данных EPSG
 // id - GML-идентификатор объекта
 // flags - флажки вывода расширенных метаданных об объекте (см. OGCSERVICEFLAG)
 // format - формат вывода данных: GML, JSON (см. OGCSERVICETYPE) 
 // targetfilename - имя выходного файла
 // mapid  - указатель на идентификатор карты, который записывается в каждый объект карты или 0
 // completed - признак необходимости записи элементов начала и конца файла
 //             (0 - не формировать, 1 - только закрывающий, 2 - только начальные теги, -1 - начало и конец данных)
 // JSON - { "type": "FeatureCollection",    ... }
 // GML  - <gml:FeatureCollection> ... </gml:FeatureCollection>
 // WFS  - <wfs:FeatureCollection> ... </wfs:FeatureCollection>
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlGetFeatureByIdPro(HGML hGml, HMAP hMap, HSITE hSite, long int list,
                                              long int epsgcode, const char *id,
                                              long int flags, long int format,
                                              const WCHAR* targetfilename,
                                              const WCHAR * mapid,
                                              long int completed, const WCHAR* requestId = 0);

 // Преобразовать координаты рамки из одной геодезической системы координат
 // в другую геодезическую систему координат
 // epsgcodesource - код исходной геодезической системы координат в базе данных EPSG,
 // epsgcodedest - код выходной геодезической системы координат в базе данных EPSG,
 // Коды могут быть в диапазоне 4326 - 4937
 // dframe - координаты рамки в исходной системе координат в радианах
 // Возвращает по адресу dframe значения координат в СК epsgcodedest
 // При ошибке возвращает ноль.

_MAPIMP long int _MAPAPI gmlGeoToGeo(long int epsgcodesource, long int epsgcodedest,
                                     DFRAME* dframe);

 // Установить отбор объектов по идентификатору слоя
 // hGml - идентификатор открытой схемы для записи GML
 // hSelect - идентификатор условий отбора объектов
 // featuretype - идентификатор слоя в кодировке UTF-8
 // При ошибке возвращает ноль
  
_MAPIMP long int _MAPAPI gmlSetSelectByFeaturieType(HGML hGml, HSELECT hSelect, const char * featuretype);
                                     
 /*********************************************************
 *                                                        *
 *           Загрузка данных из geoJSON                   *
 *                                                        *
 *********************************************************/

 // Cоздание объектов из файла формата geoJSON
 // hmap  - идентификатор открытой карты
 // hsite - идентификатор открытой пользовательской карты
 // jsonname - имя файла формата GeoJSON
 // Код создаваемого объекта ищется в поле code ("code": 115001010,)
 // Если код не задан в объекте, то он выбирается из входных параметров
 // squareCode - код создаваемых площадных объектов или 0
 // pointCode  - код создаваемых точечных объектов или 0
 // lineCode   - код создаваемых линейных объектов или 0
 // textCode   - код создаваемых подписей или 0
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlCreateObjectsFromJSON(HMAP hMap, HSITE hSite,
                                                  const WCHAR* jsonname,
                                                  long int squareCode = 0, long int pointCode = 0,  
                                                  long int lineCode = 0, long int textCode = 0);

 // Cоздание объектов из файла формата geoJSON
 // hmap  - идентификатор открытой карты
 // hsite - идентификатор открытой пользовательской карты
 // jsonname - имя файла формата GeoJSON
 // Код создаваемого объекта ищется в поле code ("code": 115001010,)
 // Если код не задан в объекте, то он выбирается из входных параметров
 // squareCode - код создаваемых площадных объектов или 0
 // pointCode  - код создаваемых точечных объектов или 0
 // lineCode   - код создаваемых линейных объектов или 0
 // textCode   - код создаваемых подписей или 0
 // hwnd - идентификатор окна для приема сообщений или ноль, посылаются сообщения WM_PROGRESSBAR                                                  
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlCreateObjectsFromJSONEx(HMAP hMap, HSITE hSite,
                                                    const WCHAR* jsonname,
                                                    long int squareCode = 0, long int pointCode = 0,  
                                                    long int lineCode = 0, long int textCode = 0, HWND hwnd = 0);
                                                  
 // Записать завершающие теги
 // hGml - идентификатор открытой схемы для записи GML
 // format - формат вывода данных: GML, JSON (см. OGCSERVICETYPE)
 // targetfilename - имя выходного файла
 // matched  - количество найденных объектов
 // returned - количество объектов, записанных в GML
 // data - данные которые необходимо дописать в конец файла
 // datasize - размер данных
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlSaveFileEnd(long int format, const WCHAR* targetfilename, long int matched, long int returned);

_MAPIMP long int _MAPAPI gmlSaveFileEndInMemory(HANDLE handle, long int matched, long int returned);

_MAPIMP long int _MAPAPI gmlSaveFileEndEx(long int format, const WCHAR* targetfilename,
                                        long int matched, long int returned,
                                        const char* data, long int datasize);
 
 // Запрос габаритов объектов файла geojson
 // name   - имя файла geojson
 // border - габариты объектов в радианах в СК WGS-84
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI gmlGetJSONBorder(const WCHAR* name, DFRAME * border);

 /*********************************************************
 *                                                        *
 *           Загрузка данных из GML/XML, OGC WFS,         *
 *                 OGC WFS-T (Transaction)                *
 *                                                        *
 *********************************************************/

 // Cоздание объектов из потока транзакций по стандарту OGC WFS-T
 // hMap, hSite - идентификатор карты
 // stream - указатель на данные, в котором находится xml файл транзакции OGC WFS-T
 // streamlength - длина потока
 // squareCode - код создаваемых площадей
 // pointCode  - код создаваемых точечных объектов
 // lineCode - код создаваемых линий
 // textCode - код создаваемых подписей
 // Возвращает идентификатор данных HGMLCLASS, который
 // должен быть освобожден функцией mapFreeGmlClassHandle
 // При ошибке возвращает 0

_MAPIMP HGMLCLASS _MAPAPI gmlLoadGmlTransactionToMap(HMAP hMap, HSITE hSite,
              const char* stream, long int streamlength,
              long int squareCode = 0, long int pointCode = 0, long int lineCode = 0, long int textCode = 0);

 // Cоздание объектов из потока gml/xml файла
 // hMap, hSite - идентификатор карты
 // stream - указатель на данные, в котором находится gml/xml файл или файл gml OGC WFS
 // streamlength - длина потока
 // squareCode - код создаваемых площадей
 // pointCode  - код создаваемых точечных объектов
 // lineCode - код создаваемых линий
 // textCode - код создаваемых подписей
 // Возвращает идентификатор данных HGMLCLASS, который
 // должен быть освобожден функцией mapFreeGmlClassHandle
 // При ошибке возвращает 0

_MAPIMP HGMLCLASS _MAPAPI gmlCreateObjectsFromXmlStream(HMAP hMap, HSITE hSite,
               const char* stream, long int streamlength,
               long int squareCode = 0, long int pointCode = 0,  long int lineCode = 0, long int textCode = 0);
_MAPIMP HGMLCLASS _MAPAPI gmlCreateObjectsFromXmlStreamPro(HMAP hMap, HSITE hSite,
               const char* stream, long int streamlength, long int protocol,
               long int squareCode = 0, long int pointCode = 0,  long int lineCode = 0, long int textCode = 0, HWND hwnd = 0);

 // Cоздание объектов из файла gml/xml
 // hMap, hSite - идентификатор карты
 // xmlname - имя входного файла gml/xml или файла gml OGC WFS
 // protocol   - признак ведения протокола ошибок в формате JSON (запрашивается gmlGetGmlReport)
 // squareCode - код создаваемых площадей
 // pointCode  - код создаваемых точечных объектов
 // lineCode - код создаваемых линий
 // textCode - код создаваемых подписей
 // hwnd - идентификатор окна для приема сообщений или ноль, посылаются сообщения WM_PROGRESSBAR
 // Возвращает идентификатор данных HGMLCLASS, который
 // должен быть освобожден функцией mapFreeGmlClassHandle
 // При ошибке возвращает 0

_MAPIMP HGMLCLASS _MAPAPI gmlCreateObjectsFromXmlPro(HMAP hMap, HSITE hSite, const WCHAR* xmlname, long int protocol,
               long int squareCode, long int pointCode,  long int lineCode, long int textCode, HWND hwnd = 0);
_MAPIMP HGMLCLASS _MAPAPI gmlCreateObjectsFromXml(HMAP hMap, HSITE hSite, const WCHAR* xmlname,
               long int squareCode, long int pointCode,  long int lineCode, long int textCode);
_MAPIMP HGMLCLASS _MAPAPI gmlCreateObjectsFromXmlEx(HMAP hMap, HSITE hSite, const WCHAR* xmlname,
               long int squareCode = 0, long int pointCode = 0,  long int lineCode = 0, long int textCode = 0, HWND hwnd = 0);

 // Освободить идентификатор данных gml/xml
 // hGmlClass - идентификатор данных, полученный в функциях mapCreateObjFromXml,
 // mapCreateObjFromXmlStream, mapLoadGmlTransactionToMap

_MAPIMP void _MAPAPI gmlFreeGmlClassHandle(HGMLCLASS hGmlClass);

 // Запросить протокол ошибок в формате JSON при загрузке GML функцией gmlCreateObjectsFromXmlPro
 // hGmlClass - идентификатор данных, полученный в функциях mapCreateObjFromXml,
 // buffer    - адрес буфера для чтения протокола
 // size      - размер буфера для чтения протокола
 // Для запроса размера протокола параметр buffer должен быть равен нулю
 // При ошибке возвращает 0

_MAPIMP int _MAPAPI gmlGetGmlReport(HGMLCLASS hGmlClass, char * buffer, long int size);

 // Запросить массив уникальных номеров созданных объектов
 // hGmlClass - идентификатор данных
 // ObjCrNum - массив уникальных номеров созданных объектов
 // Возвращает указатель на класс THandleList
 // При ошибке возвращает 0

_MAPIMP HANDLE _MAPAPI gmlGetGmlObjectsNumber(HGMLCLASS hGmlClass);

 // Запросить количество созданных объектов
 // hGmlClass - идентификатор данных
 // Возвращает количество созданных объектов
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI gmlGetGmlCreateObjectsCount(HGMLCLASS hGmlClass);

 // Запросить количество обновленных объектов при выполнении транзакций
 // hGmlClass - идентификатор данных
 // Возвращает количество обновленных объектов
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI gmlGetGmlUpdateObjectsCount(HGMLCLASS hGmlClass);

 // Запросить количество удаленных объектов при выполнении транзакций
 // hGmlClass - идентификатор данных
 // Возвращает количество удаленных объектов
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI gmlGetGmlDeleteObjectsCount(HGMLCLASS hGmlClass);


 // Запросить имя классификатора данных, по которому был создан gml
 // xmlname - имя входного файла gml/xml или файла gml OGC WFS
 // rscname - возвращаемое значение имени классификатора, по которому создан gml
 // size - размер переменной rscname
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI gmlGetGmlRscName(const WCHAR* xmlname, WCHAR* rscname, long int size);

 // Запросить габариты набора данных в формате GML          // 12/02/16
 // name   - имя gml-файла
 // border - габариты набора данных в радианах в системе WGS-84
 // При ошибке возвращает 0, иначе идентификатор данных

_MAPIMP long int _MAPAPI gmlGetGmlBorder(const WCHAR * name, DFRAME * border);

 /*********************************************************
 *                                                        *
 *                Устаревшие функции                      *
 *                                                        *
 *********************************************************/

 // Запросить габариты объектов карты по списку номеров типов объектов
 // hGml  - идентификатор GML данных
 // epsgcode - код геодезической системы координат в базе данных EPSG, для GML по умолчанию - 4326
 // dframe - габариты объектов карты в указанной системе координат
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI gmlFeaturiesBounds(HGML hGml, const char *featuretypenumbers,
                                            long int epsgcode, DFRAME *dframe);

 // Запросить число объектов карты, отвечающих условиям отбора
 // hGml - идентификатор открытой схемы для записи GML
 // featuretypenumbers - список номеров типов объектов, разделенных запятой
 // epsgcode - код геодезической системы координат в базе данных EPSG, 
 // для GML по умолчанию - 4326
 // dframe - область отбора объектов карты в указанной системе координат или 0
 // number - порядковый номер объекта, с которого начинать отбор (с 1)
 // count - число объектов запроса 
 // Возвращает число объектов
 // Если заданы слои, содержащие объекты, которых нет на карте, то возвращает -1
 // Если условиям поиска не соответствует ни один объект, то возвращает -2
 // Если произошел сбой при работе программы, то возвращает -4
 // Если для карты запрещено копирование, то возвращает -5

_MAPIMP long int _MAPAPI gmlGetFeaturiesCount(HGML hGml, const char *featuretypenumbers, 
                                              long int epsgcode, DFRAME* dframe,
                                              long int number, long int count); 


 // Запросить данные объектов карты по диапазону номеров
 // hGml - идентификатор открытой схемы для записи GML
 // featuretypenumbers - список номеров типов объектов, разделенных запятой
 // epsgcode - код геодезической системы координат в базе данных EPSG, 
 // для GML по умолчанию - 4326
 // dframe - область отбора объектов карты в указанной системе координат или 0
 // service - тип разметки или формат файла: GML, GML/WFS, JSON  (см. OGCSERVICETYPE)
 // number - порядковый номер объекта, с которого начинать вывод в файл (с 1)
 // count - число объектов, выводимых в файл, если равно 0, то выводятся все объекты  
 // targetfilename - имя выходного GML-файла
 // Возвращает число записанных объектов
 // Если заданы слои, содержащие объекты, которых нет на карте, то возвращает -1
 // Если условиям поиска не соответствует ни один объект, то возвращает -2
 // Если выходной файл не может быть открыт, то возвращает -3
 // Если произошел сбой при работе программы, то возвращает -4
 // Если для карты запрещено копирование, то возвращает -5

_MAPIMP long int _MAPAPI gmlGetFeaturiesRangeUn(HGML hGml, const char *featuretypenumbers,
                                                long int epsgcode, DFRAME* dframe,
                                                long int service,
                                                long int number, long int count,
                                                const WCHAR *targetfilename);
                                         
_MAPIMP long int _MAPAPI gmlGetFeaturiesRange(HGML hGml, const char *featuretypenumbers,
                                              long int epsgcode, DFRAME* dframe,
                                              long int service,
                                              long int number, long int count,  
                                              const char *targetfilename);
                               

 // Запросить данные объекта карты по GML-идентификатору
 // hGml - идентификатор открытой схемы для записи GML
 // featuretypenumber - номер типа объекта
 // epsgcode - код геодезической системы координат в базе данных EPSG
 // id - GML-идентификатор объекта
 // service - тип разметки файла: GML/WFS
 // targetfilename - имя выходного GML-файла

_MAPIMP long int _MAPAPI gmlGetFeatureByIdUn(HGML hGml, const char* featuretypenumber,
                                             long int epsgcode, const char *id,
                                             long int service,
                                             const WCHAR* targetfilename);
 
_MAPIMP long int _MAPAPI gmlGetFeatureById(HGML hGml, const char* featuretypenumber,
                                           long int epsgcode, const char *id,
                                           long int service,
                                           const char* targetfilename);

 // Запросить данные объектов карты
 // hGml - идентификатор открытой схемы для записи GML
 // featuretypenumbers - список номеров типов объектов, разделенных запятой
 // epsgcode - код геодезической системы координат в базе данных EPSG, 
 // для GML по умолчанию - 4326
 // dframe - область отбора объектов карты в указанной системе координат или 0
 // service - тип разметки файла: GML/WFS  (см. OGCSERVICETYPE)
 // targetfilename - имя выходного GML-файла. 
 // При ошибке возвращает ноль, иначе - число записанных объектов
 // Если заданы слои, содержащие объекты, которых нет на карте, то возвращает -1
 // Если условиям поиска не соответствует ни один объект, то возвращает -2
 // Если выходной файл не может быть открыт, то возвращает -3
 // Если произошел сбой при работе программы, то возвращает -4
 // Если для карты запрещено копирование, то возвращает -5

_MAPIMP long int _MAPAPI gmlGetFeaturies(HGML hGml, const char *featuretypenumbers,
                                         long int epsgcode, DFRAME* dframe,
                                         long int service,
                                         const char *targetfilename);

_MAPIMP long int _MAPAPI gmlGetFeaturiesUn(HGML hGml, const char *featuretypenumbers,
                                           long int epsgcode, DFRAME* dframe,
                                           long int service,
                                           const WCHAR *targetfilename);
                                         
 // Сформировать набор данных (dataset) в формате GML
 // hmap  - идентификатор открытого набора данных, содержащего векторные карты
 // hsite - идентификатор карты в наборе данных
 // list  - номер листа для многолистовой карты или 1, для конвертирования
 //         в один GML-файл сразу всех листов необходимо указать "-1"
 // hGml - идентификатор открытой схемы для записи GML
 // (формируется функцией gmlOpenEx)
 // featuretypenumbers - список номеров типов объектов, разделенных запятой,
 //                      например: "1,5,11"
 // metadata - URL для метаданных
 // epsgcode - код геодезической системы координат в базе данных EPSG,
 // по умолчанию - 4326
 // targetfilename - имя выходного GML-файла
 // force - признак принудительной записи объектов (1), даже если они не
 //         описаны в прикладной схеме или 0
 // hwnd - идентификатор для посылки сообщений о ходе выполнения процесса
 // При ошибке возвращает ноль, иначе - число записанных объектов
 // Если заданы слои, содержащие объекты, которых нет на карте, то возвращает -1
 // Если условиям поиска не соответствует ни один объект, то возвращает -2
 // Если выходной файл не может быть открыт, то возвращает -3
 // Если произошел сбой при работе программы, то возвращает -4
 // Если для карты запрещено копирование, то возвращает -5

_MAPIMP long int _MAPAPI gmlGetFeaturiesDataset(HMAP hmap, HSITE hsite, long int list,
                                                HGML hGml, const char * metadata,
                                                const char * featuretypenumbers,
                                                long int     epsgcode,
                                                const char * targetfilename,
                                                long int force, HWND hwnd);

_MAPIMP long int _MAPAPI gmlGetFeaturiesDatasetUn(HMAP hmap, HSITE hsite, long int list,
                                                  HGML hGml, const WCHAR * metadata,
                                                  const char  * featuretypenumbers,
                                                  long int      epsgcode,
                                                  const WCHAR * targetfilename,
                                                  long int force, HWND hwnd);
                                           
                                     
} // extern "C"

#endif  // GMLAPI_H