/****** RSCAPI.H *************** Belenkov   O.V. ****** 27/09/15 ***
******* RSCAPI.H *************** Panteleeva N.A. ****** 06/05/16 ***
******* RSCAPI.H *************** Derevyagina Zh.A.***** 28/12/12 ***
******* RSCAPI.H *************** Dishlenko S.G.  ****** 13/03/13 ***
******* RSCAPI.H *************** Shabakov D.A.   ****** 11/02/14 ***
******* RSCAPI.H *************** Tarakanova I.R. ****** 13/03/14 ***
******* RSCAPI.H *************** Kruzhkov A.E.   ****** 20/03/14 ***
******* RSCAPI.H *************** Борзов А.Ю.     ****** 04/03/16 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*       ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К ОБ'ЕКТУ "КЛАССИФИКАТОР КАРТ"    *
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
*    HRSC (WINAPI * lpfn)(HMAP,HSITE);                             *
*    (FARPROC)lpfn = GetProcAddress(libInst, "mapGetRscIdent");    *
*    HRSC hRsc = (*lpfn)(hMap,hSite);                              *
*                                                                  *
*    // Выгрузка библиотеки                                        *
*    ::FreeLibrary(libInst);                                       *
*                                                                  *
*******************************************************************/

#ifndef RSCAPI_H
#define RSCAPI_H

#ifndef MAPTYPE_H
 #include "maptype.h"
#endif

#ifndef MAPGDI_H
 #include "mapgdi.h"
#endif 

  // Порядковый номер слоев начинается с 0. Нулевой слой - служебный.
  // Идентификатором слоя служит его название. Название слоя уникально.
  // Короткое имя слоя (ключ) - уникально.

  // Внутренний код (порядковый номер ) объектов начинается с 1.
  // Внутренний код объекта не более количества объектов.
  // Ключ объекта - уникален.

  // Идентификатором семантик служит код. Семантика с кодом 0 - служебная.
  // Код семантики не является ее порядковым номером .
  // Порядковый номер семантики начинается с 1.
  // Короткое имя семантики (ключ) - уникально.

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++ ОПИСАНИЕ ФУНКЦИЙ ДОСТУПА К КЛАССИФИКАТОРУ +++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
extern "C"
{
  /*********************************************************
  *                                                        *
  * ФУНКЦИИ ЗАПРОСА ИНФОРМАЦИИ ИЗ КЛАССИФИКАТОРА ОБ'ЕКТОВ  *
  *                                                        *
  *********************************************************/

  // Запросить идентификатор классификатора карты
  // hMap  - идентификатор открытой карты
  // hSite - идентификатор открытой пользовательской карты
  // При ошибке возвращает ноль

_MAPIMP  HRSC _MAPAPI mapGetRscIdent(HMAP hMap,HSITE hSite);

  // Запросить имя файла классификатора
  // hRsc   - идентификатор классификатора карты,
  // target - строка для размещения полного имени файла,
  // size   - размер строки
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscFileName(HRSC hRsc,
                                     char * target, long int size);

  // Запросить имя файла классификатора в кодировке UNICODE              // 23/04/12
  // hRsc   - идентификатор классификатора карты,
  // target - строка для размещения полного имени файла,
  // size   - размер строки
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscFileNameUn(HRSC hRsc,
                                     WCHAR * target, long int size);

  // Запросить имя классификатора
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  const char * _MAPAPI mapGetRscName(HRSC hRsc);

  // Запросить имя файла классификатора в кодировке UNICODE              
  // hRsc    - идентификатор классификатора карты,
  // rscname - строка для размещения полного имени файла,
  // size    - размер строки
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscNameUn(HRSC hRsc, WCHAR * rscname, long int size); // 12/09/12


  // Запросить условное название классификатора               // 07/06/15
  // hRsc    - идентификатор классификатора карты,
  // name    - строка для размещения условного названия классификатора,
  // size    - размер строки
  // При ошибке возвращает ноль

  _MAPIMP long int _MAPAPI mapGetRscUserName(HRSC hRsc, char * name, long int size);


  // Установить условное название классификатора               // 07/06/15
  // hRsc    - идентификатор классификатора карты,
  // name    - условное название классификатора,
  // При ошибке возвращает ноль

  _MAPIMP long int _MAPAPI mapSetRscUserName(HRSC hRsc, const char * name);
  
  // Запросить код ошибки последней операции с классификатором карты
  // hRsc - идентификатор классификатора карты
  // Коды ошибок перечислены в maperr.rh

_MAPIMP  long int _MAPAPI mapGetRscError(HRSC hRsc);  

  // Запросить флаг изменений классификатора карты
  // hRsc - идентификатор классификатора карты
  // При изменении флага внутренние коды объектов сохраняются

_MAPIMP  long int _MAPAPI mapGetRscMode(HRSC hRsc);

  // Запросить стиль классификатора карты
  // hRsc - идентификатор классификатора карты
  // При изменении стиля могут измениться внутренние коды объектов

_MAPIMP  long int _MAPAPI mapGetRscStyle(HRSC hRsc);

  // Запросить возможность запиcи классификатора карты
  // hRsc - идентификатор классификатора карты
  // 0 - классификатор открыт на чтение, 1 - на запись

_MAPIMP  long int _MAPAPI mapGetRscIsWrite(HRSC hRsc);        // 15/12/15


  // Запросить число объектов описанных в классификаторе
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscObjectCount(HRSC hRsc);

  // Запросить число объектов описанных в классификаторе
  // в заданном слое
  // hRsc - идентификатор классификатора карты
  // layer - номер слоя
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscObjectCountInLayer(HRSC hRsc,
                                               long int layer);

  // Запросить название объекта по порядковому номеру
  // в заданном слое
  // hRsc   - идентификатор классификатора карты
  // layer  - номер слоя
  // number - номер объекта в слое
  // При ошибке возвращает ноль или пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscObjectNameInLayer(HRSC hRsc,
                                               long int layer,
                                               long int number);

 // Запросить название объекта по порядковому номеру
 // в заданном слое
 // hRsc   - идентификатор классификатора карты
 // layer  - номер слоя
 // number - номер объекта в слое
 // name   - адрес строки для размещения результата
 // size   - размер строки (не меньше 32 байт)
 // При ошибке возвращает ноль                                  // 08/05/07

_MAPIMP long int _MAPAPI mapGetRscObjectNameInLayerEx(HMAP hRsc, long int layer,
                                                      long int number,
                                                      char * name, long int size);
_MAPIMP long int _MAPAPI mapGetRscObjectNameInLayerUn(HMAP hRsc, long int layer,     // 19/01/14
                                                      long int number,
                                                      WCHAR * name, long int size);

  // Запросить классификационный код объекта
  // по порядковому номеру в заданном слое
  // hRsc - идентификатор классификатора карты
  // layer - номер слоя
  // number - номер объекта в слое
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscObjectExcodeInLayer(HRSC hRsc,
                                                      long int layer,
                                                      long int number);

  // Запросить код локализации объекта
  // по порядковому номеру в заданном слое
  // hRsc - идентификатор классификатора карты
  // layer - номер слоя
  // number - номер объекта в слое
  // При ошибке возвращает ноль (ноль допустим)

_MAPIMP  long int  _MAPAPI mapGetRscObjectLocalInLayer(HRSC hRsc,
                                            long int layer,
                                            long int number);

  // Запросить внутренний код (порядковый номер) объекта
  // по порядковому номеру в заданном слое
  // hRsc - идентификатор классификатора карты
  // layer - номер слоя
  // number - номер объекта в слое
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscObjectCodeInLayer(HRSC hRsc,
                                               long int layer,
                                               long int number);

  // Запросить размер текущей таблицы масштабов классификатора
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscScaleCount(HRSC hRsc);

  // Запросить значение (знаменатель масштаба) из текущей таблицы
  // масштабов классификатора  по порядковому номеру (с 1)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscScaleItem(HRSC hRsc,
                                       long int number);

  // Установка процента масштаба отображения знака в окнах - примерах  28/06/12
  // scale - масштаб отображения в % от 10 до 100 уменьшение, более 100 увеличение

_MAPIMP  long int _MAPAPI mapSetRscExampleScale(HRSC hRsc, long int scale);


  // Запрос процента масштаба отображения знака в окнах - примерах,
  // от 10 до 100 уменьшение, более 100 увеличение
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscExampleScale(HRSC hRsc);

  // Поиск объектов классификатора                                     // 20/04/06
  // oldnumber - номер объекта при предыдущем поиске, 0 - поиск с первого
  // seektype - условия поиска  SEEK_RSCOBJECT (maptype.h)
  // example - шаблон для поиска
  // Возвращает порядковый номер объекта или 0, если такого нет

_MAPIMP  long int  _MAPAPI mapGetRscSeekObjectCode(HRSC hRsc,long int oldnumber,
                                       long int seektype, char * example);

  // Запросить полное (с путем) имя P3D файла по коду библиотеки в кодировке
  // UNICODE по порядковому номеру
  // hRsc    - идентификатор классификатора карты,
  // name - строка для размещения полного имени файла,
  // size    - размер строки
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetP3DNameUn(HRSC hRsc, long int number,
                                         WCHAR * name, long int size); // 27/06/13

  /*********************************************************
  *                                                        *
  *        ФУНКЦИИ СОЗДАНИЯ КЛАССИФИКАТОРА ОБ'ЕКТОВ        *
  *                                                        *
  *********************************************************/

  // Создать классификатор векторной карты
  // name - имя создаваемого файла классификатора
  // RSCCREATE -   структура входных данных  (см. maptype.h)
  // PALETTE256 -  при необходимости задается палитра (не более 32 цветов)
  //               (см. maptype.h)
  // При ошибке возвращает ноль

_MAPIMP  HRSC _MAPAPI mapCreateRsc(const char * name, const RSCCREATE * rsccreate,
                                   const PALETTE256 * palette);

_MAPIMP  HRSC _MAPAPI mapCreateRscUn(const WCHAR * name, const RSCCREATEUN * rsccreate,
                                     const PALETTE256 * palette);

  // Создать классификатор векторной карты c идентификацией кодов объекта
  // по ключу (короткому имени объекта)
  // name - имя создаваемого файла классификатора
  // RSCCREATE -   структура входных данных  (см. maptype.h)
  // PALETTE256 -  при необходимости задается палитра (не более 32 цветов)
  //               (см. maptype.h)
  // При ошибке возвращает ноль                         // 21/05/02

_MAPIMP  HRSC _MAPAPI mapCreateKeyObjectRsc(char * name, RSCCREATE * rsccreate,
                            PALETTE256 * palette );


  // Запросить данные по классификатору векторной карты
  // hRsc - идентификатор классификатора карты
  // RSCCREATE -  описание классификатора  (см. maptype.h)
  // PALETTE256 - истинная палитра карты   (см. maptype.h)
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscDescribe(HRSC hRsc, RSCCREATE * rsccreate,
                                     PALETTE256 * palette);

  // Открыть классификатор
  // name - имя  файла классификатора
  // При ошибке возвращает ноль, иначе идентификатор классификатора карты

_MAPIMP  HRSC _MAPAPI mapOpenRsc(const char * name);            // 13/08/12

  // Открыть классификатор
  // name - имя  файла классификатора в кодировке UNICODE
  // size    - размер строки
  // При ошибке возвращает ноль

_MAPIMP  HRSC _MAPAPI mapOpenRscUn(const WCHAR * name);         // 12/09/12

  // Закрыть классификатор
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapCloseRsc(HRSC hRsc);

  // Сохранить классификатор на диск или на сервер после обновления
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapCommitRsc(HRSC hRsc);

  // Сохранить классификатор по указанному пути (включая имя)      // 17/01/12
  // hRsc - идентификатор классификатора карты
  // path - путь к создаваемому файлу (в UTF-16)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSaveRscAs(HRSC hRsc, const WCHAR * path);

  // Считать классификатор по указанному пути                      // 11/01/13
  // hRsc - идентификатор обновляемого классификатора карты
  // path - путь к считываемому файлу (в UTF-16)
  // Чтобы сразу сохранить считанный классификатор в тот, что был открыт
  // изначально, необходимо вызвать mapCommitRsc
  // Чтобы отменить результаты считывания нужно вызвать mapRevertRsc
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapLoadRscFrom(HRSC hRsc, const WCHAR * unpath);

  // Восстановить классификатор с диска
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapRevertRsc(HRSC hRsc);

  // Сохранить описание классификатора в виде прикладной XSD схемы      // 08/04/15
  // hRsc   - идентификатор классификатора карты
  // name   - имя создаваемой XSD схемы
  // layers - имя XML файла, содержащего указания на переименование и
  //          обобщение слоев в прикладной схеме
  // comment - текст комментария для размещения в прикладной схеме
  // isselectonly - признак вывода в схему только тех слоев, что заданы 
  //                в файле слоев (layers)
  // prefix - префикс идентификатора схемы (targetnamespace),
  //          если значение равно нулю, то в качества идентификатора присваивается
  //          имя классификатора RSC без расширения
  // (Например, для operator.rsc в схеме будут сгенерированы строки:
  // "<xsd:schema "xmlns:operator=\"http://www.gisinfo.net/bsd/operator" 
  //  targetNamespace="http://www.gisinfo.net/bsd/operator">"
  // Имена прикладных элементов в данной схеме будут начинаться с префикса "operator:"
  // Подробнее о содержании прикладной схемы можно прочитать 
  // в документе "Спецификация GML для ЦТК" 
  // При ошибке возвращает ноль

 _MAPIMP long int _MAPAPI mapRscSaveToXSDPro(HRSC hRsc, const WCHAR * name, const WCHAR * layers, 
                                             const WCHAR * comment, long int isselectonly,
                                             const WCHAR * prefix);
 _MAPIMP long int _MAPAPI mapRscSaveToXSDEx(HRSC hRsc, const WCHAR * name, const WCHAR * layers, 
                                            const WCHAR * comment, long int isselectonly);
 _MAPIMP long int _MAPAPI mapRscSaveToXSD(HRSC hRsc, const WCHAR * name, const WCHAR * layers, 
                                          const WCHAR * comment);
                                            

 /****************************************************************
 *                                                               *
 *     ДОСТУП К ОПИСАНИЮ ЛОКАЛИЗАЦИЙ ЭЛЕКТРОННОЙ КАРТЫ           *
 *    локализации начинаются с 0                                 *
 *                                                               *
 ****************************************************************/

  // Запрос количества локализаций
  // hRsc - идентификатор классификатора карты

_MAPIMP  long int  _MAPAPI mapGetRscLocalCount(HRSC hRsc);

  // Запрос названия локализации по ее номеру (0 - линейный, ...)
  // hRsc - идентификатор классификатора карты
  // local - тип локализации
  // language - язык (по умолчанию - русский)
  // Названия хранятся на двух языках ...

_MAPIMP  const char * _MAPAPI mapGetRscLocalName(HRSC hRsc,long int local,
                                                 long int language = ML_RUSSIAN);

  // Запрос названия локализации по ее номеру (0 - линейный, ...)
  // hRsc - идентификатор классификатора карты
  // local - тип локализации
  // name - адрес строки для размещения результата
  // size - размер строки
  // language - язык (по умолчанию - русский)
  // Названия хранятся на двух языках ...                  // 13/02/07
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscLocalNameEx(HRSC hRsc, long int local,
                                               char * name, long int size,
                                               long int language = ML_RUSSIAN);

_MAPIMP  long int _MAPAPI mapGetRscLocalNameUn(HRSC hRsc, long int local,
                                               WCHAR * name, long int size,
                                               long int language);

  // Запрос длины имени локализации
  // hRsc - идентификатор классификатора карты
  // (обычно все названия до 32 символов)

_MAPIMP  long int _MAPAPI mapGetRscLocalNameSize(HRSC hRsc);


  /*********************************************************
  *                                                        *
  *  ФУНКЦИИ РАБОТЫ СО СЛОЯМИ ИЗ КЛАССИФИКАТОРА ОБ'ЕКТОВ   *
  *                                                        *
  *********************************************************/

  // При наличии открытой карты  с данным классификатором  после
  // изменения порядка вывода слоев на экран, после перемещения объ-
  // ектов из слоя в слой и после удаления слоев необходимо привести
  // карту в соответствие с классификатором - вызвать mapAdjustData()
  // и при необходимости перерисовать карту

  // Запрос количества слоев
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSegmentCount(HRSC hRsc);


  // Запрос имени слоя по порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // При ошибке возвращает ""

_MAPIMP  const char * _MAPAPI mapGetRscSegmentName(HRSC hRsc,long int incode);

  // Запрос имени слоя в кодировке UNICODE по порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // name - адрес строки для размещения результата
  // size - размер строки (может быть до 2048 байт)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscSegmentNameUn(HRSC hRsc,long int incode,
                                                 WCHAR * name, long int size);
_MAPIMP  long int _MAPAPI mapGetRscSegmentNameUnicode(HRSC hRsc,long int incode,
                                                      WCHAR * name, long int size);

  // Запрос порядкового номера слоя по имени
  // Номера слоев начинаются с 0 !
  // hRsc - идентификатор классификатора карты
  // name - имя слоя
  // При отсутствии слоя возвращает - 0, код ошибки IDS_NOTFIND

_MAPIMP  long int _MAPAPI mapGetSegmentByName(HRSC hRsc, const char * name);

_MAPIMP  long int _MAPAPI mapGetSegmentByNameUn(HRSC hRsc, const WCHAR * name);  // 13/03/14


  // Запрос максимальной длины имени слоя
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSegmentNameSize(HRSC hRsc);


  // Запрос порядка вывода слоя на экран по порядковому номеру (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSegmentOrder(HRSC hRsc,long int incode);


  // Запрос количества объектов слоя по  порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSegmentObjectCount(HRSC hRsc,long int incode);


  // Установка имени слоя по порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // name   - имя слоя                                          // 15/07/09
  // При ошибке возвращает ноль, иначе порядковый номер слоя.
  // Если вернулся 0, проверьте код последней ошибки функцией mapGetRscError.
  // При установке уже имеющегося имени слоя
  // функция mapGetRscError возвращает IDS_RSCEXITSEGMENTERROR (MAPERR.RH)

_MAPIMP  long int _MAPAPI mapSetRscSegmentName(HRSC hRsc,
                                        long int incode,char * name);


  // Установка порядка вывода слоя по порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // order  - порядок вывода
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapSetRscSegmentOrder(HRSC hRsc,
                                         long int incode,
                                         long int order);


  // Удалить слой по порядковому номеру слоя (с 0)
  // Слой удаляется  вместе с объектами
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapDeleteRscSegment(HRSC hRsc,long int incode);


  // Перенести объекты из одного слоя в другой
  // hRsc - идентификатор классификатора карты
  // oldcode, newcode - номер слоя
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapMoveRscSegmentObjects(HRSC hRsc,
                                            long int oldcode,
                                            long int newcode);


  // Создать слой в классификаторе карты
  // RSCSEGMENT -  структура входных данных  (см. maptype.h)
  // hRsc - идентификатор классификатора карты                   // 15/07/09
  // При ошибке возвращает ноль, иначе порядковый номер слоя(с 0)
  // Если вернулся 0, проверьте код последней ошибки функцией mapGetRscError.
  // При установке уже имеющегося имени слоя
  // функция mapGetRscError возвращает IDS_RSCEXITSEGMENTERROR (MAPERR.RH)

_MAPIMP  long int _MAPAPI mapAppendRscSegment(HRSC hRsc, RSCSEGMENT * segment);


  // Заполнить структуру описания слоев
  // RSCSEGMENT -  структура входных данных  (см. maptype.h)
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер слоя
  // При ошибке возвращает ноль и код ошибки
  // иначе порядковый номер слоя  (с 0)

_MAPIMP  long int _MAPAPI mapGetRscSegment(HRSC hRsc, RSCSEGMENT * segment,
                                    long int incode);


  // Запросить короткое имя (ключ) слоя
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер слоя (с 0)
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscSegmentShortName(HRSC hRsc,long int incode);
_MAPIMP  long int _MAPAPI mapGetRscSegmentShortNameUn(HRSC hRsc,long int incode,WCHAR * shortname, long int size); // 12/09/12
_MAPIMP  const char * _MAPAPI mapGetRscSegmentKey(HRSC hRsc,long int incode);  //18/09/01


  // Установить короткое имя(ключ) слоя
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер слоя (с 0)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSegmentShortName(HRSC hRsc,long int incode,
                                             char * shortname );
_MAPIMP  long int _MAPAPI mapSetRscSegmentKey(HRSC hRsc,long int incode,    //18/09/01
                                             char * key);


  // Запросить порядковый номер слоя (с 0) по короткому имени(ключу) слоя   //11/03/00
  // hRsc - идентификатор классификатора карты
  // shortname - короткое имя(ключ) слоя
  // При ошибке возвращает ноль и код ошибки
  // иначе порядковый номер слоя  (с 0)

_MAPIMP  long int _MAPAPI mapGetRscSegmentByShortName(HRSC hRsc,char * shortname);
_MAPIMP  long int _MAPAPI mapGetRscSegmentByShortNameUn(HRSC hRsc, WCHAR * shortname);  // 11/02/14
_MAPIMP  long int _MAPAPI mapGetRscSegmentByKey(HRSC hRsc,char * key);     //18/09/01


  // Запросить количество семантик слоя
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер слоя (с 0)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscSegmentSemanticCount(HRSC hRsc,long int incode);


  // Запросить код семантики слоя по порядковому номеру
  // number семантики в списке (с 1)
  // hRsc - идентификатор классификатора карты
  // layer - порядковый номер слоя (с 0)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscSegmentSemanticCode(HRSC hRsc,long int layer,
                                                 long int number);


  // Добавить семантику слою
  // semanticcode код добавляемой семантики
  // hRsc - идентификатор классификатора карты
  // layer - порядковый номер слоя (с 0)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapAppendRscSegmentSemantic(HRSC hRsc,long int layer,
                                                long int semanticcode);


  // Удалить семантику из слоя
  // semanticcode код удаляемой семантики
  // hRsc - идентификатор классификатора карты
  // layer - порядковый номер слоя (с 0)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapDeleteRscSegmentSemantic(HRSC hRsc,long int layer,
                                                    long int semanticcode);


  // Установить семантику для слоя
  // hRsc - идентификатор классификатора карты
  // layer - порядковый номер слоя (с 0)
  // type = 0 - собрать всю семантику,1 - только обязательную
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapBuildRscSegmentSemantic(HRSC hRsc,long int layer,
                                              long int type);

  // Установить имя слоя в кодировке UNICODE по порядковому номеру слоя (с 0)
  // hRsc - идентификатор классификатора карты
  // incode - номер слоя
  // name - имя слоя в кодировке UNICODE
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSegmentNameUn(HRSC hRsc,long int incode,
                                                 const WCHAR * name);

_MAPIMP  long int _MAPAPI mapSetRscSegmentNameUnicode(HRSC hRsc,long int incode,
                                                      const WCHAR * name);

  // Запросить количество классов слоев
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscClassCount(HRSC hRsc);


  // Запросить идентификатор класса слоя по порядковому номеру среди классов
  // слоев (с 1)
  // hRsc - идентификатор классификатора карты
  // number - порядковый номер класса
  // при ошибке возвращает 0 иначе идентификатор класса слоя

_MAPIMP long int _MAPAPI mapGetRscClassIdent(HRSC hRsc,long int number);

  // Запросить уровень класса слоя (больше равен 1)
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // при ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRscClassLevel(HRSC hRsc,long int ident);

  // Запросить идентификатор родителя класса слоя
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // возвращает идентификатор класса или порядковый номер слоя (если класс 1 уровня)
  // при ошибке возвращает 0 (для класса 1 уровня 0 допустим)

_MAPIMP long int _MAPAPI mapGetRscClassParent(HRSC hRsc,long int ident);

  // Запросить имя класса слоя в кодировке UNICODE по идентификатору класса
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // name - адрес строки для размещения результата
  // size - размер строки (может быть до 2048 байт)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscClassNameUnicode(HRSC hRsc,long int incode,
                                                 WCHAR * name, long int size);

  // Установить имя класса слоя в кодировке UNICODE по идентификатору класса
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // buffer - полное имя класса (в UNICODE)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscClassNameUnicode(HRSC hRsc,long int ident,
                                                       const WCHAR * buffer);

  // Создать новый класс слоя
  // hRsc - идентификатор классификатора карты
  // parent - идентификатор класса - родителя или порядковый номер основного слоя,
  // key  - ключ, короткое имя класса (в в UNUCODE) (сохраняется как char[32],
  //        c завершающим нулем - уникальное
  // name - имя класса (в UNUCODE)
  // возвращает идентификатор класса
  // при ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapAppendRscClass(HRSC hRsc,long int parent,
                                        const WCHAR * key, const WCHAR * name);

  // Удалить класс слоя
  // класс удаляется со всеми классами - потомками, объекты переносятся в
  // класс - родитель или в слой - родитель
  // ident - идентификатор слоя
  // при ошибке возвращает 0, иначе 1

_MAPIMP  long int _MAPAPI mapDeleteRscClass(HRSC hRsc,long int ident);

  // Запросить слой для класса слоя по идентификатору класса
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // возвращает номер слоя (может быть 0)

_MAPIMP long int _MAPAPI mapGetRscClassGenericSegment(HRSC hRsc,long int ident);

  // Запросить ключ класса слоя в кодировке UNICODE по идентификатору класса
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор класса
  // name - адрес строки для размещения результата
  // size - размер строки (не менее 64 байта)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscClassKeyUnicode(HRSC hRsc,long int ident,
                                                   WCHAR * name, long int size);

  // Запросить идентификатор класса по ключу класса(в UNICODE)   // 10/10/14
  // buffer - уникальный ключ слоя
  // возвращает идентификатор класса или 0
_MAPIMP  long int _MAPAPI mapGetRscClassIdentbyKeyUn(HRSC hRsc,
                                                     const WCHAR * buffer);

  // Записать ключ класса(в UNICODE) по идентификатору класса      06/10/14
  // ident - идентификатор класса(более 255)
  // name - уникальный ключ слоя (в UNICODE) не более 31 символа
  // ( записывается в char[32])
  // возвращает длину ключа в байтах или 0
_MAPIMP  long int _MAPAPI mapSetRscClassKeyUnicode(HRSC hRsc,long int ident,
                                                   const WCHAR * name);

 // Запрос количества дочерних классов слоя
 // hRsc - идентификатор классификатора карты
 // number - номер слоя (c 0)
 // возвращает количество дочерних классов слоя

 _MAPIMP  long int _MAPAPI mapGetRscSegmentClassCount(HRSC hRsc,long int number);

 // Запрос количества семантик - идентификаторов поиска для слоя
 // hRsc - идентификатор классификатора карты
 // number - номер слоя (c 0)
 // возвращает количество семантик - идентификаторов поиска

 _MAPIMP  long int _MAPAPI mapGetRscSegmentSeekIdentCount(HRSC hRsc, long int number);

 // Запрос кода семантик - идентификаторов поиска для слоя
 // hRsc - идентификатор классификатора карты
 // number - номер слоя (c 0)
 // semnumber - порядковый номер семантики ( от 1 до 4)
 // возвращает код семантики - идентификатора поиска или 0

 _MAPIMP  long int _MAPAPI mapGetRscSegmentSeekIdentCode(HRSC hRsc,long int number,
                                                         long int semnumber);

 // Запрос флага является ли семантика - идентификатором поиска для слоя
 // hRsc - идентификатор классификатора карты
 // number - номер слоя (c 0)
 // semcode - код семантики
 // возвращает 1 - является или 0 - не является

 _MAPIMP  long int _MAPAPI mapGetRscSegmentSeekIdentFlag(HRSC hRsc,long int number,
                                                         long int semcode);

 // Установить семантику как идентификатор поиска для слоя
 // hRsc - идентификатор классификатора карты
 // number - номер слоя (c 0)
 // semnumber - код семантики
 // flag - 1 назначить семантику идентификатором поиска для слоя
 //      - 0 удалить семантику из списка семантик - идентификаторов поиска
 // если у слоя уже 4 семантики - идентификатора семантика не устанавливается
 // и возвращается 0
 // если семантика не входит в список семантик слоя - семантика будет добавлена
 // в список семантик слоя
 // возвращает 1, при ошибке возвращает 0

 _MAPIMP  long int _MAPAPI mapSetRscSegmentSeekIdentFlag(HRSC hRsc,long int number,
                                                long int semcode,long int flag);

  /***********************************************************
  *                                                          *
  *  ФУНКЦИИ РАБОТЫ С ОБ'ЕКТАМИ ИЗ КЛАССИФИКАТОРА ОБЪЕКТОВ   *
  *                                                          *
  ***********************************************************/

  // Создать объект
  // RSCOBJECT -  структура входных данных  (см. maptype.h)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль , иначе порядковый номер объекта (с 1)

_MAPIMP  long int  _MAPAPI mapAppendRscObject(HRSC hRsc, RSCOBJECT * object);


  // Скопировать объект
  // hRsc - идентификатор классификатора карты
  // oldcode - порядковый номер объекта с которого копируют
  // При ошибке возвращает ноль , иначе порядковый номер нового объекта (с 1)
  // Копируется заголовок объекта,вид изображения,семантика объекта
  // Код  FIRSTSERVEXCODE
  // Для того,чтобы данный объект сохранился,
  // пользователь должен переопределить внешний код

_MAPIMP  long int  _MAPAPI mapCopyRscObject(HRSC hRsc, long int oldcode);


  // Обновить объект
  // RSCOBJECT -  структура входных данных  (см. maptype.h)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль , иначе порядковый номер объекта (с 1)
  // При наличии серии внешний код и локализация и слой - не меняются
  // Если внешний вид объекта не соответствует локализации - записывается
  // умалчиваемый внешний вид

_MAPIMP  long int  _MAPAPI mapUpdateRscObject(HRSC hRsc,long int code,
                                       RSCOBJECT * object);


  // Удалить объект
  // hRsc - идентификатор классификатора карты
  // сode - порядковый номер объекта который удаляют (с 1)
  // При ошибке возвращает ноль , иначе порядковый номер удаленного объекта
  // если объект входит в серию - удаление не делается

_MAPIMP  long int  _MAPAPI mapDeleteRscObject(HRSC hRsc,long int code);


  // Заполнить структуру описания объекта
  // RSCOBJECT -  структура входных данных  (см. maptype.h)
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (с 1)
  // При ошибке возвращает ноль , иначе порядковый номер объекта

_MAPIMP  long int  _MAPAPI mapGetRscObject(HRSC hRsc,long int incode,
                                    RSCOBJECT * object);


  // Запросить порядковый номер объекта в серии однотипных
  // объектов (с общим классификационным кодом и локализацией)
  // по внутреннему коду объекта
  // (Противоположная функция - mapGetRscObjectCodeByNumber)
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (с 1)
  // При ошибке или отсутствии серии возвращает ноль ,
  // иначе номер объекта в серии

_MAPIMP  long int _MAPAPI mapGetRscObjectNumber(HRSC hRsc,long int incode);


  // Запросить размеры в микронах и свойства экранного вида объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (с 1)
  // IMAGESIZE -  структура входных данных  (см. maptype.h)
  // Строка string длиной length задается для
  // определения горизонтального размера подписи
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscImageSize(HRSC hRsc,
                            long int incode, IMAGESIZE * imagesize,
                            long int length = 0,char * string = 0);

  // Запросить размеры в микронах и свойства принтерного вида объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (с 1)
  // IMAGESIZE -  структура входных данных  (см. maptype.h)
  // Строка string длиной length задается для
  // определения горизонтального размера подписи
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscPrnImageSize(HRSC hRsc,
                            long int incode, IMAGESIZE * imagesize,
                            long int length = 0,char * string = 0);


 // Запросить габаритную рамку изображения объекта (точечный, векторный)
 // с учетом поворота объекта (IMAGEFRAME - см. maptype.h)
 // Все размеры в микронах на "бумажном" изображении (в базовом масштабе)
 // относительно первой точки метрики объекта в картографической системе
 // Для пересчета полученных координат в метры на местности нужно
 // их поделить на 1 000 000, умножить на базовый масштаб карты
 // и добавить координаты первой точки метрики
 // number - номер функции отображения (mapgdi.h)
 // param  - параметры отображения (mapgdi.h)
 // angle  - угол поворота объекта в радианах по часовой стрелке
 // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscMarkFrame(HRSC hRsc, long int number,
                                     const char * param,
                                     double angle,
                                     IMAGEFRAME * imageframe);


  // Запросить внутренний код (порядковый номер) объекта
  // по внешнему коду , локализации  и порядковому номеру среди аналогичных
  // объектов(с 1)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscObjectCodeByNumber(HRSC hRsc,
                                                long int excode,
                                                long int local,
                                                long int number = 1);

  // Запросить количество объектов с заданным внешним кодом и локализацией
  // hRsc - идентификатор классификатора карты
  // excode - внешний код объекта
  // local  - тип локализации
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscObjectsCount(HRSC hRsc,
                                          long int excode,
                                          long int local);

  // Запросить имя объекта по внутреннему  коду (порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscObjectName(HRSC hRsc,long int incode);

  // Запросить имя объекта по внутреннему  коду (порядковому номеру) объекта (с 1)
  // в кодировке UNICODE
  // hRsc - идентификатор классификатора карты                 // 28/05/09
  // incode - внутренний код объекта (номер по порядку)
  // name - адрес строки для размещения результата
  // size - зарезервированный размер строки (может быть до 2048 байт)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscObjectNameUn(HRSC hRsc,long int incode,
                                                WCHAR * name, long int size);

_MAPIMP  long int _MAPAPI mapGetRscObjectNameUnicode(HRSC hRsc,long int incode,
                                                     WCHAR * name, long int size);

  // Запросить ключ объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscObjectKey(HRSC hRsc,long int incode);

  // Запросить ключ объекта по внутреннему  коду (порядковому номеру) // 29/05/09
  // объекта (с 1) в кодировке UNICODE
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает пустую строку

_MAPIMP  long int _MAPAPI mapGetRscObjectKeyUn(HRSC hRsc,long int incode,
                                               WCHAR * name, long int size);

  // Установить ключ объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // (Короткое имя должно быть уникально)
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapSetRscObjectKey(HRSC hRsc,long int incode,
                                                const char * key);

  // Запросить код локализации объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты                 // 28/05/09
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает ноль (ноль допустим)

_MAPIMP  long int  _MAPAPI mapGetRscObjectLocal(HRSC hRsc,
                                                long int incode);

  // Запросить номер слоя объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты                 // 28/05/09
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает ноль (ноль допустим)

_MAPIMP  long int  _MAPAPI mapGetRscObjectSegment(HRSC hRsc,
                                                  long int incode);

  // Запросить идентификатор класса слоя объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты                 // 25/09/14
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает ноль

_MAPIMP long int  _MAPAPI mapGetRscObjectClass(HRSC hRsc, long int incode);

  // Установить идентификатор класса слоя объекта по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты                 // 25/09/14
  // incode - внутренний код объекта (номер по порядку)
  // ident - идентификатор класса
  // При ошибке возвращает ноль

 _MAPIMP long int  _MAPAPI mapSetRscObjectClass(HRSC hRsc, long int incode,
                                                 long int ident);

  // Запросить идентификатор объекта (постоянное уникальное значение
  // в пределах данного классификатора) по внутреннему  коду (порядковому номеру)
  // объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectIdent(HRSC hRsc,long int incode);


  // Запросить внутренний код (порядковый номер) объекта
  // по идентификатору
  // hRsc - идентификатор классификатора карты
  // ident - идентификатор объекта
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectIdentIncode(HRSC hRsc,long int ident);


  // Запросить внутренний код (порядковый номер) объекта
  // по ключу
  // hRsc - идентификатор классификатора карты
  // key  - ключ объекта
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectCodeByKey(HRSC hRsc,const char * key);
_MAPIMP  long _MAPAPI mapGetRscObjectKeyIncode(HRSC hRsc,const char * key);
_MAPIMP  long _MAPAPI mapGetRscObjectCodeByKeyUn(HRSC hRsc, const WCHAR * key);  // 29/05/09

  // Запросить внутренний  код (порядковый номер) объекта       //22/06/01
  // по имени
  // hRsc - идентификатор классификатора карты
  // name - имя объекта
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectCodeByName(HRSC hRsc,char * name);


  // Запросить число семантик влияющих на внещний вид объекта по внутреннему
  // коду (порядковому номеру)  объекта (с 1)
  // incode - внутренний код объекта (номер по порядку)
  // hRsc   - идентификатор классификатора карты

_MAPIMP  long  _MAPAPI mapGetRscImageSemanticCount(HRSC hRsc,long int incode);


  // Запрос кода семантики влияющей на изображение по внутреннему коду
  // (порядковому номеру) объекта и порядковому номеру такой семантики (c 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // number - номер семантики
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscImageSemanticCode(HRSC hRsc,long int incode,
                                                 long int number);

  // Запpосить количество связанных подписей объекта        //01/11/01
  // по внутреннему коду (порядковому номеру) объекта
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)

_MAPIMP  long _MAPAPI mapGetRscObjectRelateCount(HRSC hRsc,long int incode);


  // Запpосить описание связанной подписи по внутреннему коду   //01/11/01
  // (порядковому номеру) объекта и по порядковому номеру связанной подписи (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // relate - описание связанной подписи (maptype.h)
  // Order  - порядковый номер связанной подписи
  // Возвращает идентификатор подписи,
  // при ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectRelateOrder(HRSC hRsc,long int incode,
                                                 long int order,RSCRELATION * relate);

  // Запросить параметры шрифта для подписи семантики объекта   // 14/04/06
  // по внутреннему  коду (порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // semanticcode - код семантики объекта
  // viewtype - вид отображения объекта 0 - экранный, 1 - принтерный
  // При ошибке или если в параметрах объекта нет подписи возвращает  0

_MAPIMP  long _MAPAPI mapGetRscObjectSemanticFont(HRSC hRsc, long int incode,
                                                  long int semanticcode, LOGFONT * font,
                                                  long int viewtype);

  // Установить имя объекта по внутреннему  коду (порядковому номеру) объекта (с 1)
  // в кодировке UNICODE
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // name - имя объекта в кодировке UNICODE
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscObjectNameUn(HRSC hRsc,long int incode,
                                                const WCHAR * name);

_MAPIMP  long int _MAPAPI mapSetRscObjectNameUnicode(HRSC hRsc,long int incode,
                                                     const WCHAR * name);


  /**************************************************************
  *                                                             *
  *  ФУНКЦИИ РАБОТЫ С ПАРАМЕТРАМИ ОТОБРАЖЕНИЯ ОБЪЕКТОВ          *
  *                                                             *
  **************************************************************/

  // Запросить номер функции отображения объекта по внутреннему  коду
  // (порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long  _MAPAPI mapGetRscObjectFunction(HRSC hRsc,long int incode);


  // Запросить длину параметров отображения объекта по внутреннему  коду
  // (порядковому номеру)объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscObjectParametersSize(HRSC hRsc,long int incode);


  // Запросить параметры отображения объекта по внутреннему  коду
  // (порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  const char * _MAPAPI mapGetRscObjectParameters(HRSC hRsc,long int incode);


  // Запросить количество примитивов в параметрах отображения объекта по
  // внутреннему коду (порядковому номеру) объекта (с 1) и виду отображения
  // viewtype: 0 - экранный, 1 - принтерный
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long  _MAPAPI mapGetRscPrimitiveCount(HRSC hRsc,long int incode,
                                               long int viewtype = 0);


  // Запросить номер функции отображения примитива по порядковому
  // номеру примитива в параметрах отображения объекта ,
  // внутреннему коду (порядковому номеру) объекта (с 1) и виду отображения
  // viewtype: 0 - экранный, 1 - принтерный
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long  _MAPAPI mapGetRscPrimitiveFunction(HRSC hRsc, long int incode,
                                                  int number,
                                                  long int viewtype = 0);

  // Запросить длину параметров примитива по порядковому
  // номеру примитива в параметрах отображения объекта ,
  // внутреннему коду (порядковому номеру) объекта (с 1) и виду отображения
  // viewtype: 0 - экранный, 1 - принтерный
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // number - номер примитива
  // При ошибке возвращает 0

_MAPIMP  long  _MAPAPI mapGetRscPrimitiveLength(HRSC hRsc,long int incode,
                                                int number,
                                                long int viewtype = 0);

  // Запросить адрес параметров примитива по порядковому
  // номеру примитива в параметрах отображения объекта ,
  // внутреннему коду (порядковому номеру) объекта (с 1) и виду отображения
  // viewtype: 0 - экранный, 1 - принтерный
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // number - номер примитива
  // При ошибке возвращает 0

_MAPIMP  char * _MAPAPI mapGetRscPrimitiveParameters(HRSC hRsc,long int incode,
                                                     int number,
                                                     long int viewtype = 0);

  // Проверка соответствия локализации и вида отображения объекта
  // по внутреннему коду  объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long _MAPAPI mapGetRscImageSuitable(HRSC hRsc,long int local,
                                     long int incode);


  // Установить внешний вид объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (с 1)
  // length - длина параметров
  // number - номер функции отображения
  // param  - указатель на параметры функции
  // При ошибке возвращает ноль , иначе порядковый номер объекта

_MAPIMP  long int  _MAPAPI mapSetRscObjectImage(HRSC hRsc,
                                         long int incode,
                                         long int length,
                                         long int number,char * param);


  // Запросить номер функции (принтерного) отображения объекта по внутреннему коду
  // (порядковому номеру) объекта
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0

_MAPIMP  long  _MAPAPI mapGetRscPrintObjectFunction(HRSC hRsc,long int incode);   //26/11/01


  // Запросить длину параметров (принтерного)отображения объекта по внутреннему  коду
  //(порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0                                              //05/02/02

_MAPIMP  long _MAPAPI mapGetRscPrintObjectParametersSize(HRSC hRsc,long int incode);


  // Запросить параметры отображения (принтерного)объекта по внутреннему  коду
  //  объекта
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // При ошибке возвращает 0
                                                       //26/11/01
_MAPIMP  const char * _MAPAPI mapGetRscPrintObjectParameters(HRSC hRsc,long int incode);


  // Установить принтерный вид объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // length - длина параметров
  // number - номер функции отображения
  // param  - указатель на параметры функции
  // При ошибке возвращает ноль , иначе порядковый номер объекта
                                                     //26/11/01
_MAPIMP  long int  _MAPAPI mapSetRscPrintObjectImage(HRSC hRsc,
              long int incode,long int length,long int number,char * param);


  // Найти "основной" цвет изображения объекта                  // 11/04/05
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // viewtype - вид отображения  0 - экранный, 1 - принтерный
  // При отсутствии цвета возвращает 0xFF000000
  // При ошибке возвращает 0

_MAPIMP  COLORREF  _MAPAPI  mapGetRscObjectBaseColor(HRSC hRsc, long int incode,
                                          long int viewtype);


  // Запросить параметры шрифта объекта по внутреннему  коду     // 14/04/06
  // (порядковому номеру) объекта (с 1)
  // hRsc - идентификатор классификатора карты
  // incode - внутренний код объекта (номер по порядку)
  // viewtype -вид отображения  0 - экранный, 1 - принтерный
  // При ошибке или если в параметрах объекта нет подписи возвращает  0

_MAPIMP long int _MAPAPI mapGetRscObjectFont(HRSC hRsc,long int incode,LOGFONT * font,
                                             long int viewtype);

  // Заполнить параметры шрифта по параметрам текста
  // Параметры шрифта заполняются корректно, если среди открытых карт есть
  // карта с переданным идентификатором классификатора
  // Размер шрифта вычисляется для базового масштаба карты с текущим
  // разрешением экрана
  // hRsc - идентификатор классификатора карты
  // text - параметры функции отображения текста
  // font - возвращаемые параметры
  // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapGetRscTextFont(HRSC hRsc, const IMGTEXT *text, LOGFONT *font);

 // Запросить список кодов семантик, формирующих подписи в векторных знаках,
 // являющихся частью вида объекта
 // incode - внутренний код (порядковый номер объекта с 1)        // 15/02/11
 // size - размерность массива(количество элементов)
 // code - адрес массива кодов семантики
 // viewtype -вид отображения  0 - экранный, 1 - принтерный, 2 - все
 // возвращает число семантик, от которых зависят подписи
 // если размер массива меньше, чем число семантик "лишние" семантики не пишутся

_MAPIMP long int _MAPAPI mapGetRscObjectLabelSemantics(HRSC hrsc, long int incode,
                               long int viewtype, long int size, int * code);

 // Удалить принтерный вид объекта
 // incode - внутренний код (порядковый номер объекта с 1)        // 06/05/16
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapDeleteRscPrintObjectParameters(HRSC hRsc,
                                                           long int incode);


  /**************************************************************
  *                                                             *
  *     ФУНКЦИИ УПРАВЛЕНИЯ МАСШТАБИРОВАНИЕМ ОБЪЕКТОВ            *
  *                                                             *
  **************************************************************/

  // Запросить флаг сжатия изображения объекта                     // 18/04/05
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // Возвращает 0 - изображение сжимается, 1 - нет.

_MAPIMP  int _MAPAPI  mapGetRscObjectPressure(HRSC hRsc, long int incode);


  // Установить флаг сжатия изображения объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // flag - флаг сжатия изображения 0 - сжимается, 1 - нет.
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI  mapSetRscObjectPressure(HRSC hRsc, long int incode,long int flag);

  // Запросить размер максимального сжатия изображения объекта      // 18/12/07
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // Возвращает коэффмциент максимального сжатия, умноженный на 10.

_MAPIMP  int _MAPAPI  mapGetRscObjectPressLimit(HRSC hRsc, long int incode);

  // Установить размер максимального сжатия изображения объекта      // 18/12/07
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // presslimit - коэффмциент максимального сжатия, умноженный на 10.
  //              в интервале от 10 до 250)
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI  mapSetRscObjectPressLimit(HRSC hRsc, long int incode,
                                                 long int presslimit);

  // Запросить флаг масштабирования изображения объекта    // 02/11/12
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // Возвращает 1 - изображение масштабируется, 0 - нет.

_MAPIMP  int _MAPAPI  mapGetRscObjectScale(HRSC hRsc, long int incode);

  // Установить флаг масштабирования изображения объекта
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // flag - флаг масштабирования изображения 1 - масштабируется, 0 - нет.
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI  mapSetRscObjectScale(HRSC hRsc, long int incode,long int flag);

  // Запросить размер максимального увеличения изображения объекта      // 18/12/07
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // Возвращает коэффмциент максимального увеличения, умноженный на 10.

_MAPIMP  int _MAPAPI  mapGetRscObjectScaleLimit(HRSC hRsc, long int incode);

  // Установить размер максимального увеличения изображения объекта   // 18/12/07
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // scalelimit - коэффмциент максимального сжатия, умноженный на 10.
  //              в интервале от 10 до 250)
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI  mapSetRscObjectScaleLimit(HRSC hRsc, long int incode,
                                                long int scalelimit);

  // Установить границы видимости объекта на карте (диапазон масштабов    // 02/07/08
  // видимости объекта на карте
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (внутренний код в классификаторе)
  // bottom - минимальное значение знаменателя масштаба при котором виден
  //          объект
  // top    - максимальное значение знаменателя масштаба при котором виден
  //          объект
  // При установке значений масштабов они округляются до ближайших
  // значений из таблицы стандартных масштабов с установкой признака "включая"
  // или "исключая" границу (в зависимости от устанавливаемого значения)
  // Например, для значений bottom = 1, top = 199999, объект будет
  // виден в масштабах крупнее 1:200 000 (при увеличении более 200 000),
  // у другого объекта может быть bottom = 200000, top = 1000000 - он
  // будет виден в масштабах 1:200 000 и мельче и может подменять предыдущий
  // объект (вместе они никогда не отобразятся).
  // Но если объектам поставить границы 160000 и 170000 - они приравняются
  // к 200 000 то есть будут одинаковыми.
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI mapSetRscObjectScaleBorder(HRSC hRsc, long int incode,
                                                long int bottom, long int top);

  // Запросить границы видимости объекта на карте (диапазон масштабов
  // видимости объекта на карте
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (внутренний код в классификаторе)
  // bottom - минимальное значение знаменателя масштаба при котором виден
  //          объект
  // top    - максимальное значение знаменателя масштаба при котором виден
  //          объект
  // При ошибке возвращает 0

_MAPIMP  int _MAPAPI mapGetRscObjectScaleBorder(HRSC hRsc, long int incode,
                                                long int * bottom, long int * top);

  // Запросить вхождение верхней границы видимости объекта на карте    // 11/12/15
  // в диапазон видимости
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (внутренний код в классификаторе)
  // Входит - возвращает 1, иначе 0

 _MAPIMP  int _MAPAPI mapGetRscObjectBotScaleInclude(HRSC hRsc, long int incode);

  // Запросить вхождение нижней границы видимости объекта на карте
  // в диапазон видимости
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта (внутренний код в классификаторе)
  // Входит - возвращает 1, иначе 0

 _MAPIMP  int _MAPAPI mapGetRscObjectTopScaleInclude(HRSC hRsc, long int incode);


  /**************************************************************
  *                                                             *
  *  ФУНКЦИИ РАБОТЫ С СЕМАНТИКАМИ ИЗ КЛАССИФИКАТОРА ОБ'ЕКТОВ    *
  *                                                             *
  **************************************************************/

  // Создать новую семантику - возвращает  код созданной семантики
  // hRsc - идентификатор классификатора карты
  // структура RSCSEMANTICEX описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapAppendRscSemanticEx(HRSC hRsc,RSCSEMANTICEX * rsem);


  // Обновить семантику - возвращает  код обновленной семантики
  // code - код обновляемой семантики
  // classupdate - 1, классификатор семантики удаляется для последующего
  // обновления(например при смене типа семантики), 0 - тип семантики остается
  // прежний и обновления классификатора данной семантики не нужно.
  // hRsc - идентификатор классификатора карты
  // структура RSCSEMANTICEX описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapUpdateRscSemanticEx(HRSC hRsc,long int code,
                                    RSCSEMANTICEX * rsem,long int classupdate);


  // Удалить семантику
  // code - код удаляемой семантики
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapDeleteRscSemantic(HRSC hRsc,long int code);


  // Запросить информацию о применении семантики для объектов карты
  // code - код семантики
  // applysemantic - структура для информации (maptype.h)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapGetRscApplySemantic(HRSC hRsc,long int code,
                                           APPLYSEMANTIC * applysemantic);


  // Объявить принадлежность семантики объекту
  // hRsc - идентификатор классификатора карты
  // objincode - внутренний код объекта,
  // code      - код семантики,
  // enable    - код доступа к семантике
  // (2 - обязательная, 1 - возможная)
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapEnableRscSemantic(HRSC hRsc,
                                         long int objincode,
                                         long int code,
                                         long int enable);


  // Запросить количество семантик в классификаторе
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticCount(HRSC hRsc);

  // Запросить код семантики по порядковому номеру
  // hRsc   - идентификатор классификатора карты
  // number - порядковый номер семантики с 1
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticCodeByNumber(HRSC hRsc, long int number);

  // Запросить код семантики по короткому имени (ключу)семантики   // 18/09/15
  // (ключ семантики должен быть строкой символов латинского алфавита, цифр, знаков подчеркивания)
  // hRsc      - идентификатор классификатора карты
  // shortname - короткое имя семантики
  // При ошибке возвращает ноль
  
_MAPIMP long int _MAPAPI mapGetRscSemanticCodeByKey(HRSC hRsc, const char * key);
_MAPIMP long int _MAPAPI mapGetRscSemanticCodeByKeyUn(HRSC hRsc, const WCHAR * key);

  // Запросить порядковый номер семантики
  // по короткому имени (ключу)семантики                        
  // hRsc      - идентификатор классификатора карты
  // shortname - короткое имя семантики
  // При ошибке возвращает ноль
  // иначе порядковый номер семантики (с 1)

_MAPIMP  long int _MAPAPI mapGetRscSemanticByShortName(HRSC hRsc,const char * shortname);
_MAPIMP  long int _MAPAPI mapGetRscSemanticByKey(HRSC hRsc,const char * key);
_MAPIMP  long int _MAPAPI mapGetRscSemanticByShortNameUn(HRSC hRsc,const WCHAR * shortname);
_MAPIMP  long int _MAPAPI mapGetRscSemanticByKeyUn(HRSC hRsc, const WCHAR * key);

  // Запросить название семантики по порядковому номеру
  // hRsc   - идентификатор классификатора карты
  // number - порядковый номер семантики с 1
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscSemanticNameByNumber(HRSC hRsc, long int number);

  // Запросить название семантики по порядковому номеру в кодировке UNICODE
  // hRsc - идентификатор классификатора карты
  // number - номер семантики
  // name   - адрес строки для размещения результата
  // size   - максимальный размер выходной строки
  // При ошибке возвращает пустую строку

_MAPIMP  long int _MAPAPI mapGetRscSemanticNameByNumberUn(HRSC hRsc, long int number,
                                                          WCHAR * name, long int size);

  // Запросить название семантики по коду в кодировке UNICODE
  // hRsc - идентификатор классификатора карты                  // 15/06/09
  // code - код семантики
  // name - адрес строки для размещения результата
  // size - максимальный размер выходной строки (может быть до 2048 байт)
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapGetRscSemanticNameUn(HRSC hRsc, long int code,
                                                  WCHAR * name, long int size);

_MAPIMP  long int _MAPAPI mapGetRscSemanticNameUnicode(HRSC hRsc, long int code,
                                                       WCHAR * name, long int size);

  // Установить название семантики по коду в кодировке UNICODE
  // hRsc - идентификатор классификатора карты                  // 17/04/11
  // code - код семантики
  // name - адрес строки с новым названием
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSemanticNameUn(HRSC hRsc, long int code,
                                                  const WCHAR * name);

_MAPIMP  long int _MAPAPI mapSetRscSemanticNameUnicode(HRSC hRsc, long int code,
                                                       const WCHAR * name);

  // Запросить тип семантики по ее внешнему коду
  // Коды типов семантик - см. maptype.h (SEMTYPE)
  // hRsc - идентификатор классификатора карты
  // code - внешений код семантики
  // При ошибке возвращает ноль (символьная семантика имеет тип ноль!)

_MAPIMP  long int _MAPAPI mapGetRscSemanticTypeByCode(HRSC hRsc, long int code);


  // Запросить короткое имя (ключ) семантики
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscSemanticShortName(HRSC hRsc,long int code);
_MAPIMP  long int _MAPAPI mapGetRscSemanticShortNameUn(HRSC hRsc,long int code,
                                                WCHAR * shortname, long int size);
_MAPIMP  const char * _MAPAPI mapGetRscSemanticKey(HRSC hRsc,long int code);

  // Запросить короткое имя (ключ) семантики в кодировке UNICODE
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // name   - адрес строки для размещения результата
  // size   - максимальный размер строки
  // При ошибке возвращает пустую строку

_MAPIMP  long int _MAPAPI mapGetRscSemanticKeyUn(HRSC hRsc,long int code,
                                                 WCHAR * name, long int size);

  // Установить короткое имя (ключ) семантики
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSemanticShortName(HRSC hRsc, long int code,
                                                     const char * shortname);
_MAPIMP  long int _MAPAPI mapSetRscSemanticKey(HRSC hRsc,long int code,
                                               const char * key);


  // Установить короткое имя (ключ) семантики
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSemanticShortNameUn(HRSC hRsc, long int code,
                                                       const WCHAR * shortname);
_MAPIMP  long int _MAPAPI mapSetRscSemanticKeyUn(HRSC hRsc,long int code,
                                                 const WCHAR * key);

  // Установить размер и точность значения семантики 
  // hRsc - идентификатор классификатора карты
  // size - размер значения семантики (включая десятичную точку)
  // decimal - количество знаков после запятой (у символьных - 0)
  // code - код семантики
  // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSemanticDecimal(HRSC hRsc,long int code,
                                                   long int size,long int decimal);


  // Заполнить структуру описания семантической характеристики
  // по коду семантики
  // hRsc - идентификатор классификатора карты
  // code - внешний код семантики
  // структура RSCSEMANTICEX описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticExByCode(HRSC hRsc,RSCSEMANTICEX * semtype,
                                                    long int code);


  // Запросить количество значений классификатора семантической
  // характеристики по коду семантики (если ее тип TCODE)
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticClassificatorCount(HRSC hRsc,
                                                    long int code);

  //  Запросить название значения характеристики из
  //  классификатора семантики по коду семантики и
  //  последовательному номеру в классификаторе
  //  hRsc - идентификатор классификатора карты
  //  code - код семантики
  //  number - последовательный номер в классификаторе(1,2,3...)
  //  При ошибке возвращает пустую строку

_MAPIMP  const char * _MAPAPI mapGetRscSemanticClassificatorName(HRSC hRsc,long int code,
                                                                 long int number);

  //  Запросить название значения характеристики из
  //  классификатора семантики по коду семантики и
  //  последовательному номеру в классификаторе в кодировке UNICODE
  //  hRsc - идентификатор классификатора карты
  //  code   - код семантики
  //  number - последовательный номер в классификаторе(1,2,3...)
  //  name   - адрес строки для размещения результата
  //  size   - максимальный размер строки (может быть до 2048 байт)
  //  При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticClassificatorNameUn(HRSC hRsc,long int code,
                                                               long int number,
                                                               WCHAR * name, long int size);

  // Запросить код значения характеристики из
  // классификатора семантики по коду семантики и
  // последовательному номеру в классификаторе
  // hRsc - идентификатор классификатора карты
  // number - последовательный номер в классификаторе(1,2,3...)
  // code - код семантики
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetRscSemanticClassificatorCode(HRSC hRsc,
                                                             long int code,
                                                             long int number);

  // Записать новую "строчку" классификатора
  // (числовое значение и символьное) возвращает
  // номер записанной строки с 1
  // hRsc - идентификатор классификатора карты
  // code - код семантической характеристики
  // value - числовое значение
  // name  - символьное значение семантической характеристики
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapAppendRscClassificator(HRSC hRsc,
                                                     long int code,
                                                     long int value, const char * name);
_MAPIMP  long int  _MAPAPI mapAppendRscClassificatorUn(HRSC hRsc,                        // 07/12/12
                                                     long int code,
                                                     long int value, const WCHAR * name);

  // Обновить символьное значение классификатора по номеру строки (с 1)
  // Возвращает номер исправленной строки с 1
  // hRsc - идентификатор классификатора карты
  // code - код семантической характеристики
  // index - номер строки
  // name  - символьное значение семантической характеристики
  // При ошибке возвращает ноль

_MAPIMP  long int  _MAPAPI mapUpdateRscClassificatorName(HRSC hRsc,
                                                         long int code,
                                                         long int index, const char * name);

_MAPIMP  long int  _MAPAPI mapUpdateRscClassificatorNameUn(HRSC hRsc,               // 12/09/12
                                                         long int code,
                                                         long int index, const WCHAR * name);

 // Найти числовой идентификатор записи семантики-классификатора по строковому значению
  // Поиск осуществляется по совпадению заданной строки с коротким и полным
  // именем в семантике-классификаторе  
  // hRsc - идентификатор классификатора карты
  // code - код семантической характеристики типа классификатор
  // name  - символьное значение семантической характеристики, которое нужно найти
  // Если не нашли возвращает 0, иначе - числовое значение классификатора семантики

_MAPIMP  long int  _MAPAPI mapFindRscClassificatorCode(HRSC hRsc,
                                                       long int code, const char * name);
_MAPIMP  long int  _MAPAPI mapFindRscClassificatorCodeUn(HRSC hRsc,
                                                       long int code, const WCHAR * name);     // 28/12/12

 // Запросить ключ строки классификатора семантики  16/02/10
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора

_MAPIMP  const char * _MAPAPI mapGetRscClsKey(HRSC hRsc,long code,long value);

 // Записать ключ классификатора семантики                       16/02/10
 // Для одной семантики ключ классификатора должен быть уникален
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора
 // возвращает значение классификатора с данным ключом, если это значение
 // не совпадает с входным - ключ повторялся и не записан

_MAPIMP  long int _MAPAPI mapSetRscClsKey(HRSC hRsc,long code,long value,
                                           const char * key);

 // Запрос полного имени классификатора семантики (в UNICODE)
 // по коду семантики и значению классификатора
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора
 // size - размер буфера для размещения строки         // 16/02/10
 // buffer - буфер для размещения строки
 // возвращает длину полного имени или 0

_MAPIMP  long int _MAPAPI mapGetRscClsFullNameUn(HRSC hRsc, long int code, long int value,
                                                 long int size, WCHAR * buffer);

_MAPIMP  long int _MAPAPI mapGetRscSemanticClassificatorFullNameUn(HRSC hRsc,   // 15/05/13
                          long int code, long int value, WCHAR * buffer, int size);

 // Записать полное имя классификатора семантики (в UNICODE)
 // по коду семантики и значению классификатора
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора
 // buffer - полное имя объекта (в UNICODE)           // 16/02/10
 // возвращает длину полного имени или 0

_MAPIMP  long int _MAPAPI mapSetRscClsFullNameUn(HRSC hRsc,long int code,long int value,
                                                 const WCHAR * buffer);

 // Запрос короткого имени классификатора семантики   // 17/12/14
 // по коду семантики и числовому значению классификатора
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора
 // При ошибке возвращает адрес пустой строки,
 // при успешном выполнении - адрес строки

 _MAPIMP const char * _MAPAPI mapRscSemanticClassificatorShortName(HRSC hRsc,
                                       long int code,long int value);

 // Запросить значение классификатора семантики по ключу   16/02/10
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // value - значение классификатора
_MAPIMP  long int _MAPAPI mapGetRscClsKeyValue(HRSC hRsc,long int code,char * key);


  // Запросить является ли семантика списком 3D изображений по коду  // 27/06/13
  // hRsc - идентификатор классификатора карты
  // code - код семантики
  // Если семантика - список 3D изображений возвращает 1,
  // иначе - 0

_MAPIMP  long int _MAPAPI mapGetRscSemantic3DListFlag(HRSC hRsc,long int code);

 // Установить признак семантики - список 3D изображений     // 28/06/13
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // При ошибке возвращает 0

_MAPIMP  long int _MAPAPI mapSetRscSemantic3DListFlag(HRSC hRsc,long int code);

 // Запросить флаг запрета редактирования семантики          // 23/08/13
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // возвращает 0 или 1

_MAPIMP long int _MAPAPI mapGetRscSemanticNotEditFlag(HRSC hRsc,long int code);

 // Установить флаг запрета редактирования семантики     // 23/08/13
 // hRsc - идентификатор классификатора карты
 // code - код семантики
 // flag - 1 - установить запрет, 0 - снять
 // При ошибке возвращает 0

_MAPIMP long int _MAPAPI mapSetRscSemanticNotEditFlag(HRSC hRsc,long int code,long int flag);

  // Запросить количество семантик - списков 3D изображений  в данной библиотеке
  // libcode  - код библиотеки                                      // 27/06/13
  // hRsc - идентификатор классификатора карты

_MAPIMP  long int _MAPAPI mapGetRscSemantic3DListCount(HRSC hRsc,long int libcode);

  // Запросить список кодов семантик - списков 3D изображений     // 27/06/13
  // в данной библиотеке
  // hRsc - идентификатор классификатора карты
  // libcode  - код библиотеки
  // code - адрес массива кодов семантик
  // countlimit - размер массива
  // Возвращает число записанных кодов семантики
  // иначе 0

_MAPIMP  long int _MAPAPI mapGetRscSemantic3DList(HRSC hRsc, long int libcode,
                                                  long int *code, long int countlimit);

  /**************************************************************
  *                                                             *
  *  ФУНКЦИИ РАБОТЫ С СЕМАНТИКАМИ ДЛЯ ОТДЕЛЬНЫХ  ОБ'ЕКТОВ       *
  *                                                             *
  **************************************************************/

  // Запросить количество семантик для данного объекта
  // по значимости семантики - см. maptype.h (SEMANTIC_FOR_OBJECT )
  // по внутреннему коду (порядковому номеру) объекта (c 1)
  // hRsc - идентификатор классификатора карты
  // Возвращает количество семантик требуемой значимости
  // hRsc - идентификатор классификатора карты
  // incode - порядковый номер объекта
  // importance - важность

_MAPIMP  long int  _MAPAPI mapGetRscSemanticObjectCount( HRSC hRsc,long int incode,
                                                  long int importance);


  // Запросить использование семантики для данного объекта -
  // incode - внутренний код (порядковый номер)объекта
  // semanticcode - код семантики
  // hRsc - идентификатор классификатора карты
  // Возвращает значимость семантики  см. maptype.h (SEMANTIC_FOR_OBJECT )

_MAPIMP  long int  _MAPAPI mapGetRscSemanticObjectUsed(HRSC hRsc,long int incode,
                                      long int semanticcode);

  // Запросить использование семантики для данного объекта -    // 19/02/09
  // без учета общих семантик и семантик слоя
  // incode - внутренний код(индекс)объекта
  // semanticcode - код семантики
  // hRsc - идентификатор классификатора карты
  // Возвращает значимость семантики  см. maptype.h (SEMANTIC_FOR_OBJECT )

_MAPIMP  long int  _MAPAPI mapGetRscSemanticOnlyObjectUsed(HRSC hRsc,long int incode,
                                      long int semanticcode);


  // Запросить код семантики  для данного объекта по порядковому номеру
  // данной семантики для объекта (с 1) и значимости семантики
  // см. maptype.h (SEMANTIC_FOR_OBJECT ) При значимости семантики
  // ALL_SEMANTIC - возвращает семантику в порядке сортировки
  // incode - внутренний код (порядковый номер) объекта
  // importance - значимости семантики
  // hRsc - идентификатор классификатора карты
  // Возвращает код семантики
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscSemanticObjectCode(HRSC hRsc,long int incode,
                                                     long int number,long int importance);

  // Запросить список всех кодов семантики для данного объекта
  // incode - внутренний код(индекс)объекта
  // hRsc   - идентификатор классификатора карты
  // code   - адрес массива семантик
  // countlimit - размер массива
  // Возвращает число считанных кодов семантики
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscSemanticObjectCodeList(HRSC hRsc,long int incode,
                                                         long int * code, long int countlimit);
                                                           
  // Добавить код семантики  для данного объекта по значимости семантики
  // (POSSIBLE_SEMANTIC или MUST_SEMANTIC)
  // semanticcode - код семантики
  // incode - внутренний код (порядковый номер) объекта
  // hRsc - идентификатор классификатора карты
  // Возвращает номер добавленной семантики в семантиках данного типа
  // При ошибке возвращает ноль

_MAPIMP   long int  _MAPAPI mapAppendRscSemanticObject(HRSC hRsc,long int incode,
                                     long int semanticcode,long int importance);


  // Изменить значимость семантики для данного объекта
  // (POSSIBLE_SEMANTIC или MUST_SEMANTIC)
  // semanticcode - код семантики
  // incode - внутренний код (порядковый номер) объекта
  // hRsc - идентификатор классификатора карты
  // Возвращает число семантик требуемой значимости
  // При ошибке возвращает ноль

_MAPIMP   long int  _MAPAPI mapUpdateRscSemanticObject(HRSC hRsc,long int incode,
                                     long int semanticcode,long int importance);


  // Удалить семантику для данного объекта
  // (POSSIBLE_SEMANTIC или MUST_SEMANTIC)
  // semanticcode - код семантики
  // incode - внутренний код (порядковый номер) объекта
  // hRsc - идентификатор классификатора карты
  // Возвращает общее число семантик объекта
  // При ошибке возвращает ноль

_MAPIMP   long int  _MAPAPI mapDeleteRscSemanticObject(HRSC hRsc,long int incode,
                                     long int semanticcode);


   // Пересортировать массив семантик для объекта в соответствии с  // 16/02/04
   // порядковыми номерами
   // incode - внутренний код (порядковый номер) объекта
   // hRsc - идентификатор классификатора карты
   // count - размер массива семантик объекта для сортировки
   // semantics - указатель на список номеров требуемых семантик объекта.
   // order     - указатель на выходной массив заполнен отсортированными
   // порядковыми номерами входных семантик(с 1 )
   // при нормальном завершении возвращает 1

_MAPIMP   long int  _MAPAPI mapSortRscObjSemantic(HRSC hRsc,long int incode,
                            long int count,long int * semantics,long int * order);


   // Записать порядок семантик для объекта в соответствии с входным
   // списком
   // incode - внутренний код (порядковый номер) объекта
   // hRsc - идентификатор классификатора карты
   // count - размер массива семантик объекта
   // semantics - указатель на сортированный список семантик объекта.
   // Семантики которые не наначены объекту - пропускаются,
   // Если какие - то семантики пропущены пишутся в конец списка
   // при нормальном завершении возвращает количество семантик объекта

_MAPIMP   long int  _MAPAPI mapSetRscObjSemanticOrder(HRSC hRsc,long int incode,
                                       long int count,long int * semantics);

   // Составить список всех объектов классификатора, использующих семантики с
   // признаком - список 3D вида.
   // hRsc - идентификатор классификатора карты
   // buffer - указатель на массив байтов 
   // count - размер буфера в байтах (не менее количества объектов классификатора) 
   // Если для объекта разрешена семантика - список 3D видов, в буфер
   // на место, определяемое внутренним кодом объекта заносится 1, иначе 0.
   // при нормальном завершении возвращает количество объектов, для которых
   // могут использоваться семантики - списки 3D видов
                                                                   // 16/09/13
_MAPIMP long int  _MAPAPI mapGetRscAll3DSemObjects(HRSC hRsc,char * buffer,
                                                long int count);

  /**************************************************************
  *                                                             *
  *  ФУНКЦИИ РАБОТЫ С ПАЛИТРАМИ КЛАССИФИКАТОРОВ                 *
  *                                                             *
  **************************************************************/

  // Запросить цвет из палитры
  // color - цвет в COLORREF
  // index - порядковый номер цвета в палитре (с 1)
  // number - порядковый номер палитры (с 1)
  // hRsc - идентификатор классификатора карты
  // возвращает цвет в COLORREF

_MAPIMP  COLORREF _MAPAPI mapGetRscColor(HRSC hRsc,long int index,
                                         long int number = 1);


  // Установить цвет в данную палитру
  // color - цвет в COLORREF
  // index - порядковый номер цвета в палитре (с 1)
  // number - порядковый номер палитры (с 1)
  // hRsc - идентификатор классификатора карты
  // возвращает единицу
  // При ошибке возвращает ноль

_MAPIMP   long int  _MAPAPI mapSetRscColor(HRSC hRsc,COLORREF color,long int index,
                                           long int number = 1);

  // Запросить количество цветов в палитре классификатора
  // hRsc - идентификатор классификатора карты

_MAPIMP  long int  _MAPAPI mapGetRscColorCount(HRSC hRsc);


  // Запросить количество палитр классификатора
  // hRsc - идентификатор классификатора карты

_MAPIMP  long int  _MAPAPI mapGetRscPaletteCount(HRSC hRsc);


  // Запрос имени палитры по ее номеру
  // number - порядковый номер палитры (с 1)
  // hRsc - идентификатор классификатора карты

_MAPIMP  char * _MAPAPI mapGetRscPaletteName(HRSC hRsc,int number);
_MAPIMP long int _MAPAPI mapGetRscPaletteNameUn(HRSC hRsc, int number, WCHAR * name, long int size);

  // Запросить цвет из CMYK - палитры                       // 13/10/06
  // index - порядковый номер цвета в палитре (с 1)
  // hRsc - идентификатор классификатора карты
  // возвращает цвет из 4 составляющих (С,M,Y,K)
  // (каждая в интервале от 0 до 255)

_MAPIMP unsigned long _MAPAPI mapGetRscCMYKColor(HRSC hRsc,long int index);

  // Установить цвет в CMYK - палитру                      // 13/10/06
  // color - 4 составляющих (С,M,Y,K)
  // (каждая в интервале от 0 до 255)
  // index - порядковый номер цвета в палитре (с 1)
  // hRsc - идентификатор классификатора карты
  // возвращает единицу
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapSetRscCMYKColor(HRSC hRsc,unsigned long int color,
                                             long int index);


  /************************************************************
  *                                                           *
  *  ФУНКЦИИ РАБОТЫ С ШРИФТАМИ КЛАССИФИКАТОРОВ  //04/10/01    *
  *                                                           *
  ************************************************************/

  // Kод шрифта - проставляется в параметры объекта (IMGTEXT: поле Type)
  // (см. mapgdi.h) для ссылки на таблицу шрифтов.

  // Запрос количества шрифтов
  // При ошибке возвращает ноль
  // hRsc - идентификатор классификатора карты

_MAPIMP  long int _MAPAPI mapGetFontCount(HRSC hRsc);


  // Добавить шрифт - возвращает код шрифта
  // hRsc - идентификатор классификатора карты
  // RSCFONT - см. maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapAppendFont(HRSC hRsc,RSCFONT * font);


  // Заменить шрифт - возвращает код шрифта
  // hRsc - идентификатор классификатора карты
  // index - порядковый номер шрифта (с 1)
  // RSCFONT - см. maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapReplaceFont(HRSC hRsc,int index,RSCFONT * font);


  // Запрос кода шрифта по порядковому номеру(c 1)
  // hRsc - идентификатор классификатора карты
  // number - номер шрифта
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetFontCode(HRSC hRsc,int number);


  // Запрос шрифта по порядковому номеру (c 1)
  // Возвращает код шрифта
  // hRsc - идентификатор классификатора карты
  // number - номер шрифта
  // RSCFONT - см. maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetFont(HRSC hRsc,int number,RSCFONT * font);


  // Запрос шрифта по коду
  // Возвращает порядковый номер шрифта (с 1)
  // hRsc - идентификатор классификатора карты
  // code - код шрифта
  // RSCFONT - см. maptype.h
  // При ошибке возвращает ноль

_MAPIMP  long int _MAPAPI mapGetFontByCode(HRSC hRsc,int code,RSCFONT * font);

  // Дополнение параметров шрифта                              // 16/12/15
  // hRsc - идентификатор классификатора карты
  // code - код шрифта
  // Возвращает порядковый номер шрифта в таблице шрифтов
  // При ошибке возвращает ноль
_MAPIMP  long int _MAPAPI mapSetFontByCode(HRSC hRsc,int code);


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  // 19/07/06
// +++++     ОПИСАНИЕ ФУНКЦИЙ ПРОВЕРКИ И ВЫЧИСЛЕНИЯ ФОРМУЛ           ++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Проверка символьной формулы и вычисление ее значений
  // допустимые операции: +,  -,  *,  /,
  // (разделитель между операндами запятая ,)
  // MAX(... ,...,   ),MIN(... ,...,   )- максимум, минимум
  // ARM(... ,...,   ) - среднее арифметическое
  // (разделитель между операндами у MAX, MIN, ARM - запятая ,)
  // SIN,COS,TG,CTG - тригонометрические функции от углов в градусах и долях градуса
  // SQRT2(),SQRT3(),POW2(),POW3() - корень и возведение во 2,3 степень
  // PI - число PI
  // P - Периметр объекта,
  // S - Площадь объекта
  // # - Указывает, что дальше идет код семантики, в которой лежит значение,
  //     далее в ()значение по умолчанию.
  // (пример: #1(0) - взять значение семантики 1, при отсутствии взять 0)
  // цифры 0- 9,(разделитель точка .)
  // Скобки (),[],{}
  // Возвращает 0 при ошибке, и в переменной errcode - код ошибки


  // Вычисление формулы
  // HOBJ info - идентификатор объекта карты
  // formula - строка, содержащая формулу
  // value - значение формулы
  // код ошибки в errcode (maperror.rh)
  // Возвращает 0 при ошибке, и в переменной errno - код ошибки

_MAPIMP long int _MAPAPI GetFormulaValue(HOBJ info, const char * formula,
                                double * value,  long * errcode);

  // Проверка синтаксиса формулы
  // HRSC rsc - идентификатор классификатора карты
  // formula - строка, содержащая формулу
  // Возвращает 0 при ошибке, и в переменной errno - код ошибки

_MAPIMP long int _MAPAPI CheckFormula(HRSC rsc, const char * formula, long * errcode);

  // Создание символьной строки с вставками по значениям семантики
  // На входе символьная строка любого содержания source, в которой указаны места
  // для вставки семантики следующим образом
  // # - Указывает, что дальше идет код семантики, в которой лежит значение
  //     любого типа
  //     далее в ()значение по умолчанию.
  // (пример: #9(без названия) - взять значение семантики 9, при отсутствии
  // вставить строчку "без названия"
  // dest - указатель на буфер для размещения строки
  //        размер буфера - размер строки + 256*количество вставок
  // Возвращает 0 при ошибке, и в переменной errcode - код ошибки

_MAPIMP long int _MAPAPI AgregateString(HRSC hrsc, HOBJ info, const char * source,     // 15/05/09
                              char * dest,long int destlength,long int * errcode);
_MAPIMP long int _MAPAPI AgregateStringUn(HRSC hrsc, HOBJ info, const WCHAR * source,     // 18/12/12
                              WCHAR * dest,long int destlength,long int * errcode);
  // Запросить количество формул в классификаторе
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscFormuleCount(HRSC hrsc);

  // Запросить код формулы в классификаторе по порядковому номеру с 1
  // hRsc   - идентификатор классификатора карты
  // number - порядковый номер формулы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscFormuleCodeByNumber(HRSC hrsc, long int number);

  // Заполнить структуру описания формулы                           // 07/03/13
  // по коду формулы
  // hRsc - идентификатор классификатора карты
  // code - код формулы
  // структура RSCFORMULE описана в maptype.h
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapGetRscDescribeFormuleByCode(HRSC hrsc, RSCFORMULE * form,
                                                        long int code);

  // Заменить текст формулы по коду формулы
  // проверка на синтактическую правильность не делается
  // bufflength - длина формулы в байтах
  // buff - текст формулы
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapReplaceRscFormulaText(HRSC hrsc, long int code,
                                       long int bufflength, WCHAR * buff);

  // Назначить семантике формулу для расчетов
  // Возвращает код формулы или ноль

_MAPIMP long int _MAPAPI mapSetRscFormulaToSemantic(HRSC hrsc,long int code,
                                                       long int semanticcode);

  // Запросить текст формулы по коду формулы
  // hRsc - идентификатор классификатора карты
  // code - код формулы
  // При ошибке возвращает ноль

_MAPIMP const wchar_t * _MAPAPI mapGetRscTextFormuleByCode(HRSC hrsc, long int code);

  // Записать новую формулу - возвращает код формулы
  // form - описание формулы
  // text - текст формулы
  // errcode - код ошибки при проверке формулы (maperr.rh)
  // hRsc - идентификатор классификатора карты
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapSetRscFormula(HRSC hrsc, RSCFORMULE * form, WCHAR * text,
                                          long * errcode);

  // Запросить количество библиотек формул для семантики       14/11/14
  // hRsc - идентификатор классификатора карты

_MAPIMP long int _MAPAPI mapCountRscFormulaLibraries(HRSC hrsc);

  // Добавить библиотеку в список для вычисления значений семантики
  // hRsc - идентификатор классификатора карты
  // name - имя библиотеки
  // message - 0 не выдавать сообщений
  // Возвращает номер библиотеки в списке (1,...)
  // При ошибке возвращает ноль

_MAPIMP long int _MAPAPI mapOpenRscFormulaLibrary(HRSC hrsc, const WCHAR * name,
                                                  long int message);
  // Запросить описание формул для семантики по номеру библиотеки в списке (1,...)
  // hRsc - идентификатор классификатора карты
  // number - библиотеки в списке
  // При ошибке возвращает ноль

_MAPIMP SEMLIBLIST * _MAPAPI mapGetRscFormulaLibraryFunction(HRSC hrsc,
                                                             long int number);

  // Запросить по номеру библиотеки в списке (1,...)
  // имя библиотеки
  // При ошибке возвращает ноль

_MAPIMP WCHAR * _MAPAPI mapGetRscFormulaLibraryName(HRSC hrsc, int number);


} // extern "C"


#endif

