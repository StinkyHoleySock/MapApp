/****** MAPOLD.H *************** Belenkov O.V.   ****** 03/04/02 ***
******* MAPOLD.H *************** Gorbunov A.V.   ****** 21/01/02 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2002              *
*                      All Rights Reserved                         *
*                                                                  *
********************************************************************
*                                                                  *
*           УСТАРЕВШИЕ ОПИСАНИЯ БАЗОВЫХ ТИПОВ ПЕРЕМЕННЫХ           *
*                                                                  *
*******************************************************************/

#ifndef MAPOLD_H
#define MAPOLD_H


typedef struct MTRBUILDPARM  // ПАРАМЕТРЫ СОЗДАНИЯ МАТРИЦЫ
{
  double BeginX;         // Прямоугольные координаты начала
  double BeginY;         // (юго-западного угла) матрицы  в метрах

  double Width;          // Ширина матрицы в метрах
  double Height;         // Высота матрицы в метрах

  double ElemSizeMeters; // Размер стороны элементарного участка
                         // в метрах на местности
  long int ReliefType;   // Тип матрицы (0 - абсолютные высоты,
                         //              1 - абсолютные + относительные)
  long int Filter;       // Признак использования файла MTRCREA.TXT
                         // (0 - не использовать, 1 - использовать)
  long int UserType;     // Произвольное число, связываемое
                         // с создаваемой матрицей
}
  MTRBUILDPARM;

typedef struct MTRBUILDPARMEX  //  ПАРАМЕТРЫ СОЗДАНИЯ МАТРИЦЫ ВЫСОТ
{
  unsigned
  long int StructSize;     // Размер данной структуры :
                           //  sizeof (MTRBUILDPARMEX)

  double   BeginX;         // Прямоугольные координаты начала
  double   BeginY;         // (юго-западного угла) матрицы в метрах

  double   Width;          // Ширина матрицы в метрах
  double   Height;         // Высота матрицы в метрах

  double   ElemSizeMeters; // Размер стороны элементарного участка
                           // в метрах на местности

  long int ElemSizeBytes;  // Pазмер элемента матрицы в байтах
                           // (допустимые значения : 1,2,4,8)

  long int Unit;           // Eдиница измерения высоты
                           // (0 - метры, 1 - дециметры,
                           //  2 - сантиметры, 3 - миллиметры )

  long int ReliefType;     // Тип матрицы (0 - абсолютные высоты,
                           //    1 - абсолютные + относительные)

  long int UserType;       // Произвольное число, связываемое
                           // с создаваемой матрицей

  long int Scale;          // Знаменатель масштаба создаваемой
                           // матричной карты (при создании матрицы
                           // по векторной карте значение данного
                           // поля будет заменено на знаменатель
                           // масштаба векторной карты)

  long int HeightSuper;    // Высота при попадании в элемент
                           // более одного объекта с абсолютной
                           // высотой (0 - средняя, 1 - максимальная)

  long int FastBuilding;   // Быстрое создание матрицы
                           // (0 - обычное, 1 - быстрое)

  char     Reserve[8];     // Должны быть нули
}
  MTRBUILDPARMEX;


typedef struct MTRCOLORDESC  // ОПИСАНИЕ ДИАПАЗОНА ВЫСОТ
                             // МАТРИЦЫ         26/09/00
{
  COLORREF Color;            // Цвет,соответствующий диапазону высот
  double MinHeight;          // Минимальная высота дипазона
  double MaxHeight;          // Максимальная высота дипазона
}
  MTRCOLORDESC;

typedef struct SEMANTICTYPE  // ОПИСАНИЕ СЕМАНТИЧЕСКОЙ
                             // ХАРАКТЕРИСТИКИ ОБ'ЕКТА
{
  long int Code;       // внешний код
  long int Type;       // тип значения
  long int Reply;      // разрешается повторение (1)
  long int Enable;     // допустимая(1),обязательная(2)
                       // (при создании семантики поле не учитывается)
  long int Service;    // служебная,допустима для всех объектов (1)
  char     Name[32];   // название характеристики
  char     Unit[8];    // единица измерения
  double   Minimum;    // минимальное значение
  double   Default;    // умалчиваемое значение
  double   Maximum;    // максимальное значение
}
  SEMANTICTYPE;

typedef SEMANTICTYPE RSCSEMANTIC;


typedef struct SEMANTICTYPEEX  // РАСШИРЕННОЕ ОПИСАНИЕ СЕМАНТИЧЕСКОЙ    //01/03/00
                               // ХАРАКТЕРИСТИКИ ОБ'ЕКТА
{
  long int Code;         // внешний код
  long int Type;         // тип значения
  long int Reply;        // разрешается повторение (1)
  long int Enable;       // допустимая(1),обязательная(2)
                         // (при создании семантики поле не учитывается)
  long int Service;      // служебная,допустима для всех объектов (1)
  char     Name[32];     // название характеристики
  char     Unit[8];      // единица измерения
  double   Minimum;      // минимальное значение
  double   Default;      // умалчиваемое значение
  double   Maximum;      // максимальное значение
  long int Size;         // общий размер поля значения семантики
  long int Decimal;      // точность поля значения семантики
  char     ShortName[16];// короткое имя семантики (связь с полем БД)
  long int Reserv;       // резерв
}
  SEMANTICTYPEEX;

typedef struct MAPADJACENTLIST
{
  char      ListName[25];           // имя листа
  long int  Key;                    // номер объекта
  long int  Excode;                 // классификационный код объекта
  long int  First;                  // первая точка участка
  long int  Last;                   // последняя точка участка
}
  MAPADJACENTLIST;


#endif
