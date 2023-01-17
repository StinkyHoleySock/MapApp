/******  MAPMACRO.H  ***********  Belenkov O.V. ******  26/02/16 ***
*******  MAPMACRO.H  ***********  Dishlenko S.G.******  24/10/13 ***
*******  MAPMACRO.H  ***********  Shabakov D.A. ******  28/08/13 ***
*******  MAPMACRO.H  ***********  Борзов А.Ю.   ******  03/02/14 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*                  ОПИСАНИЕ МАКРОРАСШИРЕНИЙ                        *
*                                                                  *
*******************************************************************/

#ifndef MAPMACRO_H
#define MAPMACRO_H

#ifdef BUILD_TIM
  #define BUILD_DLL
#endif

#ifndef MAPSYST_H
  #include "mapsyst.h"
#endif

//#define HIDENEW // Для отладки
#define HUGER  // Устаревшее определение

#ifdef WIN32API
   #define HMESSAGE HWND
#else
   #define HMESSAGE MSGHANDLER
#endif

// Идентификаторы сообщений процессов обработки карты
#define IMM_OBJECT 0x585
#define IMM_LIST   0x586
#define IMM_ERROR  0x587

#ifndef M_PI
  #define M_PI        3.14159265358979323846
  #define M_PI_2      1.57079632679489661923
  #define M_PI_4      0.785398163397448309616
#endif

#define MPI M_PI

#ifndef WM_TOUCH                     // 04/04/12
  #define WM_TOUCH 0x0240
#endif

#define MAX_PATH_LONG  1024

#ifndef Min_Body
  #define Min_Body

  // The following Min() and Max() are for compatibility
  template <class T> inline T Min(const T& a, const T& b)
    {
      return a < b ? a : b;
    }
#endif

#ifndef Max_Body
  #define Max_Body
  template <class T> inline T Max(const T& a, const T& b)
    {
      return a > b ? a : b;
    }
#endif

// Определяется экспорт/импорт классов и функций (_DLLCLASS)
#ifdef BUILD_DLL
  #ifdef BUILD_DLL64  // 28/08/13
    #ifdef WIN32API   // 24/10/13
      #define _DLLCLASS __declspec(dllexport)
      #define _MAPAPI   WINAPI
      #define _MAPIMP   __declspec(dllexport)
    #else
      #define _DLLCLASS
      #define _MAPAPI   WINAPI
      #define _MAPIMP
    #endif
  #else
    #ifdef __BORLANDC__ // Для Borland C++
      #define _DLLCLASS _export
      #define _MAPAPI   _export WINAPI
      #define _MAPIMP
    #else               // Для Visual C++
      #ifdef LINUXAPI
        #define _DLLCLASS
        #define _MAPAPI   WINAPI
        #define _MAPIMP
      #else
        #ifndef GISWEB
          #define _DLLCLASS __declspec(dllexport)
          #define _MAPAPI   WINAPI
          #define _MAPIMP   __declspec(dllexport)
        #else
          #define _DLLCLASS
          #define _MAPAPI   WINAPI
          #define _MAPIMP   __declspec(dllexport)
        #endif
      #endif // LINUXAPI
    #endif  //  __BORLANDC__
  #endif   // BUILD_DLL64         // 28/08/13

  #ifdef  HIDENEW

    #ifdef __BORLANDC__ // Для Borland C++

    // Применение для универсальной DLL new и delete - опасно !
    inline void * operator new (size_t value)
     {
       return (void *) ::AllocateTheMemory((int) value);
     }

    inline void * operator new[] (size_t value)
     {
       return (void *) ::AllocateTheMemory((int) value);
     }

    inline void operator delete (void * point)
     {
       if (point)
        ::FreeTheMemory((char *)point);
     }

    inline void operator delete[] (void * point)
     {
       if (point)
        ::FreeTheMemory((char *)point);
     }

    #endif  //  __BORLANDC__

  #endif // HIDENEW

#else // BUILD_DLL

  #ifdef __BORLANDC__ // Для Borland C++
   #ifdef _GISSERVER
      #define _DLLCLASS
      #define _MAPAPI   WINAPI
      #define _MAPIMP
   #else
    #ifdef BUILD_DLL64
      #define _DLLCLASS
      #define _MAPAPI   WINAPI
      #define _MAPIMP   __declspec( dllimport )
    #else
      #define _DLLCLASS __declspec( dllimport )
      #define _MAPAPI   WINAPI
      #define _MAPIMP   __declspec( dllimport )
    #endif
   #endif
  #else                     // Для Visual C++
   #if defined(LINUXAPI)
     #define _DLLCLASS
     #define _MAPAPI   WINAPI
     #define _MAPIMP
   #else
     #define _DLLCLASS __declspec( dllimport )
     #define _MAPAPI   WINAPI
     #define _MAPIMP   __declspec(dllimport)
   #endif
  #endif
#endif // BUILD_DLL

#ifndef BUILD_DLL
#ifdef BUILD_MEM

#ifndef MACRONEW
#define MACRONEW

extern "C"
{
    char * _MAPAPI mapAllocate(unsigned long int value);
    void   _MAPAPI mapFree(char * memory);
}

    inline void * operator new[] (size_t value)
    {
      return (void *)::mapAllocate((unsigned long int) value);
    }

    inline void operator delete[] (void * point)
    {
      ::mapFree((char *)point);
    }

    inline void * operator new (size_t value)
    {
      return (void *)::mapAllocate((unsigned long int) value);
    }

    inline void operator delete (void * point)
    {
      ::mapFree((char *)point);
    }
#endif

#endif // BUILD_MEM
#endif // !BUILD_DLL

#endif // MAPMACRO_H
