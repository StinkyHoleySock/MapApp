/******  MAPSYST.H  ************  Belenkov O.V.    ***  20/07/16 ***
*******  MAPSYST.H  ************  Dishlenko S.G.   ***  26/04/16 ***
*******  MAPSYST.H  ************  Derevyagina Zh.A.***  27/04/16 ***
*******  MAPSYST.H  ************  Борзов А.Ю.      ***  17/07/15 ***
*******  MAPSYST.H  ************  Shabakov D.A.    ***  19/06/14 ***
*******  MAPSYST.H  ************  Kruzhkov A.E.    ***  25/03/16 ***
*******  MAPSYST.H  ************  Tarakanova I.R.  ***  08/04/14 ***
*******  MAPSYST.H  ************  Vitsko D.A.      ***  08/09/14 ***
*******  MAPSYST.H  ************  Savelov A.S.     ***  06/09/16 ***
*******  MAPSYST.H  ************  Zhelezniakov V.A.***  25/05/16 ***
*******  MAPSYST.H  ************  Korjilov A.      ***  04/02/16 ***
*******  MAPSYST.H  ************  Letov V.I.       ***  01/08/16 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*      FOR WINDOWS, WINDOWS CE, LINUX, QNX, ОС-РВ, & ...           *
*                                                                  *
********************************************************************
*                                                                  *
*                ОПИСАНИЕ СИСТЕМНЫХ ФУНКЦИЙ                        *
*                                                                  *
*******************************************************************/
 
#if !defined(MAPSYST_H)
#define MAPSYST_H
 
//#define WIN32API        // Тип системного интерфейса Win32Api
#define NAME_CODEC_UTF16 ("UTF-16LE")    // 25/01/16 Korjilov
#define  LINUXAPI     // Тип системного интерфейса Linux
//#define NUMBERTURN    // Процессор типа SPARC/MIPS (разворот байт)
//#define QNX                            // 25/01/16 Korjilov
//#define SPARC_SOLARIS                  // 25/01/16 Korjilov

//#define LINUXSERVER   // Тип сборки для Linux-сервера
                        // (увеличение размеров области памяти
                        //  для отображения карты)

//#define TRACE_LOCK      // ОТЛАДКА

#ifdef WIN32API                             // 18/02/13
typedef __int64 BIGINT; // Большой int
#endif
 
// Для MINIDATA ...
//typedef unsigned long int BIGINT; // Большой int

#ifdef WIN32API

#if defined(_M_X64) || defined(BUILD_DLL64)       // 30/11/13
  #pragma option -a8
#endif

 #if !defined(STRICT)
    #define STRICT
 #endif
 #include <windows.h>
 #include <stdlib.h>
 #include <stdio.h>
 typedef long int XYTYPE;
#endif

#ifdef SPARC_SOLARIS           // 25/01/16 Korjilov
  #define INADDR_NONE -1
  #define FNM_CASEFOLD (1 << 4)
  #define  NAME_CODEC_UTF16 ("UTF-16BE")
  #define  HIDEX11
  #define  HIDE3D
  #define  HIDERSC3D
  #define  HIDE_DOMAIN_AUTH
  #define  HIDETRY
  #define  sinl(x)     sin(x)
  #define  cosl(x)     cos(x)
#endif
#ifdef QNX                     // 25/01/16 Korjilov
  #define  POWERPC_QNX
  #undef   NAME_CODEC_UTF16
  #define  NAME_CODEC_UTF16 ("UTF-16BE")
  #define  FILESYSTEMUTF8
  #define  HIDEX11
  #define  HIDE3D
  #define  HIDERSC3D
  #define  HIDE_DOMAIN_AUTH
  #define  HIDETRY
  #define  CBAUD 15
  #define  CRTSCTS 020000000000
  #define  sinl(x)     sin(x)
  #define  cosl(x)     cos(x)
#endif

#ifdef TINYDATA
  #define HIDEMTL
  #define HIDEMTD       // 03/12/07
  #define HIDETIN       // 31/03/05
  #define HIDEMTQ
  #define HIDEMETAFILE
  #define HIDEMTR3D
  #define MINIDATA      // IDSHORT2,IDDOUBLE2,IDDOUBLE3
  #define HIDESET
  #define HIDEUNION
  #define HIDELOG
  #define HIDERSC3D     // 14/06/06
#endif

#ifdef WINCE
  #define _TEXT _T
  #define itoa(value, str, dec) sprintf(str, "%i", value)
  #define HIDESECTION   // 06/07/06
#endif

#ifndef MAPTRACE_H
  #include "maptrace.h"
#endif

#ifdef WIN32API
   #define HMESSAGE HWND
#else
   #define HMESSAGE MSGHANDLER
#endif

#ifdef LINUXAPI

 #ifdef LINUXSERVER     // 02/04/09
   #define HIDEMESS
 #endif

 #define QMF_BYCOMMAND 1 // 14/01/15 Korjilov
 #define QMF_CHECKED   2 // Константы для работы контекстного меню
 #define MAX_COMPUTERNAME_LENGTH 31 // 13/02/15 Kojilov

#ifdef NUMBERTURN
  #ifdef LINUXSERVER    // 02/04/09
    #define HIDERSC45
  #endif
#endif
  #define RISCCPU       // Выравнивание переменных и структур
//  #define HIDEMTL     // 31/03/15
//  #define HIDEMTD
//  #define HIDETIN
//  #define HIDEMTQ
  #define HIDEMETAFILE
  #define HIDEMTR3D
//  #define HIDE3D      
  #define HIDEIMAGEDIT 
//  #define HIDEUNION  
  #define WINAPI
  #define TURNRASTERTEXT
  #ifdef LINUXSERVER    // 02/04/09
    #define HIDEX11    // Скрыть функции Xwindow
    #define HIDEGISSERV
    #define MULTIPLATFORM  // Одновременная работа с данными на платформах SPARC и INTEL
    #define TRANSTELECOM
  #else
//  #define HIDEGISSERV   // 15/08/07
  #define HIDEX11        // Скрыть функции Xwindow
//  #define MULTIPLATFORM  // 10/08/07 // Одновременная работа с данными на платформах SPARC и INTEL
//  #define TRANSTELECOM   // 10/08/07
  #endif
#ifdef HIDEX11
  #define HIDESCREENIMAGE
#endif
  #define CROSS_MIPS   // 20/09/05 // Поддержка кросскомпилятора под процессор MIPS
  #define HIDEPAINTSTEP  // 17/04/06 // Игнорировать шаг вывода DIB в окно
//  #define HIDESECTION
//  #define ARM15BIT       // 19/06/08    Процессор типа ARM (глубина экрана 15 бит-на-пиксел)
  #define BUILD_DLL64      // 25/03/12
//  #define HIDEWMS          // 22/06/12
  #define HIDEWFS          // 11/04/13
  #define FILESYSTEMUTF8   // 16/06/14 // Локализация файловой системы UTF8 (по умолчанию KOI8-R)  // 14/01/15 Korjilov
  #define QT_DRAW          // 24/10/14 // Подключение функций внешнего рисования в DIB (Qt)

#ifdef ANDROID             // 06/09/16 Савелов
  #undef BUILD_DLL64
  #undef FILESYSTEMUTF8
#endif

  #include <sys/types.h>
  #include <sys/stat.h>
  #include <stdio.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <time.h>
  #include <stdlib.h>
  #include <ctype.h>
  #include <errno.h>
  #include <string.h>  
  #include <dlfcn.h>
  #include <sys/vfs.h> 

#if !defined(WATCOM)
  #include <sys/vfs.h>
#endif  
#ifndef HIDESECTION                           // 28/09/06
  #include <pthread.h>
#endif

#ifndef ANDROID
  #include <iconv.h>                          // 06/12/06
#else

// Функции находятся в файле mapiconv.cpp
extern "C"
{
// Тип метода конвертирования (тип - long int*)
// Идентификатор метода устанавливается в iconv_open
typedef void *iconv_t;

#define iconv_open  mapiconv_open
#define iconv       mapiconv
#define iconv_close mapiconv_close

// Установить значение идентификатора метода конвертирования
// Доступно 4 метода ковертирования:
// 1.UTF16  -> KOI8R  (tocode = "KOI8R", fromcode = "UTF16")
// 2.UTF16  -> CP1251 (tocode = "CP1251", fromcode = "UTF16")
// 3.KOI8R  -> UTF16  (tocode = "UTF16", fromcode = "KOI8R")
// 4.CP1251 -> UTF16  (tocode = "UTF16", fromcode = "CP1251")
// При ошибке возвращает - -1
iconv_t mapiconv_open (const char *tocode, const char *fromcode);

// Конвертировать строку
// __cd  - идентификатора метода конвертирования
// inbuf - входная строка символов
// inbytesleft - ее размер
// outbuf - строка результата конвертирования
// outbytesleft - ее размер
// При ошибке возвращает - -1
long int mapiconv (iconv_t __cd, char **inbuf, size_t * inbytesleft,
                   char ** outbuf, size_t *outbytesleft);

// Освободить ресурсы идентификатора
// При ошибке возвращает - -1
int mapiconv_close (iconv_t __cd);
}

#endif  // !ANDROID

#ifndef HIDEGISSERV                           // 08/05/08
#include <sys/socket.h> 
#include <netinet/in.h>
#include <netinet/tcp.h>                      // 19/06/08
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/time.h>                         // 20/05/08
#endif

#ifdef BUILD_DLL64                            // 25/03/12
#pragma pack(push,1)
#endif

  typedef unsigned char BYTE;
  typedef struct PALETTEENTRY
  {
    BYTE peRed;
    BYTE peGreen;
    BYTE peBlue;
    BYTE peFlags;
  }
  PALETTEENTRY;
  typedef struct tagRGBTRIPLE                // 08/07/15
  {
    BYTE rgbtBlue;
    BYTE rgbtGreen;
    BYTE rgbtRed;
  }  
  RGBTRIPLE;

  typedef unsigned long int HANDLE; // 25/03/12
  typedef unsigned int UINT;
  typedef unsigned int COLORREF;
  typedef unsigned short int WORD;  
  typedef unsigned int DWORD;
  typedef int LONG;                // 25/03/12
  typedef HANDLE HBITMAP;
  typedef HANDLE HGLOBAL;
  typedef HANDLE HPALETTE;
  typedef HANDLE HDC;
  typedef HANDLE HPEN;
  typedef HANDLE HMODULE;
  typedef HANDLE HWIDGET;        // 14/01/15 Korjilov
  typedef int XYTYPE;            // 07/06/05
  typedef long int HWND;
  typedef long int HINSTANCE;
  typedef long int LPARAM;
  typedef long int WPARAM;            // 31/01/02
  typedef long int LRESULT;
  typedef long int HMENU;             // 05/03/02
  typedef void *   LPVOID;            // 10/12/04
  typedef char *   LPTSTR;
#ifdef BUILD_DLL                      // 18/02/13
  // С BUILD_DLL транслируются только библиотеки ядра
  // вместе с опцией компилятору -fshort-wchar, что
  // означает размер wchar_t равен 2 байтам
  typedef wchar_t  WCHAR;
#else
  typedef WORD     WCHAR;
#endif
#ifndef TCHAR
  typedef char TCHAR;
  #define _TEXT(x)   x                // 08/11/00
  #define TEXT       _TEXT            // 25/03/12
  #define _tcsncpy   strncpy          // 05/03/14
  #define _tcsncat   strncat          // 13/01/15 Korjilov
  #define _tcslen    strlen           // 13/01/15
#endif
  typedef int64_t __int64;            // 14/12/09
  typedef __int64 BIGINT;             // Большой int  // 18/02/13
  typedef unsigned int SOCKET;            // 14/12/09
  typedef long int (* MSGHANDLER)
    (long int hwnd, long int code, long int p1, long int p2, long int typemsg);  // Обработчик сообщений  // 04/09/15 Савелов
  #define MessageBoxA MessageBox
#if (!defined(WATCOM) && !defined(OC2000)) // 04/05/01
  #define strnicmp   strncasecmp
  #define stricmp    strcasecmp
  #define strcmpi    strcasecmp      // 25/09/00
  #define _tcscpy    strcpy          // 18/04/13
  #define _tcscat    strcat
#if ( defined(CROSS_MIPS) && defined(NUMBERTURN) || defined(ANDROID) )   // 20/09/05 // 07/10/05
  #define sinl sin
  #define cosl cos
#endif
#else
  #define strnicmp   strncmp
  #define stricmp    strcmp
  #define strcmpi    strcmp      // 25/09/00
  #define sinl       sin
  #define cosl       cos
#endif

#ifndef FALSE                        // 30/11/00
  #define FALSE      0
  #define TRUE       1
#endif

  #define _wcscmpi wcscmpi           // 28/09/15
  #define _stprintf sprintf          // 28/09/15
  #define _fastcall                  // 22/06/12
  #define PASCAL __stdcall
  #define WINAPI
  #define __stdcall 
  #define far        // 13/09/00
  #define _import    // 08/12/00
  #define near       // 13/09/00
  #define _export    // 13/09/00
  #define __cdecl    // 13/03/13
  #define RGB(r,g,b) ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16))) // 13/09/00
  #define PALETTERGB(r,g,b)   (0x02000000 | RGB(r,g,b))  // 08/12/00
  #define PALETTEINDEX(i)     ((COLORREF)(0x01000000 | (DWORD)(WORD)(i)))
  #define GetRValue(rgb)   ((BYTE) (rgb))                 // 26/09/00
  #define GetGValue(rgb)   ((BYTE) (((WORD) (rgb)) >> 8)) // 26/09/00
  #define GetBValue(rgb)   ((BYTE) ((rgb) >> 16))         // 26/09/00

  #define ERROR_ALREADY_EXISTS 183L
  #define ERROR_FILE_NOT_FOUND 2L
  #define LF_FACESIZE 32

  #define FILE_SHARE_READ (S_IRUSR|S_IRGRP|S_IROTH)           // 22/06/12
  #define FILE_SHARE_WRITE (S_IWUSR|S_IWGRP|S_IWOTH)
//  #define GENERIC_READ O_RDONLY
  #define GENERIC_READ  0x100000              // 10/03/07
//  #define GENERIC_WRITE O_RDWR
  #define GENERIC_WRITE O_WRONLY              // 10/03/07
  #define OPEN_ALWAYS O_CREAT
  #define CREATE_ALWAYS (O_CREAT|O_TRUNC)
  #define OPEN_EXISTING O_EXCL
  #define CREAT_NEW (O_CREAT|O_EXCL)
  #define TRUNCATE_EXISTING (O_CREAT|O_EXCL|O_TRUNC)
  #define FILE_BEGIN SEEK_SET
  #define FILE_CURRENT SEEK_CUR
  #define FILE_END SEEK_END
  #define MAX_PATH 260                     // 18/03/02
  #define FILE_ATTRIBUTE_READONLY (S_IRUSR|S_IRGRP|S_IROTH)
  #define FILE_ATTRIBUTE_ARCHIVE (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)
  #define FILE_ATTRIBUTE_HIDDEN (S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR)
  #define FILE_ATTRIBUTE_NORMAL (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)
  #define FILE_ATTRIBUTE_OFFLINE (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)
  #define FILE_ATTRIBUTE_SYSTEM (S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR)
  #define FILE_ATTRIBUTE_TEMPORARY (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)

  #define OFN_FILEMUSTEXIST       0x00001000        //19/03/02
  #define OFN_PATHMUSTEXIST       0x00000800
  #define OFN_CREATEPROMPT        0x00002000
  #define OFN_OVERWRITEPROMPT     0x00000002
  #define OFN_ALLOWMULTISELECT    0x00000200        // 12/04/02
  #define OFN_HIDEREADONLY        0x00000004        // 19/05/02
  #define OFN_NOREADONLYRETURN    0x00008000
  // Маски клавиш (полное соотвествие WIN32)        // 14/05/02
  #define MK_LBUTTON                  0x0001
  #define MK_RBUTTON                  0x0002
  #define MK_SHIFT                    0x0004
  #define MK_CONTROL                  0x0008
  #define MK_MBUTTON                  0x0010
  // Коды клавиш (XWindow)
  #define VK_F1                       0xffbe            // 18/04/13
  #define VK_F2                       0xffbf
  #define VK_F3                       0xffc0
  #define VK_F4                       0xffc1
  #define VK_F5                       0xffc2
  #define VK_F6                       0xffc3
  #define VK_F7                       0xffc4
  #define VK_F8                       0xffc5
  #define VK_F9                       0xffc6
  #define VK_F10                      0xFFC7             // XK_F10
  #define VK_ESCAPE                   0xFF1B             // XK_Escape
  #define VK_TAB                      0xFF09             // XK_Tab
  #define VK_HOME                     0xFF50             // XK_Home
  #define VK_PRIOR                    0xFF55             // XK_Prior
  #define VK_NEXT                     0xFF56             // XK_Next
  #define VK_END                      0xFF57             // XK_End
  #define VK_LEFT                     0xFF51             // XK_Left
  #define VK_UP                       0xFF52             // XK_Up
  #define VK_RIGHT                    0xFF53             // XK_Right
  #define VK_DOWN                     0xFF54             // XK_Down
  #define VK_SHIFT                    0xFFE1             // XK_Shift_L      // 14/05/02
  #define VK_CONTROL                  0xFFE3             // XK_Control_L
  #define VK_RETURN                   0xFF0D             // XK_Return     // 05/03/02
  #define VK_LBUTTON                  0xFEE0             // XK_Pointer_Left // 14/05/02
  #define VK_RBUTTON                  0xFEE1             // XK_Pointer_Right
  #define VK_SPACE                    0x020              //  XK_space      // 10/03/02
  #define VK_MENU                     0xFF67             //  XK_Menu
  #define VK_BACK                     0xFF08             // XK_BackSpace // back space, back char   //19/03/02
  #define VK_INSERT                   0xFF63             // XK_Insert      Insert, insert here    WIN32 Ins 45
  #define VK_ADD                      0xFFAB             // XK_KP_Add
  #define VK_DELETE                   0xFFFF             // XK_Delete  Delete, rubout WIN32 Del  0x2e
                                                         // XK_KP_Delete 0xFF9F не добавлена
  #define VK_SUBTRACT                 0xFFAD             // XK_KP_Subtract   WIN32 109

  #define MB_OK                       0x00000000L // 15/04/99
  #define MB_ICONHAND                 0x00000010L
  #define MB_OKCANCEL                 0x00000001L
  #define MB_YESNOCANCEL              0x00000003L
  #define MB_YESNO                    0x00000004L
  #define MB_ABORTRETRYIGNORE         0x00000002L
  #define MB_RETRYCANCEL              0x00000005L

  #define MB_ICONQUESTION             0x00000020L
  #define MB_ICONEXCLAMATION          0x00000030L
  #define MB_ICONWARNING              MB_ICONEXCLAMATION
  #define MB_ICONERROR                MB_ICONHAND
  #define MB_ICONASTERISK             0x00000040L             // 27/09/00
  #define MB_ICONINFORMATION          MB_ICONASTERISK
  #define MB_ICONSTOP                 MB_ICONHAND
  #define MB_TASKMODAL                0x00002000L

  #define MB_SETFOREGROUND            0

  #define IDOK                        1
  #define IDCANCEL                    2
  #define IDABORT                     3
  #define IDRETRY                     4
  #define IDIGNORE                    5
  #define IDYES                       6
  #define IDNO                        7
  #define IDCLOSE                     8
  #define IDHELP                      9

  #define DT_PLOTTER          0   // Vector plotter
  #define DT_RASDISPLAY       1   // Raster display
  #define DT_RASPRINTER       2   // Raster printer

  #define R2_COPYPEN          0x3 // GXcopy   - src  для XWindow
//  #define R2_NOT              0x6 // GXxor      //0xa // GXinvert - (NOT dst) для XWindow // 04/04/02
  #define R2_NOT              0xa // GXinvert - (NOT dst) для XWindow // 19/04/02
  #define R2_NOTXORPEN        0x9 // GXequiv    - (NOT src) XOR dst  для XWindow   //11/04/02
  #define R2_XORPEN           0x6 // GXxor    - src XOR dst для XWindow            // 21/03/02

  #define OEM_CHARSET         255
  #define RUSSIAN_CHARSET     204 // 12/04/01
  #define BI_RGB              0L

  #define TA_LEFT             0     //19/03/02
  #define TA_RIGHT            2
  #define TA_CENTER           6

  #define TA_TOP              0
  #define TA_BOTTOM           8
  #define TA_BASELINE         24

  // Стандартный вид курсора            // Коды XWindow:                // 21/04/04
  #define IDC_ARROW    (char *)68L      // XC_left_ptr
  #define IDC_ICON     IDC_ARROW
  #define IDC_IBEAM    (char *)152L     // XC_xterm
  #define IDC_WAIT     (char *)150L     // XC_watch
  #define IDC_CROSS    (char *)34L      // XC_crosshair
  #define IDC_UPARROW  (char *)22L      // XC_center_ptr
  #define IDC_SIZEALL  (char *)52L      // XC_fleur
  #define IDC_SIZE     IDC_SIZEALL
  #define IDC_SIZENWSE (char *)32L      // XC_cross_reverse
  #define IDC_SIZENESW IDC_SIZENWSE
  #define IDC_SIZEWE   (char *)108L     // XC_sb_h_double_arrow
  #define IDC_SIZENS   (char *)116L     // XC_sb_v_double_arrow

  #define INFINITE     600000               // 02/04/12
  #define MF_SEPARATOR 0                    // 12/04/12
  #define MF_BYCOMMAND 1                    // 30/10/15
  #define MF_CHECKED   2                    // 30/10/15
  #define MF_STRING    0
  #define _fastcall

typedef struct _FILETIME
{
   DWORD dwLowDateTime;
   DWORD dwHighDateTime;
}
   FILETIME;

typedef struct tagSIZE
{
   LONG cx;
   LONG cy;
}
   SIZE;

typedef struct tagLOGFONT
{
   LONG lfHeight;
   LONG lfWidth;
   LONG lfEscapement;
   LONG lfOrientation;
   LONG lfWeight;
   BYTE lfItalic;
   BYTE lfUnderline;
   BYTE lfStrikeOut;
   BYTE lfCharSet;
   BYTE lfOutPrecision;
   BYTE lfClipPrecision;
   BYTE lfQuality;
   BYTE lfPitchAndFamily;
   char lfFaceName[LF_FACESIZE];
}
   LOGFONT;

typedef struct tagLOGFONTW    // 21/09/15 Савелов
{
   LONG lfHeight;
   LONG lfWidth;
   LONG lfEscapement;
   LONG lfOrientation;
   LONG lfWeight;
   BYTE lfItalic;
   BYTE lfUnderline;
   BYTE lfStrikeOut;
   BYTE lfCharSet;
   BYTE lfOutPrecision;
   BYTE lfClipPrecision;
   BYTE lfQuality;
   BYTE lfPitchAndFamily;
   WCHAR lfFaceName[LF_FACESIZE];
}
   LOGFONTW;

typedef struct tagRGBQUAD
{
    BYTE rgbBlue;
    BYTE rgbGreen;
    BYTE rgbRed;
    BYTE rgbReserved;
}
    RGBQUAD;

typedef struct _RECT
{
    int left;                          // 25/03/12
    int top;
    int right;
    int bottom;
}
    RECT;

typedef struct tagBITMAPINFOHEADER
{
    DWORD biSize;
    LONG biWidth;
    LONG biHeight;
    WORD biPlanes;
    WORD biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG biXPelsPerMeter;
    LONG biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;
}
    BITMAPINFOHEADER;

typedef struct tagBITMAPINFO
{
    BITMAPINFOHEADER bmiHeader;
    RGBQUAD bmiColors[1];
}
    BITMAPINFO;

typedef struct tagPOINT
{
    union { int x; int X; };                 // 25/03/12
    union { int y; int Y; };
}
    POINT;

typedef struct _SYSTEMTIME
{
    WORD wYear;
    WORD wMonth;
    WORD wDayOfWeek;
    WORD wDay;
    WORD wHour;
    WORD wMinute;
    WORD wSecond;
    WORD wMilliseconds;
}
    SYSTEMTIME;

typedef struct tagMSG    // 31/01/02
{
    HWND hwnd;
    UINT message;
    WPARAM wParam;
    LPARAM lParam;
    DWORD time;
    POINT pt;
}
    MSG;

typedef struct TMessage    //12/05/03
{
    WPARAM    WParam;
    LPARAM    LParam;
    long int  Result;
}
    TMessage;

typedef struct MENUSTRINGTABLE    // 12/04/12
{
    int       IdItem;             // Идентификатор строки
    char      NameItem[MAX_PATH]; // Текст
    int       Enable;             // Доступность
    int       Check;
}
   MENUSTRINGTABLE;

// Прототипы переопределенных функций (XWindow.cpp)
char * _fastcall strlwr(char * string);
char * _fastcall itoa(int number, char * string,int index);
char * _fastcall ltoa(long number, char* string,int index);
void   _fastcall GetSystemTime(SYSTEMTIME * systemtime);
void   _fastcall GetLocalTime(SYSTEMTIME * systemtime);            // 16/10/15
int    _fastcall MessageBox(HWND hwnd, const TCHAR * message,
                            const TCHAR * title, int flag);
int    _fastcall MessageBoxW(HWND hwnd, const WCHAR * message,     // 11/04/13
                             const WCHAR * title, int flag);

// Запросить длину строки UNICODE (datbase.cpp)                    // 03/10/11
int    _fastcall strlenun(const char * text, int limit = 4096);    // 08/09/14

// Прототипы функций, определенных только для Linux (xwindow.cpp)  // 17/09/02

// Конвертировать индекс палитры в цвет вида COLORREF
// index - индекс палитры XWindow
// возвращает COLORREF
COLORREF _fastcall ConvertIndexPaletteToCOLORREF(COLORREF index);

// Конвертировать цвет вида COLORREF в индекс палитры
// color - цвет вида COLORREF
// возвращает индекс палитры
COLORREF _fastcall ConvertCOLORREFToIndexPalette(COLORREF color);

typedef int(WINAPI *FARPROC) ();

#ifdef OC2000
 #define HIDELOG               // 08/11/01
#endif

#endif     // LINUX

#ifdef WIN32API       // Windows GDI

#include <tchar.h>

#if defined(_M_X64) || defined(BUILD_DLL64)       // 20/07/09
typedef __int64 XDHANDLE;
#else
typedef unsigned long XDHANDLE;
#endif

typedef struct XDrawHandle        // 10/07/06
{
  XDHANDLE newhandle;
  XDHANDLE oldhandle;
}
  XDrawHandle;

typedef XDrawHandle  XHPEN;
typedef XDrawHandle  XHBRUSH;

#else

typedef long int   XHPEN;
typedef long int   XHBRUSH;

#endif


// MAPTRACER   - включение трассировки (TRUE/FALSE);
// TRACE_MEM   - трассировка работы с памятью;
// TRACE_MAP   - трассировка работы конструкторов,деструкторов,
//               доступа к району работ;
// TRACE_READ  - трассировка операций чтения;
// TRACE_WRITE - трассировка операций записи.

#if (MAPTRACER == TRUE)
#ifndef TRACE_MAP
 #define TRACE_MAP TRUE
#endif
#ifndef TRACE_MEM
 #define TRACE_MEM TRUE
#endif
#ifndef TRACE_READ
 #define TRACE_READ FALSE
#endif
#ifndef TRACE_WRITE
 #define TRACE_WRITE FALSE
#endif
#endif

#ifdef WINCE
  #undef  LINUXAPI
  #define HIDETRY
  #define HIDELOG
  #define MINIDATA
  #define HIDEDIB
  #define HIDECUT
  #define RISCCPU
  #define HIDEMTL
  #define HIDEMTD       // 03/12/07
  #define HIDETIN       // 31/03/05
  #define HIDEMTQ                     
  #define HIDEMTR3D
  #define HIDEMETAFILE

  #define stricmp    strcmp
  #define strcmpi    strcmp
  #define strnicmp   strncmp
  #define strncmpi   strncmp
  #define ltoa       _ltoa


  #define MB_TASKMODAL    0
  #define OEM_CHARSET     255
  #define FW_ULTRALIGHT   FW_EXTRALIGHT

  #define TA_LEFT         0
  #define TA_RIGHT        2
  #define TA_CENTER       6

  #define TA_TOP          0
  #define TA_BOTTOM       8
  #define TA_BASELINE    24

#else

#ifndef __BORLANDC__          // Кроме Borland C++
 #define strncmpi strnicmp
#endif

#endif    // WINCE

#ifdef LINUXAPI             // 05/03/14

#define GetCharWidthA GetCharWidth	// 03/02/15 Korjilov

#if (__SIZEOF_WCHAR_T__ == 2)           // 17/02/16
#define wcslen   wcslen_size2
#define wcscmp   wcscmp_size2
#define wcscmpi  wcscmpi_size2
#define wcsncmp  wcsncmp_size2
#define wcsncmpi wcsncmpi_size2
#define wcsstr   wcsstr_size2
#define wcscpy   wcscpy_size2
#define wcsncpy  wcsncpy_size2
#define wcscat   wcscat_size2
#define wcsncat  wcsncat_size2
#endif


 // Аналоги системных функций для широких символов, 
 // но wchar_t равно 2 байта
 // Только для внутреннего пользования!
 //------------------------------------------------------------------
 // Функция  wcslen  является  эквивалентом  функции  // 04/10/11
 // strlen для широких символов. Она определяет
 // длину строки широких символов, на которую указывает s,
 // не учитывая конечный символ L'\0'
 int wcslen(const WCHAR *s);

 // Функция _wtoi  является  эквивалентом  функции    // 04/10/11
 // atoi для широких символов. Она преобразует строку 
 // символов в целое число 
 int _wtoi(const WCHAR *s);

 // Функция _wtof  является  эквивалентом  функции    // 04/10/11
 // atof для широких символов. Она преобразует строку 
 // символов в число типа double
 double _wtof(const WCHAR *s);

 // Аналог функции _ltow для Windows                                         // 18/02/13
 // Возвращает в случае успеха - 1, иначе - 0
 int _ltow(long value, WCHAR *dest, int radix);

 // Аналог функции _itow для Windows                                         // 13/03/13
 // Возвращает в случае успеха - 1, иначе - 0
 int _itow(int value, WCHAR *dest, int radix);

 // Функция  wcscmp  является  эквивалентом           // 04/10/11
 // функции strcmp для широких символов
 // Возвращает ноль, если строки, на которые указывают s1 и s2, равны
 // Иначе 1
 int wcscmp(const WCHAR *s1, const WCHAR *s2);

 // Функция  wcscmpi  является  эквивалентом           // 05/09/14
 // функции strcmpi для широких символов (без учета регистра)
 // Возвращает ноль, если строки, на которые указывают s1 и s2, равны
 // Иначе 1
 int wcscmpi(const WCHAR *s1, const WCHAR *s2);

 // Функция  wcsncmp  является  эквивалентом           // 18/02/13
 // функции strncmp для широких символов
 // Сравнивает только первые  n  символов  строки s1 с s2
 // Возвращает ноль, если равны
 // Иначе 1
 int wcsncmp(const WCHAR *s1, const WCHAR *s2, int n);

 // Функция  wcsncmpi  является  эквивалентом           // 05/09/14
 // функции strncmpi для широких символов
 // Сравнивает только первые  n  символов  строки s1 с s2 (без учета регистра)
 // Возвращает ноль, если равны
 // Иначе 1
 int wcsncmpi(const WCHAR *s1, const WCHAR *s2, int n);

 // Функция wcsstr является эквивалентом
 // функции strstr для широких символов.
 // Она ищет положение первой строки широких символов s2
 // (без завершающего символа L'\0')
 // в строке широких символов s1.
 // Возвращает указатель на первое появление подстроки s2 в s1.
 WCHAR *wcsstr(const WCHAR *s1, const WCHAR *s2);

 // Функция  wcscpy  является  эквивалентом        // 03/10/11
 // функции strcpy для широких символов.
 // Копирует  строку,  указанную как src 
 // (включая завершающий символ '\0'), в
 // массив, указанный как dest.  Строки не могут 
 // перекрываться, и в целевой строке  dest  должно
 // быть достаточно места для получения копии.
 // Возвращает указатель на целевую строку dest
 WCHAR *wcscpy(WCHAR *dest, const WCHAR *src);

 // Функция  wcsncpy  является  эквивалентом        // 03/10/11
 // функции strncpy для широких символов
 // Копируются только первые n символов строки src.
 // Таким образом, если нулевой байт отсутствует
 // в первых n байтах src, то результирующая строка
 // не будет завершена символом L'\0'.
 // Возвращает указатель на целевую строку dest
 WCHAR *wcsncpy(WCHAR *dest, const WCHAR *src, int n);

 // Функция  wcscat  является  эквивалентом  strcat  для  широких  символов.   // 03/10/11
 // Она копирует строку широких символов, на которую указывает src, включая
 // конечный L'\0', в конец  строки  широких символов, на которую
 // указывает dest. Строки не должны перекрываться.
 // Программист   должен   убедиться  в  том,  что  в  dest  имеется
 // место, как минимум, для wcslen(dest)+wcslen(src)+1 широких символов.
 // Возвращает dest
 WCHAR *wcscat(WCHAR *dest, const WCHAR *src);

 // Функция  wcsncat  является  эквивалентом  strncat  для  широких  символов.   // 18/02/13
 // Добавляются только первые n символов строки src.
 // Таким образом, если нулевой байт отсутствует
 // в первых n байтах src, то результирующая строка
 // не будет завершена символом L'\0'. Строки не должны перекрываться.
 // Программист   должен   убедиться  в  том,  что  в  dest  имеется
 // место, как минимум, для wcslen(dest)+n+1 широких символов.
 // Возвращает dest
 WCHAR *wcsncat(WCHAR *dest, const WCHAR *src, int n);

 // Функция swprintf является эквивалентом sprintf для широких символов.	// 29/01/15 Korjilov
 // Поддерживаются все спецификации и модификаторы, кроме *.
 // Возвращает dest
 int swprintf(WCHAR *dest, const WCHAR *fmt, ...);

 // Функция _wfopen является эквивалентом fopen для широких символов.		// 29/01/15 Korjilov
 FILE *_wfopen(const WCHAR *p1, const  WCHAR *p2);

 // Функция fwprintf является эквивалентом fprintf для широких символов.	// 29/01/15 Korjilov
 int fwprintf(FILE *fp, const WCHAR *text);

#endif

// --------------------------------------------------------------
// РЕАЛИЗАЦИЯ СИСТЕМНЫХ ФУНКЦИЙ В БИБЛИОТЕКАХ
//  MAPACCES.DLL / GISACCES.DLL / GISACWEB.DLL
// --------------------------------------------------------------
// ПРИ ОШИБКЕ ВСЕ ФУНКЦИИ ВОЗВРАЩАЮТ НОЛЬ !
// ---------------------------------------

// ---------------------------------------
// Копирует char строку в char строку
// в отличие от strncpy в буфер всегда будет записан завершающий 0
// Если буфер меньше строки, то строка обрезается
// src      - адрес входной строки
// dest     - адрес буфера для размещения выходной строки
// destsize - размер выходной строки в байтах
// ---------------------------------------
inline char* StrCopy(char *dest, const char *src, int destsize)
{
  if (src == 0 || dest == 0 || destsize <= 0) return 0;
  dest[destsize - 1] = 0;
  return strncpy(dest, src, destsize - 1);
}

// ---------------------------------------
// Копирует TCHAR строку в TCHAR строку
// Если буфер меньше строки, то строка обрезается
// src      - адрес входной строки
// dest     - адрес буфера для размещения выходной строки
// destsize - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline TCHAR* TcsCopy(TCHAR *dest, const TCHAR *src, int destsize)
{
  if (src == 0 || dest == 0 || destsize <= 0) return 0;
  dest[destsize/sizeof(TCHAR) - 1] = 0;
  return _tcsncpy(dest, src, destsize/sizeof(TCHAR) - 1);
}

// ---------------------------------------
// Копирует UNICODE строку в UNICODE строку
// Если буфер меньше строки, то строка обрезается
// src      - адрес входной строки UTF-16
// dest     - адрес буфера для размещения выходной строки UTF-16
// destsize - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline WCHAR* WcsCopy(WCHAR *dest, const WCHAR *src, int destsize)
{
  // Если входная строка меньше 2-х байт, то даже некуда писать завершающий 0
  if (src == 0 || dest == 0 || destsize < 2) return 0;
  dest[destsize/2 - 1] = 0;
  return wcsncpy(dest, src, destsize/2 - 1);
}

// ---------------------------------------
// Присоединяет char строку src к сhar строке dest
// Если свободная часть буфера dest меньше строки src, то строка обрезается
// src      - адрес входной строки
// dest     - адрес буфера для размещения выходной строки
// destsize - размер выходной строки в байтах
// ---------------------------------------
inline char* StrCat(char *dest, const char *src, int destsize)  // 08/10/14
{
  if (src == 0 || dest == 0 || destsize <= 0) return 0;
  dest[destsize - 1] = 0;
  return strncat(dest, src, destsize - 1 - strlen(dest));
}

inline char* _StrCat(char *dest, const char *src, int destsize)  // 08/10/14
{
  if (src == 0 || dest == 0 || destsize <= 0) return 0;
  dest[destsize - 1] = 0;
  return strncat(dest, src, destsize - 1 - strlen(dest));
}

// ---------------------------------------
// Присоединяет TCHAR строку src к TCHAR строке dest
// Если свободная часть буфера dest меньше строки src, то строка обрезается
// src      - адрес входной строки
// dest     - адрес буфера для размещения выходной строки
// destsize - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline TCHAR* TcsCat(TCHAR *dest, const TCHAR *src, int destsize)  // 08/10/14
{
  if (src == 0 || dest == 0 || destsize <= 0) return 0;
  dest[destsize/sizeof(TCHAR) - 1] = 0;
  return _tcsncat(dest, src, destsize/sizeof(TCHAR) - 1 - _tcslen(dest));
}

// ---------------------------------------
// Присоединяет WCHAR строку src к WCHAR строке dest
// Если свободная часть буфера dest меньше строки src, то строка обрезается
// src      - адрес входной строки UTF-16
// dest     - адрес буфера для размещения выходной строки UTF-16
// destsize - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline WCHAR* WcsCat(WCHAR *dest, const WCHAR *src, int destsize)  // 08/10/14
{
  // Если входная строка меньше 2-х байт, то даже некуда писать завершающий 0
  if (src == 0 || dest == 0 || destsize < 2) return 0;
  dest[destsize/2 - 1] = 0;
  return wcsncat(dest, src, destsize/2 - 1 - wcslen(dest));
}

inline WCHAR* _WcsCat(WCHAR *dest, const WCHAR *src, int destsize)
{
  // Если входная строка меньше 2-х байт, то даже некуда писать завершающий 0
  if (src == 0 || dest == 0 || destsize < 2) return 0;
  dest[destsize/2 - 1] = 0;
  return wcsncat(dest, src, destsize/2 - 1 - wcslen(dest));
}

#ifdef SPARC_SOLARIS                                // 25/01/16 Korjilov
inline size_t iconv(iconv_t cd, char **inbuf,  size_t *inbytes,
                                char **outbuf, size_t *outbytes)
{
  return iconv(cd, (const char **) inbuf, inbytes, outbuf, outbytes);
}
#endif // SPARC_SOLARIS

// ---------------------------------------
// Преобразовать строку "char" в UNICODE
// src   - адрес входной строки ANSI
// dest  - адрес буфера для размещения выходной строки UTF-16
// bytes - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline int StringToUnicode(const char * src,
                           WCHAR * dest, int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API
#ifdef PANFR
  unsigned int code = 1252;        // ANSI Latin 1; Western European (Windows) 
#else
  unsigned int code = CP_ACP;      // The current system Windows ANSI code page. 
#endif
  int ret = MultiByteToWideChar(code,0,
                                src,-1,
                                dest,bytes/2);   // 07/04/11
  dest[bytes/2-1] = 0;
  return ret;
#else
#ifndef LINUXAPI
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
#ifndef _GISSERVER                            // 30/08/13
  const char *fromcode = "KOI8-R";
#else
  const char *fromcode = "CP1251";
#endif
  const char *tocode   = NAME_CODEC_UTF16;    // 25/01/16 
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)
	 ret = 0;
    else
 	ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif

}

// ---------------------------------------
// Преобразовать строку "char" в кодировке CP1251  // 26/04/16
// в UNICODE
// src   - адрес входной строки ANSI
// dest  - адрес буфера для размещения выходной строки UTF-16
// bytes - размер выходной строки в БАЙТАХ
// ---------------------------------------
inline int AnsiStringToUnicode(const char * src,
                               WCHAR * dest, int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API
  return StringToUnicode(src, dest, bytes);
#else
#ifndef LINUXAPI
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
  const char *fromcode = "CP1251";
  const char *tocode   = NAME_CODEC_UTF16;
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)
	 ret = 0;
    else
 	ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

// ---------------------------------------
// Преобразовать символы UNICODE в строку "char"
// ---------------------------------------
inline int UnicodeToString(const WCHAR * src,              // 11/12/06
                           char * dest, int bytes)
{

  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API
#ifdef PANFR
  unsigned int code = 1252;        // ANSI Latin 1; Western European (Windows)
#else
  unsigned int code = CP_ACP;      // The current system Windows ANSI code page.
#endif
  int ret = WideCharToMultiByte(code,0,
                                src,-1,
                                dest,bytes,0,0);     // bytes/sizeof(TCHAR)
  dest[bytes-1] = 0;
  return ret;
#else
#ifndef LINUXAPI                            // 06/12/06
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
#ifndef _GISSERVER                            // 30/08/13
  const char *tocode   = "KOI8-R";
#else
  const char *tocode   = "CP1251";
#endif
  const char *fromcode = NAME_CODEC_UTF16;    // 25/01/16 Korjilov
  size_t inbytes       = strlenun((const char *)src, bytes * 2); // 13/11/15
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);
  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;     // 10/07/08
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)
	 ret = 0;
    else
	ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int UnicodeToString(const WCHAR * src,              // 11/12/06
                           unsigned char * dest, int bytes)
{
  return UnicodeToString(src, (char *)dest, bytes);
}

inline int Unicode8ToString(const char * src,
                            char * dest,
                            int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API                                // 14/09/11
  WCHAR temp[300];
  if (bytes > (int)(sizeof(temp) / 2))
    bytes = sizeof(temp)/2 - 1;

  MultiByteToWideChar(CP_UTF8, 0, src, -1, temp, bytes);


  return WideCharToMultiByte(CP_ACP, 0, temp, -1,
                             dest, bytes, 0, 0);
#else
#ifndef LINUXAPI
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
#ifndef _GISSERVER                            // 30/08/13
  const char *tocode   = "KOI8-R";
#else
  const char *tocode   = "CP1251";
#endif
  const char *fromcode = "UTF-8";
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);
  if ((long int)mycodec != -1)
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int Unicode8ToUnicode(const char * src,
                             WCHAR * dest,
                             int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API                                // 14/09/11
  dest[0] = 0;
  return MultiByteToWideChar(CP_UTF8, 0, src, -1, dest, bytes/2);
#else
#ifndef LINUXAPI
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
//  const char *tocode   = "UTF-16";
  const char *tocode   = NAME_CODEC_UTF16;    // 04/02/16 Korjilov
  const char *fromcode = "UTF-8";
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);
  if ((long int)mycodec != -1)
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int UnicodeToUnicode8(const WCHAR * src, int srcbytes,
                             char * dest, int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API                                
  return WideCharToMultiByte(CP_UTF8, 0, src, srcbytes, dest, bytes, 0, 0);
#else
#ifndef LINUXAPI                           
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
  const char *fromcode = NAME_CODEC_UTF16;    // 25/01/16 Korjilov
  const char *tocode   = "UTF-8";
  size_t inbytes       = srcbytes;
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;     // 10/07/08
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int UnicodeToUnicode8(const WCHAR * src,
                             char * dest,
                             int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API                                // 14/09/11
  return WideCharToMultiByte(CP_UTF8, 0, src, -1,
                             dest, bytes, 0, 0);
#else
#ifndef LINUXAPI                            // 06/12/06
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
  const char *fromcode = NAME_CODEC_UTF16;    // 04/02/16 Korjilov
  const char *tocode   = "UTF-8";
  size_t inbytes       = strlenun((const char *)src, bytes * 2);  // 13/11/15
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;     // 10/07/08
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int StringToUnicode8(const char * src, char * dest, int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API
  WCHAR temp[4096];
  if (bytes > (int)sizeof(temp))                    // 05/02/14
    bytes = sizeof(temp);

  StringToUnicode(src, temp, bytes);

  return WideCharToMultiByte(CP_UTF8, 0, temp, -1,
                             dest, bytes, 0, 0);
#else
#ifndef LINUXAPI                            // 06/12/06
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
#ifndef _GISSERVER                            // 30/08/13
  const char *fromcode = "KOI8-R";
#else
  const char *fromcode = "CP1251";
#endif
  const char *tocode   = "UTF-8";
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;     // 10/07/08
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

inline int StringCP1251ToUnicode8(const char * src, char * dest, int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

#ifdef WIN32API
  WCHAR temp[4096];
  if (bytes > (int)sizeof(temp))                    // 05/02/14
    bytes = sizeof(temp);

  StringToUnicode(src, temp, bytes);

  return WideCharToMultiByte(CP_UTF8, 0, temp, -1,
                             dest, bytes, 0, 0);
#else
#ifndef LINUXAPI
  return (int)strncpy(dest,src,bytes);
#else
  iconv_t    mycodec;
  const char *fromcode = "CP1251";
  const char *tocode   = "UTF-8";
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
#endif
#endif
}

#ifdef LINUXAPI

inline int Unicode8ToStringCP1251(const char * src,
                            char * dest,
                            int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;


  iconv_t    mycodec;
  const char *fromcode = "UTF-8";
  const char *tocode   = "CP1251";
  size_t inbytes       = strlen(src);
  size_t outbytes      = (size_t)bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);

  if ((long int)mycodec != -1)             // 14/01/11
  {
    char **src1  = (char **)&src;     // 10/07/08
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
}

// Перевод UNICODE в CP1251                                // 25/03/13
inline int Unicode8ToStringWindows(const char * src,
                            char * dest,
                            int bytes, int *outlength)
{
  if ((src == 0) || (dest == 0)) return 0;

  iconv_t    mycodec;
  const char *tocode   = "CP1251";
  const char *fromcode = "UTF-8";
  size_t inbytes       = strlen(src);
  size_t outbytes      = bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);
  if ((long int)mycodec != -1)                                  // 11/04/13
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    *outlength = outbytes;
    if (ret == -1)                                             // 17/02/16
      ret = 0;
    else
      ret = 1;
    iconv_close(mycodec);
  }

  return ret;
}

// Перевод CP1251 в KOI8-R                                 // 12/01/15 Korjilov
inline int StringWindowsToStringKoi8(const char * src,
                            char * dest,
                            int bytes)
{
  if ((src == 0) || (dest == 0)) return 0;

  iconv_t    mycodec;
  const char *tocode   = "KOI8-R";
  const char *fromcode = "CP1251";
  size_t inbytes       = strlen(src);
  size_t outbytes      = bytes;
  int    ret = 0;

  if ((long int)src != (long int)dest) memset(dest, 0, bytes);  // 11/04/13
  mycodec = iconv_open(tocode, fromcode);
  if ((long int)mycodec != -1)                                  // 11/04/13
  {
    char **src1  = (char **)&src;
    char **dest1 = (char **)&dest;

    ret = iconv(mycodec, src1, &inbytes, dest1, &outbytes);
    if (ret == -1) ret = 0;
    iconv_close(mycodec);
  }

  return ret;
}

#ifndef HIDEMAXMIN
#ifndef max 
  #define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
  #define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#endif

// Процедура strupr для LINUX            // 14/01/15 Korjilov
 inline char * strupr(char * string)     // 16/07/14
 {
   int len = strlen(string);
   for(int ii = 0; ii < len; ii++)
    {
      string[ii] = toupper(string[ii]);
    }
    return string;
 }


#define _wcsnicmp wcsncmpi                         // 05/09/14
//#define wcsncmpi  wcsncmp                          // 11/04/13 // 05/09/14
#define _wcsicmp  wcscmpi			// 05/09/14
//#define wcscmpi   wcscmp			// 05/09/14
 //------------------------------------------------------------------
#endif

// РАБОТА С ПАМЯТЬЮ
// ----------------------

// Выделить физическую память
inline char * AllocateTheMemory(int size)
{
  void * Memory;

#ifdef WIN32API
  try
  {
    Memory = new char[size];              // 27/04/16
  }
  catch (...)
  {
    Memory = 0;
  }
#else
 #ifdef LINUXAPI
    Memory = malloc(size);
 #else
    Memory = 0;
 #endif

#endif

#if TRACE_MEM == TRUE
  Tracer2("\n+++++ %p - Allocate memory block, size - %u\n",
          Memory,size);
#endif

  return (char *) Memory;
}

// Освободить физическую память
inline int FreeTheMemory(char * address)        // 06/09/16 Савелов
{
#if TRACE_MEM == TRUE
  Tracer1("\n----- %p - Free memory block\n",address);
#endif

#ifdef WIN32API
  if (address == 0) return;
   delete [] address;
#else

  if (address == 0)
    return 0;
  free(address);


#endif
}

// Запросить объем свободной памяти в Кб
inline BIGINT AvailableTheMemory()             // 29/10/13
{
#ifdef WIN32API
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);

  GlobalMemoryStatusEx (&statex);
  return (statex.ullAvailPhys/1024L);  
#else
  return 1024*1024;     // 1Гб
#endif
}

// Запросить объем памяти в Кб, доступной приложению
inline BIGINT TotalTheMemory()
{
#ifdef WIN32API

  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);

  GlobalMemoryStatusEx (&statex);
#ifdef BUILD_DLL64
  return (statex.ullTotalPhys/1024L);
#else
  return (statex.ullTotalVirtual/1024L);
#endif

#else                                          // 08/02/16 Савелов

  // Общее число физических страниц памяти
  long physPages = sysconf(_SC_PHYS_PAGES);
  // Размер страницы памяти в байтах
  long pageSize  = sysconf(_SC_PAGESIZE);

  if ( physPages == -1 || pageSize == -1 )
    return 0;
  else
    return physPages * pageSize / 1024;

#endif  // WIN32API
}

inline int GetTheLastError()
{
 int ret;

#ifdef WIN32API

  ret = ::GetLastError();

#else

  ret = 0;

#endif

  return ret;

}

// РАБОТА С ФАЙЛАМИ
// ----------------------

inline void SplitThePath(const WCHAR *path, WCHAR *drive, WCHAR *dir,
                         WCHAR *name, WCHAR *ext)
{
#ifdef WIN32API
  _wsplitpath(path,drive,dir,name,ext);
#else
#ifdef LINUXAPI                                                 // 18/02/13
  int first = 0;
  int size, begin;
  int end;

  if(!path) return ;

  size = wcslen(path);

  // DRIVE
  if (drive) drive[0] = 0;

  begin = first;
  end = size;

  if (dir)  dir[0] = 0;

  while((path[end] != '/') &&  (end > begin))
  {
    end--;
  }

  if (begin < end || ((begin == end) && (path[end] == '/')))    // 19/02/16
  {
    end++;

    if (dir)
    {
      wcsncpy(dir, path+begin, end-begin);
      dir[end-begin] = 0;
    }
    begin = end;
  }
  else
  {
    begin = first;
  }

  if (name) name[0] = 0;
  if (ext)  ext[0]  = 0;

  if (begin < (end = size))
  {
    //EXT
    while((path[end] != '.') && (begin < end))
    {
      end--;
    }

    if (begin < end)
    {
      if (name)
      {
        wcsncpy(name, path+begin, end-begin);
        name[end-begin] = 0;
      }

      if(ext && (end < size))
      {
        wcsncpy(ext, path+end, size-end);
        ext[size-end] = 0;
      }
    }
    else
    {
      if (name)
      {
        wcsncpy(name, path + end, size - end);
        name[size - end] = 0;
      }
    }
  }
#endif       // LINUXAPI
#endif       // !WIN32API
}

inline void SplitThePath(const char *path, char *drive, char *dir,
                         char *name, char *ext)
{

#ifdef LINUXAPI
  int first = 0;                // 20/02/01
  int size, begin;
  int end;
  int drivesize;                  //20/11/01

  if(!path) return ;

  size = strlen(path);

  // DRIVE
  if (drive)
  {
    drive[0] = 0;
    drivesize = strcspn(path, ":");                    //20/11/01
    if (drivesize !=  size)  // строка содержит ':'    //20/11/01
    {
      strncpy(drive, path, drivesize+1);
      drive[drivesize+1] = 0;
      first = drivesize+1;
    }
  }

  begin = first;
  end = size;

  if (dir)  dir[0] = 0;

  while((path[end] != '/') &&  (end > begin))
  {
    end--;
  }

  if (begin < end || ((begin == end) && (path[end] == '/')))     // 19/02/16
  {
    end++;

    if (dir)   // 20/04/01
    {
      strncpy(dir, path+begin, end-begin);
      dir[end-begin] = 0;
    }
    begin = end;
  }
  else
  {
    begin = first;
  }

  if (name) name[0] = 0;
  if (ext)  ext[0]  = 0;

  if (begin < (end = size))
  {
    //EXT
    while((path[end] != '.') && (begin < end))
    {
      end--;
    }

    if (begin < end)
    {
      if (name)
      {
        strncpy(name, path+begin, end-begin);
        name[end-begin] = 0;
      }

      if(ext && (end < size))
      {
        strncpy(ext, path+end, size-end);
        ext[size-end] = 0;
      }
    }
    else
    {
      if (name)           // 18/06/02
      {
        strncpy(name, path + end, size - end);
        name[size - end] = 0;
      }
    }
  }

#else // if WIN32API

  _splitpath(path,drive,dir,name,ext);

/* 16/01/2012
  // Проверка на концевые пробелы
  if (name)                             // 03/02/11
    {
      long size = strlen(name)-1;
      for (int i = size; i >= 0; i--)
         {
          if (name[i] == ' ')
              name[i] = 0;
          else break;
         }
   }
*/

#endif

}

inline void MakeThePath(WCHAR *path, const WCHAR *drive, const WCHAR *dir,             // 23/04/12
                        const WCHAR *name, const WCHAR *ext)
{
#ifdef WIN32API                                                            // 18/02/13
  _wmakepath(path,drive,dir,name,ext);
#else
  if(!path) return ;
  path[0] = 0;
  if(drive)
  {
    wcsncat(path, drive, MAX_PATH-wcslen(path)-1);
  }

  if(dir)
  {
    wcsncat(path, dir, MAX_PATH-wcslen(path)-1);
  }

  if(name)
  {
    wcsncat(path, name, MAX_PATH-wcslen(path)-1);

    if(ext)
    {
      wcsncat(path, ext, MAX_PATH-wcslen(path)-1);
    }
  }
#endif
}

inline void MakeThePath(char *path, const char *drive, const char *dir,
                        const char *name, const char *ext)
{

#if defined(WINCE) || defined(LINUXAPI)

  if(!path) return ;
  path[0] = 0;
  if(drive)
  {
    strncat(path, drive, MAX_PATH-strlen(path)-1);
  }

  if(dir)
  {
    strncat(path, dir, MAX_PATH-strlen(path)-1);
  }

  if(name)
  {
    strncat(path, name, MAX_PATH-strlen(path)-1);

    if(ext)
    {
      strncat(path, ext, MAX_PATH-strlen(path)-1);
    }
  }

#else // if WIN32API

  _makepath(path,drive,dir,name,ext);

#endif

}

// Открыть временный файл
inline HANDLE OpenTheTempFile(const char * name,int access = 0,
                              int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                              int attribute = 0)
{
  HANDLE file;

  if (name == 0) name = "";

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)   // 18/06/02
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
       	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

#ifdef UNICODE
  TCHAR tname[MAX_PATH];
  StringToUnicode(name,tname,sizeof(tname));
#endif

  if ((file = ::CreateFile(
#ifndef UNICODE
                            name,
#else
                            tname,
#endif
                            access,mode,NULL,
                            attribute,
                            FILE_ATTRIBUTE_TEMPORARY|FILE_FLAG_DELETE_ON_CLOSE,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

  #ifdef LINUXAPI                                        // 14/01/15 Korjilov
  char cname[1028] = {0};

#ifdef FILESYSTEMUTF8                                    // 16/06/14
   // Перевод в кодировку UTF-8 файловой системы
   StringToUnicode8(name, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(name, cname, sizeof(cname) - 1);
#else
   strcpy(cname, name);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1) // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);                  // 14/01/15 Korjilov
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));              // 14/01/15 Korjilov
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", name, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheTempFile(const WCHAR * name,int access = 0,
                              int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                              int attribute = 0)
{
  HANDLE file;

#ifdef WIN32API                        // 11/04/13
  if (name == 0) name = L"";
#else
#ifdef BUILD_DLL
  if (name == 0) name = L"";
#else
  WCHAR namew[1];
  
  namew[0] = 0;
  if (name == 0) name = namew;
#endif
#endif

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)   // 18/06/02
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
       	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API
  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

  if ((file = ::CreateFileW(name,
                            access,mode,NULL,
                            attribute,
                            FILE_ATTRIBUTE_TEMPORARY|FILE_FLAG_DELETE_ON_CLOSE,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

  #ifdef LINUXAPI
   char cname[1024];
   char cname1[1024];                                    // 14/01/15 Korjilov 

   memset(cname1, 0, sizeof(cname1));                    // 11/04/13
   memset(cname, 0, sizeof(cname));                      // 11/04/13
   UnicodeToString(name, cname1, sizeof(cname1)-1);
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
   // Перевод в кодировку UTF-8 файловой системы
   StringToUnicode8(cname1, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(cname1, cname, sizeof(cname) - 1);
#else
   strcpy(cname, cname1);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1) // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  char aname[MAX_PATH];
  UnicodeToString(name, aname, sizeof(aname));
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", aname, (int)file);
#endif

  return file;
}

// Открыть файл для ассинхронного чтения и записи
inline HANDLE OpenTheOverlapFile(const char * name,int access = 0,
                                 int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                                 int attribute = 0)
{
  HANDLE file;

  if (name == 0) name = "";

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)   // 18/06/02
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
       	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

#ifdef UNICODE
  TCHAR tname[MAX_PATH];
  StringToUnicode(name,tname,sizeof(tname));
#endif

  if ((file = ::CreateFile(
#ifndef UNICODE
                            name,
#else
                            tname,
#endif
                            access,mode,NULL,
                            attribute,
                            FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

  #ifdef LINUXAPI                                         // 14/01/15 Korjilov

  char cname[1028] = {0};
#ifdef FILESYSTEMUTF8                                    // 16/06/14
   StringToUnicode8(name, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(name, cname, sizeof(cname) - 1);
#else
   strcpy(cname, name);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1) // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);                  // 14/01/15 Korjilov
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));               // 14/01/15 Korjilov
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", name, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheOverlapFile(const WCHAR * name,int access = 0,
                                 int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                                 int attribute = 0)
{
  HANDLE file;

#ifdef WIN32API                        // 30/08/13
  if (name == 0) name = L"";
#else
#ifdef BUILD_DLL
  if (name == 0) name = L"";
#else
  WCHAR namew[1];
  
  namew[0] = 0;
  if (name == 0) name = namew;
#endif
#endif

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)   // 18/06/02
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
       	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

  if ((file = ::CreateFileW(name,
                            access,mode,NULL,
                            attribute,
                            FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

  #ifdef LINUXAPI
   char cname[1024];
   char cname1[1024];                                      // 14/01/15 Korjilov

   memset(cname, 0, sizeof(cname));                        // 25/03/13
   memset(cname1, 0, sizeof(cname1));                        // 25/03/13
   UnicodeToString(name, cname1, sizeof(cname1)-1);
   cname1[sizeof(cname1)-1] = 0;
#ifdef FILESYSTEMUTF8                                      // 14/01/15 Korjilov
   StringToUnicode8(cname1, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(cname1, cname, sizeof(cname) - 1);
#else
   strcpy(cname, cname1);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1) // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  char tname[1024];

  memset(tname, 0, sizeof(tname));                        // 25/03/13
  UnicodeToString(name, tname, sizeof(tname)-1);
  tname[sizeof(tname)-1] = 0;

  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", tname, (int)file);
#endif

  return file;
}

// Открыть файл
inline HANDLE OpenTheFile(const char * name, int access = 0,
                          int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                          int attribute = OPEN_EXISTING)  // 29/06/14
{
  HANDLE file;

  if (name == 0) return 0;

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)   // 18/06/02
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

#ifdef UNICODE
  TCHAR tname[MAX_PATH];
  StringToUnicode(name,tname,sizeof(tname));
#endif

  if ((file = ::CreateFile(
#ifndef UNICODE
                            name,
#else
                            tname,
#endif
                            access,mode,NULL,
                            attribute,FILE_ATTRIBUTE_NORMAL,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

  #ifdef LINUXAPI
   char cname[1028];                                     // 14/01/15 Korjilov

   memset(cname, 0, sizeof(cname));
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
   StringToUnicode8(name, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(name, cname, sizeof(cname) - 1);
#else
   strcpy(cname, name);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1)       // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);                   // 14/01/15 Korjilov
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));               // 14/01/15 Korjilov
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", name, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheFileUn(const WCHAR * name, int access = GENERIC_READ|GENERIC_WRITE,
                            int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                            int attribute = OPEN_EXISTING)
{
  HANDLE file;

  if (name == 0)
    return 0;

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI
  if (attribute & O_CREAT)
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
       	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access=access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
#endif

#ifdef WIN32API
  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;     // 12/11/03  // 10/03/07

  int namelen = wcslen(name);
  if(namelen >= 256)
  {
    WCHAR longname[1024];
    WcsCopy(longname, L"\\\\?\\", sizeof(longname));
    WcsCat(longname, name, sizeof(longname));
    if ((file = ::CreateFileW(longname,
                            access,mode,NULL,
                            attribute,FILE_ATTRIBUTE_NORMAL,
                            0)) == INVALID_HANDLE_VALUE)
      {
        file = 0;
      }
  }
  else
  {

    if ((file = ::CreateFileW(name,
                            access,mode,NULL,
                            attribute,FILE_ATTRIBUTE_NORMAL,
                            0)) == INVALID_HANDLE_VALUE)
      {
        file = 0;
      }
  }
#else

  #ifdef LINUXAPI
   char cname[1024];
   char cname1[1024];                                       // 14/01/15 Korjilov
   
   memset(cname, 0, sizeof(cname));                         // 25/03/13
   memset(cname1, 0, sizeof(cname1));                       // 25/03/13
   UnicodeToString(name, cname1, sizeof(cname1)-1);
   cname1[sizeof(cname1)-1] = 0;
#ifdef FILESYSTEMUTF8                                       // 14/01/15 Korjilov
   StringToUnicode8(cname1, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(cname1, cname, sizeof(cname) - 1);
#else
   strcpy(cname, cname1);
#endif
#endif

   if ((int)(file = open(cname,access,mode)) == -1) // 21/08/03
   {
     file = 0;
   }

   if ((access & O_CREAT) && file)                       // 31/03/03
   {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);                 // 03/10/11
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));
   }
  #else
    file = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
   char tname[1024];
   memset(tname, 0, sizeof(tname));                        // 25/03/13
   UnicodeToString(name, tname, sizeof(tname)-1);

   Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", tname, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheFile(const WCHAR * name, int access = GENERIC_READ|GENERIC_WRITE,
                          int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                          int attribute = OPEN_EXISTING)
{
  return OpenTheFileUn(name, access, mode, attribute);
}

// Открыть файл на чтение
inline HANDLE OpenTheFileForRead(const char * name)  // 10/08/15
{
  return OpenTheFile(name, GENERIC_READ, FILE_SHARE_READ|FILE_SHARE_WRITE, OPEN_EXISTING);
}

// Открыть файл на чтение
inline HANDLE OpenTheFileForRead(const WCHAR * name)  // 10/08/15
{
  return OpenTheFile(name, GENERIC_READ, FILE_SHARE_READ|FILE_SHARE_WRITE, OPEN_EXISTING);
}

// 28/01/16
// Открыть файл в режиме разделения (Share mode)
// Именем файла должна быть строка в латинице в любой однобайтовой кодировке
inline HANDLE OpenTheShareFileLatin(const char * name,int access = 0,
                               int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                               int attribute = 0)
{
  HANDLE file;

  if (name == 0) name = "";

#ifdef LINUXAPI         // 18/06/02
  if (attribute & O_CREAT)
  {
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;
  }
#endif

  if ((mode & FILE_SHARE_WRITE) == 0)
    {
      // Доступ на запись монопольный
      return OpenTheFile(name,access,mode,attribute);
    }

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
        // Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access = access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;

  if ((file = open(name,access,mode)) == -1)  // 20/02/01
    {
      file = 0;
    }

  if ((access & O_CREAT) && file)                       // 31/03/03
  {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(name, 0, dir, 0, 0);                 // 14/01/15 Korjilov
     if (dir[0] != 0)
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(name, (statv.st_mode & ~flag));              // 14/01/15 Korjilov
  }
#else

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;        // 12/11/03 // 10/03/07


#ifdef UNICODE
  TCHAR tname[MAX_PATH];
  StringToUnicode(name,tname,sizeof(tname));
#endif


  if ((file = ::CreateFile(
#ifndef UNICODE
                           name,
#else
                           tname,
#endif
                           access,mode,NULL,
                           attribute,FILE_FLAG_WRITE_THROUGH,
                           0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

      file = 0;

#endif // WIN32API

#endif // LINUXAPI

#if TRACE_MAP == TRUE
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", name, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheShareFile(const char * name,int access = 0,
                               int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                               int attribute = 0)
{
  HANDLE file;

  if (name == 0) name = "";

#ifdef LINUXAPI         // 18/06/02
  if (attribute & O_CREAT)
  {
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;
  }    
#endif

  if ((mode & FILE_SHARE_WRITE) == 0)
    {
      // Доступ на запись монопольный
      return OpenTheFile(name,access,mode,attribute);
    }

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access = access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;
  char cname[1028];                                      // 14/01/15 Korjilov

  memset(cname, 0, sizeof(cname));
#ifdef FILESYSTEMUTF8                                    // 16/06/14
   StringToUnicode8(name, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
   // Кодировку CP1251 ГИС Сервера переводим 
   // в кодировку KOI8-R файловой системы
   StringWindowsToStringKoi8(name, cname, sizeof(cname) - 1);
#else
   strcpy(cname, name);
#endif
#endif

  if ((file = open(cname,access,mode)) == -1)  // 20/02/01
    {
      file = 0;
    }

  if ((access & O_CREAT) && file)                       // 31/03/03
  {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);                 // 14/01/15 Korjilov
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));              // 14/01/15 Korjilov
  }
#else

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;        // 12/11/03 // 10/03/07


#ifdef UNICODE
  TCHAR tname[MAX_PATH];
  StringToUnicode(name,tname,sizeof(tname));
#endif


  if ((file = ::CreateFile(
#ifndef UNICODE
                           name,
#else
                           tname,
#endif
                           access,mode,NULL,
                           attribute,FILE_FLAG_WRITE_THROUGH,
                           0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

      file = 0;

#endif // WIN32API

#endif // LINUXAPI

#if TRACE_MAP == TRUE
  Tracer2("\n->>>> Open file : %s,"
          "\n      Handle    = 0x%04X\n", name, (int)file);
#endif

  return file;
}

inline HANDLE OpenTheShareFile(const WCHAR * name,int access = 0,
                               int mode = FILE_SHARE_READ|FILE_SHARE_WRITE,
                               int attribute = 0)
{
  HANDLE file;

#ifdef WIN32API                        // 11/04/13
  if (name == 0) name = L"";
#else
#ifdef BUILD_DLL
  if (name == 0) name = L"";
#else
  WCHAR namew[1];
  
  namew[0] = 0;
  if (name == 0) name = namew;
#endif
#endif

#ifdef LINUXAPI         // 18/06/02
  if (attribute & O_CREAT)
  {
    mode = FILE_SHARE_READ | FILE_SHARE_WRITE;
  }
#endif

  if ((mode & FILE_SHARE_WRITE) == 0)
    {
      // Доступ на запись монопольный
      return OpenTheFile(name,access,mode,attribute);
    }

  if (attribute == 0) attribute = OPEN_ALWAYS;

#ifdef LINUXAPI

  if (access == 0) access = O_RDWR;                      // 10/03/07
  else
  {
    if (access & GENERIC_READ)
    {
      // Уберем GENERIC_READ(это наш флаг)
      access = access & 0xFFEFFFFF;
      if (access & GENERIC_WRITE)
      {
        // Уберем GENERIC_WRITE
        access = access & 0xFFFFFFFE;
	// Зададим аттрибуты файла на запись и чтение
        access = access | O_RDWR;
      }
    }
  }

  access = access|attribute;

  if ((access & O_CREAT) == 0) mode = 0;

  char cname[1024];                                       // 11/04/13
  char cname1[1024];                                      // 14/01/15 Korjilov

  memset(cname, 0, sizeof(cname));
  memset(cname1, 0, sizeof(cname1));
  UnicodeToString(name, cname1, sizeof(cname1)-1);
#ifdef FILESYSTEMUTF8                                     // 14/01/15 Korjilov
  StringToUnicode8(cname1, cname, sizeof(cname) - 1);
#else
#ifdef _GISSERVER
  // Кодировку CP1251 ГИС Сервера переводим 
  // в кодировку KOI8-R файловой системы
  StringWindowsToStringKoi8(cname1, cname, sizeof(cname) - 1);
#else
  strcpy(cname, cname1);
#endif
#endif

  if ((file = open(cname,access,mode)) == -1)  // 20/02/01
    {
      file = 0;
    }

  if ((access & O_CREAT) && file)                       // 31/03/03
  {
     char        dir[MAX_PATH];
     struct stat statv;
     int         flag = (S_IXUSR|S_IXGRP|S_IXOTH);

     memset(dir, 0, MAX_PATH);
     SplitThePath(cname, 0, dir, 0, 0);
     if (strlen(dir))
     {
       // Узнаем права каталога на запись и чтение
       stat(dir, &statv);
     }
     else statv.st_mode = FILE_SHARE_READ | FILE_SHARE_WRITE;

     // Установим права созданного файла на запись и чтение
     chmod(cname, (statv.st_mode & ~flag));
  }
#else

#ifdef WIN32API

  if (access == 0) access = GENERIC_READ|GENERIC_WRITE;        // 12/11/03 // 10/03/07

  if ((file = ::CreateFileW(name,
                            access,mode,NULL,
                            attribute,FILE_FLAG_WRITE_THROUGH,
                            0)) == INVALID_HANDLE_VALUE)
    {
      file = 0;
    }
#else

      file = 0;

#endif // WIN32API

#endif // LINUXAPI

#if TRACE_MAP == TRUE
   char tname[1024];
   memset(tname, 0, sizeof(tname));                        // 25/03/13
   UnicodeToString(name, tname, sizeof(tname)-1);

   Tracer2("\n->>>> Open file : %s,"
           "\n      Handle    = 0x%04X\n", tname, (int)file);
#endif

  return file;
}

// Закрыть файл
inline int CloseTheFile(HANDLE& file)
{
  int ret;

#ifdef WIN32API

  if (file != 0)
    {
      ret = ::CloseHandle(file);
    }
  else
    {
      ret = 0;
    }

#if TRACE_MAP == TRUE
  Tracer1("\n-<<<< Close for : 0x%04X\n", (int)file);
#endif

  file = 0;

#else

 #ifdef LINUXAPI
  if (file > 0)          // 20/06/02
    {
      ret = close(file); // 20/02/01

      if (ret == 0) ret = 1;  // 12/09/02
      else ret = 0;

#if TRACE_MAP == TRUE
  Tracer1("\n-<<<< Close for : 0x%04X\n", (int)file);
#endif
    }
  else
    {
      ret = 0;
    }
  #else
    file = 0;
    ret  = 0;
  #endif

#endif

  return ret;
}

// Проверить наличие файла
inline int ExistTheFileUn(const WCHAR * name)
{
  int ret;                              // 20/02/01
  HANDLE handle;

  if (name == 0) return 0;

  handle = OpenTheFileUn(name,
                         GENERIC_READ,
                         FILE_SHARE_READ|FILE_SHARE_WRITE,OPEN_EXISTING);

#ifdef LINUXAPI

  if (((int)handle) <= 0)
  {
    ret = 0;    // 20/06/02
  }
  else
  {
    ret = 1;
    CloseTheFile(handle);
  }

#else
  ret = handle && 1;

  CloseTheFile(handle);
#endif

  return ret;
}

inline int ExistTheFile(const WCHAR * name)
{
  return ExistTheFileUn(name);
}

// Проверить наличие файла
inline int ExistTheFile(const char * name)
{
  int ret;                              // 20/02/01
  HANDLE handle;

  if (name == 0) return 0;

  handle = OpenTheFile(name,
                              GENERIC_READ,
                              FILE_SHARE_READ|FILE_SHARE_WRITE,OPEN_EXISTING);

#ifdef LINUXAPI

  if (((int)handle) <= 0)
  {
    ret = 0;    // 20/06/02
  }
  else
  {
    ret = 1;
    CloseTheFile(handle);
  }

#else
  ret = handle && 1;

  CloseTheFile(handle);
#endif

  return ret;
}

// Проверить наличие файла, доступного на запись
inline int ExistTheFileForWrite(const char * name)
{
  if (name == 0) return 0;

  HANDLE handle = OpenTheFile(name,
                              GENERIC_WRITE,
                              FILE_SHARE_READ|FILE_SHARE_WRITE,OPEN_EXISTING);
  if (handle)
    {
      CloseTheFile(handle);
      return 1;
    }

  return 0;
}

// Проверить наличие файла, доступного на запись
inline int ExistTheFileForWriteUn(const WCHAR * name)         // 08/04/14
{
  if (name == 0) return 0;

  HANDLE handle = OpenTheFileUn(name,
                              GENERIC_WRITE,
                              FILE_SHARE_READ|FILE_SHARE_WRITE,OPEN_EXISTING);
  if (handle)
    {
      CloseTheFile(handle);
      return 1;
    }

  return 0;
}


// Считать блок данных из файла
inline int ReadTheFile(HANDLE file,char * address,long int length,
                       long int * result)
{
  int ret;

  if (file == 0) return 0;                                            // 14/01/15 Korjilov

#ifdef WIN32API

  ret = ::ReadFile(file,address,length,(LPDWORD)result,NULL);

#else

  #ifdef LINUXAPI
    ret = read(file,address,length);  // 20/02/01
    if (ret == -1)                    // 14/12/09
    {
      ret = 0;
      *result = ret;
    }
    else
      if (ret == 0)
      {
        *result = ret;
	ret = 1;
      }
    else {*result = ret; ret = 1;}
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n--->> Read for : 0x%04X, total = %u\n", (int)file,*ret);
#endif

  return ret;
}

#ifdef WIN32API                                             // 25/03/12
// Считать блок данных из файла асинхронно
inline int ReadTheOverlapFile(HANDLE file,char * address,long int length,
                              long int * result, OVERLAPPED * overlap)
{
  int ret;

#ifdef WIN32API

  ret = ::ReadFile(file,address,length,(LPDWORD)result,overlap);

#else

  #ifdef LINUXAPI
    ret = read(file,address,length);  // 20/02/01
    if (ret == -1)                    // 14/12/09
    {
      ret = 0;
      *result = ret;
    }
    else
      if (ret == 0)
      {
        *result = ret;
	ret = 1;
      }
    else {*result = ret; ret = 1;}
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n--->> Read for : 0x%04X, total = %u\n", (int)file,*ret);
#endif

  return ret;
}
#endif

// Установить смещение в файле перед чтением/записью
inline long int OffsetTheFile(HANDLE file,long int offset,int type)
{
  long int ret;

#ifdef WIN32API

  if (file != 0)
    {
      if (type == FILE_BEGIN)      // 26/07/00
        {
          long int top64 = 0;
          ret = ::SetFilePointer(file,offset,&top64,type);
        }
      else
          ret = ::SetFilePointer(file,offset,0,type);

      if (ret == (long int)0x0FFFFFFFF) ret = 0;
    }
  else
    {
      ret = 0;
    }
#else

 #ifdef LINUXAPI
    if (file != 0)
    {
#ifdef __USE_LARGEFILE64                     // 08/07/15
      ret = lseek64(file,offset,type);
#else
      ret = lseek(file,offset,type);
#endif
      if (ret == -1) ret = 0;
    }
    else
    {
      ret = 0;
    }
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n===== Offset for: 0x%04X, offset = %li\n",
                                      (int)file,ret);
#endif

  return ret;
}

// Установить смещение в файле перед чтением/записью
inline unsigned int OffsetTheLongFile(HANDLE file,long int offset,long int * top64,int type)
{
  unsigned int ret;

#ifdef WIN32API

  if (file != 0)
    {
      ret = ::SetFilePointer(file,offset,top64,type);

      if (ret == (unsigned int)0x0FFFFFFFF)
        {
          ret    = 0;
          *top64 = 0;
        }
    }
  else
    {
      ret    = 0;
      *top64 = 0;
    }
#else

 #ifdef LINUXAPI
    if (file != 0)
    {
      __int64 offset1 = offset + (((__int64)*top64) << 32);    // 08/07/15    // 17/05/16 Letov
#ifdef __USE_LARGEFILE64
      ret = lseek64(file,offset1,type);
#else
      ret = lseek(file,offset1,type);
#endif
      if (ret == -1) ret = 0;
    }
    else
    {
      ret = 0;
    }
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n===== Offset for: 0x%04X, offset = %li\n",
                                      (int)file,ret);
#endif

  return ret;
}

// Установить смещение в файле перед чтением/записью
inline __int64 OffsetTheLongFile(HANDLE file, __int64 offset, int type)
{
#ifdef WIN32API                                         // 08/07/15
  long int top64 = (long int)(offset >> 32);
  __int64 ret     = OffsetTheLongFile(file, (long int)(offset & 0x0FFFFFFFF), &top64, type);

  return (ret + (((__int64)top64) << 32));
#else
  __int64 ret     = 0;

  if (file != 0)
  {
#ifdef __USE_LARGEFILE64
    ret = lseek64(file,offset,type);
#else
    ret = lseek(file,offset,type);
#endif
    if (ret == -1) ret = 0;
  }
  else
  {
    ret = 0;
  }

  return ret;
#endif
}

// Запросить длину файла
inline __int64 LengthTheLongFile(HANDLE file)
{
#ifdef WIN32API                                         // 08/07/15
  long int top64 = 0;
  __int64 ret    = OffsetTheLongFile(file, 0, &top64, FILE_END);

  return (ret + (((__int64)top64) << 32));
#else
  __int64 ret     = 0;

  if (file != 0)
  {
#ifdef __USE_LARGEFILE64
    ret = lseek64(file,0,SEEK_END);
#else
    ret = lseek(file,0,SEEK_END);
#endif
    if (ret == -1) ret = 0;
  }
  else
  {
    ret = 0;
  }

  return ret;
#endif
}

inline __int64 LengthTheLongFile(const WCHAR * name)        // 30/06/14
{
  if (name == 0) return 0;

  HANDLE hfile = OpenTheFile(name, GENERIC_READ);
  if (hfile == 0) return 0;
  __int64 ret = LengthTheLongFile(hfile);
  CloseTheFile(hfile);
  return ret;
}

// Запросить длину файла
inline unsigned long int LengthTheFile(HANDLE file)
{
  unsigned long int ret;

#ifdef WIN32API

  if (file != 0)
    {
      long int top64 = 0;
      ret = ::SetFilePointer(file, 0, &top64, FILE_END);
      if (ret == (unsigned long int)0x0FFFFFFFF) ret = 0;
    }
  else
    {
      ret = 0;
    }
#else

 #ifdef LINUXAPI
  if (file != 0)
    {
#ifdef __USE_LARGEFILE64                            // 08/07/15
      ret = lseek64(file,0,SEEK_END);
#else
      ret = lseek(file,0,SEEK_END);
#endif
      if (ret == -1) ret = 0;
    }
  else
    {
      ret = 0;
    }
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n===== Length for: 0x%04X, length = %li\n",
                                     (int)file,ret);
#endif

  return ret;
}

inline double LengthTheBigFile(HANDLE file)       // 28/07/10
{
  double ret;

#ifdef WIN32API

  if (file != 0)
    {
      LARGE_INTEGER filesize;

      if (GetFileSizeEx(file, &filesize) != 0)      // 05/08/11
        ret = (double)filesize.QuadPart;
    }
  else
    {
      ret = 0;
    }
#else

 #ifdef LINUXAPI
  if (file != 0)
    {
#ifdef __USE_LARGEFILE64                            // 08/07/15
      ret = lseek64(file,0,SEEK_END);
#else
      ret = lseek(file,0,SEEK_END);
#endif
      if (ret == -1) ret = 0;
    }
  else
    {
      ret = 0;
    }
  #else
    ret = 0;
  #endif

#endif

#if TRACE_READ == TRUE
  Tracer2("\n===== Length for: 0x%04X, length = %li\n",
                                     (int)file,ret);
#endif

  return ret;
}

// Установить новую длину файла
inline int EndTheFile(HANDLE file)
{
  int ret;

#ifdef WIN32API

  if (file != 0)
    {
      ret = ::SetEndOfFile(file);
    }
  else
    {
      ret = 0;
    }

#else

  #ifdef LINUXAPI
    if (file != 0)
    {
#ifdef __USE_LARGEFILE64                            // 08/07/15
      ret = lseek64(file,0,SEEK_CUR);
#else
      ret = lseek(file,0,SEEK_CUR);
#endif
#ifdef WATCOM  // 26/04/00
      ret = ltrunc(file,ret,0);
#else
      ret = ftruncate(file,ret);
#endif
      if (ret == -1) ret = 0;         // 24/09/02
      else ret = 1;
    }
    else
    {
      ret = 0;
    }
  #else
    ret = 0;
  #endif

#endif

#if TRACE_WRITE == TRUE
  Tracer1("\n----< End   for : 0x%04X\n", file);
#endif

  return ret;
}

// Записать блок данных в файл
inline int WriteTheFile(HANDLE file,const char * address,long int length,
                        long int * result)
{
  int ret;

  if ((file == 0) || (address == 0) || (length <= 0)) return 0; 

#ifdef WIN32API

  ret = ::WriteFile(file,address,length,(LPDWORD)result,NULL);

#else

  #ifdef LINUXAPI
    ret = write(file,address,length);
    if (ret == -1)                      // 14/12/09
    {
      ret = 0;
      *result = ret;
    }
    else
      if (ret == 0)
      {
        ret = 1;
       	*result = 0;
      }
      else { *result = ret;ret = 1; }
  #else
    ret = 0;
  #endif

#endif  // WIN32API

#if TRACE_WRITE == TRUE
  Tracer2("\n---<< Write for : 0x%04X, total = %lu\n", file,result);
#endif

  return ret;

}

#ifdef WIN32API                                                           // 25/03/12
// Записать блок данных в файл асинхронно
inline int WriteTheOverlapFile(HANDLE file,const char * address,long int length,
                               long int * result, OVERLAPPED * overlap)
{
  int ret;

#ifdef WIN32API

  ret = ::WriteFile(file,address,length,(LPDWORD)result,overlap);

#else

  #ifdef LINUXAPI
    ret = write(file,address,length);
    if (ret == -1)                      // 14/12/09
    {
      ret = 0;
      *result = ret;
    }
    else
      if (ret == 0)
      {
        ret = 1;
       	*result = 0;
      }
      else { *result = ret;ret = 1; }
  #else
    ret = 0;
  #endif

#endif  // WIN32API

#if TRACE_WRITE == TRUE
  Tracer2("\n---<< Write for : 0x%04X, total = %lu\n", file,result);
#endif

  return ret;

}
#endif

// Сохранить буферы записи данных в файл на диск
inline void FlushTheFileBuffers(HANDLE file)
{
  if (file == 0) return;
#ifdef WIN32API
  FlushFileBuffers(file);
#else
  fsync(file);                                        // 11/09/15
#endif
}

inline int DeleteTheFile(const WCHAR * name)
{
  int ret;

#ifdef WIN32API
  ret = ::DeleteFileW(name);
#else

  #ifdef LINUXAPI
    char dest[MAX_PATH];                                     // 22/06/12

    memset(dest, 0, sizeof(dest));
#ifdef FILESYSTEMUTF8                                        // 14/01/15 Korjilov
   // Перевод в кодировку UTF-8 файловой системы
   UnicodeToUnicode8(name, dest, sizeof(dest) - 1);
#else
   UnicodeToString(name, dest, sizeof(dest) - 1);
#endif

    ret = unlink(dest);      // 20/02/01

    if (ret == 0) ret = 1;           // 12/09/02
    else ret = 0;
  #else
    ret = 0;
  #endif

#endif

#if TRACE_MAP == TRUE
  char aname[MAX_PATH];
  UnicodeToString(name, aname, sizeof(aname));
  Tracer2("\n===== Delete for : %s, ret = %u\n", aname,ret);
#endif

  return ret;
}

inline int DeleteTheFileUn(const WCHAR * name)               // 23/04/12
{
  return DeleteTheFile(name);
}

inline int DeleteTheFile(const char * name)
{
#ifdef WIN32API
  int ret = ::DeleteFileA(name);
#else

  char dest[MAX_PATH];                                   // 14/01/15 Korjilov

  memset(dest, 0, sizeof(dest));
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
  // Перевод в кодировку UTF-8 файловой системы
  StringToUnicode8(name, dest, sizeof(dest) - 1);
#else
  strncpy(dest, name, sizeof(dest) - 1);
#endif

  int ret = unlink(dest);      // 20/02/01

  if (ret == 0) ret = 1;           // 12/09/02
  else ret = 0;
#endif

#if TRACE_MAP == TRUE
  Tracer2("\n===== Delete for : %s, ret = %u\n", name,ret);
#endif

  return ret;
}

inline int MoveTheFile(const WCHAR * oldname,const WCHAR * newname)
{
  int ret;

#ifdef WIN32API

  ret = ::MoveFileExW(oldname,newname,
                      MOVEFILE_COPY_ALLOWED|MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH);
#else

  #ifdef LINUXAPI
    char toldname[MAX_PATH];          // 13/03/13
    memset(toldname, 0, sizeof(toldname));               // 25/03/13

#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
    // Перевод в кодировку UTF-8 файловой системы
    UnicodeToUnicode8(oldname,toldname,sizeof(toldname) - 1);
#else
    UnicodeToString(oldname,toldname,sizeof(toldname) - 1);
#endif

    char tnewname[MAX_PATH];
    memset(tnewname, 0, sizeof(tnewname));
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
    // Перевод в кодировку UTF-8 файловой системы
    UnicodeToUnicode8(newname,tnewname,sizeof(tnewname) - 1);
#else
    UnicodeToString(newname,tnewname,sizeof(tnewname) - 1);
#endif

    ret = rename(toldname,tnewname);   // 20/02/01

    if (ret == 0) ret = 1;           // 12/09/02
    else ret = 0;
  #else
    ret = 0;
  #endif
#endif

#if TRACE_MAP == TRUE
  char coldname[1024];
  char cnewname[1024];

  UnicodeToString(oldname, coldname, sizeof(coldname)-1);
  UnicodeToString(newname, cnewname, sizeof(cnewname)-1);

  Tracer2("\n===== Move from %s to %s", coldname, cnewname);
  Tracer1("\n      Move ret : %u\n", ret);
#endif

  return ret;
}

inline int MoveTheFile(const char * oldname,const char * newname)
{
  int ret;

#ifdef WIN32API
  ret = ::MoveFileExA(oldname, newname,
                      MOVEFILE_COPY_ALLOWED|MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH);
#else

  #ifdef LINUXAPI
    char toldname[MAX_PATH];                              // 14/01/15 Korjilov
    memset(toldname, 0, sizeof(toldname));

#ifdef FILESYSTEMUTF8                                     // 14/01/15 Korjilov
    // Перевод в кодировку UTF-8 файловой системы
    StringToUnicode8(oldname,toldname,sizeof(toldname) - 1);
#else
    strncpy(toldname, oldname, sizeof(toldname) - 1);
#endif

    char tnewname[MAX_PATH];
    memset(tnewname, 0, sizeof(tnewname));
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
    // Перевод в кодировку UTF-8 файловой системы
    StringToUnicode8(newname,tnewname,sizeof(tnewname) - 1);
#else
    strncpy(tnewname, newname, sizeof(tnewname) - 1);
#endif

    ret = rename(toldname, tnewname);   // 20/02/01

    if (ret == 0) ret = 1;           // 12/09/02
    else ret = 0;
  #else
    ret = 0;
  #endif
#endif

#if TRACE_MAP == TRUE
  Tracer2("\n===== Move from %s to %s", oldname,newname);
  Tracer1("\n      Move ret : %u\n", ret);
#endif

  return ret;
}

#ifdef WIN32API   // 13/03/13
inline int CopyTheFile(const char * oldname,
                       const char * newname, int exist = 0)
{
  int ret;

#ifdef WIN32API

#ifdef UNICODE
  TCHAR toldname[MAX_PATH];
  StringToUnicode(oldname,toldname,sizeof(toldname));

  TCHAR tnewname[MAX_PATH];
  StringToUnicode(newname,tnewname,sizeof(tnewname));

  ret = ::CopyFile(toldname,tnewname,exist);
#else
  ret = ::CopyFile(oldname,newname,exist);
#endif

#else
    ret = 0;
#endif

#if TRACE_MAP == TRUE
  Tracer2("\n===== Copy from %s to %s", oldname,newname);
  Tracer1("\n      Copy ret : %u\n", ret);
#endif

  return ret;
}
#else
int CopyTheFile(const char * oldname,
                const char * newname, int exist);
#endif


inline int CopyTheFileUn(const WCHAR * oldname,
                         const WCHAR * newname, int exist = 0)
{
#ifdef WIN32API   // 13/03/13
  int ret = ::CopyFileW(oldname,newname,exist);

#if TRACE_MAP == TRUE
   char coldname[1024];
   char cnewname[1024];

   UnicodeToString(oldname, coldname, sizeof(coldname)-1);
   UnicodeToString(newname, cnewname, sizeof(cnewname)-1);

   Tracer2("\n===== Copy from %s to %s", coldname, cnewname);
   Tracer1("\n      Copy ret : %u\n", ret);
#endif
#else
  char toldname[MAX_PATH];
  memset(toldname, 0, sizeof(toldname));                       // 25/03/13
  UnicodeToString(oldname,toldname,sizeof(toldname));

  char tnewname[MAX_PATH];
  memset(tnewname, 0, sizeof(tnewname));                       // 25/03/13
  UnicodeToString(newname,tnewname,sizeof(tnewname));
  int ret = CopyTheFile(toldname,tnewname,exist);
#endif

  return ret;
}

inline int CopyTheFile(const WCHAR * oldname,
                       const WCHAR * newname, int exist = 0)
{
  return CopyTheFileUn(oldname, newname, exist);
}



inline int CreateTheDirectory(const char * name)
{
 int ret;

#ifdef WIN32API
#ifdef UNICODE
  TCHAR tname[1024];
  StringToUnicode(name,tname,sizeof(tname));
  ret = ::CreateDirectory(tname, NULL);
#else
  ret = ::CreateDirectory(name, NULL);           // 08/02/12
#endif

  if (ret == 0)
    {
      DWORD error = ::GetTheLastError();
      if (error == ERROR_ALREADY_EXISTS)
        {
          return -1;     // 08/12/11
        }

      if (error == ERROR_PATH_NOT_FOUND)
        {
          return 0;     // 07/01/12
        }

#ifdef UNICODE
      if (SetCurrentDirectory(tname) == 0)           // 07/01/12
#else
      if (SetCurrentDirectory(name) == 0)
#endif
        return 0;
    }

  return 1;
#else
  #ifdef LINUXAPI  // 04/03/00
  char  tname[1024];
#ifdef FILESYSTEMUTF8                                    // 14/01/15 Korjilov
    // Перевод в кодировку UTF-8 файловой системы
    StringToUnicode8(name,tname,sizeof(tname) - 1);
#else
    strcpy(tname, name);
#endif

    ret = mkdir(tname, S_IRWXU|S_IRWXG|S_IRWXO);   // 31/03/03
    if (ret == -1)
      {
        if (errno == EEXIST) ret = -1;            // 25/03/13
        else ret = 0;
      }
    else ret = 1;

    if (ret && (errno != EEXIST))           // 31/03/03
    {
      char   currentdir[MAX_PATH];                // 14/01/15 Korjilov
      struct stat statv;
      int i = -1;

      // Узнаем имя каталога, в котором создается текущий подкаталог
      strcpy(currentdir, tname);                  // 14/01/15 Korjilov

      if (strlen(currentdir) > 1)
      {
        currentdir[strlen(currentdir) - 1] = 0;
       	while ((i > -1) && (currentdir[i] != '/'))
       	{
          currentdir[i] = 0;
       	  i--;
       	}

       	if (i > -1) currentdir[i] = 0;
      }

      if (i == -1)
        statv.st_mode = S_IRWXU|S_IRWXG|S_IRWXO;
      else
        // Узнаем права каталога на запись и чтение
        stat(currentdir, &statv);

      // Установим права созданного каталога на запись и чтение
      chmod(tname, statv.st_mode);                        // 14/01/15 Korjilov
    }
  #else
    ret = 0;
  #endif

  return ret;

#endif
}

inline int CreateTheDirectoryUn(const WCHAR * name)       // 12/09/12
{
 int ret;

#ifdef WIN32API
  ret = ::CreateDirectoryW(name, NULL);           // 08/02/12

  if (ret == 0)
    {
      DWORD error = ::GetTheLastError();
      if (error == ERROR_ALREADY_EXISTS)
        {
          return -1;
        }

      if (error == ERROR_PATH_NOT_FOUND)
        {
          return 0;
        }

      if (SetCurrentDirectoryW(name) == 0)
        return 0;
    }

  return 1;
#else
  char  tname[1024];                              // 25/03/13
  memset(tname, 0, sizeof(tname));                // 25/03/13
  UnicodeToString(name,tname,sizeof(tname));

  return CreateTheDirectory(tname);
#endif
}

inline int CreateTheDirectory(const WCHAR * name)
{
  return CreateTheDirectoryUn(name);
}

inline int DeleteTheDirectory(const char * name)
{
#ifdef WIN32API
  return ::RemoveDirectoryA(name);
#else
  int ret = rmdir(name);

  if (ret == 0) ret = 1;           // 12/09/02
  else ret = 0;

  return ret;
#endif
}

inline int DeleteTheDirectory(const WCHAR * name)
{
#ifdef WIN32API
  return ::RemoveDirectoryW(name);
#else
  char cname[1024];                                       // 11/04/13

  memset(cname, 0, sizeof(cname));
  UnicodeToString(name, cname, sizeof(cname)-1);

  int ret = rmdir(cname);

  if (ret == 0) ret = 1;           // 12/09/02
  else ret = 0;

  return ret;
#endif
}

inline int GetTheCurrentDirectory(char * name, long int size)
{
#ifdef WIN32API
  return ::GetCurrentDirectoryA(size, name);
#else
  return (long int) getcwd(name,size);   // 25/03/12
#endif
}

inline int GetTheCurrentDirectory(WCHAR * name, long int size)
{
#ifdef WIN32API
  return ::GetCurrentDirectoryW(size/2, name);
#else
  char cname[1024];                                       // 11/04/13

  memset(cname, 0, sizeof(cname));
  UnicodeToString(name, cname, sizeof(cname)-1);

  return (long int) getcwd(cname,size);   // 25/03/12
#endif
}

inline int SetTheFileAttributes(const char * name,
                                long int attributes)
{
#ifdef WIN32API
  return ::SetFileAttributesA(name,(DWORD)attributes);
#else
  int ret = chmod(name,attributes);
  if (ret == 0) ret = 1;   // 14/12/00
  else ret = 0;            // 12/09/02

  return ret;
#endif
}

inline int SetTheFileAttributes(const WCHAR * name,
                                long int attributes)
{
#ifdef WIN32API
  return ::SetFileAttributesW(name,(DWORD)attributes);
#else
  char cname[1024];                                       // 11/04/13

  memset(cname, 0, sizeof(cname));
  UnicodeToString(name, cname, sizeof(cname)-1);

  return SetTheFileAttributes(cname, attributes);
#endif
}

inline int SetTheFileWriteAttributes(const WCHAR * name, int flag)
{
#ifdef WIN32API
  DWORD value = ::GetFileAttributesW(name);
  if (flag == 0)
    value |= FILE_ATTRIBUTE_READONLY;
  else
    value &= ~FILE_ATTRIBUTE_READONLY;

  return SetTheFileAttributes(name, value);
#else
//  Атрибуты файла                                      // 14/01/15 Korjilov
  char dummy[MAX_PATH];
  memset(dummy, 0, sizeof(dummy));
  UnicodeToString(name, dummy, sizeof(dummy));
  if (flag != 0)
#ifndef ANDROID
    if (chmod(dummy, (S_IWGRP | S_IWRITE | S_IWOTH)) == 0)
#else
    if (chmod(dummy, (S_IWGRP | S_IWUSR | S_IWOTH)) == 0)      // можно ли объединить? #TODO
#endif
      return 1;
    else
      return 0;
#endif
}

inline int GetTheFileWriteAttributes(const WCHAR * name)
{
#ifdef WIN32API
  DWORD value = ::GetFileAttributesW(name);
  return ((value & FILE_ATTRIBUTE_READONLY) == 0);
#else
  //  Атрибуты файла                                     // 14/01/15 Korjilov
  char dummy[MAX_PATH];
  struct stat stbuf;
  FILE *fp;
  memset(&stbuf, 0, sizeof(stbuf));
  memset(dummy, 0, sizeof(dummy));
  UnicodeToString(name, dummy, sizeof(dummy));
  if ((fp=fopen(dummy, "rb")) != NULL)
  {
    if (fstat(fileno(fp), &stbuf) == -1)
    {
      fclose(fp);
      return 0;
    }
    fclose(fp);
#ifndef ANDROID
    return (stbuf.st_mode & (S_IWGRP | S_IWRITE | S_IWOTH));
#else
    return (stbuf.st_mode & (S_IWGRP | S_IWUSR | S_IWOTH));      // можно ли объединить? #TODO
#endif
  }
  return 0;
#endif
}

inline int GetTheTempPath(int bytes,char * buffer)
{
  if ((bytes <= 0) || (buffer == 0)) return 0;

#ifdef WIN32API
  return ::GetTempPathA(bytes,buffer);
#else

  buffer = strcpy(buffer,P_tmpdir);
  return 4;
#endif
}

inline int GetTheTempPathUn(int bytes,WCHAR * buffer)  // 12/09/12
{

  if ((bytes <= 0) || (buffer == 0)) return 0;

#ifdef WIN32API
  return ::GetTempPathW(bytes/2,buffer);
#else
  int ret;          // 20/02/01
  char *buffer_char = AllocateTheMemory(bytes / sizeof(WCHAR));  // 25/03/13
  memset(buffer_char, 0, bytes / sizeof(WCHAR));
  ret = GetTheTempPath(bytes / sizeof(WCHAR), buffer_char);
  StringToUnicode(buffer_char, buffer, bytes);
  FreeTheMemory(buffer_char);
  buffer_char = 0;
  ret = ret * sizeof(WCHAR);
  return ret;
#endif
}

inline int GetTheTempPath(int bytes,WCHAR * buffer)
{
  return GetTheTempPathUn(bytes, buffer);
}

// Запросить системное время в формате "YYYYMMDD" 
inline int GetTheSystemDate(char * dest,int size)
{
  SYSTEMTIME data;
  int form = 0;
  int delta;

  if (dest == 0) return 0;
  if (size < 10) return 0;

  GetSystemTime(&data);

  delta = data.wYear * 10000;               // ГОД
  if (delta > 0) form += delta;
  delta     = data.wMonth * 100;            // МЕСЯЦ
  if (delta > 0) form += delta;
  delta     = data.wDay;                    // ДЕНЬ
  if (delta > 0) form += delta;

  itoa(form,dest,10);

  return 1;
}

// Запросить локальную дату в формате "YYYYMMDD" 
inline int GetTheLocalDate(char * dest,int size)       // 15/10/15
{
  SYSTEMTIME data;
  int form = 0;
  int delta;

  if (dest == 0) return 0;
  if (size < 10) return 0;

  GetLocalTime(&data);

  delta = data.wYear * 10000;               // ГОД
  if (delta > 0) form += delta;
  delta     = data.wMonth * 100;            // МЕСЯЦ
  if (delta > 0) form += delta;
  delta     = data.wDay;                    // ДЕНЬ
  if (delta > 0) form += delta;

  itoa(form,dest,10);

  return 1;
}

// Запросить локальную дату в формате "YYYY-MM-DD" 
inline int GetTheLocalDateFormat(char * dest, int size)       // 15/10/15
{
  SYSTEMTIME data;

  if (dest == 0) return 0;
  if (size < 12) return 0;

  GetLocalTime(&data);

  itoa(data.wYear, dest, 10);               // ГОД
  strcat(dest, "-");
  itoa(data.wMonth, dest+strlen(dest), 10); // МЕСЯЦ
  strcat(dest, "-");
  itoa(data.wDay, dest+strlen(dest), 10);   // ДЕНЬ
  return 1;
}

// Запросить локальное время в формате "ЧЧ:ММ:CC" 
inline int GetTheLocalTimeFormat(char * dest, int size)       // 15/10/15
{
  SYSTEMTIME data;

  if (dest == 0) return 0;
  if (size < 10) return 0;

  GetLocalTime(&data);

#if !defined(WIN32API) || defined(__BORLANDC__)
  snprintf( dest, size, "%02d:%02d:%02d", data.wHour, data.wMinute, data.wSecond );  // 16/12/15 Letov V.I.
#else
  sprintf(dest, "%02d:%02d:%02d", data.wHour, data.wMinute, data.wSecond );  // 16/12/15 Letov V.I.
#endif
  return 1;
}

// Преобразование структуры SYSTEMTIME в дату в формате "YYYYMMDD" // 17/07/15
// и число секунд от 00:00:00
inline void SysTimeToValue(SYSTEMTIME &systime, unsigned int *outdate,
                                                unsigned int *outtime)
{
  if (outdate)
    {
      *outdate = 0;
       int delta = systime.wYear * 10000;       // ГОД
       if (delta > 0) *outdate += delta;
       delta = systime.wMonth * 100;            // МЕСЯЦ
       if (delta > 0) *outdate += delta;
       delta = systime.wDay;                    // ДЕНЬ
       if (delta > 0) *outdate += delta;
    }
  if (outtime)
    *outtime = (long) systime.wSecond + (long) systime.wMinute * 60L +
               (long) systime.wHour * 3600L;
}

// Запрос даты и времени в формате "YYYYMMDD" и число секунд от 00:00:00
inline void GetTheSystemDateAndTime(unsigned int *outdate, unsigned int *outtime)
{
  SYSTEMTIME systime;
  GetSystemTime(&systime);
  SysTimeToValue(systime, outdate, outtime); // 17/07/15
}

// Запрос даты и времени в формате "YYYYMMDD" и число секунд от 00:00:00
inline void GetTheLocalDateAndTime(unsigned int *outdate, unsigned int *outtime)    // 15/10/15
{
  SYSTEMTIME systime;
  GetLocalTime(&systime);
  SysTimeToValue(systime, outdate, outtime); // 17/07/15
}

// Запрос даты и времени обновления файла в UTC    // 23/06/16
inline int GetTheFileTime(HANDLE hfile, SYSTEMTIME& time)
{
  FILETIME mtime;

#ifdef LINUXAPI
  struct stat stbuf;
  memset(&stbuf, 0, sizeof(stbuf));
  if (fstat(hfile, &stbuf) == -1)            // 13/01/15
    return 0;

  time_t ft = stbuf.st_mtime;
  struct tm local;
  memset(&local, 0, sizeof(local));
  gmtime_r(&ft, &local);                      // 23/06/16   // Время по UTC

  time.wDay         = local.tm_mday;
  time.wDayOfWeek   = local.tm_wday;
  time.wHour        = local.tm_hour;
  time.wMinute      = local.tm_min;
  time.wMonth       = local.tm_mon + 1;     // в Линуксе январь = 0 в Виндовс январь = 1. Делаем пока как в Виндовс.
  time.wSecond      = local.tm_sec;
  time.wYear        = local.tm_year + 1900; // 04/09/15 Савелов
  time.wMilliseconds = 0;

  return stbuf.st_mtime;

#else
  int ret = 1;
  if (::GetFileTime(hfile,NULL,NULL,&mtime) != 0)
    return FileTimeToSystemTime(&mtime, &time);
  return 0;
#endif
}

// Запрос даты и времени обновления файла в UTC                      // 07/08/16
inline int GetTheFileTime(const WCHAR *name, SYSTEMTIME& systime)
{
  if (name == 0) return 0;

#ifdef WIN32API
  // Считываем атрибуты файла
  WIN32_FIND_DATAW finddata;
  HANDLE findhandle = FindFirstFileW(name, &finddata);
  if (findhandle != INVALID_HANDLE_VALUE)
    {
      FindClose(findhandle);

      // Расшифровываем время последней записи
      if (FileTimeToSystemTime(&finddata.ftLastWriteTime, &systime) != 0)
        return 1;
    }
#else
  HANDLE hfile = OpenTheFile(name, GENERIC_READ);
  if (hfile != 0)
    {
      int ret = GetTheFileTime(hfile, systime);
      CloseTheFile(hfile);
      return ret;
    }
#endif

  return 0;
}

inline int GetTheFileTime(const char *name, SYSTEMTIME& systime)
{
  if (name == 0) return 0;

#ifdef WIN32API
  // Считываем атрибуты файла
  WIN32_FIND_DATAA finddata;
  HANDLE findhandle = FindFirstFileA(name, &finddata);
  if (findhandle != INVALID_HANDLE_VALUE)
    {
      FindClose(findhandle);

      // Расшифровываем время последней записи
      if (FileTimeToSystemTime(&finddata.ftLastWriteTime, &systime) != 0)
        return 1;
    }
#else
  HANDLE hfile = OpenTheFile(name, GENERIC_READ);
  if (hfile != 0)
    {
      int ret = GetTheFileTime(hfile, systime);
      CloseTheFile(hfile);
      return ret;
    }
#endif

  return 0;
}

// Установка даты и времени обновления файла
inline int SetTheFileTime(HANDLE hfile, SYSTEMTIME& time)            // 10/07/14
{
#ifndef ANDROID             // TODO
  if (hfile == 0) return 0;

  FILETIME ftime;

#ifdef WIN32API
  if (SystemTimeToFileTime(&time, &ftime) != 0)
    return SetFileTime(hfile, &ftime, &ftime, &ftime);
#else    // 04/09/15 Савелов
  struct tm local;
  memset(&local, 0, sizeof(local));

  local.tm_mday   = time.wDay;
  local.tm_hour   = time.wHour;
  local.tm_min    = time.wMinute;
  local.tm_mon    = time.wMonth - 1;     // в Линуксе январь = 0 в Виндовс январь = 1. Делаем пока как в Виндовс.
  local.tm_sec    = time.wSecond;
  local.tm_year   = time.wYear - 1900;
  local.tm_isdst  = -1;

  time_t ft = mktime(&local);
  struct timeval ftimes[2];             // 30/10/15
  ftimes[0].tv_sec = ftimes[1].tv_sec = ft;
  ftimes[0].tv_usec = ftimes[1].tv_usec = 0;

  if (futimes((int)hfile, ftimes) == 0)
    return ft;
#endif

#endif  // !ANDROID

  return 0;
}

// Установка даты и времени обновления файла                         // 10/07/14
inline int SetTheFileTime(HANDLE hfile, unsigned int *date, unsigned int * ftime)
{
  if (date == 0) return 0;

  SYSTEMTIME data;
  memset((char*)&data, 0, sizeof(data));

  data.wYear = (WORD)(*date / 10000);          // ГОД
  int delta = *date - data.wYear * 10000;
  data.wMonth = (WORD)(delta / 100);           // МЕСЯЦ
  delta = delta - data.wMonth * 100;
  data.wDay = (WORD)delta;                     // ДЕНЬ

  if (ftime != 0)
    {
      data.wHour = (WORD)(*ftime/3600);          // Часы
      delta = delta - data.wHour * 3600;
      data.wMinute = (WORD)(delta / 60);         // Минуты
      delta = delta - data.wMinute * 60;
      data.wSecond = (WORD)delta;              // Секунды
    }

  return SetTheFileTime(hfile, data);
}

// Запрос даты и времени в формате "YYYYMMDD" и число секунд от 00:00:00
inline int GetTheFileSystemDateAndTime(HANDLE hfile, unsigned int *outdate, unsigned int *outtime)
{
  SYSTEMTIME systime;
  if (GetTheFileTime(hfile, systime) == 0) return 0;
  SysTimeToValue(systime, outdate, outtime); // 17/07/15
  return 1;
}

inline int GetTheFileSystemDateAndTime(const WCHAR *name, unsigned int *outdate,
                                                          unsigned int *outtime)
{
  if (name == 0) return 0;
  int ExitCode = 0;

#ifdef WIN32API // 17/07/15
  // Считываем атрибуты файла
  WIN32_FIND_DATAW finddata;
  HANDLE findhandle = FindFirstFileW(name, &finddata);
  if (findhandle != INVALID_HANDLE_VALUE)
    {
      FindClose(findhandle);

      // Расшифровываем время последней записи
      SYSTEMTIME systime;
      if (FileTimeToSystemTime(&finddata.ftLastWriteTime, &systime) != 0)
        {
          // Преобразуем расшифрованное время в дату, время
          SysTimeToValue(systime, outdate, outtime);
          ExitCode = 1;
        }  
    }  
#else
  HANDLE hfile = OpenTheFile(name, GENERIC_READ);
  if (hfile != 0)
    {
      ExitCode = GetTheFileSystemDateAndTime(hfile, outdate, outtime);
      CloseTheFile(hfile);
    }  
#endif
  if (ExitCode == 0)
    {
       if (outdate) *outdate = 0;
       if (outtime) *outtime = 0;
    }
  return ExitCode;
}

inline int GetTheFileSystemDateAndTime(const char *name, unsigned int *outdate,
                                                         unsigned int *outtime)
{
  if (name == 0) return 0;
  int ExitCode = 0;

#ifdef WIN32API // 17/07/15
  // Считываем атрибуты файла
  WIN32_FIND_DATAA finddata;
  HANDLE findhandle = FindFirstFileA(name, &finddata);
  if (findhandle != INVALID_HANDLE_VALUE)
    {
      FindClose(findhandle);

      // Расшифровываем время последней записи
      SYSTEMTIME systime;
      if (FileTimeToSystemTime(&finddata.ftLastWriteTime, &systime) != 0)
        {
          // Преобразуем расшифрованное время в дату, время
          SysTimeToValue(systime, outdate, outtime);
          ExitCode = 1;
        }  
    }   
#else
  HANDLE hfile = OpenTheFile(name, GENERIC_READ);
  if (hfile != 0)
    {
      ExitCode = GetTheFileSystemDateAndTime(hfile, outdate, outtime);
      CloseTheFile(hfile);
    }  
#endif
  if (ExitCode == 0)
    {
      if (outdate) *outdate = 0;
      if (outtime) *outtime = 0;
    }
  return ExitCode;
}

// Оставил для Windows и Linux один прототип // 14/12/09
// Для Linux: Handle - указатель на функцию обратного вызова обработки сообщений (MSGHANDLER)
inline long int SendTheMessage(HWND Handle,long int messagenumber,WPARAM code, LPARAM parm)    // 04/09/15 Савелов // 01/08/16
{
  if (Handle == 0) return 0;
#ifndef LINUXAPI
  return (long int)::SendMessage(Handle,messagenumber,code,parm);
#else
  MSGHANDLER msgHandler = (MSGHANDLER)Handle;
  return (long int)msgHandler(0, messagenumber, code, parm, 0);      // 08/02/16 Савелов
#endif
}

inline int LockTheFile(HANDLE file,long int offset, long int size)
{
#ifdef WIN32API
#ifndef WINCE
  return LockFile(file,offset,0,size,0);
#else
  return 1;
#endif
#else
  return 1;
#endif
}

inline int UnlockTheFile(HANDLE file,long int offset,
                         long int size)
{
#ifdef WIN32API
#ifndef WINCE
  return UnlockFile(file,offset,0,size,0);
#else
  return 1;
#endif
#else
  return 1;
#endif
}

inline void SleepTheThread(unsigned int milliseconds = 1)
{
#ifdef WIN32API
  Sleep(milliseconds);
#else
  usleep(milliseconds*1000);      // 03/08/16 Савелов
#endif
}

//-----------------------------------------------------------------
// Проверить наличие свободного места на диске
// При ошибке возвращает ноль
//-----------------------------------------------------------------
inline int CheckTheDiskFreeSpace(const char *name, double length)
{
#ifdef WIN32API
#ifndef WINCE

  DWORD    SectorsPerCluster;
  DWORD    BytesPerSector;
  DWORD    NumberOfFreeClusters;
  DWORD    TotalNumberOfClusters;
  char     RootDisk[MAX_PATH];

  if (name == 0)  return 0;

  ::SplitThePath(name,RootDisk,NULL,NULL,NULL);

  if (::GetDiskFreeSpaceA                   // 09/03/10
          (strcat(RootDisk,"\\"),
           &SectorsPerCluster,
           &BytesPerSector,
           &NumberOfFreeClusters,
           &TotalNumberOfClusters) == 0)
    {
      // Не могу определить свободное место на диске
      return 1;
    }

  if (NumberOfFreeClusters == 0 ||   // 17/09/07
      (SectorsPerCluster*BytesPerSector) < (length/NumberOfFreeClusters))
    {
      // Отсутствует свободное место на диске
      return 0;
    }
#endif       // !WINCE

  return 1;

#else
#ifndef NUMBERTURN                                  // 13/11/06
    struct statfs stbuf;                            // 10/06/04

    if (statfs(name,&stbuf) == -1) return 0;

    double freebytecount = (double)stbuf.f_bsize*(double)stbuf.f_bfree;   // 14/12/09
    if ((freebytecount) < length)
    {
      return 0;
    }

    return 1;                 // 16/07/04
#else
    return 1;
#endif
#endif
}

inline int CheckTheDiskFreeSpace(const WCHAR *name, double length)            // 19/06/14
{
#ifdef WIN32API
#ifndef WINCE

  DWORD    SectorsPerCluster;
  DWORD    BytesPerSector;
  DWORD    NumberOfFreeClusters;
  DWORD    TotalNumberOfClusters;
  WCHAR    RootDisk[MAX_PATH];

  if (name == 0)  return 0;

  ::SplitThePath(name, RootDisk, NULL, NULL, NULL);
  wcscat(RootDisk, L"\\");

  if (::GetDiskFreeSpaceW(
           RootDisk,
           &SectorsPerCluster,
           &BytesPerSector,
           &NumberOfFreeClusters,
           &TotalNumberOfClusters) == 0)
    {
      // Не могу определить свободное место на диске
      return 1;
    }

  if (NumberOfFreeClusters == 0 ||
      (SectorsPerCluster*BytesPerSector) < (length/NumberOfFreeClusters))
    {
      // Отсутствует свободное место на диске
      return 0;
    }
#endif       // !WINCE

  return 1;

#else
#ifndef NUMBERTURN                                                            // 30/10/15
    struct statfs stbuf;
    char  tname[1024];

    memset(tname, 0, sizeof(tname));
#ifdef FILESYSTEMUTF8
    // Перевод в кодировку UTF-8 файловой системы
    UnicodeToUnicode8(name,tname,sizeof(tname));
#else
    UnicodeToString(name,tname,sizeof(tname));
#endif

    if (statfs(tname,&stbuf) == -1) return 0;

    double freebytecount = (double)stbuf.f_bsize*(double)stbuf.f_bfree;
    if ((freebytecount) < length)
    {
      return 0;
    }

    return 1;
#else
    return 1;
#endif
#endif
}

inline int CheckTheDiskFreeSpaceUn(const WCHAR *name, double length)            // 19/06/14
{
  return CheckTheDiskFreeSpace(name, length);
}


/***** example *******************************************************
inline int LockTheFileOrSleep(HANDLE file,long int offset,
                              long int size,long int seconds = 5)
{
  int count = seconds * 1000;
  while(count-- > 0)
   {
     if (LockTheFile(file,offset,size)) return 1;
     SleepTheThread(1000);
   }

  return 0;
}
*******************************************************************/

//-----------------------------------------------------------------
// Получить короткое имя файла
// При ошибке возвращает ноль
//-----------------------------------------------------------------
inline int GetShortFileName(char * filename,char * shortname, long int size)
{
  if ( (filename == 0) || (shortname == 0) || (size <= 0) ) return 0;

#ifdef WIN32API
#ifndef WINCE                                          // 22/04/03

 HANDLE handle = ::OpenTheFile(filename,GENERIC_READ,
                               FILE_SHARE_READ|FILE_SHARE_WRITE,
                               OPEN_EXISTING);
 if (handle)
   {
     ::GetShortPathNameA(filename,shortname,size);
     ::CloseTheFile(handle);
     return 1;
   }

#endif
#endif

 strncpy(shortname,filename,size);
 return 0;

}

//-----------------------------------------------------------------
// Получить длинное имя файла
// При ошибке возвращает ноль
//-----------------------------------------------------------------
inline int GetLongFileName(char * filename,char * longname, long int size)
{
 if ( (filename == 0) || (longname == 0) || (size <= 0) ) return 0;

#ifdef WIN32API
#ifndef WINCE                                                     // 22/04/03

 HANDLE handle = ::OpenTheFile(filename,GENERIC_READ,
                               FILE_SHARE_READ|FILE_SHARE_WRITE,
                               OPEN_EXISTING);
 if (handle)
   {
     WIN32_FIND_DATAA name;                          // 09/03/10
     ::FindFirstFileA(filename,&name);
     strncpy(longname, name.cFileName, size);
     ::CloseTheFile(handle);
     return 1;
   }

#endif
#endif

 strncpy(longname,filename,size);
 return 0;

}

#ifndef HIDESECTION   // 28/09/06
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++ РАЗГРАНИЧЕНИЕ ДОСТУПА НА ЗАПИСЬ ПОТОКОВ++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32API
typedef CRITICAL_SECTION MAPCRITICALSECTION;
#else
typedef pthread_mutex_t MAPCRITICALSECTION;
#endif

//-----------------------------------------------------------------
// Инициализация критической секции
//-----------------------------------------------------------------
inline void InitializeTheCriticalSection(MAPCRITICALSECTION *section)
{
#ifdef WIN32API
  InitializeCriticalSection(section);
#else
  pthread_mutexattr_t attr;                                       // 18/12/06

  pthread_mutexattr_init(&attr);
#ifndef NUMBERTURN
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP); // 08/05/08
  // Для МСВС раскомментировать и закомментировать строчку выше // 19/06/08
//  attr.__mutexkind = PTHREAD_MUTEX_RECURSIVE_NP;
#else
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
#endif
  pthread_mutex_init(section, &attr);

  pthread_mutexattr_destroy(&attr);
#endif
}

//-----------------------------------------------------------------
// Удаление критической секции
//-----------------------------------------------------------------
inline void DeleteTheCriticalSection(MAPCRITICALSECTION *section)
{
#ifdef WIN32API
  DeleteCriticalSection(section);
#else
  pthread_mutex_destroy(section);
#endif
}

//-----------------------------------------------------------------
// Занять критическую секцию
//-----------------------------------------------------------------
inline void EnterTheCriticalSection(MAPCRITICALSECTION *section)
{
#ifdef TRACE_LOCK
  TracerLoc1("-----> Lock section = %p\n", section);
#endif

#ifdef WIN32API
  EnterCriticalSection(section);
#else
  pthread_mutex_lock(section);
#endif
}

//-----------------------------------------------------------------
// Освободить критическую секцию
//-----------------------------------------------------------------
inline void LeaveTheCriticalSection(MAPCRITICALSECTION *section)
{
#ifdef TRACE_LOCK
  TracerLoc1("<----- UnLock section = %p\n", section);
#endif

#ifdef WIN32API
  LeaveCriticalSection(section);
#else
  pthread_mutex_unlock(section);
#endif
}
#endif // !HIDESECTION

#ifdef WIN32API
typedef HANDLE HMUTEX;
#else
typedef pthread_mutex_t HMUTEX;
#endif

//-----------------------------------------------------------------
// Инициализация mutex
//-----------------------------------------------------------------
inline void CreateTheMutex(HMUTEX& mutex)
{
#ifdef WIN32API
  mutex = CreateMutex(NULL, FALSE, NULL);
#else
  pthread_mutexattr_t attr;

  pthread_mutexattr_init(&attr);
#ifndef NUMBERTURN
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);  // 07/05/08
  // Для МСВС раскомментировать и закомментировать строчку выше  // 19/06/08
//  attr.__mutexkind = PTHREAD_MUTEX_RECURSIVE_NP;                 // 20/05/08
#else
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
#endif
  pthread_mutex_init(&mutex, &attr);

  pthread_mutexattr_destroy(&attr);
#endif
}

//-----------------------------------------------------------------
// Удаление критической секции
//-----------------------------------------------------------------
inline void DeleteTheMutex(HMUTEX& mutex)
{
#ifdef WIN32API
  if (mutex != 0)
    {
      CloseHandle(mutex);
      mutex = 0;
    }
#else
  pthread_mutex_destroy(&mutex);
#endif
}

//-----------------------------------------------------------------
// Ожидание освобождения объекта
//-----------------------------------------------------------------
inline int WaitTheObject(HANDLE object, DWORD dwMilliseconds = INFINITE)
{
#ifdef WIN32API
   if (object != 0)
     {
       return (WaitForSingleObject(object, dwMilliseconds) == WAIT_OBJECT_0);    // 0xFFFFFFFF
     }

   return 0;
#else
//    pthread_mutex_lock(&mutex);
   return 1;
#endif
}

#ifdef WIN32API                                                     // 25/03/12
//-----------------------------------------------------------------
// Ожидание освобождения объекта и запрос результата
//-----------------------------------------------------------------
inline unsigned int GetTheOverlapResult(HANDLE object, OVERLAPPED * overlap)
{
#ifdef WIN32API
   DWORD result = 0;
   if ((object != 0) && (GetOverlappedResult(object, overlap, &result, TRUE) != 0))
     return result;

   return 0;
#else
//    pthread_mutex_lock(&mutex);
   return 1;
#endif
}
#endif

//-----------------------------------------------------------------
// Занять критическую секцию на неопределенное время
//-----------------------------------------------------------------
inline int WaitTheMutex(HMUTEX& mutex)
{
#ifdef WIN32API
   if (mutex != 0)
     {
       return (WaitForSingleObject(mutex, INFINITE) == WAIT_OBJECT_0);    // 0xFFFFFFFF
     }

   return 0;
#else
   pthread_mutex_lock(&mutex);
   return 1;
#endif
}

//-----------------------------------------------------------------
// Занять критическую секцию в пределах заданного времени
//-----------------------------------------------------------------
inline int WaitTheMutexByTime(HMUTEX& mutex, int msec)
{
#ifdef WIN32API
   if (mutex == 0)
     return 0;

   DWORD ret = WaitForSingleObject(mutex, msec);
   if ((ret == WAIT_OBJECT_0) ||
       (ret == WAIT_ABANDONED))            // 27/07/16 
     return 1;

   if (ret == WAIT_TIMEOUT)                // Время истекло
     return -1;

   return 0;                               // Ошибка
#else
   struct timeval now; 
   struct timespec timeout;
   int ret = 0;

#ifndef NUMBERTURN                             // 01/07/08

#ifndef ANDROID

   gettimeofday(&now, NULL);
   timeout.tv_sec = now.tv_sec + msec / 1000;
   timeout.tv_nsec = now.tv_usec * 1000 + (msec - msec / 1000) * 1000;
   ret = pthread_mutex_timedlock(&mutex, &timeout);   // 14/09/11
   if (ret == 0)
     ret = 1;
   else
   // ETIMEDOUT, EAGAIN, EDEADLK, EINVAL
     ret = -1;
#else
// Для МСВС раскомментировать и закомментировать весь код выше
   ret = 1;
   pthread_mutex_lock(&mutex);
#endif  // !ANDROID

#else
   pthread_mutex_lock(&mutex);
   ret = 1;                                    // 02/04/09
#endif

   return ret;
#endif
}

//-----------------------------------------------------------------
// Освободить критическую секцию
//-----------------------------------------------------------------
inline void ReleaseTheMutex(HMUTEX& mutex)
{
#ifdef WIN32API
   if (mutex != 0)
     {
       ReleaseMutex(mutex);
     }
#else
   pthread_mutex_unlock(&mutex);
#endif
}

#if defined(BUILD_DLL64) && defined(LINUXAPI)                            // 25/03/12
#pragma pack(pop)
#endif

#ifdef WIN32API
#if defined(_M_X64) || defined(BUILD_DLL64)       // 30/11/13
//  #pragma option pop /*P_O_Pop*/
#endif
#endif


#endif  // MAPSYST_H



