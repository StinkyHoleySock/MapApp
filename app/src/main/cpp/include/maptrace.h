/***** MAPTRACE.H ************** Belenkov O.V.   ****** 31/07/14 ***
****** MAPTRACE.H ************** Dishlenko S.G.  ****** 01/12/15 ***
****** MAPTRACE.H ************** Kruzhkova E.A.  ****** 02/04/09 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2015              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*        ÙÚ·ÛÛÈÚÔ˜Î· ˜˘ÔÏÓÂÓÈÒ ÚÔÁÚ·ÌÌ ‰ÏÒ ÔÈÛÎ· Ô˚È‚ÔÎ         *
*                                                                  *
*******************************************************************/

#ifndef MAPTRACE_H
#define MAPTRACE_H

#include "mapversn.h"

//#define MAPTRACER TRUE
//#define TRACE_LOCK TRUE
//#define TRACE_MEM TRUE
//#define TRACE_MAP TRUE

#ifndef MAPTRACER
  #define MAPTRACER 0      // Â”Ã… ‘“¡””…“œ◊À¡ Œ’÷Œ¡ -
                            //  –œ”‘¡◊…‘ÿ TRUE
#endif

#ifndef LINUXAPI
 #ifndef LOGFILENAME
  #define LOGFILENAME "c:\\ProgramData\\maptrace.log"
//  #define LOGFILENAME "c:\\maptrace.log"
 #endif
#else
 #ifdef LINUXSERVER
  #define LOGFILENAME "/export/home/maptrace.log"
 #else
  #define LOGFILENAME "/maptrace.log"
 #endif
#endif

#ifndef TRAC_LOC
  #include <stdio.h>
  #define TracerLoc1(str, mes) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, str, mes);\
            fclose( hFile ); } }
#endif

#if MAPTRACER != 0

  #include <stdio.h>


#ifndef LINUXAPI   

//*******************************************************************
//  “…Õ≈“ ◊Ÿ⁄œ◊¡ ‘“¡””…“œ◊À… :
//   Tracer("0x%04X|message = 0x%04X BEGIN.\n", hwnd, uMsg);
//*******************************************************************

  #define Tracer2(str, num, mes) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );\
            if (hFile) { fprintf(hFile, str, num, mes); fclose( hFile );} }

  #define Tracer1(str, mes) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );  \
            if (hFile) { char timebuff[40]; timebuff[0] = 0; \
            GetTimeFormatA(NULL,0,NULL, "HH':'mm':'ss", timebuff, sizeof(timebuff)); \
            strcat(timebuff, "  "); fprintf(hFile, timebuff); \
            fprintf( hFile, str, mes); fclose( hFile );} }

  #define Tracer(message) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, message); fclose( hFile );} }

  #define TraceCode(str, nwin, nmes) Tracer2(str, nwin, nmes)
  #define TraceMessage(message) Tracer(message)
  #define TraceMessage1(message, mes) Tracer1(message, mes)
  #define TraceMessage2(message, v1, v2) Tracer2(message, v1, v2)

#else  // 02/04/09


  #define TracerSun(message) \
          { FILE* HFile = fopen( LOGFILENAME, "a+" );\
            if (HFile) { fprintf( HFile, message);\
           fclose( HFile ); } }

  #define Tracer2(str, num, mes) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, str, num, mes);\
            fclose( hFile ); } }

  #define Tracer1(str, mes) \
          { FILE* hFile = fopen( LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, str, mes);\
            fclose( hFile ); } }

  #define Tracer(message) \
          { FILE* HFile = fopen( LOGFILENAME, "a+" );\
            if (HFile) { fprintf( HFile, message);\
            fclose( HFile ); } }

  #define TraceCode(str, nwin, nmes) \
            { FILE* hFile = fopen( LOGFILENAME, "a+" );\
              if (hFile) { fprintf( hFile, str, nwin, nmes);\
              fclose( hFile ); } }

  #define TraceMessage(message) \
            { FILE* hFile = fopen( LOGFILENAME, "a+" );\
              if (hFile) { fprintf( hFile, message);\
              fclose( hFile ); } }

  #define TraceMessage1(message, v1) \
            { FILE* hFile = fopen( LOGFILENAME, "a+" );\
              if (hFile) { fprintf( hFile, message, v1);\
              fclose( hFile ); } }

  #define TraceMessage2(message, v1, v2) \
            { FILE* hFile = fopen( LOGFILENAME, "a+" );\
              if (hFile) { fprintf( hFile, message, v1, v2);\
              fclose( hFile ); } }
#endif

#else
  #define Tracer2(str, num, mes)
  #define Tracer1(str, mes)
  #define Tracer(message)

  #define TraceCode(str, nwin, nmes)
  #define TraceMessage(message)
  #define TraceMessage1(message, v1)
  #define TraceMessage2(message, v1, v2)

  #define TracerSun(message)
#endif

#ifdef GISWEB
  #define GisTraceCode(str, v1, v2) TraceMessage2(str, v1, v2)
  #define GisTraceMessage(message)  TraceMessage(message)
  #define TracerSun(message)
#else
#ifndef LINUXAPI
 #if MAPTRACER != 0

  #include <stdio.h>

  #define GisTraceCode(str, nwin, nmes) \
          { FILE* hFile = fopen(LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, str, nwin, nmes);\
            fclose( hFile ); } }
  #define GisTraceMessage(message) \
          { FILE* hFile = fopen(LOGFILENAME, "a+" );\
            if (hFile) { fprintf( hFile, message);\
            fclose( hFile ); } }
 #else
   #define GisTraceCode(str, nwin, nmes)
   #define GisTraceMessage(message) 
 #endif
#else
  #define GisTraceCode(str, nwin, nmes)
  #define GisTraceMessage(message) 
#endif

#endif    // GISWEB

#endif
