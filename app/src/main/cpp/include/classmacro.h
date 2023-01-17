/****  classmacro.h  ********  Zheleznyakov  V.A. ***** 16/09/15 ***
*****  classmacro.h  ********  Savelov A.S.       ***** 23/05/16 ***
*****  classmacro.h  ********  Dishlenko S.G.     ***** 21/04/16 ***
*                                                                  *
*              Copyright (c) PANORAMA Group 1991-2016              *
*                     All Rights Reserved                          *
*                                                                  *
********************************************************************
*                                                                  *
*                      FOR WINDOWS & LINUX                         *
*                                                                  *
********************************************************************
*                                                                  *
*                  Определение имён классов                        *
*            Необходимо устнавить в настройках проекта             *
*        condition defines CLASSPREFIX=Название префикса класса    *
*                                                                  *
*******************************************************************/

#ifndef CLASSMACRO_H
#define CLASSMACRO_H

// Для обеспечения надежной работы 64-ех разрядных приложений, 
// включающих несколько динамических библиотек, в которых применяются
// одни и те же общие классы, необходимо обеспечить, чтобы компилятор
// автоматически формировал разные имена классам в разных библиотеках.
// Для этого в проектах библиотек необходимо определить уникальный "префикс"
// проекта. 
// Например, в gisu64mapacces: CLASSPREFIX=MapAcc, в objectmath: CLASSPREFIX=Math
// Определение префикса необходимо делать в Windows и Linux

#ifndef CLASSNAME
  #define CLASSNAME
  #define _XClassName(a,b) _ClassName(a,b) // стрингификация
  #define _ClassName(a,b) a##b // склеивание
#endif

// simlist
#define THandleList _XClassName(THandleList, CLASSPREFIX)
#define TSimpleList _XClassName(TSimpleList, CLASSPREFIX)

// cmdparser
#define TParmList _XClassName(TParmList, CLASSPREFIX)
#define TLayerList _XClassName(TLayerList, CLASSPREFIX)
#define TRoleList _XClassName(TRoleList, CLASSPREFIX)
#define TParmSet _XClassName(TParmSet, CLASSPREFIX)
#define TParmParser _XClassName(TParmParser, CLASSPREFIX)

// xmlrpc
#define TParmXmlRpcParser _XClassName(TParmXmlRpcParser, CLASSPREFIX)

// longstring
#define TLongString _XClassName(TLongString, CLASSPREFIX)
#define TLongStringArray _XClassName(TLongStringArray, CLASSPREFIX)
#define TLongStringUn _XClassName(TLongStringUn, CLASSPREFIX)
#define TLongStringArrayUn _XClassName(TLongStringArrayUn, CLASSPREFIX)  // 21/04/16
#define TDataArray _XClassName(TDataArray, CLASSPREFIX)                  // 21/04/16

// xmlfile
#define XMLATTRVALUEUN _XClassName(XMLATTRVALUEUN, CLASSPREFIX)
#define XMLATTRVALUE _XClassName(XMLATTRVALUE, CLASSPREFIX)
#define XMLNODE _XClassName(XMLNODE, CLASSPREFIX)
#define XMLATTRNODE _XClassName(XMLATTRNODE, CLASSPREFIX)
#define TXmlFile _XClassName(TXmlFile, CLASSPREFIX)

// translate
//#define TTranslateRule _XClassName(TTranslateRule, CLASSPREFIX)
//#define TTranslate _XClassName(TTranslate, CLASSPREFIX)

// transaction
#define TTransaction _XClassName(TTransaction, CLASSPREFIX)      // 21/04/16 Савелов
#define TSitX _XClassName(TSitX, CLASSPREFIX)                    // 21/04/16 Савелов
#define LFILE _XClassName(LFILE, CLASSPREFIX)                    // 21/04/16 Савелов

#endif
