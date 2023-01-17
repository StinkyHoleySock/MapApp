/******* MapLib.H ************** Belenkov O.V.   **** 25/04/16 ****
******** MapLib.H ************** Shishkova L.A.  **** 15/09/03 ****
******** MapLib.H ************** Dishlenko S.G.  **** 30/10/13 ****
******** MapLib.H ************** Shabakov D.A.   **** 27/04/16 ****
******** MapLib.H ************** Tarakanova I.R. **** 16/12/13 ****
******** MapLib.H ************** Zheleznyakov V.A.*** 10/07/13 ****
******** MapLib.H ************** Borzov A.U.     **** 21/09/15 ****
******** MapLib.H ************** Derevyagina Zh.A.*** 15/04/16 ****
******** MapLib.H ************** Osipov D.A.      *** 07/09/15 ****
******** MapLib.H ************** Kruzhkov A.E.   **** 16/09/15 ****
*******************************************************************/

#ifndef MAPLIB_H
#define MAPLIB_H

#if defined(__BORLANDC__)
#ifndef MAPVERSN_H
  #include "mapversn.h"
#endif
#endif

#ifndef MAPSYST_H
  #include "mapsyst.h"
#endif

#ifdef PANVECTOR
 #define MAPEXE       "VECTOR.exe"
 #define HELPNAMERU   "VECTOR.chm"
 #define HELPNAMEENGL "VECTOR.ENGL.chm" 
 #define INIFILE      "VECTOR.INI"
 #define PICFILE      "VECTOR.PIC"
 #define AXIAPILIB    "Panorama.dll"
#else
 #define HELPNAMERU    "Panorama.chm" 
 #define HELPNAMERU    "Panorama.chm" 
 #define HELPNAMEFR    "Panorama.fr.chm" 
 #define HELPNAMEUA    "Panorama.ua.chm" 
#ifdef PANFR
 #define HELPNAMEENGL  "Panorama.fr.chm" 
#else
#ifdef PANUA
 #define HELPNAMEENGL  "Panorama.ua.chm" 
#else
 #define HELPNAMEENGL  "Panorama.engl.chm" 
#endif
#endif

#define HELPNAME      HELPNAMERU
#define PICFILE       "Panorama.pic"
#define MAPTREE       "Panorama.tre"

#ifdef MEJDELO                                 // 09/07/15
 #define MAPEXE        "MapSurvey.exe"
 #define INIFILE       "MapSurvey.ini"
#else
#ifdef GISARMY                                 // 30/10/13
 #define MAPEXE        "Operator.exe"
 #define INIFILE       "Operator.ini"
#else
#ifdef PANDESK
 #define MAPEXE        "PanDesk.exe"  
 #define INIFILE       "PanDesk.ini"
#else
#ifdef NAVIGATOR
 #define MAPEXE        "Navigator.exe"
 #define INIFILE       "Navigator.ini"
#else
#ifdef PANEDIT
 #define MAPEXE        "PanEdit.exe"
 #define INIFILE       "PanEdit.ini"
#else
#ifdef PANMINI
 #define MAPEXE        "Mapview.exe"
 #define INIFILE       "Mapview.ini"
#else
 #define MAPEXE        "Panorama.exe"
 #define INIFILE       "Panorama.ini"

#endif // PANMINI
#endif // PANEDIT
#endif // NAVIGATOR
#endif // PANDESK
#endif // GISARMY 
#endif // PANVECTOR
#endif // MEJDELO

#ifdef  LINUXAPI  
#define MAPACCLIB     "libqdmapacces.so"
#define MTRLIB        "libqdmapmtrex.so"
#define MAPTASKLIB    "libqdmaptask.so"
#define MAPSELECLIB   "libqdmapselec.so"      // 16/10/02
#define MAPOBJLIB     "libqdmapobj.so"        // 04/12/02
#define MAPPICTRLIB   "libqtmappictr.so"
#define MAPPASPLIB    "libqdmappasp.so"
#define MAPSHP2MAPLIB "libshptomap.so"     // 06/08/03
#define MAP2SHPLIB    "libqdmaptoshp.so"     // 09/10/03
#define MAPEDITLIB   "libqdmapedits.so"
#define MAPCOMPLIB   "libqdmapcomp.so"      // 29/03/02
#define MAPEDFRMLIB  "libqdmapedfrm.so"     // 10/04/02
#define MAPCOMMLIB   "libqdmapcomm.so"      // 14/04/02
#define MAPRCLIB     "libqdmaprc.so"        // 15/09/03
#define MAPQTFRMLIB  "libqdmapqtfrm.so"     // 12/05/02
#define MAPDBLIB     "libqdmapdb.so"        // 22/05/02
#define MAPPICEXLIB  "libqdmappicex.so"     // 20/09/02  Gorbunov
#define MAPVECEXLIB  "libqdmapvecex.so"     // 14/03/03
#define MAPRSCLIB    "librscedit.so"      // 15/04/03
#define MAP2MIFLIB    "libqdmaptomif.so"    // 23/07/03
#define MAPMIF2MAPLIB "libmiftomap.so"    // 06/08/03
#define MAPCNTEXLIB   "libqdmapcntex.so"    // 01/08/03
#define MAPVECTRLIB   "libqdmapvectr.so"    // 06/08/03
#define MAPS572MAPLIB "libs57tomap.so"    // 06/08/03
#define MAPS57LIB     "libs57tomap.so"    // 06/05/13
#define MAPDXF2MAPLIB "libdxftomap.so"    // 06/08/03
#define PREPRINTLIB   "libpreprint.so"    // 17/01/06
#define WMSACCESLIB   "libqdwmsacces.so"    // 12/09/11
#define MAPMATHLIB    "libqdmapmath.so"
#define MAPSCENALIB   "libqdmapscena.so" 
// 18/01/13 Библиотека удалена. #define JPGCOMPRLIB   "libjpgcompr.so"    // 02/12/11
#define JPGVIEWLIB    "libqdjpgview.so"     // 18/01/13
#define MAPLEGLIB     "libqdmapclass.so"    // 12/04/12
#define MAPRSWLIB     "librswedit.so"
#define GPSTESTLIB    "libgpstest.so"
#define EDITROADLIB   "libeditroad.so"
#define GEODESYLIB    "libgeodesy.so"
#define GEOEDITLIB    "libgeoedit.so"
#define MAP2D14LIB    "libqdmaptod14.so"
#define MTR2TXTLIB    "libmtr2txt.so"
#define GEOIMPRTLIB   "libgeoimpr.so"
#define GRDTOMTWLIB   "libgrdtomtw.so"
#define GEOCODELIB    "libgeocode.so"
#define MAPUPSEMEXL   "libupsemexl.so"
#define RSWGISTLIB    "librswgist.so"
#define DBCREATELIB   "libdbcreate.so"
#define MAPPRINTLIB   "libqdmapprintt.so"
#define CREATMTQLIB   "libcreatmtq.so"
#define MAP3DLIB      "libqdmap3d.so"         // 16/04/12
#define WCSACCESLIB   "libqdwcsacces.so"      // 30/07/13
#define GDALLIB       "libqdgdalserv.so"      // 24/10/13
#define GDALSERVLIB   "libqdgdalserv.so"
#define SID2RSWLIB    "libsid2rsw.so"
#define MODIFYLIB     "libmodify.so"
#define TRANSGEOLIB   "libtransgeo.so"
#define CALCMSKLIB    "libcalcmsk.so"
#define CALCDATMLIB   "libcalcdatm.so"
#define MP2MAPLIB     "libmp2map.so"
#define GML2MAPLIB    "libgml2map.so"
#define KML2MAPLIB    "libkml2map.so"
#define MAPOBSETLIB   "libqdmapobset.so"      // 26/10/13
#define MAPLUALIB     "libqdmaplua.so"
#endif  // LINUXAPI

#ifdef WIN32API
#ifdef GISWEB
#if defined(_M_X64) || defined(BUILD_DLL64)       // 30/12/11
  #define MAPACCLIB   "GISACWEB64.DLL"
  #define WMSACCESLIB "GISWEBWMSACCES64.DLL"
  #define GDALLIB     "GISU64GDAL111.DLL"        // 01/10/14
#else
  #define MAPACCLIB  "GISACWEB.DLL"
  #define WMSACCESLIB "GISWEBWMSACCES.DLL"
  #define GDALLIB     "GDAL111.DLL"              // 19/09/14
#endif
#else

#ifdef GISLIB
 #ifndef UNICODE11                       // 07/03/13
   #define MAPACCLIB    "GISACCES.DLL"
   #define MTRLIB       "GISMTREX.DLL"
   #define MAPVECEXLIB  "GISVECEX.DLL"
   #define MAPVECTRLIB  "GISVECTR.DLL"       // 01/07/10
   #define MTLLIB       "GISMTLEX.DLL"
   #define MAPPICEXLIB  "GISPICEX.DLL"
   #define MAPPICTRLIB  "GISFORMS.DLL"
   #define MAPRUSLIB    "GISFORMS.DLL"
   #define MAPENGLLIB   "GISFORME.DLL"
   #define MAPUALIB     "GISFORMS.DLL"
   #define MAPFRLIB     "GISFORMS.DLL"
   #define MAPESLIB     "GISFORMS.DLL"       // 16/09/15
   #define MAPOWLLIB    "GISFORMS.DLL"

   #define MAPSELECLIB  "GISSELEC.DLL"  // 24/09/2008
   #define MAPOBJLIB    "GISFORMS.DLL"
   #define MAPLISTLIB   "GISFORMS.DLL"
   #define MAPEDFRMLIB  "GISFORMS.DLL"  // 24/05/2006
   #define MAPEDFRM     "GISFORMS.DLL"  // 24/05/2006
   #define MAPPASPLIB   "GISPASP.DLL"   // 01/07/2010
   #define MAPRSCLIB    "GISFORMS.DLL"  // 21/08/2006
   #define MAPGPSLIB    "GISGPS.DLL"
   #define MAPOBJLIB    "GISFORMS.DLL"    // 02/05/2007
   #define MAPSCENALIB  "GISDLGS.DLL"     // 24/09/2008
   #define RSWGISTLIB   "GISRSWG.DLL"
   #define MAPPRINTLIB  "GISPRINT.DLL"    // 10/07/13
   #define MAPMATHLIB   "GISMATH.DLL"      // 06/07/11
   #define JPGVIEWLIB   "GISJPGVIEW.DLL"   // 18/01/13
   #define WMSACCESLIB  "GISWMSACCES.DLL"  // 08/12/11
   #define SID2RSWLIB   "GISSIDTORSW.DLL"  // 29/03/12
   #define GDALSERVLIB  "GISGDSRV.DLL"     // 04/10/12
   #define WFSACCESLIB  "GISWFSACCES.DLL"  // 29/03/13
   #define WCSACCESLIB  "GISWCSACCES.DLL"  // 07/04/13
   #define MAPLUALIB    "GISMAPLUA.DLL"    // 03/04/13
   #define MAP3DEXLIB   "GIS3D.DLL"        // 09/09/13
   #define GDALLIB      "GDAL111.DLL"      // 19/09/14
   #define MRSIDLIB     "LTI_DSDK_9.1.DLL" // 02/10/14
   #define MAPS57LIB    "giss57.dll"       // 18/12/15
   #define RSCTOOLSLIB  "gisrsctools.dll"  // 24/04/16
 #else

  #if defined(_M_X64) || defined(BUILD_DLL64)  // 21/10/15

  #ifdef BUILD_DLLVC64
   #define MAPACCLIB    "GISU64VCACCES.DLL"    // 26/08/13
   #define MTRLIB       "GISU64VCMTREX.DLL"    // 26/08/13
   #define MAPVECEXLIB  "GISU64VCVECEX.DLL"    // 26/08/13
   #define MAPVECTRLIB  "GISU64VCVECTR.DLL"    // 26/08/13
   #define MTLLIB       "GISU64VCMTLEX.DLL"    // 26/08/13
   #define MAPPICEXLIB  "GISU64VCPICEX.DLL"    // 26/08/13
   #define MAPPICTRLIB  "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPRUSLIB    "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPENGLLIB   "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPUALIB     "GISU64VCFORMS.DLL"    // 10/06/15
   #define MAPFRLIB     "GISU64VCFORMS.DLL"    // 10/06/15
   #define MAPESLIB     "GISU64VCFORMS.DLL"    // 16/09/15
   #define MAPOWLLIB    "GISU64VCFORMS.DLL"
   #define MAPSELECLIB  "GISU64VCSELEC.DLL"    // 26/08/13
   #define MAPOBJLIB    "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPLISTLIB   "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPEDFRMLIB  "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPEDFRM     "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPPASPLIB   "GISU64VCPASP.DLL"     // 26/08/13
   #define MAPRSCLIB    "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPGPSLIB    "GISU64VCGPS.DLL"      // 26/08/13
   #define MAPOBJLIB    "GISU64VCFORMS.DLL"    // 26/08/13
   #define MAPSCENALIB  "GISU64VCDLGS.DLL"     // 26/08/13
   #define RSWGISTLIB   "GISU64VCRSWG.DLL"     // 26/08/13
   #define MAPPRINTLIB  "GISU64VCPRINT.DLL"    // 26/08/13
   #define MAPMATHLIB   "GISU64VCMATH.DLL"     // 26/08/13
   #define JPGVIEWLIB   "GISU64VCJPGVIEW.DLL"  // 26/08/13
   #define WMSACCESLIB  "GISU64VCWMSACCES.DLL" // 26/08/13
   #define WFSACCESLIB  "GISU64VCWFSACCES.DLL" // 25/11/13
   #define WCSACCESLIB  "GISU64VCWCSACCES.DLL" // 28/08/13
   #define SID2RSWLIB   "GISU64VCSIDRW.DLL"    // 26/08/13
   #define GDALSERVLIB  "GISU64VCGDSRV.DLL"    // 26/08/13
   #define GPSTESTLIB   "GISU64VCGPS.DLL"      // 26/08/13
   #define MAP3DEXLIB   "GISU64VC3D.DLL"       // 09/09/13
   #define MAP3DLIB     "GISU64VC3D.DLL"       // 11/09/13
   #define ADDRESSLIB   "GISU64VCADDRESS.dll"  // 29/01/15
   #define GDALLIB      "GISU64VCGDAL111.DLL"  // 01/10/14
   #define MRSIDLIB     "LTI_DSDK_9.1.DLL"     // 02/10/14
   #define GRIDPVOLIB   "GISU64VCGRIDPVO.DLL"  // 07/09/15

#else

   #define MAPACCLIB    "GISU64ACCES.DLL"    // 26/08/13
   #define MTRLIB       "GISU64MTREX.DLL"    // 26/08/13
   #define MAPVECEXLIB  "GISU64VECEX.DLL"    // 26/08/13
   #define MAPVECTRLIB  "GISU64VECTR.DLL"    // 26/08/13
   #define MTLLIB       "GISU64MTLEX.DLL"    // 26/08/13
   #define MAPPICEXLIB  "GISU64PICEX.DLL"    // 26/08/13
   #define MAPPICTRLIB  "GISU64FORMS.DLL"    // 26/08/13
   #define MAPRUSLIB    "GISU64FORMS.DLL"    // 26/08/13
   #define MAPENGLLIB   "GISU64FORMS.DLL"    // 26/08/13
   #define MAPUALIB     "GISU64FORMS.DLL"    // 10/06/15
   #define MAPFRLIB     "GISU64FORMS.DLL"    // 10/06/15
   #define MAPESLIB     "GISU64FORMS.DLL"    // 16/09/15
   #define MAPOWLLIB    "GISU64FORMS.DLL"
   #define MAPSELECLIB  "GISU64SELEC.DLL"    // 26/08/13
   #define MAPOBJLIB    "GISU64FORMS.DLL"    // 26/08/13
   #define MAPLISTLIB   "GISU64FORMS.DLL"    // 26/08/13
   #define MAPEDFRMLIB  "GISU64FORMS.DLL"    // 26/08/13
   #define MAPEDFRM     "GISU64FORMS.DLL"    // 26/08/13
   #define MAPPASPLIB   "GISU64PASP.DLL"     // 26/08/13
   #define MAPRSCLIB    "GISU64FORMS.DLL"    // 26/08/13
   #define MAPGPSLIB    "GISU64GPS.DLL"      // 26/08/13
   #define MAPOBJLIB    "GISU64FORMS.DLL"    // 26/08/13
   #define MAPSCENALIB  "GISU64DLGS.DLL"     // 26/08/13
   #define RSWGISTLIB   "GISU64RSWG.DLL"     // 26/08/13
   #define MAPPRINTLIB  "GISU64PRINT.DLL"    // 26/08/13
   #define MAPMATHLIB   "GISU64MATH.DLL"     // 26/08/13
   #define JPGVIEWLIB   "GISU64JPGVIEW.DLL"  // 26/08/13
   #define WMSACCESLIB  "GISU64WMSACCES.DLL" // 26/08/13
   #define WFSACCESLIB  "GISU64WFSACCES.DLL" // 25/11/13
   #define WCSACCESLIB  "GISU64WCSACCES.DLL" // 28/08/13
   #define SID2RSWLIB   "GISU64SIDRW.DLL"    // 26/08/13
   #define GDALSERVLIB  "GISU64GDSRV.DLL"    // 26/08/13
   #define GPSTESTLIB   "GISU64GPS.DLL"      // 26/08/13
   #define MAP3DEXLIB   "GISU643D.DLL"       // 09/09/13
   #define MAP3DLIB     "GISU643D.DLL"       // 11/09/13
   #define ADDRESSLIB   "GISU64ADDRESS.dll"  // 29/01/15
   #define GDALLIB      "GISU64GDAL111.DLL"  // 01/10/14
   #define MRSIDLIB     "LTI_DSDK_9.1.DLL"   // 02/10/14
   #define GRIDPVOLIB   "GISU64GRIDPVO.DLL"  // 07/09/15
   #define MAPS57LIB    "gisu64s57.dll"      // 18/12/15
   #define RSCTOOLSLIB   "GISU64RSCTOOLS.DLL"// 21/04/16

#endif

  #else

   #define MAPACCLIB    "GISUACCES.DLL"    // 22/03/13
   #define MTRLIB       "GISUMTREX.DLL"    // 22/03/13
   #define MAPVECEXLIB  "GISUVECEX.DLL"    // 22/03/13
   #define MAPVECTRLIB  "GISUVECTR.DLL"    // 22/03/13
   #define MTLLIB       "GISUMTLEX.DLL"    // 22/03/13
   #define MAPPICEXLIB  "GISUPICEX.DLL"    // 22/03/13
   #define MAPPICTRLIB  "GISUFORMS.DLL"    // 22/03/13
   #define MAPRUSLIB    "GISUFORMS.DLL"    // 22/03/13
   #define MAPENGLLIB   "GISUFORMS.DLL"    // 22/03/13
   #define MAPUALIB     "GISUFORMS.DLL"    // 10/06/15
   #define MAPFRLIB     "GISUFORMS.DLL"    // 10/06/15
   #define MAPESLIB     "GISUFORMS.DLL"    // 16/09/15
   #define MAPSELECLIB  "GISUSELEC.DLL"    // 22/03/13
   #define MAPOBJLIB    "GISUFORMS.DLL"    // 22/03/13
   #define MAPLISTLIB   "GISUFORMS.DLL"    // 22/03/13
   #define MAPEDFRMLIB  "GISUFORMS.DLL"    // 22/03/13
   #define MAPEDFRM     "GISUFORMS.DLL"    // 22/03/13
   #define MAPOWLLIB    "GISUFORMS.DLL" 
   #define MAPPASPLIB   "GISUPASP.DLL"     // 22/03/13
   #define MAPRSCLIB    "GISUFORMS.DLL"    // 22/03/13
   #define MAPGPSLIB    "GISUGPS.DLL"      // 22/03/13
   #define MAPOBJLIB    "GISUFORMS.DLL"    // 22/03/13
   #define MAPSCENALIB  "GISUDLGS.DLL"     // 22/03/13
   #define RSWGISTLIB   "GISURSWG.DLL"     // 22/03/13
   #define MAPPRINTLIB  "GISUPRINT.DLL"    // 22/03/13
   #define MAPMATHLIB   "GISUMATH.DLL"     // 22/03/13
   #define JPGVIEWLIB   "GISUJPGVIEW.DLL"  // 22/03/13
   #define WMSACCESLIB  "GISUWMSACCES.DLL" // 22/03/13
   #define WFSACCESLIB  "GISUWFSACCES.DLL" // 25/11/13
   #define SID2RSWLIB   "GISUSIDRW.DLL"    // 22/03/13
   #define GDALSERVLIB  "GISUGDSRV.DLL"    // 22/03/13
   #define MAPLUALIB    "GISUMAPLUA.DLL"   // 03/04/13
   #define GPSTESTLIB   "GISUGPS.DLL"      // 26/08/13
   #define MAP3DEXLIB   "GISU3D.DLL"       // 09/09/13
   #define MAP3DLIB     "GISU3D.DLL"       // 11/09/13
   #define WCSACCESLIB  "GISUWCSACCES.DLL" // 11/09/13
   #define ADDRESSLIB   "GISUADDRESS.dll"  // 29/01/15
   #define GDALLIB      "GDAL111.DLL"      // 19/09/14
   #define MRSIDLIB     "LTI_DSDK_9.1.DLL" // 02/10/14
   #define GRIDPVOLIB   "GISUGRIDPVO.DLL"  // 07/09/15
   #define MAPS57LIB    "gisus57.dll"      // 18/12/15
   #define RSCTOOLSLIB  "GISURSCTOOLS.DLL" // 21/04/16

  #endif   // BUILD_DLL64                  // 26/08/13
 #endif   // UNICODE11                     

#else
  #if defined(_M_X64) || defined(BUILD_DLL64)     // 02/12/14
    #define MAPACCLIB     "MAPACCES64.DLL"
    #define MAPRUSLIB     "MAPRUS64.DLL"
    #define MAPENGLLIB    "MAPENGL64.DLL"
    #define MAPENGLLIBX   "MAPENGL64.DLL"
    #define MAPUALIB      "MAPUA64.DLL"
    #define MAPFRLIB      "MAPFR64.DLL"
    #define MAPESLIB      "MAPES64.DLL"           // 16/09/15
    #define MAPOWLLIB     "MAPOWL64.DLL"
    #define MAPTASKLIB    "MAPTASK64.DLL"
    #define MTRLIB        "MAPMTREX64.DLL"
    #define MTLBUILD      "MTLBUILD64.DLL"
    #define MAPSELECLIB   "MAPSELEC64.DLL"
    #define MAPOBJLIB     "MAPOBJ64.DLL"
    #define MAPPICTRLIB   "MAPLIST64.DLL"
    #define MAPPASPLIB    "MAPPASP64.DLL"
    #define MAPSHP2MAPLIB "SHPTOMAP64.DLL"
    #define MAP2SHPLIB    "MAPTOSHP64.DLL"
    #define MAPTREELIB   "MAPTREE64.DLL"
    #define MAPCOMMLIB   "MAPCOMM64.DLL"
    #define MAPEDITLIB   "MAPEDIT64.DLL"
    #define MAPCOMPLIB   "MAPCOMP64.DLL"
    #define MAPEDFRMLIB  "MAPLIST64.DLL"
    #define MAPDBLIB     "MAPDB64.DLL"
    #define MAPDBALIB    "MAPDBA64.DLL"        // 02/12/14
    #define MAPPICEXLIB  "MAPPICEX64.DLL"
    #define MAPVECEXLIB  "MAPVECEX64.DLL"
    #define MAPRSCLIB    "RSCEDIT64.DLL"
    #define MAP2MIFLIB    "MAPTOMIF64.DLL"
    #define MAPMIF2MAPLIB "MIFTOMAP64.DLL"
    #define MAPCNTEXLIB  "MAPVECEX64.DLL"
    #define MAPVECTRLIB  "MAPVECTR64.DLL"
    #define MAPS572MAPLIB "S57TOMAP64.DLL"      // 04/05/00
    #define MAPS57LIB     "MAPS5764.DLL"        // 24/01/06
    #define MAPDGN2MAPLIB "DGNTOMAP64.DLL"      // 23/03/04
    #define MAPDXF2MAPLIB "DXFTOMAP64.DLL"      // 10/04/00
    #define PREPRINTLIB   "PREPRINT64.DLL"      // 08/11/05

    #define MAPRSWLIB    "RSWEDIT64.DLL"

    #define MAPLEGLIB    "MAPCLASS64.DLL"
    #define MAPPOINTLIB  "MAPLIST64.DLL"
    #define MAPPRINTLIB  "MAPPRINT64.DLL"
    #define MAPLISTLIB   "MAPLIST64.DLL"
    #define MAPCONTRLIB  "MAPCONTR64.DLL"       //22.03.00
    #define MAPKWALIB    "MAPKWA64.DLL"         //22.03.00
    #define MAPTITLELIB  "MAPTITLE64.DLL"       //22.03.00

    #define MAP2D14LIB    "MAPTOD1464.DLL"      //04/05/00
    #define MAP2TIMLIB    "MAP2TIM64.DLL"
    #define RSWGISTLIB    "RSWGIST64.DLL"       // 15/12/02
    #define MAPSCENALIB   "MAPSCENA64.DLL"      // 28/01/04
    #define GEOEDITLIB    "GEOEDIT64.DLL"       // 02/02/04
    #define MAP3DLIB      "MAP3D64.DLL"         // 22/07/04
    #define MAPOBSETLIB   "MAPOBSET64.DLL"      // 22/10/04
    #define GEOIMPRTLIB   "GEOIMPRT64.DLL"      // 16/11/04
    #define MTR2TXTLIB    "MTR2TXT64.DLL"       // 06/12/04
    #define GRDTOMTWLIB   "GRDTOMTW64.DLL"      // 06/12/04
    #define GEOCODELIB    "GEOCODE64.DLL"       // 09/12/04
    #define GEODESYLIB    "GEODESY64.DLL"       // 14/03/05
    #define GPSPROTCLIB   "GPSPROTC64.DLL"      // 25/03/05
    #define TINBUILDLIB   "TINBUILD64.DLL"      // 12/04/05
    #define GEOLOADLIB    "GEOLOAD64.DLL"       // 09/06/05
    #define PREPRINTLIB   "PREPRINT64.DLL"      // 08/11/05
    #define GPSTESTLIB    "GPSTEST64.DLL"       // 25/01/06
    #define RSCVIEWLIB    "RSCVIEW64.DLL"       // 16/02/06
    #define CREATMTQLIB   "CREATMTQ64.DLL"      // 16/11/06
    #define MTLZONELIB    "MTLZONE64.DLL"
    #define OBJNETLIB     "OBJNET64.DLL"
    #define ISOL2MTWLIB   "ISOL2MTW64.DLL"
    #define SRTM2MTWLIB   "SRTM2MTW64.DLL"      // 29/11/06
    #define MTWCOMPLIB    "MTWCOMP64.DLL"       // 24/04/07
    #define MTWADJUSLIB   "MTWADJUS64.DLL"
    #define MAPHYDROLIB   "MAPHYDRO64.DLL"
    #define MTWSUMLIB     "MTWSUM64.DLL"
    #define MTWCUTLIB     "MTWCUT64.DLL"
    #define MASSESLIB     "MASSES64.DLL"
    #define EDITROADLIB   "EDITROAD64.DLL"      // 07/03/09
    #define MAPUPSEMEXL   "UPSEMEXL64.DLL"      // 22/07/09
    #define GLOBLOADLIB   "GLOBLOAD64.DLL"      // 03/06/10
    #define GEOLOGYLIB    "GEOLOGY64.DLL"       // 18/10/10
    #define MAPMASSESLIB  "MASSES64.DLL"        // 18/10/10
    #define MIFMIDLIB     "MIFTOMAP64.DLL"      // 03/06/11
    #define MAPMATHLIB    "MAPMATH64.DLL"       // 04/06/11
    #define JPGVIEWLIB    "JPGVIEW64.DLL"       // 18/01/13
    #define WMSACCESLIB   "WMSACCES64.DLL"      // 08/12/11
    #define WFSACCESLIB   "WFSACCES64.DLL"      // 29/03/13
    #define WCSACCESLIB   "WCSACCES64.DLL"      // 17/04/13
    #define AGROEDITLIB   "AGROEDIT64.DLL"      // 15/02/12
    #define SID2RSWLIB    "SID2RSW64.DLL"       // 29/03/12
    #define URBANLIB      "URBAN64.DLL"         // 01/06/12
    #define GDALSERVLIB   "GDALSERV64.DLL"      // 10/08/12
    #define MRSIDLIB      "LTI_DSDK_9.164.DLL"  // 19/09/14
    #define VECADAPTLIB   "MAPVECTR64.DLL"      // 25/10/12
    #define TMPLEDITLIB   "TMPLEDT64.DLL"       // 19/04/13
    #define RSC3DLIB      "RSC3D64.DLL"         // 07/03/13
    #define MAP3DEXLIB    "MAP3DEX64.DLL"       // 07/03/13
    #define MODIFYLIB     "MAPVECTR64.DLL"      
    #define TRANSGEOLIB   "TRANSGEO64.DLL"      // 07/03/13
    #define CALCMSKLIB    "CALCMSK64.DLL"       // 07/03/13
    #define CALCDATMLIB   "CALCDATM64.DLL"      // 07/03/13
    #define MAP2MPLIB     "MAP2MP64.DLL"        // 07/03/13
    #define MP2MAPLIB     "MP2MAP64.DLL"        // 07/03/13
    #define MAP2GMLLIB    "mapconv64.dll"       // 07/04/16
    #define GML2MAPLIB    "mapconv64.DLL"       // 07/04/16
    #define MAP2KMLLIB    "mapconv64.dll"  
    #define MAPCONVLIB    "mapconv64.dll"  
    #define KML2MAPLIB    "KML2MAP64.DLL"       // 07/03/13
    #define MAP2GDFLIB    "MAP2GDF64.DLL"       // 07/03/13
    #define GDF2MAPLIB    "GDF2MAP64.DLL"       // 07/03/13
    #define DEMO3DLIB     "DEMO3D64.DLL"        // 07/03/13
    #define RSCIMPLIB     "RSCIMP64.DLL"        // 07/03/13
    #define ADDTOMAPLIB   "MAPVECTR64.DLL"      // 07/03/13
    #define LOADEXLLIB    "LOADEXL64.DLL"       // 07/03/13
    #define TINBUILDLIB   "TINBUILD64.DLL"      // 07/03/13
    #define GPX2MAPLIB    "GPX2MAP64.DLL"       // 07/03/13
    #define TABLETLIB     "TABLET64.DLL"        // 07/03/13
    #define LOADWFSLIB    "LOADWFS64.DLL"       // 07/03/13
    #define GOOGVIEWLIB   "GOOGVIEW64.DLL"      // 07/03/13
    #define ZEMTOMAPLIB   "ZEMTOMAP64.DLL"      // 07/03/13
    #define MAPLUALIB     "MAPLUA64.DLL"        // 03/04/13
    #define GDALLIB       "GDAL11164.DLL"       // 19/09/14
    #define GEOPDFLIB     "GEOPDF64.DLL"        // 18/09/13
    #define SURVEYLIB     "survey64.DLL"        // 23/10/13
    #define CRYPTOECPLIB  "CryptoECP64.DLL"     // 23/10/13
    #define XSDCHECKLIB   "XSDCheck64.DLL"      // 16/12/13
    #define ADDRESSLIB    "address64.dll"       // 14/01/15
    #define MAPSOCKLIB    "MAPSOCK64.DLL"       // 05/02/15
    #define GRIDPVOLIB    "GRIDPVO64.DLL"       // 07/09/15
    #define REPEDITLIB    "REPEDIT64.DLL"       // 03/11/15
    #define MAPARINCLIB   "maparinc64.dll"      // 27/01/16
    #define TIFFPROFLIB   "TIFFPROF64.DLL"      // 27/01/16
    #define RSCTOOLSLIB   "RSCTOOLS64.DLL"      // 15/04/16
 #else
    #define MAPACCLIB     "MAPACCES.DLL"
    #define MAPRUSLIB     "MAPRUS.DLL"
    #define MAPENGLLIB    "MAPENGL.DLL"
    #define MAPENGLLIBX   "MAPENGL.DLL"
    #define MAPUALIB      "MAPUA.DLL"
    #define MAPFRLIB      "MAPFR.DLL"
    #define MAPESLIB      "MAPES.DLL"           // 16/09/15
    #define MAPOWLLIB     "MAPOWL.DLL"
    #define MAPTASKLIB    "MAPTASK.DLL"
    #define MTRLIB        "MAPMTREX.DLL"
    #define MTLBUILD      "MTLBUILD.DLL"
    #define MAPSELECLIB   "MAPSELEC.DLL"
    #define MAPOBJLIB     "MAPOBJ.DLL"
    #define MAPPICTRLIB   "MAPLIST.DLL"
    #define MAPPASPLIB    "MAPPASP.DLL"
    #define MAPSHP2MAPLIB "SHPTOMAP.DLL"
    #define MAP2SHPLIB    "MAPTOSHP.DLL"
    #define MAPTREELIB   "MAPTREE.DLL"
    #define MAPCOMMLIB   "MAPCOMM.DLL"
    #define MAPEDITLIB   "MAPEDIT.DLL"
    #define MAPCOMPLIB   "MAPCOMP.DLL"
    #define MAPEDFRMLIB  "MAPLIST.DLL"
    #define MAPDBLIB     "MAPDB.DLL"
    #define MAPDBALIB    "MAPDBA.DLL"         // 02/12/14
    #define MAPPICEXLIB  "MAPPICEX.DLL"
    #define MAPVECEXLIB  "MAPVECEX.DLL"
    #define MAPRSCLIB    "RSCEDIT.DLL"
    #define MAP2MIFLIB    "MAPTOMIF.DLL"
    #define MAPMIF2MAPLIB "MIFTOMAP.DLL"
    #define MAPCNTEXLIB   "MAPVECEX.DLL"
    #define MAPVECTRLIB   "MAPVECTR.DLL"
    #define MAPS572MAPLIB "S57TOMAP.DLL"      // 04/05/00
    #define MAPS57LIB     "MAPS57.DLL"        // 24/01/06
    #define MAPDGN2MAPLIB "DGNTOMAP.DLL"      // 23/03/04
    #define MAPDXF2MAPLIB "DXFTOMAP.DLL"      // 10/04/00
    #define PREPRINTLIB   "PREPRINT.DLL"      // 08/11/05

    #define MAPRSWLIB    "RSWEDIT.DLL"

    #define MAPLEGLIB    "MAPCLASS.DLL"
    #define MAPPOINTLIB  "MAPLIST.DLL"
    #define MAPPRINTLIB  "MAPPRINT.DLL"
    #define MAPLISTLIB   "MAPLIST.DLL"
    #define MAPCONTRLIB  "MAPCONTR.DLL"    
    #define MAPKWALIB    "MAPKWA.DLL"      
    #define MAPTITLELIB  "MAPTITLE.DLL"    

    #define MAP2D14LIB    "MAPTOD14.DLL"   
    #define MAP2TIMLIB    "MAP2TIM.DLL"
    #define RSWGISTLIB    "RSWGIST.DLL"
    #define MAPSCENALIB   "MAPSCENA.DLL"
    #define GEOEDITLIB    "GEOEDIT.DLL"
    #define MAP3DLIB      "MAP3D.DLL"  
    #define MAPOBSETLIB   "MAPOBSET.DLL"
    #define GEOIMPRTLIB   "GEOIMPRT.DLL"
    #define MTR2TXTLIB    "MTR2TXT.DLL" 
    #define GRDTOMTWLIB   "GRDTOMTW.DLL"
    #define GEOCODELIB    "GEOCODE.DLL" 
    #define GEODESYLIB    "GEODESY.DLL" 
    #define GPSPROTCLIB   "GPSPROTC.DLL"
    #define TINBUILDLIB   "TINBUILD.DLL"
    #define GEOLOADLIB    "GEOLOAD.DLL" 
    #define PREPRINTLIB   "PREPRINT.DLL"
    #define GPSTESTLIB    "GPSTEST.DLL" 
    #define RSCVIEWLIB    "RSCVIEW.DLL" 
    #define CREATMTQLIB   "CREATMTQ.DLL"
    #define MTLZONELIB    "MTLZONE.DLL"
    #define OBJNETLIB     "OBJNET.DLL"
    #define ISOL2MTWLIB   "ISOL2MTW.DLL"
    #define SRTM2MTWLIB   "SRTM2MTW.DLL"
    #define MTWCOMPLIB    "MTWCOMP.DLL" 
    #define MTWADJUSLIB   "MTWADJUS.DLL"
    #define MAPHYDROLIB   "MAPHYDRO.DLL"
    #define MTWSUMLIB     "MTWSUM.DLL"
    #define MTWCUTLIB     "MTWCUT.DLL"
    #define MASSESLIB     "MASSES.DLL"
    #define EDITROADLIB   "EDITROAD.DLL" 
    #define MAPUPSEMEXL   "UPSEMEXL.DLL" 
    #define GLOBLOADLIB   "GLOBLOAD.DLL" 
    #define GEOLOGYLIB    "GEOLOGY.DLL"  
    #define MAPMASSESLIB  "MASSES.DLL"   
    #define MIFMIDLIB     "MIFTOMAP.DLL" 
    #define MAPMATHLIB    "MAPMATH.DLL"  
    #define JPGVIEWLIB    "JPGVIEW.DLL"  
    #define WMSACCESLIB   "WMSACCES.DLL" 
    #define WFSACCESLIB   "WFSACCES.DLL" 
    #define WCSACCESLIB   "WCSACCES.DLL" 
    #define AGROEDITLIB   "AGROEDIT.DLL" 
    #define SID2RSWLIB    "SID2RSW.DLL"  
    #define URBANLIB      "URBAN.DLL"    
    #define GDALSERVLIB   "GDALSERV.DLL" 
    #define MRSIDLIB      "LTI_DSDK_9.1.DLL"  // 19/09/14
    #define VECADAPTLIB   "MAPVECTR.DLL" 
    #define TMPLEDITLIB   "TMPLEDT.DLL"       // 19/04/13
    #define RSC3DLIB      "RSC3D.DLL"         // 07/03/13
    #define MAP3DEXLIB    "MAP3DEX.DLL"       // 07/03/13
    #define MODIFYLIB     "MAPVECTR.DLL"  
    #define TRANSGEOLIB   "TRANSGEO.DLL"      // 07/03/13
    #define CALCMSKLIB    "CALCMSK.DLL"       // 07/03/13
    #define CALCDATMLIB   "CALCDATM.DLL"      // 07/03/13
    #define MAP2MPLIB     "MAP2MP.DLL"        // 07/03/13
    #define MP2MAPLIB     "MP2MAP.DLL"        // 07/03/13
    #define MAP2GMLLIB    "mapconv.dll"       // 07/04/16
    #define GML2MAPLIB    "mapconv.dll"       // 07/04/16
    #define MAP2KMLLIB    "mapconv.dll"
    #define MAPCONVLIB    "mapconv.dll"  
    #define KML2MAPLIB    "KML2MAP.DLL"       // 07/03/13
    #define MAP2GDFLIB    "MAP2GDF.DLL"       // 07/03/13
    #define GDF2MAPLIB    "GDF2MAP.DLL"       // 07/03/13
    #define DEMO3DLIB     "DEMO3D.DLL"        // 07/03/13
    #define RSCIMPLIB     "RSCIMP.DLL"        // 07/03/13
    #define ADDTOMAPLIB   "MAPVECTR.DLL"   
    #define LOADEXLLIB    "LOADEXL.DLL"       // 07/03/13
    #define TINBUILDLIB   "TINBUILD.DLL"      // 07/03/13
    #define GPX2MAPLIB    "GPX2MAP.DLL"       // 07/03/13
    #define TABLETLIB     "TABLET.DLL"        // 07/03/13
    #define LOADWFSLIB    "LOADWFS.DLL"       // 07/03/13
    #define GOOGVIEWLIB   "GOOGVIEW.DLL"      // 07/03/13
    #define ZEMTOMAPLIB   "ZEMTOMAP.DLL"      // 07/03/13
    #define MAPLUALIB     "MAPLUA.DLL"        // 03/04/13
    #define GDALLIB       "GDAL111.DLL"       // 19/09/14
    #define GEOPDFLIB     "GEOPDF.DLL"        // 18/09/13
    #define SURVEYLIB     "survey.dll"        // 23/10/13
    #define CRYPTOECPLIB  "CryptoECP.dll"     // 23/10/13
    #define XSDCHECKLIB   "XSDCheck.dll"      // 16/12/13
    #define ADDRESSLIB    "address.dll"       // 14/01/15
    #define MAPSOCKLIB    "MAPSOCK.DLL"       // 05/02/15
    #define MAPARINCLIB   "maparinc.dll"      // 25/06/15
    #define GRIDPVOLIB    "GRIDPVO.DLL"       // 07/09/15
    #define TIFFPROFLIB   "TIFFPROF.DLL"      // 21/09/15
    #define REPEDITLIB    "REPEDIT.DLL"       // 03/11/15
    #define RSCTOOLSLIB   "RSCTOOLS.DLL"      // 15/04/16
 #endif

#endif  // GISLIB
#endif  // GISWEB
#endif  // WIN32API

#endif // MAPLIB_H


