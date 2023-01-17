#include <jni.h>
#include <string>
#include <cmath>
#include <android/bitmap.h>
#include <android/log.h>
#include "mapapi.h"

#define  LOG_TAG    "native-lib"
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


namespace {
    const HMAP INVALID_MAP_HANDLE = 0;
    const long int MAPAPI_ERROR = 0;

    struct Point {
        float x{std::numeric_limits<float>::quiet_NaN()}, y{std::numeric_limits<float>::quiet_NaN()};

        Point() = default;
        Point(float x, float y):
            x(x), y(y) {}

        bool isValid() const noexcept {
            return !(std::isnan(x) || std::isnan(y));
        }
    };

    struct GeoPoint {
        float b{std::numeric_limits<float>::quiet_NaN()}, l{std::numeric_limits<float>::quiet_NaN()};

        GeoPoint() = default;
        GeoPoint(float b, float l):
                b(b), l(l) {}

        bool isValid() const noexcept {
            return !(std::isnan(b) || std::isnan(l));
        }
    };

    struct Viewport {
        Point leftTop, rightBottom;

        bool isValid() const noexcept {
            return leftTop.isValid() && rightBottom.isValid();
        }
    };

    std::string string_from_jstr(JNIEnv *env, jstring jstr) {
        auto isCopy = jboolean(JNI_FALSE);
        auto strPtr = env->GetStringUTFChars(jstr, &isCopy);

        std::string ret(strPtr);

        if (isCopy == JNI_TRUE)
            env->ReleaseStringUTFChars(jstr, strPtr);

        return ret;
    }

    Point point_from_jobj(JNIEnv *env, jobject obj) {
        if (!obj)
            return {};

        jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapPoint");

        jmethodID getX = env->GetMethodID(clazz, "getX", "()F");
        jmethodID getY = env->GetMethodID(clazz, "getY", "()F");

        if (getX == nullptr || getY == nullptr)
            return {};

        float x = env->CallFloatMethod(obj, getX);
        float y = env->CallFloatMethod(obj, getY);

        return Point{x, y};
    }

    GeoPoint geopoint_from_jobj(JNIEnv *env, jobject obj) {
        if (!obj)
            return {};

        jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapGeoPoint");

        jmethodID getB = env->GetMethodID(clazz, "getB", "()F");
        jmethodID getL = env->GetMethodID(clazz, "getL", "()F");

        if (getB == nullptr || getL == nullptr)
            return {};

        float b = env->CallFloatMethod(obj, getB);
        float l = env->CallFloatMethod(obj, getL);

        return GeoPoint{b, l};
    }

    Viewport viewport_from_jobj(JNIEnv *env, jobject obj) {
        jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapViewport");

        jmethodID getLT = env->GetMethodID(clazz, "getLeftTop", "()Lru/radioavionica/drone/activities/MapPoint");
        jmethodID getRB = env->GetMethodID(clazz, "getRightBottom", "()Lru/radioavionica/drone/activities/MapPoint");

        if (getLT == nullptr || getRB == nullptr)
            return Viewport{};

        auto lt = env->CallObjectMethod(obj, getLT);
        auto rb = env->CallObjectMethod(obj, getRB);

        return {point_from_jobj(env, lt), point_from_jobj(env, rb)};
    }
}

extern "C" {

JNICALL JNIEXPORT jlong
Java_ru_radioavionica_drone_activities_MapActivity_openMapFile(JNIEnv *env,
                                                                    jobject /* this */,
                                                                    jstring filePath) {
    std::string path = string_from_jstr(env, filePath);

    auto ret = mapOpenData(path.c_str(), GENERIC_READ);
    LOGE("OPEN MAP %s, %i " , path.c_str(), ret);

    return ret;
}

JNICALL JNIEXPORT void
Java_ru_radioavionica_drone_activities_MapActivity_closeMap(JNIEnv *env,
                                                                 jobject /* this */,
                                                                 jlong handle) {
    mapCloseData(handle);
}

JNICALL JNIEXPORT jobject
Java_ru_radioavionica_drone_activities_MapActivity_getMapSize(JNIEnv *env,
                                                                   jobject javaThis,
                                                                   jlong handle)
{
    if (handle == INVALID_MAP_HANDLE)
        return nullptr;

    DFRAME df = {};

    if (mapGetTotalBorder(handle, &df, PP_PICTURE) == MAPAPI_ERROR)
        return nullptr;

    jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapSize");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FF)V");

    auto width  = float(df.X2 - df.X1),
         height = float(df.Y2 - df.Y1);

    jobject obj = env->NewObject(clazz, constructor, width, height);

    return obj;
}

JNICALL JNIEXPORT jboolean
Java_ru_radioavionica_drone_activities_MapActivity_renderViewport(JNIEnv *env,
                                                                       jobject javaThis,
                                                                       jlong handle,
                                                                       jobject pointobj,
                                                                       jobject bitmapobj)
{
    if (handle == INVALID_MAP_HANDLE || pointobj == nullptr || bitmapobj == nullptr)
        return JNI_FALSE;

    long rc = 0;
    AndroidBitmapInfo bitmapInfo;

    rc = AndroidBitmap_getInfo(env, bitmapobj, &bitmapInfo);

    if (rc < 0 || bitmapInfo.format != ANDROID_BITMAP_FORMAT_RGB_565) {
        LOGE("Invalid bitmap!");

        return JNI_FALSE;
    }

    Point leftTop = point_from_jobj(env, pointobj);

    if (!leftTop.isValid()) {
        LOGE("Invalid leftTop point!");

        return JNI_FALSE;
    }

    DFRAME df = {};

    if (mapGetTotalBorder(handle, &df, PP_PICTURE) == MAPAPI_ERROR)
        return JNI_FALSE;

    RECT viewPort;
    viewPort.left = leftTop.x;
    viewPort.top = leftTop.y;
    viewPort.right = leftTop.x + bitmapInfo.width;
    viewPort.bottom = leftTop.y + bitmapInfo.height;

    XIMAGEDESC ximage;
    ximage.Width = bitmapInfo.width;
    ximage.Height = bitmapInfo.height;
    ximage.Depth = 16;
    ximage.CellSize = ximage.Depth / 8;
    ximage.RowSize = ximage.Width * ximage.CellSize;

    if ((rc = AndroidBitmap_lockPixels(env, bitmapobj, (void**)&(ximage.Point))) < 0) {
        LOGE("AndroidBitmap_lockPixels() failed ! error=%ll", rc);

        return JNI_FALSE;
    }

    rc = mapPaintToXImage(handle, &ximage, 0, 0, &viewPort);

    AndroidBitmap_unlockPixels(env, bitmapobj);

    return (rc != MAPAPI_ERROR);
}

JNICALL JNIEXPORT jdouble
Java_ru_radioavionica_drone_activities_MapActivity_mapScaleUp(JNIEnv *env,
                                                                          jobject javaThis,
                                                                          jlong handle)
{
    if (handle == INVALID_MAP_HANDLE)
        return 0;

    auto currentScale = mapGetRealShowScale(handle);

    return mapSetRealShowScale(handle, currentScale * 2.0);
}

JNICALL JNIEXPORT jdouble
Java_ru_radioavionica_drone_activities_MapActivity_mapScaleDown(JNIEnv *env,
                                                                   jobject javaThis,
                                                                   jlong handle)
{
    if (handle == INVALID_MAP_HANDLE)
        return 0;

    auto currentScale = mapGetRealShowScale(handle);

    return mapSetRealShowScale(handle, currentScale / 2.0);
}

JNICALL JNIEXPORT jobject
Java_ru_radioavionica_drone_activities_MapActivity_mapXYtoGeo(JNIEnv *env,
                                                                   jobject javaThis,
                                                                   jlong handle,
                                                                   jobject pointXYobj)
{
    if (handle == INVALID_MAP_HANDLE || !mapIsGeoSupported(handle))
        return nullptr;

    auto pointXY = point_from_jobj(env, pointXYobj);

    if (!pointXY.isValid())
        return nullptr;

    double B = pointXY.x, L = pointXY.y; // so stupid...


    mapPictureToPlane(handle, &B, &L);
    mapPlaneToGeo(handle, &B, &L);

    jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapGeoPoint");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FF)V");

    auto Bf = float(B),
         Lf = float(L);

    return env->NewObject(clazz, constructor, Bf, Lf);
}

JNICALL JNIEXPORT jobject
Java_ru_radioavionica_drone_activities_MapActivity_mapXYtoGeoWGS84(JNIEnv *env,
                                                              jobject javaThis,
                                                              jlong handle,
                                                              jobject pointXYobj)
{
    if (handle == INVALID_MAP_HANDLE || !mapIsGeoSupported(handle))
        return nullptr;

    auto pointXY = point_from_jobj(env, pointXYobj);

    if (!pointXY.isValid())
        return nullptr;

    double B = pointXY.x, L = pointXY.y; // so stupid...


    mapPictureToPlane(handle, &B, &L);
    mapPlaneToGeoWGS84(handle, &B, &L);

    jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapGeoPoint");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FF)V");

    auto Bf = float(B),
         Lf = float(L);

    return env->NewObject(clazz, constructor, Bf, Lf);
}

JNICALL JNIEXPORT jobject
Java_ru_radioavionica_drone_activities_MapActivity_mapGeoToXY(JNIEnv *env,
                                                                   jobject javaThis,
                                                                   jlong handle,
                                                                   jobject pointGeoObj)
{
    if (handle == INVALID_MAP_HANDLE || !mapIsGeoSupported(handle))
        return nullptr;

    auto pointGeo = geopoint_from_jobj(env, pointGeoObj);

    if (!pointGeo.isValid())
        return nullptr;

    double B = pointGeo.b, L = pointGeo.l; // so stupid...

    mapGeoToPlane(handle, &B, &L);
    mapPlaneToPicture(handle, &B, &L);

    jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapPoint");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FF)V");

    auto Xf = float(B),
         Yf = float(L);

    return env->NewObject(clazz, constructor, Xf, Yf);
}

JNICALL JNIEXPORT jobject
Java_ru_radioavionica_drone_activities_MapActivity_mapGeoWGS84ToXY(JNIEnv *env,
                                                              jobject javaThis,
                                                              jlong handle,
                                                              jobject pointGeoObj)
{
    if (handle == INVALID_MAP_HANDLE || !mapIsGeoSupported(handle))
        return nullptr;

    auto pointGeo = geopoint_from_jobj(env, pointGeoObj);

    if (!pointGeo.isValid())
        return nullptr;

    double B = pointGeo.b, L = pointGeo.l, H = 0; // so stupid...

    mapGeoWGS84ToPlane3D(handle, &B, &L, &H);
    mapPlaneToPicture(handle, &B, &L);

    jclass clazz = env->FindClass("ru/radioavionica/drone/activities/MapPoint");
    jmethodID constructor = env->GetMethodID(clazz, "<init>", "(FF)V");

    auto Xf = float(B),
         Yf = float(L);

    return env->NewObject(clazz, constructor, Xf, Yf);
}

}


extern "C"
JNIEXPORT jlong JNICALL
Java_ru_radioavionica_drone_activities_AddRouteActivity_openMapFile(JNIEnv *env, jobject thiz,
                                                                    jstring file_path) {
    // TODO: implement openMapFile()
}