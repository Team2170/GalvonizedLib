/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI */

#ifndef _Included_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
#define _Included_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    createDetector
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_createDetector
  (JNIEnv *, jclass);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    destroyDetector
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_destroyDetector
  (JNIEnv *, jclass, jlong);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    setDetectorConfig
 * Signature: (JLBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagDetector/Config;)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_setDetectorConfig
  (JNIEnv *, jclass, jlong, jobject);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    getDetectorConfig
 * Signature: (J)LBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagDetector/Config;
 */
JNIEXPORT jobject JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_getDetectorConfig
  (JNIEnv *, jclass, jlong);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    setDetectorQTP
 * Signature: (JLBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagDetector/QuadThresholdParameters;)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_setDetectorQTP
  (JNIEnv *, jclass, jlong, jobject);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    getDetectorQTP
 * Signature: (J)LBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagDetector/QuadThresholdParameters;
 */
JNIEXPORT jobject JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_getDetectorQTP
  (JNIEnv *, jclass, jlong);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    addFamily
 * Signature: (JLjava/lang/String;I)Z
 */
JNIEXPORT jboolean JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_addFamily
  (JNIEnv *, jclass, jlong, jstring, jint);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    removeFamily
 * Signature: (JLjava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_removeFamily
  (JNIEnv *, jclass, jlong, jstring);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    clearFamilies
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_clearFamilies
  (JNIEnv *, jclass, jlong);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    detect
 * Signature: (JIIIJ)[LBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagDetection;
 */
JNIEXPORT jobjectArray JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_detect
  (JNIEnv *, jclass, jlong, jint, jint, jint, jlong);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    estimatePoseHomography
 * Signature: ([DDDDDD)Ledu/wpi/first/math/geometry/Transform3d;
 */
JNIEXPORT jobject JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_estimatePoseHomography
  (JNIEnv *, jclass, jdoubleArray, jdouble, jdouble, jdouble, jdouble, jdouble);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    estimatePoseOrthogonalIteration
 * Signature: ([D[DDDDDDI)LBobcatLib/Subsystems/Vision/Components/Utility/apriltag/AprilTagPoseEstimate;
 */
JNIEXPORT jobject JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_estimatePoseOrthogonalIteration
  (JNIEnv *, jclass, jdoubleArray, jdoubleArray, jdouble, jdouble, jdouble, jdouble, jdouble, jint);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    estimatePose
 * Signature: ([D[DDDDDD)Ledu/wpi/first/math/geometry/Transform3d;
 */
JNIEXPORT jobject JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_estimatePose
  (JNIEnv *, jclass, jdoubleArray, jdoubleArray, jdouble, jdouble, jdouble, jdouble, jdouble);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    generate16h5AprilTagImage
 * Signature: (Ledu/wpi/first/util/RawFrame;JI)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_generate16h5AprilTagImage
  (JNIEnv *, jclass, jobject, jlong, jint);

/*
 * Class:     BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI
 * Method:    generate36h11AprilTagImage
 * Signature: (Ledu/wpi/first/util/RawFrame;JI)V
 */
JNIEXPORT void JNICALL Java_BobcatLib_Subsystems_Vision_Components_Utility_apriltag_JNI_AprilTagJNI_generate36h11AprilTagImage
  (JNIEnv *, jclass, jobject, jlong, jint);

#ifdef __cplusplus
}
#endif
#endif
