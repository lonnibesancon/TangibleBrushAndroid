#include "native_app.h"

#include "apps/app.h"
#include "thirdparty/yuv2rgb.h"
#include "rendering/surface_2d.h"

#include <cstdlib>
#include <cstring>
#include <time.h>

#include "AR/ar.h"
#include "AR/param.h"
#include "AR/gsub.h"

// ======================================================================
// JNI interface

extern "C" {
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_init(JNIEnv* env, jobject obj, jint appType, jstring baseDir); //, jint videoWidth, jint videoHeight);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_rebind(JNIEnv* env, jobject obj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_reshape(JNIEnv* env, jobject obj, jint width, jint height);

	// JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_processVideoFrame(JNIEnv* env, jobject obj, jbyteArray inArray);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_render(JNIEnv* env, jobject obj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_getSettings(JNIEnv* env, jobject obj, jobject settingsObj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_setSettings(JNIEnv* env, jobject obj, jobject settingsObj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_getState(JNIEnv* env, jobject obj, jobject stateObj);
}

// (end of JNI interface)
// ======================================================================

NativeApp::NativeApp(const InitParams& params)
 : settings(new Settings), state(new State)
{
	init(params);
}

NativeApp::NativeApp(const InitParams& params,
	SettingsPtr settings_, StatePtr state_)
 : settings(settings_), state(state_)
{
	init(params);
}

void NativeApp::init(const InitParams& params)
{
	android_assert(settings);
	android_assert(state);

	// Default values
	screenWidth = 0;
	screenHeight = 0;

	// Create an orthographic projection matrix
//	orthoProjMatrix = Matrix4::ortho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
	const float aspect = (float)screenWidth/screenHeight;
	projMatrix    = Matrix4::perspective(35.0f, float(screenWidth)/screenHeight, 50.0f, 2500.0f);
	orthoProjMatrix = Matrix4::ortho(-1.0f, 1.0f, -1/aspect, 1/aspect, 1.0f, -1.0f);
	orthoProjMatrix[1][1] *= -1;
	orthoProjMatrix[2][2] *= -1;
	orthoProjMatrix[2][3] *= -1;
}

NativeApp::~NativeApp()
{
}

// (GL context)
void NativeApp::rebind()
{
	// cameraSurface->bind();
}

// (GL context)
void NativeApp::reshape(unsigned int width, unsigned int height)
{
	android_assert(width > 0 && height > 0);
	screenWidth = width;
	screenHeight = height;
	LOGD("screen size: %d x %d", screenWidth, screenHeight);

	// Create a perspective projection matrix
	projMatrix    = Matrix4::perspective(35.0f, float(screenWidth)/screenHeight, 50.0f, 2500.0f);
//	scaleFrustumY = std::tan(35.0*0.5*M_PI/180)/2.0f;
//	scaleFrustumX = scaleFrustumY * float(screenWidth)/screenHeight;
	Vector3_f pN = projMatrix * Vector3_f(1.0, 1.0, 50.0);
	Vector3_f pF = projMatrix * Vector3_f(1.0, 1.0, 51.0);

	scaleFrustumX = (pN.x-pF.x)/pF.x/2.0;
	scaleFrustumY = (pN.y-pF.y)/pF.y/2.0;

	LOGE("frstrum %f, %f", scaleFrustumX, scaleFrustumY);
	LOGE("fX %f, %f", pN.x, pF.x);
	LOGE("fY %f, %f", pN.y, pF.y);
	zNear         = 50;

	float aspect = screenWidth / (float)screenHeight;
	projMatrix = Matrix4::ortho(-100.0f, 100.0f, -100.0f/aspect, 100.0f/aspect, 50.0f, 2500.0f);
	projMatrix[1][1] *= -1;
	projMatrix[2][2] *= -1;
	projMatrix[2][3] *= -1;

	projNearClipDist = -projMatrix[3][2] / (1+projMatrix[2][2]); // 50.0f
	projFarClipDist  =  projMatrix[3][2] / (1-projMatrix[2][2]); // 2500.0f

//	projMatrix = Matrix4::ortho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

	glViewport(0, 0, screenWidth, screenHeight);
}

void NativeApp::render()
{
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderObjects();
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_init(JNIEnv* env,
	jobject obj, jint appType, jstring baseDir) //, jint videoWidth, jint videoHeight)
{
	try {
		if (App::getInstance())
			throw std::runtime_error("init() was already called");

		const char* ptr = env->GetStringUTFChars(baseDir, nullptr);
		if (!ptr) throw std::runtime_error("GetStringUTFChars() returned null");
		const std::string baseDirStr(ptr);
		env->ReleaseStringUTFChars(baseDir, ptr);

		InitParams params;
		params.baseDir = baseDirStr;

		App::create(appType, params);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

// (GL context)
JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_rebind(JNIEnv* env, jobject obj)
{
	try {
		// LOGD("(JNI) rebind()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		// Forcefully rebind all GL objects, since the GL context may
		// have changed
		App::getInstance()->rebind();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

// (GL context)
JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_reshape(JNIEnv* env,
	jobject obj, jint width, jint height)
{
	try {
		// LOGD("(JNI) reshape(%d, %d)", width, height);

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (width <= 0 || height <= 0) {
			throw std::runtime_error("Invalid screen size: "
				"(" + Utility::toString(width) + "," + Utility::toString(height) + ")");
		}

		App::getInstance()->reshape(width, height);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

// (GL context)
JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_render(JNIEnv* env, jobject obj)
{
	try {
		// LOGD("(JNI) render()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		App::getInstance()->render();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_getSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		NativeApp::Settings* settings = App::getInstance()->getSettings().get();
		android_assert(settings);
		settings->read(env, settingsObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_setSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		// LOGD("(JNI) setSettings()");

		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		NativeApp::Settings* settings = App::getInstance()->getSettings().get();
		android_assert(settings);
		settings->write(env, settingsObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_NativeApp_getState(JNIEnv* env, jobject obj, jobject stateObj)
{
	try {
		// LOGD("(JNI) getState()");

		if (!stateObj)
			throw std::runtime_error("\"State\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		jclass cls = env->GetObjectClass(stateObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		NativeApp::State* state = App::getInstance()->getState().get();
		android_assert(state);
		state->read(env, stateObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}
