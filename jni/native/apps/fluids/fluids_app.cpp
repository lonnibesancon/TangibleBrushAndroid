#include "fluids_app.h"

#include "apps/app.h"

#include "vtk_output_window.h"
#include "vtk_error_observer.h"
#include "volume.h"
#include "volume3d.h"
#include "isosurface.h"
#include "slice.h"
#include "rendering/cube.h"
#include "rendering/rectangle.h"
#include "tracking/multi_marker.h"
#include "tracking/multi_marker_objects.h"
#include "loaders/loader_obj.h"
#include "rendering/mesh.h"
#include "rendering/lines.h"

#include <array>
#include <time.h> 

#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkDataSetReader.h>
#include <vtkXMLImageDataReader.h>
#include <vtkImageData.h>
#include <vtkImageResize.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkProbeFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <QCAR/QCAR.h>
#include <QCAR/CameraDevice.h>
#include <QCAR/Renderer.h>
#include <QCAR/VideoBackgroundConfig.h>
#include <QCAR/Trackable.h>
#include <QCAR/TrackableResult.h>
#include <QCAR/Tool.h>
#include <QCAR/Tracker.h>
#include <QCAR/TrackerManager.h>
#include <QCAR/ImageTracker.h>
#include <QCAR/CameraCalibration.h>
#include <QCAR/UpdateCallback.h>
#include <QCAR/DataSet.h>
#include "interactionMode.h"
// #include <QCAR/Image.h>

#define NEW_STYLUS_RENDER

// ======================================================================
// JNI interface

extern "C" {
	JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadDataset(JNIEnv* env, jobject obj, jstring filename);
	JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadVelocityDataset(JNIEnv* env, jobject obj, jstring filename);

	// JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_initQCAR(JNIEnv* env, jobject obj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_releaseParticles(JNIEnv* env, jobject obj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonPressed(JNIEnv* env, jobject obj);
	JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonReleased(JNIEnv* env, jobject obj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSettings(JNIEnv* env, jobject obj, jobject settingsObj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setSettings(JNIEnv* env, jobject obj, jobject settingsObj);

	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getState(JNIEnv* env, jobject obj, jobject stateObj);
	JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setTangoValues(JNIEnv* env, jobject obj, jdouble tx, jdouble ty, jdouble tz, jdouble rx, jdouble ry, jdouble rz, jdouble q);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setGyroValues(JNIEnv* env, jobject obj, jdouble rx, jdouble ry, jdouble rz, jdouble q);
    JNIEXPORT jstring JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getData(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setInteractionMode(JNIEnv* env, jobject obj, jint mode);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_updateFingerPositions(JNIEnv* env, jobject obj, jfloat x, jfloat y, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_addFinger(JNIEnv* env, jobject obj, jfloat x, jfloat y, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_removeFinger(JNIEnv* env, jobject obj, jint fingerID);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_reset(JNIEnv* env, jobject obj);
    JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_resetParticles(JNIEnv* env, jobject obj);

	//Data to send via computer
	JNIEXPORT jstring JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSelectionData(JNIEnv* env, jobject obj);
}

// (end of JNI interface)
// ======================================================================

struct Particle
{
	Vector3 pos;
	bool valid;
	int delayMs, stallMs;
	timespec lastTime;
};

struct FluidMechanics::Impl
{
	Impl(const std::string& baseDir);

	bool loadDataSet(const std::string& fileName);
	bool loadVelocityDataSet(const std::string& fileName);

	// void initQCAR();

	template <typename T>
	vtkSmartPointer<vtkImageData> loadTypedDataSet(const std::string& fileName);

	// (GL context)
	void rebind();

	// void detectObjects(ARMarkerInfo* markerInfo, int markerNum);
	void setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix);
	void updateSlicePlanes();
	void updateMatrices();

	// (GL context)
	void renderObjects();

	void updateSurfacePreview();

	void buttonPressed();
	float buttonReleased();

	void releaseParticles();
	Vector3 particleJitter();
	void integrateParticleMotion(Particle& p);

	bool computeCameraClipPlane(Vector3& point, Vector3& normal);
	bool computeAxisClipPlane(Vector3& point, Vector3& normal);
	bool computeStylusClipPlane(Vector3& point, Vector3& normal);

	void setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q);
	void setGyroValues(double rx, double ry, double rz, double q);
	std::string getData();
	void setInteractionMode(int mode);
	void updateFingerPositions(float x, float y, int fingerID);
	void addFinger(float x, float y, int fingerID);
	void removeFinger(int fingerID);
	void computeFingerInteraction();
	bool computeSeedingPlacement();
	void reset();
	int getFingerPos(int fingerID);
	void onTranslateBar(float pos);
	bool checkPosition();
	void resetParticles();

	Vector3 posToDataCoords(const Vector3& pos); // "pos" is in eye coordinates
	Vector3 dataCoordsToPos(const Vector3& dataCoordsToPos);
	Vector3 posToSliceCoords(const Vector3& pos);

	Quaternion currentSliceRot ;
	Vector3 currentSlicePos ;
	Quaternion currentDataRot ;
	Vector3 currentDataPos ;
	//Quaternion representing the orientation of the tablet no matter the interaction mode or constrains
	Quaternion currentTabRot ;

	Synchronized<std::vector<Vector3> > fingerPositions;
	Synchronized<std::vector<Vector3> > prevFingerPositions ;
	Synchronized<std::vector<std::vector<Vector3>>> movementPositions;
	bool newData;
	int lastFingerID;
	Vector3 lastSelection[2];

	std::vector<Quaternion> selectionRotMatrix; //All the quaternion rotation of the selection;
	std::vector<Vector3> selectionTransMatrix; //All the Vector3 translation of the selection;
	std::vector<Vector3> selectionLastPos; //All the last position during a selection;

	Matrix4 tangibleMatrix; //Matrix used for tangible selection

	Vector2 initialVector ;
	Vector3 prevVec ;
	float translateBarPrevPos ;
	bool isAboveThreshold = false ;
	bool mInitialPinchDistSet = false ;
	Synchronized<Vector3> seedingPoint ;
	float screenW = 1920 ;
	float screenH = 1104 ;
	float mInitialZoomFactor ;
	float mInitialPinchDist ;



	bool tangoEnabled = false ;
	int interactionMode = dataTangible ;
	bool seedPointPlacement = false ;



	FluidMechanics* app;
	std::shared_ptr<FluidMechanics::Settings> settings;
	std::shared_ptr<FluidMechanics::State> state;

	Synchronized<MultiMarker> tangible;
	Synchronized<MultiMarker> stylus;

	CubePtr cube, axisCube;

	vtkSmartPointer<vtkImageData> data, dataLow;
	int dataDim[3];
	Vector3 dataSpacing;

	vtkSmartPointer<vtkImageData> velocityData;

	typedef LinearMath::Vector3<int> DataCoords;
	Synchronized<std::array<Particle, 200>> particles;
	timespec particleStartTime;
	static constexpr float particleSpeed = 0.15f;
	static constexpr int particleReleaseDuration = 700; // ms
	static constexpr int particleStallDuration = 1000; // ms

	static constexpr float stylusEffectorDist = 24.0f;

	Synchronized<VolumePtr> volume;
	Synchronized<IsoSurfacePtr> isosurface, isosurfaceLow;
	Synchronized<SlicePtr> slice;
	Synchronized<CubePtr> outline;
	Vector3 slicePoint, sliceNormal;
	float sliceDepth;
	Synchronized<std::vector<Vector3>> slicePoints; // max size == 6

	MeshPtr particleSphere, cylinder;
	LinesPtr lines;

	Lines tangibleSelection;

	//Matching part
	std::vector<Matrix4> planeMatrices ;
	std::vector<Matrix4> dataMatrices ;
	int targetId = 0 ;

	vtkSmartPointer<vtkProbeFilter> probeFilter;

	Synchronized<Vector3> effectorIntersection;
	bool effectorIntersectionValid;

	bool buttonIsPressed;
};

FluidMechanics::Impl::Impl(const std::string& baseDir)
 : currentSliceRot(Quaternion(Vector3::unitX(), 0)),
   currentSlicePos(Vector3::zero()),
   currentDataPos(Vector3::zero()),
   currentDataRot(Quaternion(Vector3::unitX(), M_PI)),
   prevVec(Vector3::zero()),
   currentTabRot(Quaternion(Vector3::unitX(), M_PI)),
   buttonIsPressed(false) 
{
	seedingPoint = Vector3(-10000.0,-10000.0,-10000.0);
	cube.reset(new Cube);
	axisCube.reset(new Cube(true));
	particleSphere = LoaderOBJ::load(baseDir + "/sphere.obj");
	cylinder = LoaderOBJ::load(baseDir + "/cylinder.obj");
	lines.reset(new Lines);

	isAboveThreshold = false ;

	
	targetId = 0 ;
	for (Particle& p : particles)
		p.valid = false;

	tangibleSelection.setColor(Vector3(1.0, 1.0, 0.0));
	lastFingerID=-1;
	newData = false;
	tangibleMatrix = Matrix4::identity();
}

void FluidMechanics::Impl::reset(){
	seedingPoint = Vector3(-1,-1,-1);
	currentSliceRot = Quaternion(Vector3::unitX(), M_PI);
	currentDataRot = Quaternion(Vector3::unitX(), M_PI);
	currentSlicePos = Vector3(0, 0, 0);
	currentDataPos = Vector3(0,0,400);
	buttonIsPressed = false ;

	for (Particle& p : particles)
		p.valid = false;

	setMatrices(Matrix4::makeTransform(Vector3(0, 0, 400)),Matrix4::makeTransform(Vector3(0, 0, 400)));
	
}

bool FluidMechanics::Impl::checkPosition(){
	Matrix4 targetData = dataMatrices[targetId];
	Matrix4 targetSlice = planeMatrices[targetId];

	Vector3 posData(targetData[3][0],targetData[3][1],targetData[3][2]);
	Vector3 posSlice(targetSlice[3][0],targetSlice[3][1],targetSlice[3][2]);

	Vector3 diffData = posData - currentDataPos ;
	Vector3 diffSlice = posSlice - currentSlicePos ;
	float distData = sqrt(diffData.x * diffData.x + diffData.y * diffData.y + diffData.z * diffData.z);
	float distSlice = sqrt(diffSlice.x * diffSlice.x + diffSlice.y * diffSlice.y + diffSlice.z * diffSlice.z);

	LOGD("Distance to data target = %f",distData);
	LOGD("Distance to slice target = %f",distSlice);
}

void FluidMechanics::Impl::rebind()
{
	LOGD("OpenGL version: %s", glGetString(GL_VERSION));
	LOGD("GLSL version: %s", glGetString(GL_SHADING_LANGUAGE_VERSION));
	LOGD("OpenGL extensions: %s", glGetString(GL_EXTENSIONS));

	cube->bind();
	axisCube->bind();
	lines->bind();
	particleSphere->bind();
	cylinder->bind();

	synchronized_if(volume) { volume->bind(); }
	synchronized_if(isosurface) { isosurface->bind(); }
	synchronized_if(isosurfaceLow) { isosurfaceLow->bind(); }
	synchronized_if(slice) { slice->bind(); }
	synchronized_if(outline) { outline->bind(); }
}

template <typename T>
vtkSmartPointer<vtkImageData> FluidMechanics::Impl::loadTypedDataSet(const std::string& fileName)
{
	vtkNew<T> reader;

	LOGI("Loading file: %s...", fileName.c_str());
	reader->SetFileName(fileName.c_str());

	vtkNew<VTKErrorObserver> errorObserver;
	reader->AddObserver(vtkCommand::ErrorEvent, errorObserver.GetPointer());

	reader->Update();

	if (errorObserver->hasError()) {
		// TODO? Throw a different type of error to let Java code
		// display a helpful message to the user
		throw std::runtime_error("Error loading data: " + errorObserver->getErrorMessage());
	}

	vtkSmartPointer<vtkImageData> data = vtkSmartPointer<vtkImageData>::New();
	data->DeepCopy(reader->GetOutputDataObject(0));

	return data;
}

bool FluidMechanics::Impl::loadDataSet(const std::string& fileName)
{
	synchronized (particles) {
		// Unload velocity data
		velocityData = nullptr;

		// Delete particles
		for (Particle& p : particles)
			p.valid = false;
	}

	VTKOutputWindow::install();

	const std::string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "vtk")
		data = loadTypedDataSet<vtkDataSetReader>(fileName);
	else if (ext == "vti")
		data = loadTypedDataSet<vtkXMLImageDataReader>(fileName);
	else
		throw std::runtime_error("Error loading data: unknown extension: \"" + ext + "\"");

	data->GetDimensions(dataDim);

	double spacing[3];
	data->GetSpacing(spacing);
	dataSpacing = Vector3(spacing[0], spacing[1], spacing[2]);

	// Compute a default zoom value according to the data dimensions
	// static const float nativeSize = 128.0f;
	static const float nativeSize = 110.0f;
	state->computedZoomFactor = nativeSize / std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2]));
	// FIXME: hardcoded value: 0.25 (minimum zoom level, see the
	// onTouch() handler in Java code)
	state->computedZoomFactor = std::max(state->computedZoomFactor, 0.25f);

	dataLow = vtkSmartPointer<vtkImageData>::New();
	vtkNew<vtkImageResize> resizeFilter;
	resizeFilter->SetInputData(data.GetPointer());
	resizeFilter->SetOutputDimensions(std::max(dataDim[0]/3, 1), std::max(dataDim[1]/3, 1), std::max(dataDim[2]/3, 1));
	resizeFilter->InterpolateOn();
	resizeFilter->Update();
	dataLow->DeepCopy(resizeFilter->GetOutput());

	probeFilter = vtkSmartPointer<vtkProbeFilter>::New();
	probeFilter->SetSourceData(data.GetPointer());

	synchronized(outline) {
		LOGD("creating outline...");
		outline.reset(new Cube(true));
		outline->setScale(Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing);
	}

	synchronized(volume) {
		LOGD("creating volume...");
		volume.reset(new Volume(data));
		// volume.reset(new Volume3d(data));
		if (fileName.find("FTLE7.vtk") != std::string::npos) { // HACK
			// volume->setOpacity(0.25f);
			volume->setOpacity(0.15f);
		}
	}

	if (fileName.find("FTLE7.vtk") == std::string::npos) { // HACK
		synchronized(isosurface) {
			LOGD("creating isosurface...");
			isosurface.reset(new IsoSurface(data));
			isosurface->setPercentage(settings->surfacePercentage);
		}

		synchronized(isosurfaceLow) {
			LOGD("creating low-res isosurface...");
			isosurfaceLow.reset(new IsoSurface(dataLow, true));
			isosurfaceLow->setPercentage(settings->surfacePercentage);
		}
	} else {
		isosurface.reset();
		isosurfaceLow.reset();
	}

	synchronized(slice) {
		LOGD("creating slice...");
		slice.reset(new Slice(data));
	}

	return true;
}

bool FluidMechanics::Impl::loadVelocityDataSet(const std::string& fileName)
{
	if (!data)
		throw std::runtime_error("No dataset currently loaded");

	VTKOutputWindow::install();

	const std::string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "vtk")
		velocityData = loadTypedDataSet<vtkDataSetReader>(fileName);
	else if (ext == "vti")
		velocityData = loadTypedDataSet<vtkXMLImageDataReader>(fileName);
	else
		throw std::runtime_error("Error loading data: unknown extension: \"" + ext + "\"");

	int velocityDataDim[3];
	velocityData->GetDimensions(velocityDataDim);

	if (velocityDataDim[0] != dataDim[0]
	    || velocityDataDim[1] != dataDim[1]
	    || velocityDataDim[2] != dataDim[2])
	{
		throw std::runtime_error(
			"Dimensions do not match: "
			"vel: " + Utility::toString(velocityDataDim[0]) + "x" + Utility::toString(velocityDataDim[1]) + "x" + Utility::toString(velocityDataDim[2])
			+ ", data: " + Utility::toString(dataDim[0]) + "x" + Utility::toString(dataDim[1]) + "x" + Utility::toString(dataDim[2])
		);
	}

	int dim = velocityData->GetDataDimension();
	if (dim != 3)
		throw std::runtime_error("Velocity data is not 3D (dimension = " + Utility::toString(dim) + ")");

	if (!velocityData->GetPointData() || !velocityData->GetPointData()->GetVectors())
		throw std::runtime_error("Invalid velocity data: no vectors found");

	return true;
}

Vector3 FluidMechanics::Impl::posToSliceCoords(const Vector3& pos)
{
	return (app->getProjMatrix()).inverse().transformPos(pos, true);
/*  
	synchronized(state->modelMatrix) {
		// Transform "pos" into object space
		result = state->modelMatrix.inverse() * pos;
	}

	// Compensate for the scale factor
//	result *= 1/settings->zoomFactor;

	return result;
	*/
}

Vector3 FluidMechanics::Impl::posToDataCoords(const Vector3& pos)
{
	Vector3 result;

	synchronized(state->modelMatrix) {
		// Transform "pos" into object space
		result = state->modelMatrix.inverse() * pos;
	}

	// Compensate for the scale factor
	result *= 1/settings->zoomFactor;

	// The data origin is on the corner, not the center
	result += Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing;

	return result;
}

Vector3 FluidMechanics::Impl::particleJitter()
{
	return Vector3(
		(float(std::rand()) / RAND_MAX),
		(float(std::rand()) / RAND_MAX),
		(float(std::rand()) / RAND_MAX)
	) * 1.0f;
	// ) * 0.5f;
}

void FluidMechanics::Impl::buttonPressed()
{
	tangoEnabled = true ;

	//buttonIsPressed = true;
}

float FluidMechanics::Impl::buttonReleased()
{
	tangoEnabled = false ;
	return 0 ;
}

void FluidMechanics::Impl::resetParticles(){
	LOGD("Reset Particle case");
	for (Particle& p : particles) {
		p.pos = Vector3(0,0,0);
		p.delayMs = 0 ;
		p.stallMs = 0;
		p.valid = false ;
	}		
	seedingPoint = Vector3(-1000000,-1000000,-1000000);
}

void FluidMechanics::Impl::releaseParticles()
{
	Matrix4 smm;
	synchronized (state->stylusModelMatrix) {
		smm = state->stylusModelMatrix;
	}
	Vector3 dataPos = posToDataCoords(seedingPoint) ;
	if (dataPos.x < 0 || dataPos.y < 0 || dataPos.z < 0
	    || dataPos.x >= dataDim[0] || dataPos.y >= dataDim[1] || dataPos.z >= dataDim[2])
	{
		LOGD("outside bounds");
		seedPointPlacement = false ;
		return;
	}
	LOGD("Coords correct");
	DataCoords coords(dataPos.x, dataPos.y, dataPos.z);

	clock_gettime(CLOCK_REALTIME, &particleStartTime);

	int delay = 0;
	LOGD("Starting Particle Computation");
	synchronized (particles) {
		for (Particle& p : particles) {
			p.pos = Vector3(coords.x, coords.y, coords.z) + particleJitter();
			p.lastTime = particleStartTime;
			p.delayMs = delay;
			delay += particleReleaseDuration/particles.size();
			p.stallMs = 0;
			p.valid = true;
		}
	}

	seedPointPlacement = true ;
}

void FluidMechanics::Impl::integrateParticleMotion(Particle& p)
{
	if (!p.valid)
		return;

	// Pause particle motion when the data is not visible
	if (!state->tangibleVisible)
		return;

	timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	int elapsedMs = (now.tv_sec - p.lastTime.tv_sec) * 1000
		+ (now.tv_nsec - p.lastTime.tv_nsec) / 1000000;

	p.lastTime = now;

	if (p.delayMs > 0) {
		p.delayMs -= elapsedMs;
		if (p.delayMs < 0)
			elapsedMs = -p.delayMs;
		else
			return;
	}

	if (p.stallMs > 0) {
		p.stallMs -= elapsedMs;
		if (p.stallMs < 0)
			p.valid = false;
		return;
	}

	vtkDataArray* vectors = velocityData->GetPointData()->GetVectors();

	while (elapsedMs > 0) {
		--elapsedMs;

		DataCoords coords = DataCoords(p.pos.x, p.pos.y, p.pos.z);

		if (coords.x < 0 || coords.y < 0 || coords.z < 0
		    || coords.x >= dataDim[0] || coords.y >= dataDim[1] || coords.z >= dataDim[2])
		{
			// LOGD("particle moved outside bounds");
			p.valid = false;
			return;
		}

		double* v = vectors->GetTuple3(coords.z*(dataDim[0]*dataDim[1]) + coords.y*dataDim[0] + coords.x);

		Vector3 vel(v[1], v[0], v[2]); // XXX: workaround for a wrong data orientation

		if (!vel.isNull()) {
			p.pos += vel * particleSpeed;
		} else {
			// LOGD("particle stopped");
			p.stallMs = particleStallDuration;
			break;
		}
	}
}

bool FluidMechanics::Impl::computeCameraClipPlane(Vector3& point, Vector3& normal)
{
	static const float weight = 0.8f;
	static bool wasVisible = false;
	static Vector3 prevPos;

	if (!state->tangibleVisible) {
		wasVisible = false;
		return false;
	}

	Matrix4 slicingMatrix;
	slicingMatrix = Matrix4((app->getProjMatrix() * state->modelMatrix).inverse().get3x3Matrix());

	// Compute the slicing origin location in data coordinates:

	// Center of the screen (at depth "clipDist")
	Vector3 screenSpacePos = Vector3(0, 0, settings->clipDist);

	// Transform the position in object space
	Vector3 pos = state->modelMatrix.inverse() * screenSpacePos;

	// Transform the screen normal in object space
	Vector3 n = (state->modelMatrix.transpose().get3x3Matrix() * Vector3::unitZ()).normalized();

	// Filter "pos" using a weighted average, but only in the
	// "n" direction (the screen direction)
	// TODO: Kalman filter?
	if (wasVisible)
		pos += -n.project(pos) + n.project(pos*weight + prevPos*(1-weight));
	wasVisible = true;
	prevPos = pos;

	// Transform the position back in screen space
	screenSpacePos = state->modelMatrix * pos;

	// Store the computed depth
	sliceDepth = screenSpacePos.z;

	// Unproject the center of the screen (at the computed depth
	// "sliceDepth"), then convert the result into data coordinates
	Vector3 pt = app->getProjMatrix().inverse() * Vector3(0, 0, app->getDepthValue(sliceDepth));
	Vector3 dataCoords = posToDataCoords(pt);
	slicingMatrix.setPosition(dataCoords);

	synchronized(slice) {
		slice->setSlice(slicingMatrix, sliceDepth, settings->zoomFactor);
	}

	point = pt;
	normal = -Vector3::unitZ();

	return true;
}

bool FluidMechanics::Impl::computeAxisClipPlane(Vector3& point, Vector3& normal)
{
	if (state->tangibleVisible) {
		Matrix3 normalMatrix = state->modelMatrix.inverse().transpose().get3x3Matrix();
		float xDot = (normalMatrix*Vector3::unitX()).normalized().dot(Vector3::unitZ());
		float yDot = (normalMatrix*Vector3::unitY()).normalized().dot(Vector3::unitZ());
		float zDot = (normalMatrix*Vector3::unitZ()).normalized().dot(Vector3::unitZ());
		// Prevent back and forth changes between two axis (unless no
		// axis is defined yet)
		const float margin = (state->clipAxis != CLIP_NONE ? 0.1f : 0.0f);
		if (std::abs(xDot) > std::abs(yDot)+margin && std::abs(xDot) > std::abs(zDot)+margin) {
			state->clipAxis = (xDot < 0 ? CLIP_AXIS_X : CLIP_NEG_AXIS_X);
		} else if (std::abs(yDot) > std::abs(xDot)+margin && std::abs(yDot) > std::abs(zDot)+margin) {
			state->clipAxis = (yDot < 0 ? CLIP_AXIS_Y : CLIP_NEG_AXIS_Y);
		} else if (std::abs(zDot) > std::abs(xDot)+margin && std::abs(zDot) > std::abs(yDot)+margin) {
			state->clipAxis = (zDot < 0 ? CLIP_AXIS_Z : CLIP_NEG_AXIS_Z);
		}

		if (state->lockedClipAxis != CLIP_NONE) {
			Vector3 axis;
			ClipAxis neg;
			switch (state->lockedClipAxis) {
				case CLIP_AXIS_X: axis = Vector3::unitX(); neg = CLIP_NEG_AXIS_X; break;
				case CLIP_AXIS_Y: axis = Vector3::unitY(); neg = CLIP_NEG_AXIS_Y; break;
				case CLIP_AXIS_Z: axis = Vector3::unitZ(); neg = CLIP_NEG_AXIS_Z; break;
				case CLIP_NEG_AXIS_X: axis = -Vector3::unitX(); neg = CLIP_AXIS_X; break;
				case CLIP_NEG_AXIS_Y: axis = -Vector3::unitY(); neg = CLIP_AXIS_Y; break;
				case CLIP_NEG_AXIS_Z: axis = -Vector3::unitZ(); neg = CLIP_AXIS_Z; break;
				default: android_assert(false);
			}
			float dot = (normalMatrix*axis).normalized().dot(Vector3::unitZ());
			if (dot > 0)
				state->lockedClipAxis = neg;
		}

	} else {
		state->clipAxis = state->lockedClipAxis = CLIP_NONE;
	}

	const ClipAxis ca = (state->lockedClipAxis != CLIP_NONE ? state->lockedClipAxis : state->clipAxis);

	if (ca == CLIP_NONE)
		return false;

	Vector3 axis;
	Quaternion rot;
	switch (ca) {
		case CLIP_AXIS_X: axis = Vector3::unitX(); rot = Quaternion(Vector3::unitY(), -M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_AXIS_Y: axis = Vector3::unitY(); rot = Quaternion(Vector3::unitX(),  M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_AXIS_Z: axis = Vector3::unitZ(); rot = Quaternion::identity(); break;
		case CLIP_NEG_AXIS_X: axis = -Vector3::unitX(); rot = Quaternion(Vector3::unitY(),  M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_NEG_AXIS_Y: axis = -Vector3::unitY(); rot = Quaternion(Vector3::unitX(), -M_PI/2)*Quaternion(Vector3::unitZ(), M_PI); break;
		case CLIP_NEG_AXIS_Z: axis = -Vector3::unitZ(); rot = Quaternion(Vector3::unitX(),  M_PI); break;
		default: android_assert(false);
	}

	// Project "pt" on the chosen axis in object space
	Vector3 pt = state->modelMatrix.inverse() * app->getProjMatrix().inverse() * Vector3(0, 0, app->getDepthValue(settings->clipDist));
	Vector3 absAxis = Vector3(std::abs(axis.x), std::abs(axis.y), std::abs(axis.z));
	Vector3 pt2 = absAxis * absAxis.dot(pt);

	// Return to eye space
	pt2 = state->modelMatrix * pt2;

	Vector3 dataCoords = posToDataCoords(pt2);

	// static const float size = 128.0f;
	const float size = 0.5f * std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2]));

	Matrix4 proj = app->getProjMatrix(); proj[0][0] = -proj[1][1] / 1.0f; // same as "projMatrix", but with aspect = 1
	Matrix4 slicingMatrix = Matrix4((proj * Matrix4::makeTransform(dataCoords, rot)).inverse().get3x3Matrix());
	slicingMatrix.setPosition(dataCoords);
	synchronized(slice) {
		slice->setSlice(slicingMatrix, -proj[1][1]*size*settings->zoomFactor, settings->zoomFactor);
	}

	synchronized(state->sliceModelMatrix) {
		state->sliceModelMatrix = Matrix4(state->modelMatrix * Matrix4::makeTransform(state->modelMatrix.inverse() * pt2, rot, settings->zoomFactor*Vector3(size, size, 0.0f)));
	}

	if (!slice->isEmpty())
		state->lockedClipAxis = ca;
	else
		state->lockedClipAxis = CLIP_NONE;

	point = pt2;
	normal = state->modelMatrix.inverse().transpose().get3x3Matrix() * axis;

	return true;
}

// From: "Jittering Reduction in Marker-Based Augmented Reality Systems"
Matrix4 filter(const Matrix4& in, const Matrix4& prev, float posWeight, float rotWeight)
{
	// TODO: Kalman filter for position?

	Matrix4 result;

	for (unsigned int col = 0; col < 4; ++col) {
		for (unsigned int row = 0; row < 4; ++row) {
			if (row == 3) {
				// The last row is left unchanged
				result[col][row] = in[col][row];

			} else if (row == 0 && col < 3) {
				// Skip the first axis (side vector)
				continue;

			} else if (col < 3) { // orientation
				// Average the last 1/rotWeight values
				result[col][row] = in[col][row]*rotWeight + prev[col][row]*(1-rotWeight);

			} else { // position
				// Average the last 1/posWeight values
				result[col][row] = in[col][row]*posWeight + prev[col][row]*(1-posWeight);
			}
		}
	}

	Vector3 forward(result[0][2], result[1][2], result[2][2]);
	forward.normalize();
	result[0][2] = forward.x;
	result[1][2] = forward.y;
	result[2][2] = forward.z;

	Vector3 up(result[0][1], result[1][1], result[2][1]);
	up.normalize();

	// Recompute the side vector, then the up vector, to make sure the
	// coordinate system remains orthogonal

	Vector3 side = forward.cross(-up);
	side.normalize();
	result[0][0] = side.x;
	result[1][0] = side.y;
	result[2][0] = side.z;

	up = forward.cross(side);
	up.normalize();
	result[0][1] = up.x;
	result[1][1] = up.y;
	result[2][1] = up.z;

	return result;
}

bool FluidMechanics::Impl::computeStylusClipPlane(Vector3& point, Vector3& normal)
{
	if (!state->stylusVisible)
		return false;

	// FIXME: state->stylusModelMatrix may be invalid (non-invertible) in some cases
	try {

	const float size = 0.5f * (60.0f + std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2])));

	Matrix4 planeMatrix = state->stylusModelMatrix;

	Vector3 dataPosInStylusSpace = state->stylusModelMatrix.inverse() * state->modelMatrix * Vector3::zero();

	// The slice will be rendered from the viewpoint of the plane
	Matrix4 proj = app->getProjMatrix(); proj[0][0] = -proj[1][1] / 1.0f; // same as "projMatrix", but with aspect = 1
	Matrix4 slicingMatrix = Matrix4((proj * planeMatrix.inverse() * state->modelMatrix).inverse().get3x3Matrix());

	Vector3 pt2 = planeMatrix * Vector3::zero();

	// Position of the stylus tip, in data coordinates
	Vector3 dataCoords = posToDataCoords(pt2);
	slicingMatrix.setPosition(dataCoords);

	synchronized(slice) {
		slice->setSlice(slicingMatrix, -proj[1][1]*size*settings->zoomFactor, settings->zoomFactor);
	}

	synchronized(state->sliceModelMatrix) {
		state->sliceModelMatrix = Matrix4(planeMatrix * Matrix4::makeTransform(Vector3::zero(), Quaternion::identity(), settings->zoomFactor*Vector3(size, size, 0.0f)));
	}

	point = pt2;
	normal = state->stylusModelMatrix.inverse().transpose().get3x3Matrix() * Vector3::unitZ();

	} catch (const std::exception& e) { LOGD("%s", e.what()); return false; }

	return true;
}

Vector3 FluidMechanics::Impl::dataCoordsToPos(const Vector3& dataCoords)
{
	Vector3 result = dataCoords;

	// The data origin is on the corner, not the center
	result -= Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing;

	// Compensate for the scale factor
	result *= settings->zoomFactor;

	synchronized(state->modelMatrix) {
		// Transform "result" into eye space
		result = state->modelMatrix * result;
	}

	return result;
}

template <typename T>
T lowPassFilter(const T& cur, const T& prev, float alpha)
{ return prev + alpha * (cur-prev); }

void FluidMechanics::Impl::setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix)
{
	synchronized(state->modelMatrix) {
		state->modelMatrix = volumeMatrix;
	}

	synchronized(state->stylusModelMatrix) {
		state->stylusModelMatrix = stylusMatrix;
	}

	updateSlicePlanes();
}

void FluidMechanics::Impl::setInteractionMode(int mode){
	this->interactionMode = mode ;
	//Reinit selection values
	selectionRotMatrix.clear();
	selectionTransMatrix.clear();
	selectionLastPos.clear();

	currentSlicePos = Vector3::zero();
	currentSliceRot = Quaternion(Vector3::unitX(), 0);
}

void FluidMechanics::Impl::setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q){
	if(!data ){
		return ;
	}

	Vector3 vec(tx,ty,tz);

	if(tangoEnabled){
		//LOGD("Tango Enabled");
		Quaternion quat(rx,ry,rz,q);

		//LOGD("autoConstraint == %d",settings->autoConstraint);
		if(settings->autoConstraint){
			/*- n = normale du plan
			- v1 = ramener n dans le repère écran
			- v2 = ramener le déplacement de la tablette dans le repère écran
			- l = v2.length()
			- d = v1.normalized().dot(v2.normalized())
			- position du plan += n * l*d*/

			Vector3 trans = quat.inverse() * (vec-prevVec);
			float l = trans.length();
			float d = sliceNormal.normalized().dot(trans.normalized());
			trans = sliceNormal*l*d ;
			trans *= 300 ;
			trans *= -1 ;
			trans *= settings->precision ;
			trans.x *= settings->considerX ; //* settings->considerTranslation ;
			trans.y *= settings->considerY ; //* settings->considerTranslation ;
			trans.z *= settings->considerZ ; //* settings->considerTranslation ;

			printAny(sliceNormal,"SliceNormal = ");
			printAny(trans,"Trans = ");
			currentSlicePos += trans ;
			//LOGD("D = %f  --  L = %f",d,l);
			printAny(trans, "Trans: ");
		}
		
		else{
			//Normal interaction
			Vector3 trans = quat.inverse() * (vec-prevVec);
			trans *= Vector3(1,-1,-1);	//Tango... -_-"
			trans *= 300 ;
			//trans.z *= -1 ;
			trans *= settings->precision ;

			//To constrain interaction

			if( interactionMode == planeTangible ||
			    interactionMode == planeTouchTangible ||
			    interactionMode == dataPlaneTouchTangible)
			{

				//currentSlicePos += trans ;	Version with the plane moving freely in the world
				Vector3 cons(settings->considerX,settings->considerY,settings->considerZ);

				if(!(cons==Vector3(1,1,1)) ){
					Vector3 tmp = state->modelMatrix.get3x3Matrix() * cons ;
					trans = tmp.normalized() * trans.dot(tmp);
					printAny(cons, "CONS = ");
					printAny(tmp, "TMP = ");
					printAny(trans, "TRANS = ");
				}
				
				currentSlicePos += trans ; 	//Version with a fix plane
				tangibleMatrix = Matrix4::makeTransform(-currentSlicePos, -currentSliceRot);
				newData = true;
			}
			else if( interactionMode == dataTangible || 
				interactionMode == dataTouchTangible ||
				interactionMode == dataPlaneTangibleTouch)
			{
				trans.x *= settings->considerX ;//* settings->considerTranslation ;
				trans.y *= settings->considerY ;//* settings->considerTranslation ;
				trans.z *= settings->considerZ ; //* settings->considerTranslation ;

				currentDataPos +=trans ;
			}
		}
	}
	prevVec = vec ;

}

void FluidMechanics::Impl::setGyroValues(double rx, double ry, double rz, double q){
	if(!data){
		return ;
	}

	if(settings->considerX == 0 || settings->considerZ == 0 || settings->considerY == 0){
		return ;
	}

	//Now we update the rendering according to constraints and interaction mode
	rz *=settings->precision ;
	ry *=settings->precision ;
	rx *=settings->precision ;
	if(tangoEnabled){
	   if(  interactionMode == planeTangible ||
		    interactionMode == planeTouchTangible ||
		    interactionMode == dataPlaneTouchTangible)
		{
			Quaternion rot = currentSliceRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
			currentSliceRot = rot ;
			tangibleMatrix = Matrix4::makeTransform(-currentSlicePos, -currentSliceRot);
			newData = true;

		}
		else if( interactionMode == dataTangible || 
				interactionMode == dataTouchTangible ||
				interactionMode == dataPlaneTangibleTouch)
		{
			Quaternion rot = currentDataRot;
			rot = rot * Quaternion(rot.inverse() * (-Vector3::unitZ()), rz);
			rot = rot * Quaternion(rot.inverse() * -Vector3::unitY(), ry);
			rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), rx);
			currentDataRot = rot;
		}

		//Now for the automatic constraining of interaction
	}
	
}
//Code adapted from 
//http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection
bool intersectPlane(const Vector3& n, const Vector3& p0, const Vector3& l0, const Vector3& l, float& t) 
{ 
	// Here we consider that if the normal in the direction of the screen, there is no intersection
    // assuming vectors are all normalized
    float denom = n.dot(l);//dotProduct(n, l); 
    if (denom < 1e-6) { 
        Vector3 p0l0 = p0 - l0; 
        t = p0l0.dot(n)/denom ;
        return (t >= 0); 
    } 
 
    return false; 
} 

//Returns true if the seeding is successful
bool FluidMechanics::Impl::computeSeedingPlacement(){
	Vector2 currentPos ;

	//The seeding button is pressed but no finger on the data and there are already particles
	if(fingerPositions.size()!=0){
		synchronized(fingerPositions){
			currentPos = Vector2(fingerPositions[0].x,fingerPositions[0].y);
		}

		//Put screen coordinate between -1 and 1, and inverse Y to correspond to OpenGL conventions
		currentPos.x = (currentPos.x * 2 /screenW)-1 ;
		currentPos.y = (currentPos.y * 2 /screenH)-1 ;
		currentPos.y *= -1 ;

		//Get the world cornidates of the finger postion accoriding to the depth of the plane
		Vector3 ray(currentPos.x, currentPos.y, 1);
		ray = app->getProjMatrix().inverse() * ray ;
		ray.normalize();
		
		float t ;

		bool success = intersectPlane(sliceNormal, slicePoint, Vector3::zero(),ray, t) ;

		if(success){
			//To save energy with less rendering and computation, we do not render the particles on the tablet
			seedingPoint = ray*t ;
			releaseParticles();
			return true ;
		}
		else{
			//LOGD("FAIL");
			return false ;
			
		}
	}
	

}

void FluidMechanics::Impl::onTranslateBar(float pos){
}

void FluidMechanics::Impl::computeFingerInteraction(){
/*  Vector2 currentPos ;
	Vector2 prevPos ;
	if(fingerPositions.size() == 0){
		return ;
	}
	//Rotation case
	//For the plane, it gives both rotations AND translations, depending on the state of the button

	if(fingerPositions.size() == 1){
		synchronized(fingerPositions){
			currentPos = Vector2(fingerPositions[0].x,fingerPositions[0].y);
		}
		synchronized(prevFingerPositions){
			prevPos = Vector2(prevFingerPositions[0].x, prevFingerPositions[0].y);
		}

		Vector2 diff = currentPos - prevPos ;

		diff /=1000 ;
		diff *= settings->precision ;

		if( interactionMode == dataTouch || 
			interactionMode == dataTouchTangible ||
			interactionMode == dataPlaneTouchTangible)
		{
					Quaternion rot = currentDataRot;
					rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), 0);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), -diff.x);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), diff.y);
					currentDataRot = rot;
		}
		else if (interactionMode == planeTouch ||
			     interactionMode == planeTouchTangible ||
			     interactionMode == dataPlaneTangibleTouch)
		{
	  			Quaternion rot = currentSliceRot;
					rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), 0);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), -diff.x);
					rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), diff.y);
					currentSliceRot = rot;
					
		}
	}

	else if(fingerPositions.size() == 2){
		//LOGD("Two finger interaction");
		Vector2 diff = Vector2(0,0);

		float x1,x2,y1,y2 ;
		x1 = fingerPositions[0].x ;
		x2 = fingerPositions[1].x ;
		y1 = fingerPositions[0].y ;
		y2 = fingerPositions[1].y ;

		Vector2 newVec = Vector2(x2-x1,y2-y1);
		

		//Translation Computation

		for(int i = 0 ; i < 2 ; i++){
			synchronized(fingerPositions){
			currentPos = Vector2(fingerPositions[i].x,fingerPositions[i].y);
			}
			synchronized(prevFingerPositions){
				prevPos = Vector2(prevFingerPositions[i].x, prevFingerPositions[i].y);
			}

			diff += currentPos - prevPos ;
		}

	
	
		//FIXME Hardcoded
		diff /=2 ;
		diff /=4 ;
		diff *= settings->precision ;
		diff *= settings->considerTranslation * settings->considerX * settings->considerY;

		Vector3 trans = Vector3(diff.x, diff.y, 0);
		//LOGD("Diff = %f -- %f", diff.x, diff.y);
		
		//Compute distance between the two fingers
		float distance = sqrt(    (x1-x2)*(x1-x2) + (y1-y2) * (y1-y2)    );
		//LOGD("Distance %f", distance);
		if( interactionMode == planeTouch ||
			interactionMode == planeTouchTangible ||
			interactionMode == dataPlaneTangibleTouch)
		{
  		//We just translate along the z axis of the plane
			trans = Vector3(0,0,diff.x);		//Consider z vector only, on screen
			trans = state->stylusModelMatrix.get3x3Matrix() * trans ;	//Transform to plane's Z vector
			currentSlicePos +=trans ;
			LOGD("PLane Interaction Translation");

		}
		else if(interactionMode == dataTouch || 
			    interactionMode == dataTouchTangible ||
			    interactionMode == dataPlaneTouchTangible)
		{
			currentDataPos +=trans ;
		}

		//Rotation on the z axis --- Spinning 
		if(distance> thresholdRST || isAboveThreshold){
			isAboveThreshold = true ;
			
			//LOGD("Spinning considered");
			
	        
	        float dot = initialVector.x * newVec.x + initialVector.y * newVec.y ;
	        float det = initialVector.x * newVec.y - initialVector.y * newVec.x ;

	        float angle = atan2(det,dot);
	        //FIXME : to correct for the case when I remove one finger and put it back, rotate 360°
	        if(angle == 3.141593){	//FIXME hardcoded
	        	angle = 0 ;	
	        }

	        angle *=settings->precision ;
	        angle *= settings->considerZ * settings->considerRotation ;

	        //if(interactionMode == planeTouch){
	        if( interactionMode == planeTouch ||
			interactionMode == planeTouchTangible ||
			interactionMode == dataPlaneTouchTangible)
	        {
  			Quaternion rot = currentSliceRot;
				rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), angle);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), 0);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), 0);
				//currentSliceRot = rot ; //Version with the plane moving freely in the world
				currentSliceRot = rot ;
				
			}
			else if(interactionMode == dataTouch || 
			    	interactionMode == dataTouchTangible ||
			    	interactionMode == dataPlaneTouchTangible)
			{
				Quaternion rot = currentDataRot;
				rot = rot * Quaternion(rot.inverse() * Vector3::unitZ(), angle);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitY(), 0);
				rot = rot * Quaternion(rot.inverse() * Vector3::unitX(), 0);
				currentDataRot = rot;
			}


			//Scale case:
	        if(mInitialPinchDistSet == false){
	        	mInitialPinchDist = distance ;
	        	mInitialPinchDistSet = true ;
	        }
        	LOGD("Initial Pinch = %f, && zoom = %f",mInitialPinchDist,settings->zoomFactor);
            settings->zoomFactor = mInitialZoomFactor * distance/mInitialPinchDist;
            if (settings->zoomFactor <= 0.25f){
            	settings->zoomFactor = 0.25f;
            }
                
			//We set the initialVector to the new one, because relative mode
		}

		//Should be done no matter what is happening with the distance between the two fingers
		//Otherwise, spinning happens at one
		initialVector = newVec ;

	}
*/
}

void FluidMechanics::Impl::updateMatrices(){
	Matrix4 statem ;
	Matrix4 slicem ;
	
	if(settings->isSeeding && velocityData){
		//LOGD("Seeding Case");
		computeSeedingPlacement();
	}

	else if( interactionMode == dataTouch ||
		interactionMode == planeTouch ||
		interactionMode == dataTouchTangible ||
		interactionMode == planeTouchTangible ||
		interactionMode == dataPlaneTouchTangible ||
		interactionMode == dataPlaneTangibleTouch
		)
	{

			//LOGD("Interaction Needs touch");
			computeFingerInteraction();
	}
	slicem = Matrix4::makeTransform(currentSlicePos, currentSliceRot);	//Version with the plane moving freely
	statem = Matrix4::makeTransform(currentDataPos, currentDataRot);


	//First we update the slice
	//Plane moving freely
	synchronized(state->stylusModelMatrix) {
		state->stylusModelMatrix = slicem;
	}


	//Then the data
	synchronized(state->modelMatrix) {
		state->modelMatrix = statem ;
	}

	updateSlicePlanes();
}

std::string FluidMechanics::Impl::getData(){
	
  	std::ostringstream oss;
  	Matrix4 m ;

  	//First we set the zoomFactor
  	oss << settings->zoomFactor << ";" ;
  	int tmp = (settings->showVolume) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showSurface) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showStylus) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showSlice) ? 1 : 0 ;
  	oss << tmp << ";" ;
  	tmp = (settings->showOutline) ? 1 : 0 ;
  	oss << tmp << ";" ;

  	synchronized(state->modelMatrix){
  		m = state->modelMatrix ;	
  	}  	
	oss << m.data_[0] << ";" 
		<< m.data_[1] << ";" 
		<< m.data_[2] << ";" 
		<< m.data_[3] << ";" 
		<< m.data_[4] << ";" 
		<< m.data_[5] << ";" 
		<< m.data_[6] << ";" 
		<< m.data_[7] << ";" 
		<< m.data_[8] << ";" 
		<< m.data_[9] << ";" 
		<< m.data_[10] << ";" 
		<< m.data_[11] << ";" 
		<< m.data_[12] << ";" 
		<< m.data_[13] << ";" 
		<< m.data_[14] << ";" 
		<< m.data_[15] << ";" ;

	synchronized(state->stylusModelMatrix){
  		m = state->stylusModelMatrix ;	
  	}

	oss << m.data_[0] << ";" 
		<< m.data_[1] << ";" 
		<< m.data_[2] << ";" 
		<< m.data_[3] << ";" 
		<< m.data_[4] << ";" 
		<< m.data_[5] << ";" 
		<< m.data_[6] << ";" 
		<< m.data_[7] << ";" 
		<< m.data_[8] << ";" 
		<< m.data_[9] << ";" 
		<< m.data_[10] << ";" 
		<< m.data_[11] << ";" 
		<< m.data_[12] << ";" 
		<< m.data_[13] << ";" 
		<< m.data_[14] << ";" 
		<< m.data_[15] << ";" ;

	oss << seedingPoint.x << ";"
		<< seedingPoint.y << ";"  
		<< seedingPoint.z << ";" ;

	std::string s = oss.str();

	return s ;
}

void FluidMechanics::Impl::addFinger(float x, float y, int fingerID){
	Vector3 pos(x,y, fingerID);
	lastFingerID = fingerID;

	synchronized(prevFingerPositions){
		prevFingerPositions.push_back(pos);
	}
	synchronized(fingerPositions){
		fingerPositions.push_back(pos);
	}
	synchronized(movementPositions){
		movementPositions.push_back(std::vector<Vector3>(1, pos));
	}

	if(fingerPositions.size() == 2){
		float x1,x2,y1,y2 ;
		synchronized(fingerPositions){
			x1 = fingerPositions[0].x ;
			x2 = fingerPositions[1].x ;
			y1 = fingerPositions[0].y ;
			y2 = fingerPositions[1].y ;
		}
		initialVector = Vector2(x2-x1, y2-y1);
		float dx = x1  - x2;
        float dy = y1 - y2;
        float dist = sqrt(dx*dx + dy*dy);
        mInitialPinchDist = dist;
        mInitialZoomFactor = settings->zoomFactor;
	}
}
void FluidMechanics::Impl::removeFinger(int fingerID){
	lastFingerID=-1;
	int position = getFingerPos(fingerID);
	if(position == -1){
		return ;
	}

	if(getFingerPos(fingerID) == 0)
	{
		selectionRotMatrix.clear();
		selectionTransMatrix.clear();
		selectionLastPos.clear();
		currentSlicePos = Vector3::zero();
		currentSliceRot = Quaternion(Vector3::unitX(), 0);
	}
	//LOGD("Size = %d, position == %d", fingerPositions.size(), position);
	synchronized(prevFingerPositions){
		prevFingerPositions.erase(prevFingerPositions.begin()+position);
	}
	synchronized(fingerPositions){
		fingerPositions.erase(fingerPositions.begin()+position);
	}
	synchronized(movementPositions){
		movementPositions.erase(movementPositions.begin()+position);
	}

	//Reset all bools that are related to 2-finger interaction
	if(fingerPositions.size()<2){
		mInitialPinchDistSet = false ;
		isAboveThreshold = false ;
	}
}

int FluidMechanics::Impl::getFingerPos(int fingerID){
	for(int i = 0 ; i < fingerPositions.size() ; i++){
		if(fingerPositions[i].z == fingerID){
			return i ;
		}
	}
	return -1 ;
}


void FluidMechanics::Impl::updateFingerPositions(float x, float y, int fingerID){
	Vector3 pos(x,y, fingerID);
	
	int position = getFingerPos(fingerID);
	if(position == -1){
		//LOGD("Error in Finger ID");
		return ;
	}
	synchronized(prevFingerPositions){
		prevFingerPositions[position] = fingerPositions[position];	
	}
	synchronized(fingerPositions){
		fingerPositions[position] = pos ;	
	}

	synchronized(movementPositions){
		movementPositions[position].push_back(pos);
	}
}


void FluidMechanics::Impl::updateSlicePlanes()
{
	if(!data){
		return ;
	}
	if (state->stylusVisible) {
		if (state->tangibleVisible) { // <-- because of posToDataCoords()
			// Effector 2
			const float size = 0.5f * (stylusEffectorDist + std::max(dataSpacing.x*dataDim[0], std::max(dataSpacing.y*dataDim[1], dataSpacing.z*dataDim[2])));
			Vector3 dataPos = posToDataCoords(state->stylusModelMatrix * Matrix4::makeTransform(Vector3(-size, 0, 0)*settings->zoomFactor) * Vector3::zero());

			if (dataPos.x >= 0 && dataPos.y >= 0 && dataPos.z >= 0
			    && dataPos.x < dataDim[0]*dataSpacing.x && dataPos.y < dataDim[1]*dataSpacing.y && dataPos.z < dataDim[2]*dataSpacing.z)
			{
				const auto rayAABBIntersection = [](const Vector3& rayPoint, const Vector3& rayDir, const Vector3& aabbMin, const Vector3& aabbMax, float& tmin, float& tmax) -> bool {
					// http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-box-intersection/
					float tmin_ = (aabbMin.x - rayPoint.x) / rayDir.x;
					float tmax_ = (aabbMax.x - rayPoint.x) / rayDir.x;
					if (tmin_ > tmax_) std::swap(tmin_, tmax_);
					float tymin = (aabbMin.y - rayPoint.y) / rayDir.y;
					float tymax = (aabbMax.y - rayPoint.y) / rayDir.y;
					if (tymin > tymax) std::swap(tymin, tymax);
					if ((tmin_ > tymax) || (tymin > tmax_))
						return false;
					if (tymin > tmin_)
						tmin_ = tymin;
					if (tymax < tmax_)
						tmax_ = tymax;
					float tzmin = (aabbMin.z - rayPoint.z) / rayDir.z;
					float tzmax = (aabbMax.z - rayPoint.z) / rayDir.z;
					if (tzmin > tzmax) std::swap(tzmin, tzmax);
					if ((tmin_ > tzmax) || (tzmin > tmax_))
						return false;
					if (tzmin > tmin_)
						tmin_ = tzmin;
					if (tzmax < tmax_)
						tmax_ = tzmax;
					if ((tmin_ > tmax) || (tmax_ < tmin)) return false;
					if (tmin < tmin_) tmin = tmin_;
					if (tmax > tmax_) tmax = tmax_;
					// LOGD("tmin = %f, tmax = %f", tmin, tmax);
					return true;
				};

				// Same as posToDataCoords(), but for directions (not positions)
				// (direction goes from the effector to the stylus: +X axis)
				Vector3 dataDir = state->modelMatrix.transpose().get3x3Matrix() * state->stylusModelMatrix.inverse().transpose().get3x3Matrix() * Vector3::unitX();

				float tmin = 0, tmax = 10000;
				synchronized (effectorIntersection) {
					effectorIntersectionValid = false;
					if (rayAABBIntersection(dataPos, dataDir, Vector3::zero(), Vector3(dataDim[0], dataDim[1], dataDim[2])*dataSpacing, tmin, tmax) && tmax > 0) {
						effectorIntersection = dataCoordsToPos(dataPos + dataDir*tmax);
						effectorIntersectionValid = true;
					}
				}


				if (buttonIsPressed) {
					// settings->showSurface = true;
					settings->surfacePreview = true;

					vtkNew<vtkPoints> points;
					points->InsertNextPoint(dataPos.x, dataPos.y, dataPos.z);
					vtkNew<vtkPolyData> polyData;
					polyData->SetPoints(points.GetPointer());
					probeFilter->SetInputData(polyData.GetPointer());
					probeFilter->Update();

					vtkDataArray* scalars = probeFilter->GetOutput()->GetPointData()->GetScalars();
					android_assert(scalars);
					unsigned int num = scalars->GetNumberOfTuples();
					android_assert(num > 0);
					double value = scalars->GetComponent(0, 0);
					static double prevValue = 0.0;
					if (prevValue != 0.0)
						value = lowPassFilter(value, prevValue, 0.5f);
					prevValue = value;
					double range[2] = { volume->getMinValue(), volume->getMaxValue() };
					// LOGD("probed value = %f (range = %f / %f)", value, range[0], range[1]);
					settings->surfacePercentage = (value - range[0]) / (range[1] - range[0]);

					// NOTE: updateSurfacePreview cannot be used to update isosurfaceLow
					// from settings->surfacePercentage, since isosurfaceLow and isosurface
					// have different value ranges, hence expect different percentages.
					// updateSurfacePreview();

					// Directly use setValue() instead
					synchronized_if(isosurfaceLow) {
						isosurfaceLow->setValue(value);
					}
				}
			} else {
				effectorIntersectionValid = false;

				// // settings->showSurface = false;
				// settings->showSurface = true;
				// settings->surfacePreview = true;
			}
		}
	}

	// if (!state->tangibleVisible) // && !state->stylusVisible)
	// 	app->resetThreshold(); // reset the threshold in case its value went too far due

	bool clipPlaneSet = false;

	if (settings->showSlice && slice) {
		switch (settings->sliceType) {
			case SLICE_CAMERA:
				clipPlaneSet = computeCameraClipPlane(slicePoint, sliceNormal);
				break;

			case SLICE_AXIS:
				clipPlaneSet = computeAxisClipPlane(slicePoint, sliceNormal);
				break;

			case SLICE_STYLUS:
				clipPlaneSet = computeStylusClipPlane(slicePoint, sliceNormal);
				break;
		}
	}

	if (clipPlaneSet) {
		synchronized_if(isosurface) { isosurface->setClipPlane(sliceNormal.x, sliceNormal.y, sliceNormal.z, -sliceNormal.dot(slicePoint)); }
		synchronized_if(isosurfaceLow) { isosurfaceLow->setClipPlane(sliceNormal.x, sliceNormal.y, sliceNormal.z, -sliceNormal.dot(slicePoint)); }
		synchronized_if(volume) { volume->setClipPlane(sliceNormal.x, sliceNormal.y, sliceNormal.z, -sliceNormal.dot(slicePoint)); }

		// pt: data space
		// dir: eye space
		const auto rayPlaneIntersection = [this](const Vector3& pt, const Vector3& dir, float& t) -> bool {
			// float dot = dir.dot(posToDataCoords(sliceNormal));
			// float dot = dataCoordsToPos(dir).dot(sliceNormal);
			float dot = dir.dot(sliceNormal);
			if (dot == 0)
				return false;
			// t = -(pt.dot(posToDataCoords(sliceNormal)) - sliceNormal.dot(slicePoint)) / dot;
			t = -(dataCoordsToPos(pt).dot(sliceNormal) - sliceNormal.dot(slicePoint)) / dot;
			// t = -(pt.dot(sliceNormal) - sliceNormal.dot(slicePoint)) / dot;
			// LOGD("t = %f", t);
			return true;
		};


		// Slice-cube intersection
		float t;
		Vector3 dir;
		synchronized(slicePoints) {
			slicePoints.clear();

			static const float min = 0.0f;
			const float max = settings->zoomFactor;// * settings->zoomFactor;

			// Same as dataCoordsToPos(), but for directions (not positions)
			dir = state->modelMatrix.inverse().transpose().get3x3Matrix() * (Vector3(dataDim[0]*dataSpacing.x, 0, 0));// / settings->zoomFactor);

			if (rayPlaneIntersection(Vector3(0, 0, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, 0, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(0, dataDim[1]*dataSpacing.y, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, dataDim[1]*dataSpacing.y, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(0, 0, dataDim[2]*dataSpacing.z), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, 0, dataDim[2]*dataSpacing.z)) + dir*t);
			if (rayPlaneIntersection(Vector3(0, dataDim[1]*dataSpacing.y, dataDim[2]*dataSpacing.z), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, dataDim[1]*dataSpacing.y, dataDim[2]*dataSpacing.z)) + dir*t);

			dir = state->modelMatrix.inverse().transpose().get3x3Matrix() * (Vector3(0, dataDim[1]*dataSpacing.y, 0));// / settings->zoomFactor);
			if (rayPlaneIntersection(Vector3(0, 0, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, 0, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(dataDim[0]*dataSpacing.x, 0, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(dataDim[0]*dataSpacing.x, 0, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(0, 0, dataDim[2]*dataSpacing.z), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, 0, dataDim[2]*dataSpacing.z)) + dir*t);
			if (rayPlaneIntersection(Vector3(dataDim[0]*dataSpacing.x, 0, dataDim[2]*dataSpacing.z), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(dataDim[0]*dataSpacing.x, 0, dataDim[2]*dataSpacing.z)) + dir*t);

			dir = state->modelMatrix.inverse().transpose().get3x3Matrix() * (Vector3(0, 0, dataDim[2]*dataSpacing.z));// / settings->zoomFactor);
			if (rayPlaneIntersection(Vector3(0, 0, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, 0, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(dataDim[0]*dataSpacing.x, 0, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(dataDim[0]*dataSpacing.x, 0, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(0, dataDim[1]*dataSpacing.y, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(0, dataDim[1]*dataSpacing.y, 0)) + dir*t);
			if (rayPlaneIntersection(Vector3(dataDim[0]*dataSpacing.x, dataDim[1]*dataSpacing.y, 0), dir, t) && t >= min && t <= max)
				slicePoints.push_back(dataCoordsToPos(Vector3(dataDim[0]*dataSpacing.x, dataDim[1]*dataSpacing.y, 0)) + dir*t);

		}
	} else {
		synchronized_if(isosurface) { isosurface->clearClipPlane(); }
		synchronized_if(isosurfaceLow) { isosurfaceLow->clearClipPlane(); }
		synchronized_if(volume) { volume->clearClipPlane(); }
	}
}

// (GL context)
void FluidMechanics::Impl::renderObjects()
{
	//Comment this line to show the slice
	settings->showSlice = false;
	updateMatrices();
	//checkPosition();

	const Matrix4 proj = app->getProjMatrix() * tangibleMatrix;
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDepthMask(true); // requires "discard" in the shader where alpha == 0

	 if (settings->clipDist > 0.0f) {
		// Set a depth value for the slicing plane
		Matrix4 trans = Matrix4::identity();
		// trans[3][2] = app->getDepthValue(settings->clipDist); // relative to trans[3][3], which is 1.0
		trans[3][2] = app->getDepthValue(sliceDepth);
		// LOGD("%s", Utility::toString(trans).c_str());

		trans[1][1] *= -1; // flip the texture vertically, because of "orthoProjMatrix"

		synchronized(slice) {
			slice->setOpaque(false);
			slice->render(app->getOrthoProjMatrix(), trans);
		}
	}

	// Stylus (paddle?) z-buffer occlusion
	// TODO: correct occlusion shape for the real *stylus*
	if (false && state->stylusVisible && cube /*&& (settings->sliceType != SLICE_STYLUS || slice->isEmpty())*/) {
		glColorMask(false, false, false, false);
		glDepthMask(true);

			synchronized (state->stylusModelMatrix) {
				// if (!isRfduinoStylus) {
					cube->render(proj, state->stylusModelMatrix
					             * Matrix4::makeTransform(
						             Vector3(10.0, 0, 10.0),
						             Quaternion::identity(),
						             Vector3(59, 40, 3)/2
					             ));
					cube->render(proj, state->stylusModelMatrix
					             * Matrix4::makeTransform(
						             Vector3(10.0, -10.0, -5.0),
						             // Quaternion(Vector3::unitX(),  2.146),
						             Quaternion(Vector3::unitX(),  2.09),
						             Vector3(59, 40, 3)/2
					             ));
					cube->render(proj, state->stylusModelMatrix
					             * Matrix4::makeTransform(
						             Vector3(10.0, 10.0, -5.0),
						             // Quaternion(Vector3::unitX(), -2.146),
						             Quaternion(Vector3::unitX(), -2.09),
						             Vector3(59, 40, 3)/2
					             ));
					// Handle
					if (cylinder) {
						cylinder->render(proj, state->stylusModelMatrix
						                 * Matrix4::makeTransform(
							                 Vector3(75.0, 0.0, 0.0),
							                 Quaternion(Vector3::unitY(), M_PI/2),
							                 Vector3(0.01f, 0.01f, 0.017f)*2
						                 ));
					}
			}

		glColorMask(true, true, true, true);
	}

	if (settings->showStylus && state->stylusVisible && cube) {
		glDepthMask(true);
		glEnable(GL_CULL_FACE);

		Matrix4 smm;
		synchronized(state->stylusModelMatrix) {
			smm = state->stylusModelMatrix;
		}
	}

	if (state->tangibleVisible) {
		Matrix4 mm;
		synchronized(state->modelMatrix) {
			mm = state->modelMatrix;
		}

		// Apply the zoom factor
		mm = mm * Matrix4::makeTransform(
			Vector3::zero(),
			Quaternion::identity(),
			Vector3(settings->zoomFactor)
		);

		//Render the outline
		if(settings->showOutline){
			synchronized_if(outline) {
				glDepthMask(true);
				glLineWidth(2.0f);
				if(tangoEnabled && (interactionMode == dataTangible ||
									interactionMode == planeTangible ||
									interactionMode == dataTouchTangible ||
									interactionMode == planeTouchTangible ||
									interactionMode == dataPlaneTouchTangible ||
									interactionMode == dataPlaneTangibleTouch))
				{
					outline->setColor(Vector3(0, 1.0f, 0));
				}
				else{
					outline->setColor(Vector3(1.0f, 0, 0));	
				}
				outline->render(proj, mm);
			}
		}

		// Render the surface
		if (settings->showSurface) {
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
			glDepthMask(true);

			if (!settings->surfacePreview || !isosurfaceLow) {
				synchronized_if(isosurface) {
					isosurface->render(proj, mm);
				}
			}

			if (settings->surfacePreview) {
				synchronized_if(isosurfaceLow) {
					isosurfaceLow->render(proj, mm);
				}
			}
		}

		// Render particles
		synchronized (particles) {
			for (Particle& p : particles) {
				if (!p.valid)
					continue;
				integrateParticleMotion(p);
				if (!p.valid || p.delayMs > 0)
					continue;
				Vector3 pos = p.pos;
				pos -= Vector3(dataDim[0]/2, dataDim[1]/2, dataDim[2]/2) * dataSpacing;
				particleSphere->render(proj, mm * Matrix4::makeTransform(pos, Quaternion::identity(), Vector3(0.15f)));
			}
		}

		// NOTE: must be rendered before "slice" (because of
		// transparency sorting)
		if (settings->showSlice && state->clipAxis != CLIP_NONE && state->lockedClipAxis == CLIP_NONE) {
			Vector3 scale;
			Vector3 color;
			switch (state->clipAxis) {
				case CLIP_AXIS_X: case CLIP_NEG_AXIS_X: scale = Vector3(150, 0, 0); color = Vector3(1, 0, 0); break;
				case CLIP_AXIS_Y: case CLIP_NEG_AXIS_Y: scale = Vector3(0, 150, 0); color = Vector3(0, 1, 0); break;
				case CLIP_AXIS_Z: case CLIP_NEG_AXIS_Z: scale = Vector3(0, 0, 150); color = Vector3(0, 1, 1); break;
				case CLIP_NONE: android_assert(false);
			}

			const Matrix4 trans = Matrix4::makeTransform(
				Vector3::zero(),
				Quaternion::identity(),
				scale
			);

			glDepthMask(true);
			glLineWidth(5.0f);
			axisCube->setColor(color);
			axisCube->render(proj, mm*trans);
		}

		// FIXME: slight misalignment error?
		// Render the volume
		if (settings->showVolume) {// && sliceDot <= 0) {
			glEnable(GL_DEPTH_TEST);
			synchronized_if(volume) {
				glDepthMask(true);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // modulate
				glDisable(GL_CULL_FACE);
				volume->render(proj, mm);
			}
		}

		if (slice && settings->showSlice) {
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_CULL_FACE);
			glDepthMask(true); // requires "discard" in the shader where alpha == 0

			switch (settings->sliceType) {
				case SLICE_CAMERA: {
					if (settings->clipDist > 0.0f) {
						// Set a depth value for the slicing plane
						Matrix4 trans = Matrix4::identity();
						trans[3][2] = app->getDepthValue(sliceDepth);

						trans[1][1] *= -1; // flip the texture vertically, because of "orthoProjMatrix"

						synchronized(slice) {
							slice->setOpaque(false);
							slice->render(app->getOrthoProjMatrix(), trans);
						}
					}

					break;
				}

				case SLICE_AXIS:
				case SLICE_STYLUS: {
					if (settings->sliceType != SLICE_STYLUS || state->stylusVisible) {
						Matrix4 s2mm;
						synchronized(state->sliceModelMatrix) {
							s2mm = state->sliceModelMatrix;
						}

						synchronized(slice) {
							slice->setOpaque(true || slice->isEmpty() /*settings->sliceType == SLICE_STYLUS || slice->isEmpty()*/);
							slice->render(proj, s2mm);
						}

					}

					break;
				}
			}
		}

		//Render the rectangle selection
		if(interactionMode==planeTouch || interactionMode == planeTouchTangible)
		{
			synchronized(movementPositions)
			{
				int position = getFingerPos(lastFingerID);
				if(position != -1 && movementPositions[position].size() >= 2)
				{
					//Update all the selection
					
					Vector3 firstPos = movementPositions[position][0];
					firstPos.y *= -1;
					firstPos.y += screenH;
					firstPos -= Vector3(screenW/2.0, screenH/2.0, 0.0);
					firstPos   *= Vector3(2.0/screenW, 2.0/screenH, 1.0);

					Vector3 lastPos = movementPositions[position][movementPositions[position].size()-1];
					lastPos.y *= -1;
					lastPos.y += screenH;
					lastPos   -= Vector3(screenW/2.0, screenH/2.0, 0.0);
					lastPos   *= Vector3(2.0/screenW, 2.0/screenH, 1.0);

					Vector3 v  = posToSliceCoords(lastPos) - posToSliceCoords(firstPos);

					//Add what is needed to show the selection
					if(newData)
					{
						selectionRotMatrix.push_back(-currentSliceRot);
						selectionTransMatrix.push_back(-currentSlicePos);
						selectionLastPos.push_back(lastPos);
						LOGE("NEW DATA");
						newData = false;
					}

					/*  
					Matrix4 rectangleMat = Matrix4(Matrix4::identity());
					rectangleMat.setScale(2.0/screenW, 2.0/screenH, 1.0);
					rectangleLines.push_back(firstPos);
					rectangleLines.push_back(firstPos + Vector3(v.x, 0.0, 0.0));
					rectangleLines.push_back(firstPos + Vector3(v.x, 0.0, 0.0));
					rectangleLines.push_back(lastPos);
					rectangleLines.push_back(lastPos);
					rectangleLines.push_back(lastPos - Vector3(v.x, 0.0, 0.0));
					rectangleLines.push_back(lastPos - Vector3(v.x, 0.0, 0.0));
					rectangleLines.push_back(firstPos);
					lines->setLines(rectangleLines);

					lines->render(, Matrix4::identity());
					*/

					Rectangle* r = new Rectangle(v.x, v.y);
					r->render(app->getProjMatrix(), Matrix4(Matrix4::identity()).translate(posToSliceCoords(firstPos)));
					delete r;

/*  				for(uint32_t i=0; i < selectionRotMatrix.size(); i++)
					{
						selectionRect[i]->render(proj, Matrix4(Matrix4::identity()).translate(posToSliceCoords(firstPos)));//Matrix4::makeTransform(selectionTransMatrix[i] + posToSliceCoords(firstPos), selectionRotMatrix[i]));
					}
	*/
				}
			}
		}
	}
}


void FluidMechanics::Impl::updateSurfacePreview()
{
	if (!settings->surfacePreview) {
		synchronized_if(isosurface) {
			isosurface->setPercentage(settings->surfacePercentage);
		}
	} else if (settings->surfacePreview) {
		synchronized_if(isosurfaceLow) {
			isosurfaceLow->setPercentage(settings->surfacePercentage);
		}
	}
	if(settings->considerX+settings->considerY+settings->considerZ == 3){
		state->clipAxis = CLIP_NONE ;
	}

	else if(settings->considerX == 1 ){
		state->clipAxis = CLIP_AXIS_X ;
	}
	else if(settings->considerY == 1 ){
		state->clipAxis = CLIP_AXIS_Y ;
	}
	else if(settings->considerZ == 1 ){
		state->clipAxis = CLIP_AXIS_Z ;
	}
}

JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadDataset(JNIEnv* env,
	jobject obj, jstring filename)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		const char* javaStr = env->GetStringUTFChars(filename, nullptr);
		if (!javaStr)
			throw std::runtime_error("GetStringUTFChars() returned null");
		const std::string filenameStr(javaStr);
		env->ReleaseStringUTFChars(filename, javaStr);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->loadDataSet(filenameStr);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return false;
	}
}

JNIEXPORT jboolean JNICALL Java_fr_limsi_ARViewer_FluidMechanics_loadVelocityDataset(JNIEnv* env,
	jobject obj, jstring filename)
{
	try {
		// LOGD("(JNI) [FluidMechanics] loadVelocityDataSet()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		const char* javaStr = env->GetStringUTFChars(filename, nullptr);
		if (!javaStr)
			throw std::runtime_error("GetStringUTFChars() returned null");
		const std::string filenameStr(javaStr);
		env->ReleaseStringUTFChars(filename, javaStr);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->loadVelocityDataSet(filenameStr);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return false;
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_releaseParticles(JNIEnv* env, jobject obj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] releaseParticles()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->releaseParticles();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] getSettings()");

		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::Settings* settings = dynamic_cast<FluidMechanics::Settings*>(App::getInstance()->getSettings().get());
		android_assert(settings);
		return settings->read(env, settingsObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setSettings(JNIEnv* env, jobject obj, jobject settingsObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] setSettings()");

		if (!settingsObj)
			throw std::runtime_error("\"Settings\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(settingsObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::Settings* settings = dynamic_cast<FluidMechanics::Settings*>(App::getInstance()->getSettings().get());
		android_assert(settings);
		settings->write(env, settingsObj, cls);

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(settings);
		instance->updateSurfacePreview();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getState(JNIEnv* env, jobject obj, jobject stateObj)
{
	try {
		// LOGD("(JNI) [FluidMechanics] getState()");

		if (!stateObj)
			throw std::runtime_error("\"State\" object is null");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		jclass cls = env->GetObjectClass(stateObj);
		if (!cls)
			throw std::runtime_error("GetObjectClass() returned null");

		FluidMechanics::State* state = dynamic_cast<FluidMechanics::State*>(App::getInstance()->getState().get());
		android_assert(state);
		state->read(env, stateObj, cls);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonPressed(JNIEnv* env, jobject obj)
{
	try {
		 //LOGD("(JNI) [FluidMechanics] buttonPressed()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->buttonPressed();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setTangoValues(JNIEnv* env, jobject obj,jdouble tx, jdouble ty, jdouble tz, jdouble rx, jdouble ry, jdouble rz, jdouble q){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setTangoValues(tx,ty,tz,rx,ry,rz,q);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void JNICALL Java_fr_limsi_ARViewer_FluidMechanics_setGyroValues(JNIEnv* env, jobject obj, jdouble rx, jdouble ry, jdouble rz, jdouble q){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setGyroValues(rx,ry,rz,q);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT jfloat JNICALL Java_fr_limsi_ARViewer_FluidMechanics_buttonReleased(JNIEnv* env, jobject obj)
{
	try {
		 //LOGD("(JNI) [FluidMechanics] buttonReleased()");

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return instance->buttonReleased();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
		return 0.0f;
	}
}

JNIEXPORT jstring Java_fr_limsi_ARViewer_FluidMechanics_getData(JNIEnv* env, jobject obj)
{
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		return (env->NewStringUTF(instance->getData().c_str())) ;

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
	
}


JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_setInteractionMode(JNIEnv* env, jobject obj, jint mode){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->setInteractionMode(mode);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_updateFingerPositions(JNIEnv* env, jobject obj,jfloat x, jfloat y, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->updateFingerPositions(x,y,fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_addFinger(JNIEnv* env, jobject obj,jfloat x, jfloat y, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->addFinger(x,y,fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_reset(JNIEnv* env, jobject obj){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->reset();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_removeFinger(JNIEnv* env, jobject obj, jint fingerID){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->removeFinger(fingerID);

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}

JNIEXPORT jstring JNICALL Java_fr_limsi_ARViewer_FluidMechanics_getSelectionData(JNIEnv* env, jobject obj)
{
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);

		return env->NewStringUTF(instance->getSelectionData().c_str());

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}

	return NULL;
}

JNIEXPORT void Java_fr_limsi_ARViewer_FluidMechanics_resetParticles(JNIEnv* env, jobject obj){
	try {

		if (!App::getInstance())
			throw std::runtime_error("init() was not called");

		if (App::getType() != App::APP_TYPE_FLUID)
			throw std::runtime_error("Wrong application type");

		FluidMechanics* instance = dynamic_cast<FluidMechanics*>(App::getInstance());
		android_assert(instance);
		instance->resetParticles();

	} catch (const std::exception& e) {
		throwJavaException(env, e.what());
	}
}


FluidMechanics::FluidMechanics(const InitParams& params)
 : NativeApp(params, SettingsPtr(new FluidMechanics::Settings), StatePtr(new FluidMechanics::State)),
   impl(new Impl(params.baseDir)), indiceSelection(0)
{
	impl->app = this;
	impl->settings = std::static_pointer_cast<FluidMechanics::Settings>(settings);
	impl->state = std::static_pointer_cast<FluidMechanics::State>(state);
}

bool FluidMechanics::loadDataSet(const std::string& fileName)
{
	impl->setMatrices(Matrix4::makeTransform(Vector3(0, 0, 400)),
	                  Matrix4::makeTransform(Vector3(0, 0, 400)));
	impl->currentSlicePos = Vector3(0, 0, 400);
	impl->currentDataPos = Vector3(0,0,400);
	return impl->loadDataSet(fileName);
}

bool FluidMechanics::loadVelocityDataSet(const std::string& fileName)
{
	return impl->loadVelocityDataSet(fileName);
}

void FluidMechanics::releaseParticles()
{
	impl->releaseParticles();
}

void FluidMechanics::buttonPressed()
{
	impl->buttonPressed();
}

float FluidMechanics::buttonReleased()
{
	return impl->buttonReleased();
}

void FluidMechanics::rebind()
{
	NativeApp::rebind();
	impl->rebind();
}

void FluidMechanics::resetParticles(){
	impl->resetParticles();
}

void FluidMechanics::setMatrices(const Matrix4& volumeMatrix, const Matrix4& stylusMatrix)
{
	impl->setMatrices(volumeMatrix, stylusMatrix);
}

void FluidMechanics::renderObjects()
{
	impl->renderObjects();
}

void FluidMechanics::updateSurfacePreview()
{
	impl->updateSurfacePreview();
}

void FluidMechanics::reset(){
	impl->reset();
}


void FluidMechanics::setInteractionMode(int mode){
	indiceSelection = 0;
	impl->setInteractionMode(mode);
}

std::string FluidMechanics::getData(){
	return impl->getData();
}

void FluidMechanics::updateFingerPositions(float x, float y, int fingerID){
	impl->updateFingerPositions(x,y,fingerID);
}

void FluidMechanics::addFinger(float x, float y, int fingerID){
	impl->addFinger(x,y,fingerID);
}

void FluidMechanics::removeFinger(int fingerID){
	impl->removeFinger(fingerID);
}

void FluidMechanics::setTangoValues(double tx, double ty, double tz, double rx, double ry, double rz, double q){
	impl->setTangoValues(tx,ty,tz,rx,ry,rz,q);
}

void FluidMechanics::setGyroValues(double rx, double ry, double rz, double q){
	impl->setGyroValues(rx,ry,rz,q);
}

//get data to send via UDP
std::string FluidMechanics::getSelectionData()
{
	if(impl->interactionMode != planeTouchTangible)
		return "3";
	//2 is for adding array
	//3 is for cleaning the selection array
	//The array is : "2;firstPoint;matrix;lastPoint;matrix;lastPoint;...;
	
	int position = impl->getFingerPos(impl->lastFingerID);
	if(position == -1 || impl->movementPositions[position].size() <= 2 || indiceSelection > impl->selectionRotMatrix.size()-1)
		return " ";

	std::string data = "2;";
	Vector3 firstPos = impl->movementPositions[position][0];
	firstPos.y *= -1;
	firstPos.y += impl->screenH;
	firstPos -= Vector3(impl->screenW/2.0, impl->screenH/2.0, 0.0);
	firstPos   *= Vector3(2.0/impl->screenW, 2.0/impl->screenH, 1.0);

	char c[1024];
	sprintf(c, "%.2f;%.2f;", firstPos.x, firstPos.y);
	data += c;

	int i;
	for(i=indiceSelection; i < impl->selectionRotMatrix.size() && i < indiceSelection+1; i++)
	{
		Matrix4 m = Matrix4::makeTransform(getProjMatrix().inverse() * firstPos - impl->selectionTransMatrix[i], impl->selectionRotMatrix[i]);
		const float* mData = m.data_;
		for(uint32_t j=0; j < 16; j++)
		{
			sprintf(c, "%.2f", mData[j]);
			data += c;
		    data += ";";
		}

		sprintf(c, "%.2f;%.2f;", impl->selectionLastPos[i].x, impl->selectionLastPos[i].y);
		data += c;
	}
	if(i==indiceSelection)
		return " ";
	indiceSelection = i;
	return data;
}
