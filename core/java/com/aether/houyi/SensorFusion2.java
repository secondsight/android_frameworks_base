package com.aether.houyi;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.ServiceManager;
import android.os.Parcel;
import android.os.RemoteException;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.Window;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

public class SensorFusion2 implements SensorEventListener {

    // interface begin
    public float mAzimuth;
    public float mInclination;
    public HCamera mCam = new HCamera(new HVector(0, 0, 0), new HVector(0, 0, -1), 0);
    public float mInitInclination;
    public float mInitAzimuth;
    public boolean mOverrideTouchEvent = true;
    public float mFOV = 45;
    public float mDefZ = -2.4142f;
    public float mZScale = 0.8f;
    public float mCamRot = 0f;
    public float mCamDis = 0.13f;
    // interface end
    
    // private
    private static final int SENSOR_INIT_DELAY = 100;
    private static final int SENSOR_WAITING_COUNT = 300;
    
    private Activity mActivity;
    private SensorManager mSensorManager;

    // angular speeds from gyro
    private float[] gyro = new float[3];
 
    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];
 
    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];
 
    // magnetic field vector
    private float[] magnet = new float[3];
 
    // accelerometer vector
    private float[] accel = new float[3];
 
    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];
 
    // final orientation angles from sensor fusion
    private float[] fusedOrientationFinal = new float[3];
    private float[] fusedOrientation = new float[3];
 
    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];
    
    private static final float EPSILON = 0.000000001f;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private float timestamp;
    private boolean initState = true;
    
    private static final int TIME_CONSTANT = 10;
    private static final float FILTER_COEFFICIENT = 0.94f;
    private Timer fuseTimer = new Timer();
    private boolean mIsActive;
    
    private static final int CMD_UPDATE_SENSOR = 2000;
	
    private static final int INVALID_AZIMUTH = 1;
    private static final int IGNORING_AZIMUTH = 2;
    private static final int COLLECT_AZIMUTH = 3;

    private float[] mAcceleratorData = new float[3];
    private float[] mSensorMagData = new float[3];
    
    private float[] mRotationMatrix = new float[16];
    private float[] mFinalRotationMatrix = new float[16];
    private float[] mInclinationMatrix = new float[16];
    private float[] mFinalInclinationMatrix = new float[16];
    private float[] mDefOrientation = new float[3];//portrait orientation
    private float[] mDefOrientationFiltered = new float[3];
    private float[] mOrientation = new float[3];
    private float[] mQuaternion = new float[4];
    
    private float[] mProjectionMatrix = new float[16];
    private float[] mProjectionMatrixInv = new float[16];
    private float[] mViewMatrix = new float[16];
    private float[] mViewMatrixInv = new float[16];
    private float[] mMVPMatrix = new float[16];
    private float[] mBgMatrix = new float[16];
    private float[] mBgMatrixFlipped = new float[16];
    private float[] mTempMatrix = new float[16];
    private float[] mCamMat = new float[16];
    private float[] mResLookat = new float[4];
    private float[] mResUp = new float[4];
    private float[] mSrcLookat = {0, 0, -1, 1};
    private float[] mSrcUp = {0, 1, 0, 1};

    private boolean mHasSensor;
    private boolean mHasGravity;
    private boolean mMirrable = true;
    
    private float mDefInclination = (float)(90.0 * Math.PI / 180);
    private int mInitInclinationState = STATE_INVALID;
    private static final int STATE_INVALID = 1;
    private static final int STATE_IGNORE = 2;
    private static final int STATE_COLLECT = 3;
    private int mTick;
    private int mInitAzimuthState = STATE_INVALID;
    private float mAzimuthRange = (float)(30.0 * Math.PI / 180);
    private float mLastA;
    private float mLastI;
    
    private int mScreenRotation;
    private static SensorFusion2 mIns;
    
    public static synchronized SensorFusion2 getInstance(Activity activity) {
    	if (mIns == null) {
    		mIns = new SensorFusion2();
    	}
    	
    	mIns.mActivity = activity;
    	mIns.mSensorManager = (SensorManager)activity.getSystemService(Context.SENSOR_SERVICE);
    	return mIns;
    }
    
    private SensorFusion2() {
    }
    
    public float getAzimuth() {
        return fusedOrientationFinal[0];
    }
    
    public float getInclination() {
        return -fusedOrientationFinal[2];
    }    
    
    public void resetSenorInitialValues() {
    	Log.i("Lance", "resetSensor");
    	mInitAzimuthState = STATE_INVALID;
        mInitInclinationState = STATE_INVALID;
        mTick = 0;
        mDefOrientationFiltered = new float[3];
    }
    
    public void start() {
        if(mSensorManager != null){
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),
                    SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                    SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                    SensorManager.SENSOR_DELAY_FASTEST);
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                    SensorManager.SENSOR_DELAY_FASTEST);
            reset();
            resetSenorInitialValues();
            mIsActive = true;
        }
    }
    
    public void stop() {
        if(mSensorManager != null){
            mSensorManager.unregisterListener(this);
        }
        mIsActive = false;
    }
    
    public void reset() { 
        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;
 
        // initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f; gyroMatrix[1] = 0.0f; gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f; gyroMatrix[4] = 1.0f; gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f; gyroMatrix[7] = 0.0f; gyroMatrix[8] = 1.0f;      
    }
    
    public void setScreenRotation(int rotation) {
        if (mScreenRotation != rotation) {
            mScreenRotation = rotation;
            reset();
            resetSenorInitialValues();
        }        
    }

    public void dispatchTouchEvent(MotionEvent ev) {
    	// for testing app, set mOverrideTouchEvent false and override dispatchTouchEvent in Activity
        // if mirroring is turned off, no need to do anything
    	if (!mOverrideTouchEvent || !mMirrable || !mActivity.isSBSEnabled()) return;
    	
    	boolean print = ev.getAction() == MotionEvent.ACTION_DOWN;
    	float width = mActivity.getWindow().getDecorView().getWidth();
    	float height = mActivity.getWindow().getDecorView().getHeight();
		int o = mScreenRotation;
		
		float azimuth = mAzimuth;
		float initAzimuth = mInitAzimuth;
        float delAzimuth = azimuth - initAzimuth;

        float initInc = mInitInclination;
        float inclination = mInclination;
        float delInc = inclination - initInc;
        if (o == Surface.ROTATION_270) {
        	delInc = -delInc;
        }
        if (o == Surface.ROTATION_90) {
        	delAzimuth = -delAzimuth;
        }
        
        if (print) Log.d("LanceT", "i = " + (inclination - initInc) + " a = " + (azimuth - initAzimuth));
        
    	Matrix.setIdentityM(mCamMat, 0);
    	Matrix.rotateM(mCamMat, 0, (float)(delInc * 180 / Math.PI), 1, 0, 0);
        Matrix.rotateM(mCamMat, 0, (float)(-delAzimuth * 180 / Math.PI), 0, 1, 0);
        Matrix.multiplyMV(mResLookat, 0, mCamMat, 0, mSrcLookat, 0);
        Matrix.multiplyMV(mResUp, 0, mCamMat, 0, mSrcUp, 0);
        mCam.setLookAt(mResLookat[0], mResLookat[1], mResLookat[2]);
        mCam.setUp(mResUp[0], mResUp[1], mResUp[2]);
        
        mCam.update();
        
        float camDis = ev.getX() > width / 2 ? -mCamDis : mCamDis;
        float camRot = ev.getX() > width / 2 ? mCamRot : -mCamRot;
        mDefZ = (float)(- 1 / Math.tan(mFOV/2 * Math.PI / 180));
        Matrix.setIdentityM(mTempMatrix, 0);
        Matrix.translateM(mTempMatrix, 0, camDis, 0, mDefZ);
        Matrix.rotateM(mTempMatrix, 0, camRot, 0, 1, 0);
        Matrix.scaleM(mTempMatrix, 0, mZScale, mZScale/2, 1);
        
        float[] camMat = mCam.getViewMatrix();
        Matrix.multiplyMM(mViewMatrix, 0, camMat, 0, mTempMatrix, 0);
    	
        // get content view bound
    	float sx = ev.getX();
    	float sy = ev.getY();
        if (sx > width / 2) {
        	sx -= width / 2;
        }
        if (print) Log.d("LanceT", "w = " + width + " h = " + height + " sx = " + sx + " sy = " + sy);
        
        // generate perspective projection matrix
        Util.glhPerspectivef(mProjectionMatrix, mFOV, 1, 0.1f, 1000);
        
        float mirrorWidth = width / 2;
        float nx = 2 * sx / mirrorWidth - 1;
        float ny = 1 - 2 * sy / height;
        if (print) Log.d("LanceT", "nx = " + nx + " ny = " + ny);
        
        Matrix.invertM(mProjectionMatrixInv, 0, mProjectionMatrix, 0);
        HVector eye = HMatrix.multiply(mProjectionMatrixInv, nx, ny, 0);
        if (print) Log.i("LanceT", "eyex = " + eye.x + " eyey = " + eye.y + " eyez = " + eye.z);
        
        Matrix.invertM(mViewMatrixInv, 0, mViewMatrix, 0);
        HVector world = HMatrix.multiply(mViewMatrixInv, eye.x * mDefZ / (eye.z), eye.y * mDefZ / (eye.z), mDefZ);
        if (world != null) {
	        if (print) Log.w("LanceT", "worldx = " + world.x + " worldy = " + world.y + " worldz = " + world.z);
	        
	        float outx = (0.5f + world.x/2) * width;
	        float outy = (0.5f - world.y/2) * height;
	        if (print) Log.e("LanceT", "screenx = " + outx + " screeny = " + outy);
	        ev.setLocation(outx, outy);
        }
    }
    
    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
        case Sensor.TYPE_GRAVITY:
            System.arraycopy(event.values, 0, accel, 0, 3);
            calculateAccMagOrientation();
            mHasGravity = true;
            break;

        case Sensor.TYPE_ACCELEROMETER:
            if (!mHasGravity) {
                // copy new accelerometer data into accel array and calculate
                // orientation
                System.arraycopy(event.values, 0, accel, 0, 3);
                calculateAccMagOrientation();
            }
            break;

        case Sensor.TYPE_GYROSCOPE:
            // process gyro data
            gyroFunction(event);
            calculateFusedOrientation();
            notifySensorChanged();
            break;

        case Sensor.TYPE_MAGNETIC_FIELD:
            // copy new magnetometer data into magnet array
            System.arraycopy(event.values, 0, magnet, 0, 3);
            break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
    
    // calculates orientation angles from accelerometer and magnetometer output
    public void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }
    
    // This function is borrowed from the Android reference
    // at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
    // It calculates a rotation vector from the gyroscope angular speed values.
    private void getRotationVectorFromGyro(float[] gyroValues,
            float[] deltaRotationVector,
            float timeFactor)
    {
        float[] normValues = new float[3];
        
        // Calculate the angular speed of the sample
        float omegaMagnitude =
        (float)Math.sqrt(gyroValues[0] * gyroValues[0] +
        gyroValues[1] * gyroValues[1] +
        gyroValues[2] * gyroValues[2]);
        
        // Normalize the rotation vector if it's big enough to get the axis
        if(omegaMagnitude > EPSILON) {
        normValues[0] = gyroValues[0] / omegaMagnitude;
        normValues[1] = gyroValues[1] / omegaMagnitude;
        normValues[2] = gyroValues[2] / omegaMagnitude;
        }
        
        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }
    
    // This function performs the integration of the gyroscope data.
    // It writes the gyroscope based orientation into gyroOrientation.
    public void gyroFunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;
     
        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }
     
        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
	        System.arraycopy(event.values, 0, gyro, 0, 3);
	        getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }
     
        // measurement done, save current time for next interval
        timestamp = event.timestamp;
     
        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);
     
        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);
     
        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }
    
    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];
     
        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);
     
        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;
     
        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;
     
        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;
     
        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }
    
    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];
     
        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
     
        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
     
        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
     
        return result;
    }
    

    class calculateFusedOrientationTask extends TimerTask {
        public void run() {
        	calculateFusedOrientation();
        }
    }
    
    private void calculateFusedOrientation() {
    	int o = mActivity.getWindowManager().getDefaultDisplay().getRotation();
        setScreenRotation(o);
        
        float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
        
        /*
         * Fix for 179 <--> -179 transition problem:
         * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
         * If so, add 360 (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360 from the result
         * if it is greater than 180  This stabilizes the output in positive-to-negative-transition cases.
         */
        
        // azimuth
        if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
            fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]);
            fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
        }
        else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
            fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
            fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
        }
        else {
            fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
        }
        
        // pitch
        if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
            fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]);
            fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
        }
        else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
            fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
            fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
        }
        else {
            fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
        }
        
        // roll
        if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
            fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]);
            fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
        }
        else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
            fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
            fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
        }
        else {
            fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
        }
 
        // overwrite gyro matrix and orientation with fused orientation
        // to comensate gyro drift
        gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
        System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
        fusedOrientationFinal = lowPass(fusedOrientation.clone(), fusedOrientationFinal, 0.2f);
        
        setAzimuth(getAzimuth());
        setInclination(getInclination());
        
        mTick++;
    }
    
    private void setInclination(float inclination) {
        if (mScreenRotation == Surface.ROTATION_270) {
            // inclination = -inclination;
        }

        if (mInitInclinationState == STATE_INVALID) {
            if (mTick >= SENSOR_INIT_DELAY) {
                mInitInclinationState = STATE_IGNORE;
                mInitInclination = inclination;
                mInclination = inclination;
            }
        } else if (mInitInclinationState == STATE_IGNORE) { // Average next 10?
        	if (mTick > SENSOR_WAITING_COUNT) {
        		mInitInclinationState = STATE_COLLECT;	
        	} else {
        		mLastI = mInclination;
        		float del = inclination - mLastI;
        		if (Math.abs(del) < 0.03 || (inclination==0&&mInitInclination==0)) {
        			mInitInclination += del;
        		} else {
        			mInitInclinationState = STATE_COLLECT;
        			mInitAzimuthState = STATE_COLLECT;
        		}
        		mInclination = inclination;
        	}
        } else if (mInitInclinationState == STATE_COLLECT){
        	mLastI = mInclination;
    		float del = inclination - mLastI;
    		if (Math.abs(del) < 0.001) {
    			mInitInclination += del;
    		}
    		mInclination = inclination;
        }
        
//        Log.d("Lance", "inc = " + inclination + " init inc = " + mInitInclination);
    }
    
    private void setAzimuth(float azimuth) {
    	if (mScreenRotation == Surface.ROTATION_90) {
    		azimuth = -azimuth;
    	}
    	
    	if (mInitAzimuthState == STATE_INVALID) {
            if (mTick >= SENSOR_INIT_DELAY) {
                mInitAzimuthState = STATE_IGNORE;
                mInitAzimuth = azimuth;
                mAzimuth = azimuth;
            }
        } else if (mInitAzimuthState == STATE_IGNORE) { // Average next 10?
        	if (mTick > SENSOR_WAITING_COUNT) {
        		mInitAzimuthState = STATE_COLLECT;	
        	} else {
        		mLastA = mAzimuth;
        		float del = azimuth - mLastA;
        		if (Math.abs(del) < 0.03 || (azimuth==0&&mInitAzimuth==0)) {
        			mInitAzimuth += del;
        		} else {
        			mInitAzimuthState = STATE_COLLECT;
        			mInitInclinationState = STATE_COLLECT;
        		}
        		mAzimuth = azimuth;
        	}
        } else if (mInitAzimuthState == STATE_COLLECT){
        	mLastA = mAzimuth;
    		float del = azimuth - mLastA;
    		if (Math.abs(del) < 0.001) {
    			mInitAzimuth += del;
    		}
    		mAzimuth = azimuth;
        }
        
//        if (Math.abs(mInitAzimuth - azimuth) > Math.PI / 4) {
//            Log.e("Lance", "a = " + azimuth + " init a = " + mInitAzimuth);
//        }
    }
    
    private float clampInclination(float input, float range) {
    	float res = input - mDefInclination;
    	if (res > range) {
    		res = range;
    	} else if (res < -range) {
    		res = -range;
    	}
    	return res;
    }
    
    private void notifySensorChanged() {
        try {
        	if (mTick < SENSOR_INIT_DELAY) {
        		return;
        	}
        	
    		float azimuth = mAzimuth;
            float delAzimuth = (float)(azimuth - mInitAzimuth);
            float min = (float)clampBetweenZeroAnd2PI(mInitAzimuth - mAzimuthRange);
            float max = (float)clampBetweenZeroAnd2PI(mInitAzimuth + mAzimuthRange);
            float clampedAzimuth = delAzimuth;
            
//            if (max > min) { // Normal case
//            	float absAngleMin = (float)getSmallerAngle(azimuth, min);
//            	float absAngleMax = (float)getSmallerAngle(azimuth, max);
//            	float minRefAzimuth = (float)((azimuth > max && absAngleMin < absAngleMax) ? azimuth - PI2 : azimuth);
//            	float maxRefAzimuth = (float)((azimuth < min && absAngleMax < absAngleMin) ? azimuth + PI2 : azimuth);
//            	if (minRefAzimuth < min && absAngleMin < absAngleMax) {
//            		mInitAzimuth -= absAngleMin; 
////            		Log.d(TAG, "A " + mInitAzimuth + "; min/max = " + min + ", " + max);
//            	} else if (maxRefAzimuth > max && absAngleMax < absAngleMin) {
//            		mInitAzimuth += absAngleMax; 
////            		Log.d(TAG, "B " + mInitAzimuth + "; min/max = " + min + ", " + max);
//            	}
//            } else if (azimuth > max && azimuth < min){
//            	if (azimuth > max && azimuth - max < min - azimuth) {
//            		mInitAzimuth += azimuth - max; 
////            		Log.d(TAG, "C " + mInitAzimuth + "; min/max = " + min + ", " + max);
//            	} else if (azimuth < min && azimuth - max > min - azimuth) {
//            		mInitAzimuth -= min - azimuth; 
////            		Log.d(TAG, "D " + mInitAzimuth + "; min/max = " + min + ", " + max);
//            	} else {
////            		Log.d(TAG, "E " + mInitAzimuth + "; min/max = " + min + ", " + max);
//            	}
//            }
            
            // re-center azimuth
            double delta = getSmallerAngle(azimuth, mInitAzimuth);
            if (Math.abs(delta) < 0.01f) {
//            	mInitAzimuth = azimuth;
            } else {
            	double step = delta / 40;
            	double attemp = Math.abs(getSmallerAngle(azimuth, clampBetweenZeroAnd2PI(mInitAzimuth + step)));
//            	if (attemp < delta) {
//        			mInitAzimuth += step;
//            	} else {
//                    mInitAzimuth -= step;
//            	}
            }
//            mInitAzimuth = (float)clampBetweenZeroAnd2PI(mInitAzimuth);
            
            // re-center inclination
            float inclination = mInclination;
            delta = getSmallerAngle(inclination, mInitInclination);
            if (Math.abs(delta) < 0.01f) {
//                mInitInclination = inclination;
            } else {
                double step = delta / 40;
                double attemp = Math.abs(getSmallerAngle(inclination, clampBetweenZeroAnd2PI(mInitInclination + step)));
//                if (attemp < delta) {
//                    mInitInclination += step;
//                } else {
//                    mInitInclination -= step;
//                }
            }
            mInitInclination = (float)clampBetweenZeroAnd2PI(mInitInclination);
            
//            Log.d("Lance", "ia = " + mInitAzimuth + " a = " + azimuth);
//            Log.d("Lance", "ii = " + mInitInclination + " i = " + inclination);
            
            if (mInitAzimuthState != STATE_INVALID && mInitInclinationState != STATE_INVALID) {
	            // magic communication with surface flinger.
	            IBinder flinger = ServiceManager.getService("SurfaceFlinger");
	            if (flinger != null) {
	                Parcel data = Parcel.obtain();
	                Parcel reply = Parcel.obtain();
	                data.writeFloat(-clampedAzimuth);
	                data.writeFloat(inclination - mInitInclination);
	                flinger.transact(CMD_UPDATE_SENSOR, data, reply, 0);
	                
	                mMirrable = reply.readInt() == 1 ? true : false;
	                mFOV = reply.readInt();
	                mZScale = reply.readFloat();
	                mCamRot = reply.readFloat();
	                mCamDis = reply.readFloat();
	                reply.recycle();
	                data.recycle();
	            }
            }
        } catch (RemoteException ex) {
        }
    }
    
    // util
    private static double PI2 = Math.PI * 2;
    private static double clampBetweenZeroAnd2PI(double angle) {
		double res = angle;
		
		while (res < 0) {
			res += PI2;
		} 
		while (res > PI2) {
			res -= PI2;
		}
		return res;
	}
	
	// return angle between 0~PI
    private static double getSmallerAngle(double a1, double a2) {
		double ca1 = clampBetweenZeroAnd2PI(a1);
		double ca2 = clampBetweenZeroAnd2PI(a2);
		double angle = Math.abs(ca1 - ca2);
		if (angle > Math.PI) {
			angle = PI2 - angle;
		}
		return angle;
	}
    
    private float[] lowPass( float[] input, float[] output, float deltaWeigh) {
        if ( output == null ) return input;     
        for ( int i=0; i<input.length; i++ ) {
            float delta = input[i] - output[i];
            if (Math.abs(delta) > Math.PI) {
                output[i] = input[i];
            } else {
                output[i] = output[i] + deltaWeigh * (delta);
            }
        }
        return output;
    }
}
