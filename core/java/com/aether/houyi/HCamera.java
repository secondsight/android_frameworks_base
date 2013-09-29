package com.aether.houyi;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class HCamera extends HObject3D{
	/** Position of the camera's eye point. */
	protected HVector mPosition = new HVector(0, 0, 1);
	/** Position of the "look-at" direction point. */
	protected HVector mLookAt = new HVector(0, 0, 0);
	/** Direction of the up vector. */
	protected HVector mUp = new HVector(0, 1, 0);
	// mRight&mForward vectors are used only for performance enhancement
	// We can avoid a lot of run time memory allocation by pre-allocating them
	protected HVector mRight = new HVector(1, 0, 0);
	protected HVector mForward = new HVector(0, 0, -1);
	
	/** View matrix. */
	protected float[] mViewMatrix = new float[16];
	/** Projection matrix */
	protected float[] mProjectionMatrix = new float[16];
	// View & Projection matrix. This is for shader
	protected float[] mViewProjectionMatrix = new float[16];
	// Normal matrix. This is for shader
	protected float[] mNormalMatrix = new float[9];
	// inverse matrix of view matrix
	protected float mViewInvMatrix[] = new float[16];
	
	protected float mBankAngle;
	protected float mFocus;
	protected float mNearRange = 1;
	protected float mFarRange = 1000;
	protected float mRoll;
	protected float mFOV = 45;
	protected float mAspectRatio = 1.f;

	protected float[][] mKFPos;
	protected float[][] mKFTargetPos;
	protected float[][] mKFFOV;
	protected float[][] mKFRoll;
	protected int mCurFrame;
	protected int mMaxKFCount = 1;
	
	public HCamera() {

	}

	public HCamera(HVector position, HVector lookat, float roll) {
		lookAt(position, lookat, roll);
	}
	
	public HVector getLookAt(){
		return mLookAt;
	}

	public void setPosition(float x, float y, float z) {
		mPosition.x = x;
		mPosition.y = y;
		mPosition.z = z;
	}

	public HVector getPosition(){
		return mPosition;
	}
	
	public void setLookAt(float x, float y, float z) {
		mLookAt.x = x;
		mLookAt.y = y;
		mLookAt.z = z;
	}

	public void setUp(float x, float y, float z) {
		mUp.x = x;
		mUp.y = y;
		mUp.z = z;
	}
	
	public void setBankAngle(float angle) {
		mBankAngle = angle;
	}

	public void setFocus(float focus) {
		mFocus = focus;
	}

	public void setNearRange(float nearrange) {
		mNearRange = nearrange;
	}

	public void setFarRange(float farrange) {
		mFarRange = farrange;
	}

	public float getNearRange() {
		return mNearRange;
	}

	public float getFarRange() {
		return mFarRange;
	}

	public float getFOV() {
		return mFOV;
	}

	public void setFOV(float fov){
		mFOV = fov;
	}
	
	public float getRoll() {
		return mRoll;
	}
	
	public void setRoll(float roll){
		mRoll = roll;
	}

	public float getBankAngle() {
		return mBankAngle;
	}

	public float getFocus() {
		return mFocus;
	}

	public HVector getUp() {
		return mUp;
	}

	public void lookAt(){
		lookAt(mPosition, mLookAt, mRoll);
	}
	
	public void lookAt(HVector aPosition, HVector aLookAt, double aRoll) {
		mPosition = aPosition;
		mLookAt = aLookAt;

		mUp.x = 0;
		mUp.y = 1;
		mUp.z = 0;

		mForward = aLookAt.substract(aPosition);
		mForward.normalize();

		// calculating up vector using the following steps:
		// 1. Calculate angle of mForward & x-z plane: theta1
		// 2. Calculate angle of mForward & -z axis: theta2
		// 3. Rotate (0, 1, 0) along x axis by theta1
		// 4. Rotate then along y axis by theta2 or 180-theta2
		// 5. Rotate then along mForward by aRoll
		double theta1 = Math.asin(mForward.y);

		double theta2;
		if (mForward.z == 0) {
			theta2 = 0;
		} else {
			theta2 = Math.atan(mForward.x / mForward.z);
			if (mForward.z > 0) {
				theta2 = Math.PI / 2 - theta2;
			}
		}

		mUp.y = (float) Math.cos(theta1);
		mUp.z = (float) Math.sin(theta1);

		mUp.z = (float) (mUp.z * Math.cos(theta2) - mUp.x * Math.sin(theta2));
		mUp.x = (float) (mUp.x * Math.cos(theta2) + mUp.z * Math.sin(theta2));

		aRoll = HMath.degree2radian(aRoll);
		mUp.rotate(mForward, aRoll);
		// up.iX = up.iX * Cos(aRoll) - up.iY * Sin(aRoll);
		// up.iY = up.iY * Cos(aRoll) + up.iZ * Sin(aRoll);

		mUp.normalize();

		mRight = HVector.crossProduct(mForward, mUp);
		mRight.normalize();

		mUp = HVector.crossProduct(mRight, mForward);
		mUp.normalize();

		buildViewMatrix();
	}

	public void update(int keyFrame) {
		mCurFrame = keyFrame % mMaxKFCount;
		
		boolean lookat = false;
		if (mKFPos != null && mKFPos.length > 1){
			if (mCurFrame >= 0 && mCurFrame < mKFPos.length){
				mPosition.x = mKFPos[mCurFrame][0];
				mPosition.y = mKFPos[mCurFrame][1];
				mPosition.z = mKFPos[mCurFrame][2];
			}
			
			lookat = true;
		}
		
		if (mKFTargetPos != null && mKFTargetPos.length > 1){
			if (mCurFrame >= 0 && mCurFrame < mKFTargetPos.length){
				mLookAt.x = mKFTargetPos[mCurFrame][0];
				mLookAt.y = mKFTargetPos[mCurFrame][1];
				mLookAt.z = mKFTargetPos[mCurFrame][2];
			}
			
			lookat = true;
		}
		
		if (lookat){
			lookAt();
		}else{
			update();
		}
	}
	
	public void update(){
		mForward = mLookAt.substract(mPosition);
		mForward.normalize();
		
		mRight = HVector.crossProduct(mForward, mUp);
		mRight.normalize();

		mUp = HVector.crossProduct(mRight, mForward);
		mUp.normalize();
		
		buildViewMatrix();
	}
	
	protected void buildViewMatrix(){
		mViewMatrix[0 * 4 + 0] = mNormalMatrix[0 * 3 + 0] = mRight.x;
		mViewMatrix[1 * 4 + 0] = mNormalMatrix[1 * 3 + 0] = mRight.y;
		mViewMatrix[2 * 4 + 0] = mNormalMatrix[2 * 3 + 0] = mRight.z;
		mViewMatrix[3 * 4 + 0] = -mPosition.x*mRight.x - mPosition.y*mRight.y - mPosition.z*mRight.z;
		
		mViewMatrix[0 * 4 + 1] = mNormalMatrix[0 * 3 + 1] = mUp.x;
		mViewMatrix[1 * 4 + 1] = mNormalMatrix[1 * 3 + 1] = mUp.y;
		mViewMatrix[2 * 4 + 1] = mNormalMatrix[2 * 3 + 1] = mUp.z;
		mViewMatrix[3 * 4 + 1] = -mPosition.x*mUp.x - mPosition.y*mUp.y - mPosition.z*mUp.z;

		mViewMatrix[0 * 4 + 2] = mNormalMatrix[0 * 3 + 2] = -mForward.x;
		mViewMatrix[1 * 4 + 2] = mNormalMatrix[1 * 3 + 2] = -mForward.y;
		mViewMatrix[2 * 4 + 2] = mNormalMatrix[2 * 3 + 2] = -mForward.z;
		mViewMatrix[3 * 4 + 2] = mPosition.x*mForward.x + mPosition.y*mForward.y + mPosition.z*mForward.z;
		
		mViewMatrix[0 * 4 + 3] = 0;
		mViewMatrix[1 * 4 + 3] = 0;
		mViewMatrix[2 * 4 + 3] = 0;
		mViewMatrix[3 * 4 + 3] = 1;
	}
	
	public void perspective(){
		perspective(mFOV, mAspectRatio, mNearRange, mFarRange);
	}
	
	public void perspective(float fov, float aspectRatio, float near, float far){
		mNearRange = near;
		mFarRange = far;
		mAspectRatio = aspectRatio;
		
		float f = (float)(1 / Math.tan(HMath.degree2radian(fov/2)));
		mProjectionMatrix[0 * 4 + 0] = f / aspectRatio;
		mProjectionMatrix[1 * 4 + 0] = 0;
		mProjectionMatrix[2 * 4 + 0] = 0;
		mProjectionMatrix[3 * 4 + 0] = 0;
		
		mProjectionMatrix[0 * 4 + 1] = 0;
		mProjectionMatrix[1 * 4 + 1] = f;
		mProjectionMatrix[2 * 4 + 1] = 0;
		mProjectionMatrix[3 * 4 + 1] = 0;

		mProjectionMatrix[0 * 4 + 2] = 0;
		mProjectionMatrix[1 * 4 + 2] = 0;
		mProjectionMatrix[2 * 4 + 2] = (near + far)/(near - far);
		mProjectionMatrix[3 * 4 + 2] = 2 * near * far / (near - far);
		
		mProjectionMatrix[0 * 4 + 3] = 0;
		mProjectionMatrix[1 * 4 + 3] = 0;
		mProjectionMatrix[2 * 4 + 3] = -1;
		mProjectionMatrix[3 * 4 + 3] = 0;
	}
	
	public float[] getViewMatrix(){
		return mViewMatrix;
	}
	
	public FloatBuffer getViewMatrixAsFloatBuffer(){
		ByteBuffer bb = ByteBuffer.allocateDirect(16 * 4);
		bb.order(ByteOrder.nativeOrder());
		FloatBuffer fb = bb.asFloatBuffer();
		fb.put(mViewMatrix);
		fb.position(0);
		return fb;
	}
	
	public float[] getProjectionMatrix(){
		return mProjectionMatrix;
	}
	
	public FloatBuffer getProjectionMatrixAsFloatBuffer(){
		ByteBuffer bb = ByteBuffer.allocateDirect(16 * 4);
		bb.order(ByteOrder.nativeOrder());
		FloatBuffer fb = bb.asFloatBuffer();
		fb.put(mProjectionMatrix);
		fb.position(0);
		return fb;
	}
	
	public float[] getViewProjectionMatrix(){
		return mViewProjectionMatrix;
	}
	
	public FloatBuffer getViewProjectionMatrixAsFloatBuffer(){
		HMatrix.multiply(mViewProjectionMatrix, mProjectionMatrix, mViewMatrix);
		ByteBuffer bb = ByteBuffer.allocateDirect(16 * 4);
		bb.order(ByteOrder.nativeOrder());
		FloatBuffer fb = bb.asFloatBuffer();
		fb.put(mViewProjectionMatrix);
		fb.position(0);
		return fb;
	} 
	
	public float[] getNormalMatrix(){
		return mNormalMatrix;
	}
	
	public FloatBuffer getNormalAsFloatBuffer(){
		ByteBuffer bb = ByteBuffer.allocateDirect(16 * 4);
		bb.order(ByteOrder.nativeOrder());
		FloatBuffer fb = bb.asFloatBuffer();
		fb.put(mNormalMatrix);
		fb.position(0);
		return fb;
	} 
	
	public FloatBuffer getViewInvAsFloatBuffer(){
		ByteBuffer bb = ByteBuffer.allocateDirect(16 * 4);
		bb.order(ByteOrder.nativeOrder());
		FloatBuffer fb = bb.asFloatBuffer();
		fb.put(mViewInvMatrix);
		fb.position(0);
		return fb;
	} 
	
	public void peek(float xs1, float ys1, float xs2, float ys2) {
		// first check if two points are very close, no need to rotate
		float dis = xs1 - xs2 + ys1 - ys2;
		if (dis > -0.0001 && dis < 0.0001) {
			return;
		}

		// 1. transform screen coordinates into view coordinates with help of
		// projection matrix
		// xs1 * -n = x1 * mProjectionMatrix[0 * 4 + 0]
		float x1 = xs1 * mNearRange / mProjectionMatrix[0 * 4 + 0];
		float x2 = xs2 * mNearRange / mProjectionMatrix[0 * 4 + 0];
		float y1 = -ys1 * mNearRange / mProjectionMatrix[1 * 4 + 1];
		float y2 = -ys2 * mNearRange / mProjectionMatrix[1 * 4 + 1];
		float z1 = -mNearRange;
		float z2 = -mNearRange;

		// 2. transform view coordinates into world coordinates using inverse of
		// view matrix
		// note the view matrix` s inverse should not be calculate using normal
		// matrix
		// inverse operation because that is time consuming. Since view matrix`
		// s first 3 rows and 3 columns consists an orthogonal matrix 
		// we can get the inverse of
		// this sub matrix immediately because its inverse is its transpose.
		// Then we multiply translation to get the final matrix
		mForward = mLookAt.substract(mPosition);
		mForward.normalize();
		mRight = HVector.crossProduct(mForward, mUp);
		mRight.normalize();
		mUp = HVector.crossProduct(mRight, mForward);
		mUp.normalize();
		mViewInvMatrix[0 * 4 + 0] = mRight.x;
		mViewInvMatrix[1 * 4 + 0] = mUp.x;
		mViewInvMatrix[2 * 4 + 0] = -mForward.x;
		mViewInvMatrix[3 * 4 + 0] = mPosition.x;
		mViewInvMatrix[0 * 4 + 1] = mRight.y;
		mViewInvMatrix[1 * 4 + 1] = mUp.y;
		mViewInvMatrix[2 * 4 + 1] = -mForward.y;
		mViewInvMatrix[3 * 4 + 1] = mPosition.y;
		mViewInvMatrix[0 * 4 + 2] = mRight.z;
		mViewInvMatrix[1 * 4 + 2] = mUp.z;
		mViewInvMatrix[2 * 4 + 2] = -mForward.z;
		mViewInvMatrix[3 * 4 + 2] = mPosition.z;
		mViewInvMatrix[0 * 4 + 3] = 0;
		mViewInvMatrix[1 * 4 + 3] = 0;
		mViewInvMatrix[2 * 4 + 3] = 0;
		mViewInvMatrix[3 * 4 + 3] = 1;
		HVector v1 = HMatrix.multiply(mViewInvMatrix, x1, y1, z1);
		HVector v2 = HMatrix.multiply(mViewInvMatrix, x2, y2, z2);

		// 3. calculate rotate vector VR which is the cross product of
		// mForward&(P2 - P1)
		HVector vr = HVector.crossProduct(mLookAt.substract(mPosition), v2
				.substract(v1));
		vr.normalize();

		// 4. rotate camera position along VR by length(P1-P2) * coefficient
		float length = (xs2 - xs1) * (xs2 - xs1) + (ys2 - ys1) * (ys2 - ys1);
		mPosition.rotate(vr, length * 0.0001);

		// 5. update camera. this will update view matrix also
		// note that up vector stays the same
		update(mCurFrame);
	}
	
	public void setKFPos(int index, float x, float y, float z){
		mKFPos[index][0] = x;
		mKFPos[index][1] = y;
		mKFPos[index][2] = z;
	}
	
	public void setKFTargetPos(int index, float x, float y, float z){
		mKFTargetPos[index][0] = x;
		mKFTargetPos[index][1] = y;
		mKFTargetPos[index][2] = z;
	}
	
	public void setupKeyFrame(){
		// interpolate position
		for (int i = 0, start = 0;i < mKFPos.length;i++){
			float x = mKFPos[i][0];
			float y = mKFPos[i][1];
			float z = mKFPos[i][2];

			if (start != i && (x != 0 || y != 0 || z != 0)){
				int len = i - start;
				float stepX = (mKFPos[i][0] - mKFPos[start][0]) / len;
				float stepY = (mKFPos[i][1] - mKFPos[start][1]) / len;
				float stepZ = (mKFPos[i][2] - mKFPos[start][2]) / len;
				for (int j = start;j < i;j++){
					float deltaX = (j - start) * stepX;
					float deltaY = (j - start) * stepY;
					float deltaZ = (j - start) * stepZ;
					mKFPos[j][0] = mKFPos[start][0] + deltaX;
					mKFPos[j][1] = mKFPos[start][1] + deltaY;
					mKFPos[j][2] = mKFPos[start][2] + deltaZ;
				}
				start = i;
			}
		}
		
//		Log.d(HEngine.TAG, "Set camera key. Key frame count = " + mKFPos.length);
//		for (int i = 0;i < mKFPos.length;i++){
//			Log.d(HEngine.TAG, "KF " + i 
//					+ ": POSX = " 
//					+ mKFPos[i][0] + "; POSY = " 
//					+ mKFPos[i][1] + "; POSZ = "
//					+ mKFPos[i][2]);
//		}
		
		// interpolate target position
		for (int i = 0, start = 0;i < mKFTargetPos.length;i++){
			float x = mKFTargetPos[i][0];
			float y = mKFTargetPos[i][1];
			float z = mKFTargetPos[i][2];

			if (start != i && (x != 0 || y != 0 || z != 0)){
				int len = i - start;
				float stepX = (mKFTargetPos[i][0] - mKFTargetPos[start][0]) / len;
				float stepY = (mKFTargetPos[i][1] - mKFTargetPos[start][1]) / len;
				float stepZ = (mKFTargetPos[i][2] - mKFTargetPos[start][2]) / len;
				for (int j = start;j < i;j++){
					float deltaX = (j - start) * stepX;
					float deltaY = (j - start) * stepY;
					float deltaZ = (j - start) * stepZ;
					mKFTargetPos[j][0] = mKFTargetPos[start][0] + deltaX;
					mKFTargetPos[j][1] = mKFTargetPos[start][1] + deltaY;
					mKFTargetPos[j][2] = mKFTargetPos[start][2] + deltaZ;
				}
				start = i;
			}
		}
		
//		Log.d(HEngine.TAG, "Set camera key. Key frame count = " + mKFTargetPos.length);
//		for (int i = 0;i < mKFTargetPos.length;i++){
//			Log.d(HEngine.TAG, "KF " + i 
//					+ ": POSX = " 
//					+ mKFTargetPos[i][0] + "; POSY = " 
//					+ mKFTargetPos[i][1] + "; POSZ = "
//					+ mKFTargetPos[i][2]);
//		}
	}
	
	public void setMaxKFCount(int maxkf){
		if (maxkf > mMaxKFCount){
			mMaxKFCount = maxkf;
		}
	}
	
	public void roll(double degree){
		mForward = mLookAt.substract(mPosition);
		mForward.normalize();
		mUp.rotate(mForward, degree);
	}
}
