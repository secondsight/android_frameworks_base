package com.aether.houyi;

import android.content.Context;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.view.MotionEvent;
import android.view.OrientationEventListener;
import android.view.Surface;
import android.view.WindowManager;

import java.util.concurrent.atomic.AtomicReference;

public class SensorFusionService extends ISensorFusion.Stub implements SensorFusion2.OnTrackDataChangedListener {

    private static AtomicReference<SensorFusionService> sThis =
            new AtomicReference<SensorFusionService>();
    
    private final Context mContext;
    private final SensorFusion2 mSensorFusion;
    private final WindowManager mWindowManager;
    private OrientationEventListener mOrientationEventListener;
    
	final RemoteCallbackList<ISensorFusionCallback> mCallbacks = new RemoteCallbackList<ISensorFusionCallback>();
    /**
     * This should only be called by system code. One should only call this after the service
     * has started.
     * @return a reference to the SensorFusionService instance
     * @hide
     */
    public static SensorFusionService getSingleton() {
        return sThis.get();
    }

    public SensorFusionService(Context context) {
        mContext = context;
        mSensorFusion = new SensorFusion2(context);
        mWindowManager = (WindowManager)context.getSystemService(Context.WINDOW_SERVICE);
        mSensorFusion.registerOnTrackDataChangedListener(this);
    }

    
    @Override
    public float getAzimuth() throws RemoteException {
        return mSensorFusion.getAzimuth();
    }

    @Override
    public float getInclination() throws RemoteException {
        return mSensorFusion.getInclination();
    }

    @Override
    public MotionEvent redirectTouchEvent(MotionEvent ev, int width, int height)
            throws RemoteException {
        mSensorFusion.dispatchTouchEvent(ev, width, height);
        return ev;
    }

    @Override
	public boolean registerCallback(ISensorFusionCallback callback)
			throws RemoteException {	
    	return mCallbacks.register(callback);
	}

	@Override
	public boolean unregisterCallback(ISensorFusionCallback callback)
			throws RemoteException {	
		return mCallbacks.unregister(callback);
	}
	@Override
	public void onTrackDataChanged(float azimuth, float inclination) {
		int count = mCallbacks.beginBroadcast();
		try {
			for (int i = 0; i < count; i++) {
				ISensorFusionCallback callback = mCallbacks.getBroadcastItem(i);
				callback.onTrackDataChanged(azimuth, inclination);			
			}			
		} catch (Exception e) {		
		}
		mCallbacks.finishBroadcast();	
	}
    public void systemReady() {
        mOrientationEventListener = new OrientationEventListener(mContext) {
            @Override
            public void onOrientationChanged(int orientation) {   
                int rotaion = mWindowManager.getDefaultDisplay().getRotation();
                if (rotaion == Surface.ROTATION_90 || rotaion == Surface.ROTATION_270) {
                    mSensorFusion.start();
                    mSensorFusion.setScreenRotation(rotaion);
                } else {
                    mSensorFusion.stop();
                }
            }            
        };
        mOrientationEventListener.enable();
    } 

}
