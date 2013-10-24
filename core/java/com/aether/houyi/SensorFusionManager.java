package com.aether.houyi;

import android.content.Context;
import android.view.MotionEvent;

public class SensorFusionManager {

    private final ISensorFusion mSensorFusion;
    
    public SensorFusionManager(ISensorFusion service) {
        mSensorFusion = service;        
    }
    
    public static SensorFusionManager get(Context context) {
        return (SensorFusionManager)context.getSystemService(Context.SENSOR_FUSION_SERVICE);
    }
    
    public float getAzimuth()  {
        try {
            return mSensorFusion.getAzimuth();
        } catch (Exception e) {   
            return 0;
        }
    }

    public float getInclination() {
        try {
            return mSensorFusion.getInclination();
        } catch (Exception e) {   
            return 0;
        }
    }

    public MotionEvent redirectTouchEvent(MotionEvent ev, int width, int height) {
        try {
            return mSensorFusion.redirectTouchEvent(ev, width, height);
        } catch (Exception e) {   
            return ev;
        }
    }

	public boolean registerCallback(ISensorFusionCallback callback)  {	
		try {
			return mSensorFusion.registerCallback(callback);
		} catch(Exception e) {
			return false;
		}
	}

	public boolean unregisterCallback(ISensorFusionCallback callback)  {	
		try {
			return mSensorFusion.unregisterCallback(callback);
		} catch(Exception e) {
			return false;
		}
	}
}
