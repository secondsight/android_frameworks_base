package com.aether.houyi;

public class HMath {
	public static double PI2 = Math.PI * 2;
	
	public static double degree2radian(double degree){
		return degree * Math.PI / 180;
	}
	
	public static double radian2degree(double degree){
		return degree * 180 / Math.PI;
	}
	
	public static float clamp(float input, float min, float max) {
		return Math.min(Math.max(input, min), max);
	}
	
	public static double clampBetweenZeroAnd2PI(double angle) {
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
	public static double getSmallerAngle(double a1, double a2) {
		double ca1 = clampBetweenZeroAnd2PI(a1);
		double ca2 = clampBetweenZeroAnd2PI(a2);
		double angle = Math.abs(ca1 - ca2);
		if (angle > Math.PI) {
			angle = PI2 - angle;
		}
		return angle;
	}
}
