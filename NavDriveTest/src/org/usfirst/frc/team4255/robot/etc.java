package org.usfirst.frc.team4255.robot;

public class etc {
	public static double[][] middleL = {
		{1.0, -45.0},
		{5.25, 45.0},
	};
	
	public static double[][] middleR = {
		{1.8, 45.0},
		{4.07, -45.0}, //sensor and dropoff
	};
	public static double[][] leftL = {
		{10.38, 75.0} //raise lift and dropoff switch
	};
	public static double[][] leftR = {
		{15.44, 82.0},
		{11.72, 75.0} //dropoff switch
	};
	public static double[][] rightR = {
		{10.38, -75.0} //dropoff switch
	};
	public static double[][] rightL = {
		{15.44, -82.0},
		{13.72, -75.0} //dropoff switch
	};
	public static double[][] leftLL = {
		{9.38, 75.0}, //raise lift while, then drop in switch
		{-1.0, -90.0},
		{6.36, 0.0},
		{0.0, 90.0},
		{3.17, 0.0},
		{0.0, 90.0},
		{1.58, 0.0}, //with intake activated -> pickup cube
		{-1.0, 0.0},
		{0.0, 90.0},
		{3.79, 0.0},
		{0.0, 90.0},
		{0.5, 0.0}, //dropoff scale
		{-1.66, 0.0}
	};
	public static double[][] leftLR = {
		{12.38, 0.0}, //raise lift
		{0.0, 90.0},
		{1.59, 0.0}, //dropoff switch
		{-1.0, 0.0},
		{0.0, -90.0},
		{6.36, 0.0},
		{0.0, 90.0},
		{3.17, 0.0},
		{0.0, 90.0},
		{1.58, 0.0}, //with intake activated -> pickup cube
		{-1.0, 0.0},
		{0.0, -90.0},
		{11.82, 0.0},
		{0.0, -90.0},
		{6.81, 0.0}, //dropoff scale
		{-1.66, 0.0}
	};
	public static double[][] leftLS = {
			{22.5, 75.0}, //raise lift and dropoff switch
		};
	public static double[][] rightRS = {
			{22.5, -75.0},
	};
	
	public static double map(double val, double fromMin, double fromMax, double toMin, double toMax) {
		double output = ((val-fromMin)*((toMax-toMin)/(fromMax-fromMin)))+toMin;
		return output;
	}
	
	public static double constrain(double val, double min, double max) {
		if (val > max) val = max;
		if (val < min) val = min;
		return val;
	}
}
