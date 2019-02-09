package org.usfirst.frc.team4255.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class NavDrive {
	private AHRS navX;
	private Drive drive;
	private Timer time;
	private double driveSpeed;
	
	public NavDrive(AHRS navX, Drive drive) {
		this.navX = navX;
		this.drive = drive;
		time = new Timer();
		time.start();
		driveSpeed = 0.0;
	}
	
	void reset() {
		navX.reset();
		time.reset();
		driveSpeed = 0.0;
	}
	
	boolean turnTo(double angle) {
		drive.setDrive(driveSpeed, -driveSpeed, false);
		
		if (Math.abs(navX.getAngle()) >= Math.abs(angle) - 20.0) {
			driveSpeed += (((180.0/20.0)*(angle-navX.getAngle())) - navX.getRawGyroZ())/8000.0;
			if (Math.abs(navX.getAngle()) >= Math.abs(angle)) {
				drive.setDrive(0.0, 0.0, false);
				return true;
			}
		} else {
			driveSpeed += (180.0*(angle/Math.abs(angle)) - navX.getRawGyroZ())/8000.0;
		}
		return false;
	}
	
	boolean driveTo(double distance) { //experimental DO NOT USE!!!
		drive.setDrive(driveSpeed, driveSpeed, false);
		double acceleration = 2.0;
		double velocity = 2.0;
		double accelTime = velocity/acceleration;
		double fs = 32.174049*Math.abs(navX.getRawAccelX());
		
		if (time.get() < accelTime) {
			driveSpeed += ((acceleration - fs)*(time.get()/accelTime))/80;
		} else {
			double ftDist = (time.get()*velocity) + ((accelTime*accelTime)*acceleration)/2.0;
			if (ftDist >= distance) {
				drive.setDrive(0.0, 0.0, false);
				return true;
			}
		}
		
		return false;
	}
	
	/*boolean arcTo(double radius, double length) {
		
	}
	*/
}
