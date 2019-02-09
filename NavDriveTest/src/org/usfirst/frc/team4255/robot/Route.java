package org.usfirst.frc.team4255.robot;

public class Route {
	private double[][] ROUTE_ARRAY;
	private Drive drive;
	private NavDrive navDrive;
	public int step = 0;
	public int typeStep = 0;
	
	public Route(double[][] ROUTE_ARRAY, Drive drive, NavDrive navDrive) {
		this.ROUTE_ARRAY = ROUTE_ARRAY;
		this.drive = drive;
		this.navDrive = navDrive;
		step = 0;
		typeStep = 0;
		this.drive.reset();
		this.navDrive.reset();
	}
	
	public void setTo(int step) {
		this.step = step;
		drive.reset();
		navDrive.reset();
	}
	
	public boolean run() {
		if (step < ROUTE_ARRAY.length) {
			if (typeStep == 0) {
				if(drive.driveTo(ROUTE_ARRAY[step][typeStep]) || ROUTE_ARRAY[step][typeStep] == 0) typeStep++;
			} else {
				if(navDrive.turnTo(ROUTE_ARRAY[step][typeStep]) || ROUTE_ARRAY[step][typeStep] == 0) {
					typeStep = 0;
					step++;
					drive.reset();
					navDrive.reset();
				}
			}
			return false;
		}
		drive.setDrive(0.0, 0.0, false);
		return true;
	}
}
