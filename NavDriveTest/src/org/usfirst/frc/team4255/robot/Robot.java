package org.usfirst.frc.team4255.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends IterativeRobot {
	Timer time = new Timer();
	Timer liftTime = new Timer();
	
	//SerialPort rpi = new SerialPort(9600, SerialPort.Port.kOnboard);
	
	Joystick jLeft = new Joystick (0);
    Joystick jRight = new Joystick (1);
    Joystick jSide = new Joystick (2);
    Joystick chooser = new Joystick (3);
    Joystick Esplora1 = new Joystick (4);
    Joystick Esplora2 = new Joystick (5);
	
    AHRS navX = new AHRS(SPI.Port.kMXP);
    AnalogInput sonar = new AnalogInput(0); //5v/512
    //I2C pixyPort = new I2C(edu.wpi.first.wpilibj.I2C.Port.kOnboard, 0x45);
  	//Pixy pixy = new Pixy(pixyPort);
    
    TalonSRX leftDrive = new TalonSRX(0);
    TalonSRX leftFollow1 = new TalonSRX(1);
    TalonSRX rightDrive = new TalonSRX(3);
    TalonSRX rightFollow1 = new TalonSRX(4);
    TalonSRX leftFollow2 = new TalonSRX(2);
    TalonSRX rightFollow2 = new TalonSRX(5);
    
    //DoubleSolenoid release = new DoubleSolenoid(0,1);
    
    Drive drive = new Drive(leftDrive, FeedbackDevice.None, rightDrive, FeedbackDevice.CTRE_MagEncoder_Relative);
    NavDrive navDrive = new NavDrive(navX, drive);
    Route middleL = new Route(etc.middleL, drive, navDrive);
    Route middleR = new Route(etc.middleR, drive, navDrive);
    Route leftL = new Route(etc.leftL, drive, navDrive);
    Route leftR = new Route(etc.leftR, drive, navDrive);
    Route rightR = new Route(etc.rightR, drive, navDrive);
    Route rightL = new Route(etc.rightL, drive, navDrive);
    Route leftLS = new Route(etc.leftLS, drive, navDrive);
    Route rightRS = new Route(etc.rightRS, drive, navDrive);
    
	Spark lift = new Spark(0);
	Spark clamp = new Spark(1);
	Spark tilt = new Spark(2);
	Spark intake = new Spark(3);
	Encoder tiltEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	Encoder clampEncoder = new Encoder(2,3, false, Encoder.EncodingType.k4X);
	double clampVal = 0.0;
	
	CameraServer camserv;
	UsbCamera cam0;
	UsbCamera cam1;

	String sides;
	String position;
	int step = 0;
	boolean done = false;
	boolean stop = false;
	
	@Override
	public void robotInit() {
		//release.set(DoubleSolenoid.Value.kOff);
		
		tiltEncoder.setDistancePerPulse(12.0/350.0);
		clampEncoder.setDistancePerPulse(12.0/100.0);
		
	    rightDrive.setInverted(true);
		rightFollow1.setInverted(true);
		rightFollow2.setInverted(true);
		leftFollow1.follow(leftDrive);
		rightFollow1.follow(rightDrive);
		leftFollow2.follow(leftDrive);
		rightFollow2.follow(rightDrive);
		
		rightDrive.configPeakCurrentDuration(10, 0);
		rightDrive.configPeakCurrentLimit(50, 0);
		rightDrive.configContinuousCurrentLimit(40, 0);
		rightDrive.enableCurrentLimit(true);
		
		leftDrive.configPeakCurrentDuration(10, 0);
		leftDrive.configPeakCurrentLimit(50, 0);
		leftDrive.configContinuousCurrentLimit(40, 0);
		leftDrive.enableCurrentLimit(true);
		
		rightFollow1.configPeakCurrentDuration(10, 0);
		rightFollow1.configPeakCurrentLimit(500, 0);
		rightFollow1.configContinuousCurrentLimit(40, 0);
		rightFollow1.enableCurrentLimit(true);
		
		leftFollow1.configPeakCurrentDuration(10, 0);
		leftFollow1.configPeakCurrentLimit(50, 0);
		leftFollow1.configContinuousCurrentLimit(40, 0);
		leftFollow1.enableCurrentLimit(true);
		
		leftFollow2.configPeakCurrentDuration(10, 0);
		leftFollow2.configPeakCurrentLimit(200, 0);
		leftFollow2.configContinuousCurrentLimit(40, 0);
		leftFollow2.enableCurrentLimit(true);
		
		rightFollow2.configPeakCurrentDuration(10, 0);
		rightFollow2.configPeakCurrentLimit(200, 0);
		rightFollow2.configContinuousCurrentLimit(40, 0);
		rightFollow2.enableCurrentLimit(true);
		
		camserv = CameraServer.getInstance();
	    
	    cam0 = camserv.startAutomaticCapture(0);
	    /*cam1 = camserv.startAutomaticCapture(1);
	    cam1.setResolution(1920, 1080);*/
	    //cam0.setFPS(30); //this probably won't work if you activate it
		time.start();
	}

	@Override
	public void autonomousInit() {
		sides = DriverStation.getInstance().getGameSpecificMessage();
		if (chooser.getRawButton(1)) position = "Left";
		else if (chooser.getRawButton(2)) position = "Right";
		else position = "Middle";
		navX.reset();
		navDrive.reset();
		drive.reset();
		done = false;
		step = 0;
		middleL.setTo(0);
		middleR.setTo(0);
		leftL.setTo(0);
		leftR.setTo(0);
		rightR.setTo(0);
		rightL.setTo(0);
		leftLS.setTo(0);
		time.reset();
		liftInit();
	}

	@Override
	public void autonomousPeriodic() {
		if (done) {
		switch (position) {
			case "Left":
				switch (sides.charAt(1)) {
				case 'L':
					switch (sides.charAt(0)) {
					case 'L': //Left side Left switch Left scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = leftL.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					case 'R': //Left side Right switch Left scale
						switch (step) { //switch code
						case 0:
							boolean up = liftUp();
							boolean run = leftR.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						
						/*switch (step) { //scale code
						case 0:
							boolean up = liftToScale();
							boolean run = leftLS.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (scaleDeploy()) step++;
							break;
						}*/
						break;
					}
					break;
				case 'R':
					switch (sides.charAt(0)) {
					case 'L': //Left side Left switch Right scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = leftL.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					case 'R': //Left side Right switch Right scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = leftR.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					}
					break;
				}
				break;
				
			case "Middle":
				switch (sides.charAt(0)) {
				case 'L':
					switch (step) {
					case 0:
						boolean up = liftUp();
						boolean run = middleL.run();
						if (up && run) {
						 	resetLift();
							step++;
						}
						break;
					case 1:
						if (deploy()) step++;
						break;
					}
					break;
				case 'R':
					switch (step) {
					case 0:
						boolean up = liftUp();
						boolean run = middleR.run();
						if (up && run) {
					 		resetLift();
							step++;
						}
						break;
					case 1:
						if (deploy()) step++;
						break;
					}
					break;
				}
				break;
			case "Right":
				switch (sides.charAt(1)) {
				case 'L':
					switch (sides.charAt(0)) {
					case 'L': //Right side Left switch Left scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = rightL.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					case 'R': //Right side Right switch Left scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = rightR.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					}
					break;
				case 'R':
					switch (sides.charAt(0)) {
					case 'L': //Right side Left switch Right scale
						/*switch (step) { //scale code
						case 0:
							boolean up = liftToScale();
							boolean run = rightRS.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (scaleDeploy()) step++;
							break;
						}*/
						
						switch (step) { //switch code
						case 0:
							boolean up = liftUp();
							boolean run = rightL.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					case 'R': //Right side Right switch Right scale
						switch (step) {
						case 0:
							boolean up = liftUp();
							boolean run = rightR.run();
							if (up && run) {
								resetLift();
								step++;
							}
							break;
						case 1:
							if (deploy()) step++;
							break;
						}
						break;
					}
					break;
				}
				break;
			}
		} else {
			liftInit(); //failsafe initializer
			done = true;
		}
	}
	
	@Override
	public void teleopInit() {
		resetLift();
		
		time.reset();
		drive.zeroLeftDist();
		done = false;
	}
	
	@Override
	public void teleopPeriodic() {
		lift.set(jSide.getY()+0.2);
		
		switch (jSide.getPOV()) { //Little D-Pad
		case -1: //default
			tilt.set(-.005);
			break;
		case 0: //Up
			tilt.set(0.4);
			break;
		case 45: //Top Right
			tilt.set(0.4);
			break;
		case 90: //Right
			tilt.set(-.005);
			break;
		case 135: //Lower Right
			tilt.set(-0.5);
			break;
		case 180: //Down
			tilt.set(-0.5);
			break;
		case 225: //Lower Left
			tilt.set(-0.5);
			break;
		case 270: //Left
			tilt.set(-.005);
			break;
		case 315: //Top Left
			tilt.set(0.4);
			break;
		}
		
		System.out.println(clampEncoder.getDistance());
		if(jSide.getRawButton(6)) clamp.set(-0.3);
		else if(jSide.getRawButton(5)) clamp.set(0.3);
		else clamp.set(0);
		/*if (time.get() >= 3.0) {
			if(jSide.getRawButton(6)) clampVal = 0.0; //Open Value
			if(jSide.getRawButton(5)) clampVal = -63.39; //Close Value
			setClamp(clampVal);
		} else {
			clamp.set(-0.5);
			clampEncoder.reset();
		}
		
		/*if(jSide.getRawButton(5)) clamp.set(0.8);
		else if (jSide.getRawButton(6)) clamp.set(-0.8);
		else clamp.set(0);
		System.out.println(clampEncoder.getDistance());*/
		
		if(jSide.getRawButton(2)) intake.set(1);
		else if(jSide.getRawButton(1)) intake.set(-.7);
		else intake.set(0);
		
		/*if(jSide.getRawButton(3)) tilt.set(-.4);
		else if(jSide.getRawButton(4)) tilt.set(.4);
		else tilt.set(-.005);*/
		
		drive.setDrive(-jLeft.getY()*0.5, -jRight.getY()*0.5, false);
	}

	public void nextStep(){
		navDrive.reset();
		drive.reset();
		step++;
	}
	
	public boolean liftUp() {
		double timeToLift = 1.2;
		if (liftTime.get() >= timeToLift) {
			lift.set(0.2);
			intake.set(-0.3);
			//setTilt(-7.0); //-14.0
			if (liftTime.get() >= timeToLift+0.7) {
				tilt.set(-0.1);
				return true;
			} else tilt.set(-0.6);
		} else {
			lift.set(0.8);
			tilt.set(0.0);
			intake.set(-0.3);
		}
		return false;
	}
	
	public boolean deploy() {
		double deployDist = 20.0; //inches
		lift.set(0.2);
		//setTilt(-7.0); //-14.0
		if (sonar.getVoltage()*102.4 <= deployDist || stop) {
			stop = true;
			drive.setDrive(0.0, 0.0, false);
			if (liftTime.get() >= 1.0) {
				intake.set(0.0);
				lift.set(0.2);
				tilt.set(0.0);
				return true;
			} 
			else if(liftTime.get() >= 0.5) {
				intake.set(1.0);
				clamp.set(-0.5);
			}
		} 
		else {
			drive.setDrive(0.3, 0.3, false);
			intake.set(-0.2);
			liftTime.reset();
		}
		return false;
	}
	
	public boolean setTilt(double toVal) {
		double margin = 0.2;
		double motorVal = etc.constrain((toVal-tiltEncoder.getDistance())/5.0, -6.0, 6.0);
		intake.set(-0.5);
		tilt.set(motorVal);
		if (Math.abs(motorVal) <= margin) {
			tilt.set(-0.1);
			return true;
		}
		return false;
	}
	
	public boolean setClamp(double toVal) {
		double margin = 0.2;
		double motorVal = etc.constrain((toVal-clampEncoder.getDistance())/-20.0, -6.0, 6.0);
		clamp.set(motorVal);
		if (Math.abs(motorVal) <= margin) {
			clamp.set(0.0);
			return true;
		}
		return false;
	}
	
	public void liftInit() {
		liftTime.reset();
		liftTime.start();
		tiltEncoder.reset();
		stop = false;
	}
	
	public void resetLift() {
		liftTime.reset();
		stop = false;
	}
	
	public boolean liftToScale() {
		lift.set(0.6);
		intake.set(-0.5);
		if (liftTime.get() >= 10.0) {
			tilt.set(-0.1);
			return true;
		}
		else if (liftTime.get() >= 1.0) {
			setTilt(-18.0);
		}
		else {
			tilt.set(0.0);
		}
		return false;
	}
	
	public boolean scaleDeploy() {
		lift.set(0.2);
		setTilt(-18.0);
		drive.setDrive(0.0, 0.0, false);
		if (liftTime.get() >= 1.0) {
			intake.set(0.0);
			lift.set(0.0);
			tilt.set(0.0);
			return true;
		}
		else {
			intake.set(1.0);
			clamp.set(0.5);
		}
		return false;
	}
}
