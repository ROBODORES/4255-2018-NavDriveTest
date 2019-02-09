package org.usfirst.frc.team4255.robot;

import edu.wpi.first.wpilibj.I2C;

public class Pixy {
	private I2C port;
	private int obnum = 20;
    public int[] checksum = new int[obnum];
    public int[] signature = new int[obnum];
    public int[] objectX = new int[obnum];
    public int[] objectY = new int[obnum];
    public int[] objectWidth = new int[obnum];
    public int[] objectHeight = new int[obnum];
	
	
	public Pixy(I2C port) {
		this.port = port;
	}
	
	public boolean setPanTilt(int panPos, int tiltPos) {
		byte [] panData = new byte [2]; 
		panData[0] = (byte) (panPos & 0xFF);
		panData[1] = (byte) ((panPos >> 8) & 0xFF);
		byte [] tiltData = new byte [2]; 
		tiltData[0] = (byte) (tiltPos & 0xFF);
		tiltData[1] = (byte) ((tiltPos >> 8) & 0xFF);
		
		byte[] buff = {
				(byte) 0x00,
				(byte) 0xff,
				panData[0],
				panData[1],
				tiltData[0],
				tiltData[1]
		};
		
		if (port.writeBulk(buff)) return false; //write data and exit if failed
		
		return true;
	}
	
	public boolean update() {
		boolean track = true;
		byte[] buff = new byte[1];
		int object = 0;
	    int dataState = 0;
	    boolean frame = false;
	    boolean see = false;
	    int zeroNum = 0;
	    
		//Get Pixy I2C Data 
		for (int i = 0; i < objectX.length; i++) {
			objectX[i] = 0;
		}
				while (track) {
			    	if (port.readOnly(buff, 1)) return false; //read data and exit if failed
				
			    	switch (dataState) {
			    	case 0:
			    		if (buff[0] == (byte)0x55) {
			    			dataState++;
			    			object++;
			    			zeroNum = 0;
			    		} else if (buff[0] == (byte)0x00) {
			    			zeroNum++;
			    			if (zeroNum >= 100) {
			    				track = false;
			    				see = false;
			    			}
			    		}
			    		break;
			    	case 1:
			    		if (buff[0] == (byte)0xAA) {
			    			dataState++;
			    		} else {
			    			dataState = 0;
			    		}
			    		break;
			    	case 2:
			    		if (buff[0] == (byte)0x55) {
			    			dataState = 1;
			    			object = 0;
			    			if (frame) {
			    				track = false;
			    				frame = false;
			    			} else {
			    				frame = true;
			    				see = true;
			    			}
			    		} else {
			    		    //System.out.println("Object: " + object);
						    checksum[object] = buff[0] & 0xFF;
						    dataState++;
			    		}
						break;
			    	case 3:
			    		checksum[object] = checksum[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" Checksum: " + checksum[object]);
						dataState++;
						break;
			    	case 4:
			    		signature[object] = buff[0] & 0xFF;
						dataState++;
						break;
			    	case 5:
			    		signature[object] = signature[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" Signature #: " + signature[object]);
						dataState++;
						break;
			    	case 6:
			    		objectX[object] = buff[0] & 0xFF;
						dataState++;
						break;
			    	case 7:
			    		objectX[object] = objectX[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" X value: " + objectX[object]);
						dataState++;
						break;
			    	case 8:
			    		objectY[object] = buff[0] & 0xFF;
						dataState++;
						break;
			    	case 9:
			    		objectY[object] = objectY[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" Y value: " + objectY[object]);
						dataState++;
						break;
			    	case 10:
			    		objectWidth[object] = buff[0] & 0xFF;
						dataState++;
						break;
			    	case 11:
			    		objectWidth[object] = objectWidth[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" Width: " + objectWidth[object]);
						dataState++;
						break;
			    	case 12:
			    		objectHeight[object] = buff[0] & 0xFF;
						dataState++;
						break;
			    	case 13:
			    		objectHeight[object] = objectHeight[object] | ((buff[0] & 0xFF)*256);
						//System.out.println(" Height: " + objectHeight[object]);
						dataState = 0;
						break;
			    	}
				}
				
				return see;
	}
}
