package org.usfirst.frc.team2374.robot;

import edu.wpi.first.wpilibj.Gyro;

public class CalibratedGyro {
	Gyro gyro;
	long calibrationTime;
	double offset;
	public CalibratedGyro(int port){
		//start up the gyro
		gyro=new Gyro(port);
	}
	
	public double getAngle(){
		//returns an angle value for the gyro, adjusted for drift over time
		return gyro.getAngle()-offset*(System.currentTimeMillis()-calibrationTime);
	}
	
	public void calibrate(){//WARNING: takes 1/10 of a second!
		//start from zero
		gyro.reset();
		calibrationTime=System.currentTimeMillis();
		while(System.currentTimeMillis()<calibrationTime+100);//do nothing for 100 ms
		
		//offset is the amount the gyroscope drifts by every millisecond
		offset=gyro.getAngle()/100;
		
	}
	public void reset(){
		//resets both the gyroscope and the timer, but doesn't calibrate it
		gyro.reset();
		calibrationTime=System.currentTimeMillis();
	}
}
