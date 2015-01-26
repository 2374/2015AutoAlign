package org.usfirst.frc.team2374.robot;

public class VisionReport {
	//a simple class designed to make the camera output easier to use
	//and more relevant
	double depthOffset, horizontalOffset;
	
	public VisionReport(double x, double y, double w, double h){
		//INPUT: the coordinates of the particle from vision processing
		
		//the following code was determined experimentally
		//it's specific to the camera layout
		depthOffset=(225-y)/10;//magic
		horizontalOffset=(x+w/2 - 160)*0.5/(y-120);//more magic
	}
	
	
}
