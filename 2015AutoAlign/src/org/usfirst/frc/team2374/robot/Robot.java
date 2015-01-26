
package org.usfirst.frc.team2374.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
    Drivetrain drivetrain;
    Joystick joystick;
    VisionProcessor vision;
    public Robot() {
        drivetrain=new Drivetrain();
        joystick = new Joystick(0);
        vision=new VisionProcessor();
    }

    public void disabled(){
    	drivetrain.commandList.clear();//so we don't have any commands left over
    	drivetrain.gyro.calibrate();
    }
    public void autonomous() {
    	//this will cue commands
    	drivetrain.moveDistance(5);
    	drivetrain.moveDistance(-5);
    	
    	//the following code executes those commands
    	while(isAutonomous() && isEnabled()){
    		drivetrain.update(0, 0);//this will have the drivetrain followed the desired commands
    		/*
    		 * parameters are normally joystick values
    		 * 
    		 * since the drivetrain will follow commands until either it's done
    		 * or the driver overrides the command with their own joysticks,
    		 * putting (0,0) in will guarantee it finishes all commands uninterrupted
    		 * (and stop in place when it's done)
    		 */
    		
    	}
    	drivetrain.commandList.clear();//don't want any commands left over
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	drivetrain.commandList.clear();
    	drivetrain.resetGyro();
    	int count=0;//the camera is updated once every 10 updates, this keeps track of that
    	
        while(isOperatorControl() && isEnabled()){
        	drivetrain.update(-joystick.getRawAxis(1), -joystick.getRawAxis(3));
        	SmartDashboard.putNumber("Gyro",drivetrain.gyro.getAngle());
        	SmartDashboard.putNumber("Encoder", drivetrain.encoder.get());
        	SmartDashboard.putNumber("Commands", drivetrain.commandList.size());
        	
        	//the following code will only call once every 10 updates
        	//(so the vision doesn't hog too much CPU/bandwidth)
        	if(count%10==0){
        		//process the camera input into a vision report
        		VisionReport v=vision.processCamera();
        		if(v!=null){
        			SmartDashboard.putNumber("Horizontal Offset", v.horizontalOffset);
        			if(joystick.getRawButton(1)){
        				//auto-align!
        				drivetrain.alignWithCrate(v);
        			}
        		}
        	}
        	count++;
        	Timer.delay(0.005);
        	//wait for a little bit
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
