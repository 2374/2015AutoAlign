package org.usfirst.frc.team2374.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;

public class Drivetrain {
	//this state machine might not be necessary, but whatever
	public static final int STATE_DRIVER_CONTROL=0;
	public static final int STATE_AUTO_ALIGN=1;
	
	static final double ENCODER_COUNTS_TO_FEET=0.05;//for prototype robot
	
	static final double ANGULAR_ADJUSTMENT_SCALE=0.02;//motor speed per degree
	static final double ANGULAR_ADJUSTMENT_MAX=0.5;//maximum angle speed
	
	static final double AUTO_SPEED_SCALE=0.3;//motor speed per foot
	
	Jaguar l1, l2, r1, r2;//jaguars, pretty normal
	CalibratedGyro gyro;//sensors
	Encoder encoder;
	int state;
	
	double targetHeading, targetDistance; //used for automatic movement
	
	
	ArrayList<AutoCommand> commandList; //a list of all commands provided to the robot
	
	public Drivetrain(){
		l1=new Jaguar(0);//ports, to be changed later
		l2=new Jaguar(1);
		r1=new Jaguar(2);
		r2=new Jaguar(3);
		
		gyro=new CalibratedGyro(0);//more ports :P
		
		encoder=new Encoder(1,2);
		
		commandList=new ArrayList<AutoCommand>();
		
		targetHeading=0;
	}
	
	public void update(double lspeed, double rspeed){
		if(Math.max(Math.abs(lspeed), Math.abs(rspeed))>0.5){//if the driver does anything
			state=STATE_DRIVER_CONTROL;//give the driver control again
		}
		
		if(state==STATE_AUTO_ALIGN){
			//follow the command- if there are none left, switch back to driver control
			if(followCommand())state=STATE_DRIVER_CONTROL;
		}
		if(state==STATE_DRIVER_CONTROL){
			commandList.clear();//clear commands
			preciseTank(lspeed,rspeed);//precise tank drive movement
			targetHeading=gyro.getAngle();//sets base values for sensors
			targetDistance=getEncoderFeet();
		}
	}
	
	public boolean followCommand(){
		if(commandList.size()==0)return true;//if there aren't any commands, exit with "true"
		
		//gets the first command in the list
		AutoCommand ac=commandList.get(0);
		
		//computes the difference between the target angle/position and the current angle/position
		double angleDifference=ac.direction-gyro.getAngle();
		double posDifference=ac.distance-getEncoderFeet();
		
		//these values will eventually be what the motors are set to
		double speed=0;
		double turnSpeed=0;
		//angular adjustment, basic PID algorithm (P only)
		if(Math.abs(angleDifference)>3){
			turnSpeed=angleDifference*ANGULAR_ADJUSTMENT_SCALE;
			
			//scales for maximum and minimum values
			if(Math.abs(turnSpeed)>ANGULAR_ADJUSTMENT_MAX)turnSpeed=ANGULAR_ADJUSTMENT_MAX*Math.signum(turnSpeed);
			//0.4 seems to be enough to overcome the carpet's friction
			if(Math.abs(turnSpeed)<0.4)turnSpeed=0.4*Math.signum(turnSpeed);
		}
		//same algorithm for position adjustment
		if(Math.abs(posDifference)>0.5){
			speed=posDifference*AUTO_SPEED_SCALE;
			
			if(Math.abs(speed)>ac.speed)speed=ac.speed*Math.signum(speed);
			//0.3 also seems sufficient
			if(Math.abs(speed)<0.3)speed=0.3*Math.signum(speed);
		}
		
		//are we in position?
		if(Math.abs(angleDifference)<=3 && Math.abs(posDifference)<=0.5){
			//stop
			setMotors(0,0);
			//remove the command from the list
			commandList.remove(ac);
			//if there are no more commands, we're done
			return commandList.size()==0;
		}
		setMotors(speed+turnSpeed,speed-turnSpeed);
		
		return false;//there are more commands to follow
	}
	
	public void alignWithCrate(VisionReport vision){
		//aligns the robot to a crate, roughly
		if(state==STATE_DRIVER_CONTROL){//so we don't call it a bunch of times
			gyro.calibrate();//NOTE: takes 100 ms, and robot CANNOT be moving
			
			//turn 30 degrees in the target direction
			this.turnToHeading(30*Math.signum(vision.horizontalOffset));
			
			//move twice the horizontal offset (since sin(30)=0.5)
			this.moveDistance(Math.max(Math.abs(vision.horizontalOffset)*2,0.5));
			
			//turn back to the original heading
			this.turnToHeading(0);
		}
	}
	
	public double getEncoderFeet(){
		//returns the distance traveled, in feet
		return (double)getEncoderAdjusted()*ENCODER_COUNTS_TO_FEET;
	}
	
	public int getEncoderAdjusted(){
		//adjusts the encoder's values by the angle of the gyroscope
		//turning the robot in place changes the encoder's value, making this necessary
		return encoder.get()+(int)(gyro.getAngle()/6);
	}
	
	public void turnToHeading(double heading){
		//turns to the targeted heading, relative to the gyro's last calibration point
		
		state=STATE_AUTO_ALIGN;//so the driver can't screw things up
		targetHeading=heading;//this is used to specify the direction in future movement commands
		
		//creates a command based on the desired values
		commandList.add(new AutoCommand(targetDistance,heading,0.5));//0.5 is arbitrary
	}
	public void moveDistance(double feet){
		//moves the robot X feet in the direction last provided
		state=STATE_AUTO_ALIGN;
		targetDistance+=feet;
		commandList.add(new AutoCommand(targetDistance,targetHeading,0.5));
	}
	
	public void preciseTank(double lspeed, double rspeed){
		//an experimental algorithm to make tank drive as good at going straight/turning in place as arcade drive
		
		//converts left/right values into forwards/turn values
		double forwards=(lspeed+rspeed)/2;
		double turn=(lspeed-rspeed)/2;
		
		//scales those values
		forwards=quadraticScale(forwards);
		turn=deadbandScale(turn);
		
		//turns them back into left/right, sets motors
		setMotors(forwards+turn,forwards-turn);
	}
	
	double quadraticScale(double value){
		//scales the value quadratically, good for driving
		return value*Math.abs(value);
	}
	double deadbandScale(double value){
		//scales the value according to a deadband
		//Ian thinks this is better for rotation
		double deadband=0.1;
		if(value>deadband)return (value-deadband)/(1-deadband);
		else if(value<-deadband)return (value+deadband)/(1-deadband);
		else return 0;
	}
	
	public void setMotors(double lspeed, double rspeed){
		double ls2=lspeed;
		double rs2=rspeed;
		//normalize speeds that are too high
		if(Math.abs(ls2)>1){
			rs2/=Math.abs(ls2);
			ls2/=Math.abs(ls2);
		}
		if(Math.abs(rs2)>1){
			ls2/=Math.abs(rs2);
			rs2/=Math.abs(rs2);
		}
		//set the motors as usual
		l1.set(-ls2);
		l2.set(-ls2);
		r1.set(rs2);
		r2.set(rs2);
	}
	public void resetGyro(){
		gyro.reset();
	}
}

class AutoCommand{
	//A simple class to make storing commands in a list more convenient
	double distance, direction, speed;
	public AutoCommand(double distance, double direction, double speed){
		this.distance=distance;
		this.direction=direction;
		this.speed=speed;
	}
}

