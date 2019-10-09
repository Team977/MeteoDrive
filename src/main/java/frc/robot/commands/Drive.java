/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Drive extends Command {
  private double[] rawAxis;
	private double vx;
	private double vy;
  //private double r;
  private double[] speed = {0,0,0,0};
  //private double theta = 0;
  private double[] angle = {0,0,0,0};
	private double[] error1 = {0,0,0,0};
	private double[] error2 = {0,0,0,0};
	private double[] error3 = {0,0,0,0};
	private double[] error = {0,0,0,0};
	private double[] pOut = {0,0,0,0};
	private double[] iOut = {0,0,0,0};
	private double[] dOut = {0,0,0,0};
	private double[] output = {0,0,0,0};
	private double[] currentEncoderPos = {0,0,0,0};
	private double[] delta_error = {0,0,0,0};
	private double[] prevError = {0,0,0,0};
  private double[] sumError = {0,0,0,0};
  private double omega = 0;
  private double a,b,c,d;
 
  

  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.RobotDrive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //get joystick
    rawAxis = OI.getJoy();    
    
    vx = rawAxis[0]*Math.sqrt(1-rawAxis[1]*rawAxis[1]/2);
    vy = rawAxis[1]*Math.sqrt(1-rawAxis[0]*rawAxis[0]/2);
    omega = rawAxis[3]/25;
    
    a = vx - omega*RobotMap.Length/2;
    b = vx + omega*RobotMap.Length/2;
    c = vy - omega*RobotMap.Width/2;
    d = vy + omega*RobotMap.Width/2;

    speed[2] = Math.sqrt(b*b + c*c);
    speed[1] = Math.sqrt(b*b + d*d);
    speed[0] = Math.sqrt(a*a + d*d);
    speed[3] = Math.sqrt(a*a + c*c);
 //See note #2 under Figure 6 from Ether's note
    if (Math.abs(speed[0])>1 || Math.abs(speed[1])>1 || Math.abs(speed[2])>1 || Math.abs(speed[3])>1){
      double maxspeed = 0; 
      for (int i=0; i<4; i++){
	      if (Math.abs(maxspeed) < Math.abs(speed[i])) {maxspeed = speed[i];}
      }
      for (int i=0; i<4; i++){
        speed[i] = speed[i]/maxspeed;
      }
    }
    
    //insert deadzone code here (don't update theta if r is close to zero)
    //if (speed[i] >= RobotMap.deadzone){
    angle[2] = Math.atan2(b,c)*180/Math.PI;
    angle[1] = Math.atan2(b,d)*180/Math.PI;
    angle[0] = Math.atan2(a,d)*180/Math.PI;
    angle[3] = Math.atan2(a,c)*180/Math.PI;

double gAngle= Robot.RobotDrive.getAngle();
SmartDashboard.putNumber("gyro",gAngle);

    /*angle[2] = Math.atan2(b,c)*180/Math.PI+gAngle;
    angle[1] = Math.atan2(b,d)*180/Math.PI+gAngle;
    angle[0] = Math.atan2(a,d)*180/Math.PI+gAngle;
    angle[3] = Math.atan2(a,c)*180/Math.PI+gAngle;
*/



    //}
    SmartDashboard.putNumber("speedFR",speed[0]);
    SmartDashboard.putNumber("speedFL",speed[1]);
    SmartDashboard.putNumber("speedBL",speed[2]);
    SmartDashboard.putNumber("speedBR",speed[3]);
    SmartDashboard.putNumber("angleFR",angle[0]);
    SmartDashboard.putNumber("angleFL",angle[1]);
    SmartDashboard.putNumber("angleBL",angle[2]);
    SmartDashboard.putNumber("angleBR",angle[3]);
    
    //read encoder positions
    currentEncoderPos = Robot.RobotDrive.reportEncoders();
    
    double[] tdelta_error = {0,0,0,0};
    double[] tsumError ={0,0,0,0};
    for (int i=0; i<4; i++){
     
      //generate error arrays
     // error[i] = angle[i] - currentEncoderPos[i];

      error1[i] = angle[i] - currentEncoderPos[i];
      error2[i] = angle[i] + 360 - currentEncoderPos[i];
      error3[i] = angle[i] - 360 - currentEncoderPos[i];
      

      if (Math.abs(error1[i]) < Math.abs(error2[i]) && Math.abs(error1[i]) < Math.abs(error3[i])){
        error[i]=error1[i];
      }
      else if (Math.abs(error2[i]) < Math.abs(error1[i]) && Math.abs(error2[i]) < Math.abs(error3[i])){
        error[i] = error2[i];
      }
      else {
        error[i] = error3[i];
      }
      
      //error[i] = Math.min(error1[i],error2[i]);
      
      tdelta_error[i] = prevError[i] - error[i];
      tsumError[i] += error[i];
        
      //generate pid array

      pOut[i] = error[i] * RobotMap.KP;
      iOut[i] = tsumError[i] * RobotMap.KI;
      dOut[i] = tdelta_error[i] * RobotMap.KD;
          
      //generate output array
      output[i] = pOut[i] + iOut[i] + dOut[i];
    
      prevError[i] = error[i];
      
    }    	
    SmartDashboard.putNumber("turn",output[1]);
    SmartDashboard.putNumber("drive",speed[1]);
    Robot.RobotDrive.turn(output);
    Robot.RobotDrive.drive(speed);
    /*
    //get joystick
    rawAxis = OI.getJoy();    
    	
    x = rawAxis[0]*Math.sqrt(1-rawAxis[1]*rawAxis[1]/2);
    y = rawAxis[1]*Math.sqrt(1-rawAxis[0]*rawAxis[0]/2);
    
    //convert to polar
    r = Math.sqrt(x*x + y*y);
    
    //insert deadzone code here (don't update theta if r is close to zero)
    if (r >= RobotMap.deadzone){
      theta = Math.atan2(y,-x) + Math.PI;
    }
        
    SmartDashboard.putNumber("r", r);
    SmartDashboard.putNumber("theta", theta);
        
    //read encoder positions
    currentEncoderPos = Robot.RobotDrive.reportEncoders();
            
    
    for (int i=0; i<4; i++){
      
      //generate error arrays

      //error1[i] = theta - currentEncoderPos[i];
      //error2[i] = theta + 2*Math.PI - currentEncoderPos[i];
      //error3[i] = theta - 2*Math.PI - currentEncoderPos[i];
      error1[i] = theta - currentEncoderPos[i];
      error2[i] = theta + 2*Math.PI - currentEncoderPos[i];
      error3[i] = theta - 2*Math.PI - currentEncoderPos[i];
      


      if (Math.abs(error1[i]) < Math.abs(error2[i]) && Math.abs(error1[i]) < Math.abs(error3[i])){
        error[i]=error1[i];
      }
      else if (Math.abs(error2[i]) < Math.abs(error1[i]) && Math.abs(error2[i]) < Math.abs(error3[i])){
        error[i] = error2[i];
      }
      else {
        error[i] = error3[i];
      }
      
      //error[i] = Math.min(error1[i],error2[i]);
      
      delta_error[i] = prevError[i] - error[i];
      sumError[i] += error[i];
        
      //generate pid array

      pOut[i] = error[i] * RobotMap.KP;
      iOut[i] = sumError[i] * RobotMap.KI;
      dOut[i] = delta_error[i] * RobotMap.KD;
          
      //generate output array
      output[i] = pOut[i] + iOut[i] + dOut[i];
    
      prevError[i] = error[i];
      
    }    	
    
    Robot.RobotDrive.turn(output);
    Robot.RobotDrive.drive(r);
   */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
