/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {


	//MOTORS
	
	public static final int FrontRightDrivePWM = 0;
	public static final int FrontRightTurnCAN = 0;
	public static final int FrontLeftDrivePWM = 1;
	public static final int FrontLeftTurnCAN = 1;
	public static final int BackLeftDrivePWM = 2;
	public static final int BackLeftTurnCAN = 2;
	public static final int BackRightDrivePWM = 3;
	public static final int BackRightTurnCAN = 3;
	
	//speed modifiers
	
	//ENCODERS
	
	public static final int FrontRightEncoderAI = 0;
	public static final int FrontLeftEncoderAI = 1;
	public static final int BackLeftEncoderAI = 2;
	public static final int BackRightEncoderAI = 3;
	
	//Encoder Zeroing
	public static final double SpeedModifier = 0.6;
	public static final double KP = 0.02;
	public static final double KI = 0.0;
	public static final double KD = 0.00;
	public static final double deadzone = 0.2;
	

	//robot stuff
	public static final double Length = 23.0;
	public static final double Width = 25.0;
	

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
