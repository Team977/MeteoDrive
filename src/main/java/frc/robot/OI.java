/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public static Joystick joy = new Joystick(0);
	
	
	public static double[] getJoy(){
		double[] rawValues = {-joy.getRawAxis(1), joy.getRawAxis(0),0, joy.getRawAxis(4)};
		double[] adjValues= {0,0,0,0};
    double[] newValues= {0,0,0,0};
		for (int i=0; i<4; i++){
			if (Math.abs(rawValues[i]) >= RobotMap.deadzone){
				adjValues[i] = Math.signum(rawValues[i])*(Math.abs(rawValues[i]) - RobotMap.deadzone)/(1 - RobotMap.deadzone);
				adjValues[i] = Math.signum(adjValues[i])*adjValues[i]*adjValues[i];
			}
			else{
				adjValues[i] = 0;
			}
		}
		/*
		for (int i=0; i<4; i++){
			adjValues[i] = Math.signum(rawValues[i])*rawValues[i]*rawValues[i];
		}
    */
    double angle = Math.toRadians(Robot.RobotDrive.getAngle());
    newValues[1] = adjValues[1]*Math.cos(angle)-adjValues[0]*Math.sin(angle);
    newValues[0] = adjValues[1]*Math.sin(angle)+adjValues[0]*Math.cos(angle);
    newValues[2] = adjValues[2];
    newValues[3] = adjValues[3];
    return newValues;
    //return adjValues;
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  }
