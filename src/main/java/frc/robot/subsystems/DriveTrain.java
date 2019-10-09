/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {

  private TalonSRX frontRightTurn,  frontLeftTurn, backLeftTurn, backRightTurn;
	private SpeedController  frontRightDrive, backLeftDrive,  backRightDrive, frontLeftDrive;
	public AnalogInput frontRightEncoder, frontLeftEncoder, backLeftEncoder, backRightEncoder;
	private Gyro gyro;
	public DriveTrain() {
		super();
		frontRightDrive = new VictorSP(RobotMap.FrontRightDrivePWM);
	
		frontRightTurn = new TalonSRX(RobotMap.FrontRightTurnCAN);
		frontLeftDrive = new VictorSP(RobotMap.FrontLeftDrivePWM);
		frontLeftTurn = new TalonSRX(RobotMap.FrontLeftTurnCAN);
		backLeftDrive = new VictorSP(RobotMap.BackLeftDrivePWM);
		backLeftTurn = new TalonSRX(RobotMap.BackLeftTurnCAN);
		backRightDrive = new VictorSP(RobotMap.BackRightDrivePWM);
		backRightTurn = new TalonSRX(RobotMap.BackRightTurnCAN);
		
		frontRightEncoder = new AnalogInput(RobotMap.FrontRightEncoderAI);
		frontLeftEncoder = new AnalogInput(RobotMap.FrontLeftEncoderAI);
		backRightEncoder = new AnalogInput(RobotMap.BackRightEncoderAI);
		backLeftEncoder = new AnalogInput(RobotMap.BackLeftEncoderAI);
		
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
		
	}
	
	public void drive(double[] speed){
		frontRightDrive.set(speed[0]*RobotMap.SpeedModifier);
		frontLeftDrive.set(speed[1]*RobotMap.SpeedModifier);
		backLeftDrive.set(-speed[2]*RobotMap.SpeedModifier);
		backRightDrive.set(-speed[3]*RobotMap.SpeedModifier);
	}
	
	public void drive(double speed){
		frontRightDrive.set(speed*RobotMap.SpeedModifier);
		frontLeftDrive.set(speed*RobotMap.SpeedModifier);
		backRightDrive.set(speed*RobotMap.SpeedModifier);
		backLeftDrive.set(speed*RobotMap.SpeedModifier);
	}
	
	public void turn(double[] speed){
		frontRightTurn.set(ControlMode.PercentOutput,speed[0]);
		frontLeftTurn.set(ControlMode.PercentOutput,speed[1]);
		backLeftTurn.set(ControlMode.PercentOutput,speed[2]);
		backRightTurn.set(ControlMode.PercentOutput,speed[3]);
	}
	
	public double getAngle(){
		return gyro.getAngle();
	}
	
	public double[] reportEncoders(){
		/*double[] encoders = {Math.round(((frontRightEncoder.getVoltage()+RobotMap.FrontRightEncoderZero))*Math.PI*200d)/500d+Math.PI,
		Math.round(((frontLeftEncoder.getVoltage()+RobotMap.FrontLeftEncoderZero))*Math.PI*200d)/500d+Math.PI,
		Math.round(((backRightEncoder.getVoltage()+RobotMap.BackRightEncoderZero))*Math.PI*200d)/500d+Math.PI,
		Math.round(((backLeftEncoder.getVoltage()+RobotMap.BackLeftEncoderZero))*Math.PI*200d)/500d+Math.PI};
		*/
		/*
		double[] encoders = {Math.round(frontRightEncoder.getVoltage()*Math.PI*2)/4.95,
			Math.round(frontLeftEncoder.getVoltage()*Math.PI*2)/4.95,
			Math.round(backLeftEncoder.getVoltage()*Math.PI*2)/4.95,
			Math.round(backRightEncoder.getVoltage()*Math.PI*2)/4.95};
			*/
		double[] encoders = {frontRightEncoder.getVoltage()*360/4.95-180,
			frontLeftEncoder.getVoltage()*360/4.95-180,
			backLeftEncoder.getVoltage()*360/4.95-180,
			backRightEncoder.getVoltage()*360/4.95-180};
				
		SmartDashboard.putNumber("FrontRightEncoder", frontRightEncoder.getVoltage());
		SmartDashboard.putNumber("FrontLeftEncoder", frontLeftEncoder.getVoltage());
		SmartDashboard.putNumber("BackRightEncoder", backRightEncoder.getVoltage());
		SmartDashboard.putNumber("BackLeftEncoder", backLeftEncoder.getVoltage());

		
		SmartDashboard.putNumber("FrontRightEncoderADJ", encoders[0] );
		SmartDashboard.putNumber("FrontLeftEncoderADJ", encoders[1]);
		SmartDashboard.putNumber("BackRightEncoderADJ", encoders[2]);
		SmartDashboard.putNumber("BackLeftEncoderADJ", encoders[3]);
		
    return encoders;
    }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive());
  }
}
