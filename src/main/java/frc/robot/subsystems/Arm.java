// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax m_armMotor;
  private DutyCycleEncoder armEncoder;
  private PIDController m_armPID;

  private double DesiredLocation;

  /** Creates a new Arm. */
  public Arm(CANSparkMax arm, DutyCycleEncoder encoder, PIDController armPID) {
    
    m_armMotor = arm;
    armEncoder = encoder;
    m_armPID = armPID;

    DesiredLocation = Constants.Arm.ARM_RETRACTED;
  }

  public void lowerArm(){
    if (armEncoder.get() > Constants.Arm.ARM_RETRACTED){
      m_armMotor.set(-Constants.Arm.ARM_OUTPUT_POWER);
    }else {
      m_armMotor.set(0);
    }
  }

  public void raiseArm(){
    if (armEncoder.get() < Constants.Arm.ARM_EXTENDED){
      m_armMotor.set(Constants.Arm.ARM_OUTPUT_POWER);
    }else {
      m_armMotor.set(0);
      
    }
  }

  public void setSpeed (double speed){
    SmartDashboard.putNumber("Arm Speed", speed);
    SmartDashboard.putNumber("desired position", DesiredLocation);
    if (speed < 0 && armEncoder.get() > Constants.Arm.ARM_EXTENDED){
      SmartDashboard.putString("First Check", "Passed!");
      m_armMotor.set(speed);
    }else if(speed > 0 && armEncoder.get() < Constants.Arm.ARM_RETRACTED){
      SmartDashboard.putString("Second Check", "Passed!");
      m_armMotor.set(speed);
    }else{
      SmartDashboard.putString("Else", "Failed!");
      m_armMotor.set(0);
    }
  }
  
  public void changeDesired(boolean wantUpwards){
    if (wantUpwards && armEncoder.get() > Constants.Arm.ARM_EXTENDED + Constants.Arm.ADJUSTMENT_INCREMENT){
      DesiredLocation -= Constants.Arm.ADJUSTMENT_INCREMENT;
    }else if(!wantUpwards && armEncoder.get() < Constants.Arm.ARM_RETRACTED - Constants.Arm.ADJUSTMENT_INCREMENT){
      DesiredLocation += Constants.Arm.ADJUSTMENT_INCREMENT;
    }
  }

  public void stopArm() {
    m_armMotor.set(0);
  }

  public double getDesired(){
    return DesiredLocation;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder count:", armEncoder.get());
    if (DesiredLocation < Constants.Arm.ARM_EXTENDED){
      DesiredLocation = Constants.Arm.ARM_EXTENDED;
    }else if (DesiredLocation > Constants.Arm.ARM_RETRACTED){
      DesiredLocation = Constants.Arm.ARM_RETRACTED;
    }
    }
}
