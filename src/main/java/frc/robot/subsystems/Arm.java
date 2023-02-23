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
    DesiredLocation = Constants.Arm.ARM_RETRACTED;
    m_armMotor = arm;
    armEncoder = encoder;
    m_armPID = armPID;
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
    m_armMotor.set(speed);
  }
  
  public void changeDesired(boolean direction){
    if (direction && armEncoder.get() < Constants.Arm.ARM_EXTENDED){
      DesiredLocation += 0.05;
    }else if(!direction && armEncoder.get() > Constants.Arm.ARM_RETRACTED){
      DesiredLocation -= 0.05;
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
    }
}
