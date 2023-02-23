// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax m_armMotor;
  private DutyCycleEncoder armEncoder;

  /** Creates a new Arm. */
  public Arm(CANSparkMax arm, DutyCycleEncoder encoder) {
    m_armMotor = arm;
    armEncoder = encoder;
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
    if((speed < 0 && armEncoder.get() > Constants.Arm.ARM_RETRACTED) || 
    (speed < 0 && armEncoder.get() < Constants.Arm.ARM_EXTENDED)){
      m_armMotor.set(speed);
    }else{
      m_armMotor.set(0);
    }
  }

  public void stopArm() {
    m_armMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder count:", armEncoder.get());
    SmartDashboard.putNumber("armEncder2", m_armMotor.getEncoder().getPosition());
    }
}
