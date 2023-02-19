// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax m_arm;

  /** Creates a new Arm. */
  public Arm(CANSparkMax arm) {
    m_arm = arm;
  }

  public void lowerArm(){
    m_arm.set(-Constants.Arm.ARM_OUTPUT_POWER);
  }

  public void raiseArm(){
    m_arm.set(Constants.Arm.ARM_OUTPUT_POWER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
