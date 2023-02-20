// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;

  /** Creates a new Intake. */
  public Intake(CANSparkMax intakeMotor) {
    m_intakeMotor = intakeMotor;
    
  }

  public void setIntakeMotor(double percent, int amps){
    m_intakeMotor.set(percent);
    m_intakeMotor.setSmartCurrentLimit(amps);
  }

  @Override
  public void periodic() {
    
  }
}
