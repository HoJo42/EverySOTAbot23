// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class moveArm extends CommandBase {
  private final Arm m_Arm;
  private final CommandXboxController m_driverController;

  /** Creates a new moveArm. */
  public moveArm(Arm arm, CommandXboxController dStick) {
    m_Arm = arm;
    m_driverController = dStick;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driverController.x().getAsBoolean()){
      m_Arm.raiseArm();
    }else if (m_driverController.a().getAsBoolean()){
      m_Arm.lowerArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
