// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class DriveCommand extends CommandBase {
  private final Drive m_Drive;
  private final CommandXboxController m_dStick;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drive drive, CommandXboxController dStick) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Drive = drive;
    m_dStick = dStick;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drive.drive(m_dStick.getLeftY(), m_dStick.getLeftX());
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
