// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ControlIntake extends CommandBase {
  private final Intake m_Intake;
  private final CommandXboxController m_driverController;

  private final int CONE = 1;
  private final int CUBE = 2;
  private final int NOTHING = 3;

  /** Creates a new controlIntake. */
  public ControlIntake(Intake intake, CommandXboxController dStick) {
    m_Intake = intake;
    m_driverController = dStick;

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int lastGamePiece = NOTHING;
    double intakePower;
    int intakeAmps;
    if (m_driverController.y().getAsBoolean()) {
      // cube in or cone out
      intakePower = Constants.Intake.INTAKE_OUTPUT_POWER;
      intakeAmps = Constants.Intake.INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (m_driverController.b().getAsBoolean()) {
      // cone in or cube out
      intakePower = -Constants.Intake.INTAKE_OUTPUT_POWER;
      intakeAmps = Constants.Intake.INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = Constants.Intake.INTAKE_HOLD_POWER;
      intakeAmps = Constants.Intake.INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -Constants.Intake.INTAKE_HOLD_POWER;
      intakeAmps = Constants.Intake.INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    m_Intake.setIntakeMotor(intakePower, intakeAmps);
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
