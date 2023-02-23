// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends CommandBase {
  private DutyCycleEncoder m_armEncoder;
  private Arm m_Arm;



  /** Creates a new ArmPID. */
  public ArmPID(double setpoint, DutyCycleEncoder encoder, Arm arm) {
    m_armEncoder = encoder;
    m_Arm = arm;
    new PIDCommand(new PIDController(5, 0, 0),
     () -> m_armEncoder.get(),
    setpoint,
    (it) -> m_Arm.setSpeed(it), m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
