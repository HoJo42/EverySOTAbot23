// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPID extends PIDCommand {
  
  /** Creates a new ArmPID. */
  public ArmPID(PIDController armPID, Arm arm, DutyCycleEncoder armEncoder) {
    super(
        // The controller that the command will use
        armPID,
        // This should return the measurement
        () -> armEncoder.get(),
        // This should return the setpoint (can also be a constant)
        () -> arm.getDesired(),
        // This uses the output
        output -> {
          SmartDashboard.putNumber("Output", output);
          arm.setSpeed(MathUtil.clamp(output, -.75, .75));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(arm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
