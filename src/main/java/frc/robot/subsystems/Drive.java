// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  DifferentialDrive diffDrive;

  /** Creates a new Drive. */
  public Drive(MotorControllerGroup leftDriveMotors, MotorControllerGroup rightDriveMotors) {
    diffDrive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
  }

  public void drive(double frontBack, double leftRight){
    diffDrive.arcadeDrive(frontBack * Constants.DriveTrain.SPEED_CONTROL,leftRight * Constants.DriveTrain.SPEED_CONTROL);
  }
}
