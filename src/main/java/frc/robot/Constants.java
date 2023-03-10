// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {//TODO: CHECK MOTOR PORTS BEFORE PUSHING TO ROBOT!!!
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveTrain {
    public static final int LEFT_MOTOR_PORT = 0;
    public static final int LEFT_MOTOR_PORT_OTHER = 1;
    public static final int RIGHT_MOTOR_PORT = 2;
    public static final int RIGHT_MOTOR_PORT_OTHER = 3;

    public static final double SPEED_CONTROL = 0.6;
  }
  public static class Arm {
    public static final int ARM_MOTOR_PORT = 4;
    public static final double ARM_OUTPUT_POWER = 0.4;
  }
  public static class Intake {
    public static final int INTAKE_MOTOR_PORT = 5;
    public static final double INTAKE_OUTPUT_POWER = 1.0;
    public static final int INTAKE_CURRENT_LIMIT_A = 25;
    public static final double INTAKE_HOLD_POWER = 0.07;
    public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
  }
}
