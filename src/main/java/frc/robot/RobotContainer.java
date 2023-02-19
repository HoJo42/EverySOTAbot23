// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.lowerArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); //TODO: remove this
  
  private WPI_TalonSRX leftDriveMotor = new WPI_TalonSRX(Constants.DriveTrain.LEFT_MOTOR_PORT);
  private WPI_TalonSRX leftDriveMotorOther = new WPI_TalonSRX(Constants.DriveTrain.LEFT_MOTOR_PORT_OTHER);
  private WPI_TalonSRX rightDriveMotor = new WPI_TalonSRX(Constants.DriveTrain.RIGHT_MOTOR_PORT);
  private WPI_TalonSRX rightDriveMotorOther = new WPI_TalonSRX(Constants.DriveTrain.RIGHT_MOTOR_PORT_OTHER);

  private MotorControllerGroup leftDriveMotors = new MotorControllerGroup(leftDriveMotor, leftDriveMotorOther);
  private MotorControllerGroup rightDriveMotors = new MotorControllerGroup(rightDriveMotor, rightDriveMotorOther);

  private CANSparkMax arm = new CANSparkMax(Constants.Arm.ARM_MOTOR_PORT, MotorType.kBrushless);
  private CANSparkMax intake = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_PORT, MotorType.kBrushless);

  private Drive m_Drive = new Drive(leftDriveMotors, rightDriveMotors);
  private Arm m_Arm = new Arm(arm);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    ConfigureDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(new lowerArm(m_Arm));

  }

  private void ConfigureDefaultCommands(){
    m_Drive.setDefaultCommand(new DriveCommand(m_Drive, m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
