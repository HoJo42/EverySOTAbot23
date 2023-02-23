// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AdjustArm;
import frc.robot.commands.ArmPID;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ControlIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem(); 
  
  private WPI_TalonSRX leftDriveMotor = new WPI_TalonSRX(Constants.DriveTrain.LEFT_MOTOR_PORT);
  private WPI_TalonSRX leftDriveMotorOther = new WPI_TalonSRX(Constants.DriveTrain.LEFT_MOTOR_PORT_OTHER);
  private WPI_TalonSRX rightDriveMotor = new WPI_TalonSRX(Constants.DriveTrain.RIGHT_MOTOR_PORT);
  private WPI_TalonSRX rightDriveMotorOther = new WPI_TalonSRX(Constants.DriveTrain.RIGHT_MOTOR_PORT_OTHER);

  private MotorControllerGroup leftDriveMotors = new MotorControllerGroup(leftDriveMotor, leftDriveMotorOther);
  private MotorControllerGroup rightDriveMotors = new MotorControllerGroup(rightDriveMotor, rightDriveMotorOther);

  private CANSparkMax arm = new CANSparkMax(Constants.Arm.ARM_MOTOR_PORT, MotorType.kBrushless);
  private CANSparkMax intake = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_PORT, MotorType.kBrushless);

  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0); 

  private PIDController armPidController = new PIDController(Constants.Arm.PID_P, Constants.Arm.PID_I, Constants.Arm.PID_D);

  
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private Drive m_Drive = new Drive(leftDriveMotors, rightDriveMotors);
  private Arm m_Arm = new Arm(arm, armEncoder, armPidController);
  private Intake m_Intake = new Intake(intake);

  private AdjustArm m_AdjustArm = new AdjustArm(m_Arm, m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftDriveMotor.setInverted(true);
    leftDriveMotorOther.setInverted(true);
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

    m_driverController.x().onTrue(m_AdjustArm);
    m_driverController.b().onTrue(m_AdjustArm);
  }

  private void ConfigureDefaultCommands(){
    m_Drive.setDefaultCommand(new DriveCommand(m_Drive, m_driverController));
    m_Arm.setDefaultCommand(new ArmPID(armPidController, m_Arm, armEncoder));
    m_Intake.setDefaultCommand(new ControlIntake(m_Intake, m_driverController));
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
