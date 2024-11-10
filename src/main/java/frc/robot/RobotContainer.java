// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  // Calls the subsystems.
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intakebase = new IntakeSubsystem();
  private final ClimbSubsystem climbbase = new ClimbSubsystem();
  private final ArmSubsytem armbase = new ArmSubsytem();

  // Creates the auton sendable chooser.
  private final SendableChooser<Command> autoChooser;

  Boolean fieldRelative = false;

  // Creates the controllers.
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
    // Register the commands for FRC PathPlanner.
    NamedCommands.registerCommand("Raise Arm", armbase.MoveToSetpoint(15).withTimeout(1));
    NamedCommands.registerCommand("Lower Arm", armbase.MoveToSetpoint(.2).withTimeout(1));
    NamedCommands.registerCommand("Move To Setpoint", armbase.MoveToSetpoint(6).withTimeout(2));
    NamedCommands.registerCommand("Shoot High", intakebase.ShootCommand(1, .7, .5).withTimeout(2));
    NamedCommands.registerCommand("Shoot Low", intakebase.ShootCommand(.5, .3, .2).withTimeout(2));
    NamedCommands.registerCommand("Outdex Slightly", intakebase.IntakeCommand(-.3).withTimeout(.25));
    NamedCommands.registerCommand("Arm Encoder Up", new InstantCommand(armbase::ArmEncoderUp));
    NamedCommands.registerCommand("Reset IMU", new InstantCommand(drivebase::zeroGyro));

    autoChooser = AutoBuilder.buildAutoChooser(); // Builds auton sendable chooser for pathplanner.
    SmartDashboard.putData(autoChooser);  // Sends autoBuilder to smartdashboard.
    SmartDashboard.putBoolean("yes", fieldRelative);

    // Creating the robot centric swerve drive
    Command closedDrive = drivebase.customDriveCommand( 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftTrigger(.3),
    driverXbox.leftBumper(),
    driverXbox.rightBumper());

    // Creating the field centric swerve drive
    Command fieldDrive = drivebase.customDriveCommand( 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftTrigger().debounce(1),
    driverXbox.leftBumper(),  // Left bumper for slow speed
    driverXbox.rightBumper());  // Right bumper for high speed

    drivebase.setDefaultCommand(closedDrive); // Set default drive command to field centric drive

    driverXbox.rightTrigger().whileTrue(drivebase.goToNotePID()); // Drive to note with vision.

    // Driver Controls
    driverXbox.start().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    // Lock drive train to limit pushing.
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro)); // Zero the gyro to avoid odd drive due to gyro drift.

    // Operator Controls
    operatorXbox.y().whileTrue(armbase.ArmCommand(.3));  // Raise the arm.
    operatorXbox.a().whileTrue(armbase.ArmCommand(-.3)); // Lower the arm.
    operatorXbox.b().whileTrue(intakebase.IntakeCommand(-.3));  // Outtake the note.
    operatorXbox.leftBumper().whileTrue(intakebase.IntakeCommand(1.0)); // Run intake at speed.
    operatorXbox.x().whileTrue(intakebase.ShootCommand(.6, .5, .2)); // Spit the note into the amp.
    operatorXbox.rightBumper().whileTrue(intakebase.ShootCommand(1, .7, .7));  // Shoot the note into the speaker
    operatorXbox.start().whileTrue(climbbase.ClimbCommand(1)); // Spin the climb motor forwards.
    operatorXbox.back().whileTrue(climbbase.ClimbCommand(-1)); // Spin the climb motor in reverse. 
    operatorXbox.pov(0).whileTrue(armbase.MoveToSetpoint(6)); // Move the arm to setpoint. When held will oscillate around setpoint.
  }

  public void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); // Gets Selected Auton.
  }
}