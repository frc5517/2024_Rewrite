// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ManipulatorSuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.io.File;

public class RobotContainer {
    // Calls the subsystems.
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final IntakeSubsystem intakebase = new IntakeSubsystem();
    private final ArmSubsystem armbase = new ArmSubsystem();
    private final ManipulatorSuperStructure superStructure = new ManipulatorSuperStructure(armbase, intakebase);


    // Creates the controllers.
    CommandXboxController driverXbox = new CommandXboxController(0);
    CommandXboxController operatorXbox = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
        // Creating the robot centric swerve drive
        Command closedDrive = drivebase.customDriveCommand(false,
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox.leftBumper(),
                driverXbox.rightBumper());

        // Creating the field centric swerve drive
        Command fieldDrive = drivebase.customDriveCommand(true,
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox.leftBumper(),  // Left bumper for slow speed
                driverXbox.rightBumper());  // Right bumper for high speed

        drivebase.setDefaultCommand(closedDrive); // Set default drive command from chooser.

        // when top limit is hit stop arm reset angle then wait 2 seconds for safety, depending on behaviour may be removed.
        armbase.limitHit().onTrue(armbase.stopArm().alongWith(armbase.ArmEncoderUp()).andThen(Commands.waitSeconds(2)));    // New with superstructure

        // Also new with superstructure \/
        operatorXbox.a().whileTrue(superStructure.angleArmAndIntake());
        operatorXbox.y().whileTrue(superStructure.angleArmAndShootHigh());


        // Driver Controls
        driverXbox.leftTrigger(.3).toggleOnTrue(fieldDrive); // Toggle robot centric swerve drive.
        driverXbox.y().whileTrue(drivebase.goToNotePID()); // Drive to note with vision.
        driverXbox.x().whileTrue(drivebase.aimAtSpeaker(2));
        driverXbox.start().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    // Lock drive train to limit pushing.
        driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro)); // Zero the gyro to avoid odd drive due to gyro drift.
    }

    public void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.none(); // Gets Selected Auton. // Disabled in this code.
    }
}