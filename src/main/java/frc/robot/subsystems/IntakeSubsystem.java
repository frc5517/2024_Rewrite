// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Telemetry;
import frc.robot.Telemetry.RobotTelemetry;

public class IntakeSubsystem extends SubsystemBase {
    // Create intake and shooter motors.
    CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.intakeMotorPort, MotorType.kBrushed);
    CANSparkMax shooterMotor = new CANSparkMax(ManipulatorConstants.shooterMotorPort, MotorType.kBrushed);

    public IntakeSubsystem() {
        // Set motor inversions.
        intakeMotor.setInverted(false);
        shooterMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // Does nothing.

        if (Telemetry.robotVerbosity == RobotTelemetry.HIGH) {
            SmartDashboard.putNumber("Intake Applied Output", intakeMotor.getAppliedOutput());
            SmartDashboard.putNumber("Shooter Applied Output", shooterMotor.getAppliedOutput());
        }

    }

    /**
     * Run the intake at set speed.
     *
     * @param speed to intake note.
     * @return A {@link Command} to run the intake for set time.
     */
    public Command IntakeCommand(Double speed) {
        return runEnd(() -> {
                    intakeMotor.set(speed); // outtake note without sensor
                }, () -> {
                    intakeMotor.stopMotor();  // stop intake when done.
                }
        );
    }

    /**
     * Shoot the note at given speeds and ramp up time.
     *
     * @param shooterSpeed to shoot the note.
     * @param intakeSpeed  to index into the shooter.
     * @param time         to ramp up shooter.
     * @return A {@link Command} to shoot the note.
     */
    public Command ShootCommand(double shooterSpeed, double intakeSpeed, double time) {
        return runEnd(() -> {
                    shooterMotor.set(shooterSpeed); // run shooter at set speed.
                    Timer.delay(time);  // ramp up shooter for time.
                    intakeMotor.set(intakeSpeed); // index the note into the shooter at set speed.

                }, () -> {  // stop motors when done.
                    shooterMotor.stopMotor();
                    intakeMotor.stopMotor();
                }
        );
    }
}
