// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Telemetry;
import frc.robot.Telemetry.RobotTelemetry;

public class ClimbSubsystem extends SubsystemBase {
    // Create climb motor and climb encoder.
    CANSparkMax climbMotor = new CANSparkMax(ClimbConstants.climbMotorPort, MotorType.kBrushless);
    DigitalInput climbLimit = new DigitalInput(ClimbConstants.climbLimitPort);

    public ClimbSubsystem() {
        // Set motor to brake when not receiving input.
        climbMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {  // Periodic is called continuously.

        // Push telemetry data to the NetworkTables if verbosity is LOW or higher.
        if (Telemetry.robotVerbosity.ordinal() >= RobotTelemetry.LOW.ordinal()) {
            SmartDashboard.putBoolean("Climb Limit", climbLimit.get());
        }
        // Push telemetry data to the NetworkTables.
        if (Telemetry.robotVerbosity == RobotTelemetry.HIGH) {
            SmartDashboard.putNumber("Climb Position", climbMotor.getEncoder().getPosition());
        }
    }

    /**
     * Raise or lower the climb at given speed.
     *
     * @param speed How fast the climb moves. (E.g., 0.4)
     * @return a {@link Command} to move the climb at set speed.
     */
    public Command ClimbCommand(double speed) {
        return runEnd(() -> {

            if (speed < 0) {  // If speed is greater than zero.

                if (climbLimit.get()) { // We are going up and top limit is tripped so stop.
                    climbMotor.stopMotor();

                } else {  // We are going up but top limit is not tripped so go at commanded speed.
                    climbMotor.set(speed);

                }
            } else {  // Else speed is less than zero (reverse), go at commanded speed.
                climbMotor.set(speed);
            }
        }, () -> {  // Command is finished, stop motors.
            climbMotor.stopMotor();
        });
    }
}
