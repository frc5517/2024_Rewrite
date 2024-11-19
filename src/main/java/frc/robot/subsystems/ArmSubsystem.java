// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Telemetry;
import frc.robot.Telemetry.RobotTelemetry;

public class ArmSubsystem extends SubsystemBase {
    // Create the arms motors.
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;

    // Create the encoder and PID Controller.
    SparkPIDController armController;
    RelativeEncoder armEncoder;

    // Create the limit switches.
    DigitalInput topLimit;

    public ArmSubsystem()
    {
        leftMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);

        topLimit = new DigitalInput(0);

        // Set motors to brake when not receiving input.
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // Create encoder and PID controller.
        armEncoder = leftMotor.getEncoder();
        armController = leftMotor.getPIDController();

        // Enable PID wrapping allowing the motor to move to a set position.
        armController.setPositionPIDWrappingEnabled(true);
        armController.setSmartMotionAllowedClosedLoopError(1, 0);

        // Set the right motor mirror the left motors movements.
        rightMotor.follow(leftMotor, true);

        // Setup PID and Feedforward for the armController and flash motor settings.
        armController.setP(ManipulatorConstants.armPID.kP);
        armController.setFF(ManipulatorConstants.armFF);
        armController.setOutputRange(ManipulatorConstants.armMinOutput, ManipulatorConstants.armMaxOutput);
        leftMotor.burnFlash();
    }

    @Override
    public void periodic()
    {  // Periodic is called continuously.
        // Push telemetry data to the NetworkTables if verbosity is LOW or higher.
        if (Telemetry.robotVerbosity.ordinal() >= RobotTelemetry.LOW.ordinal()) {
            SmartDashboard.putBoolean("Top Arm Limit", topLimit.get());
        }
        // Push telemetry data to the NetworkTables if verbosity is high.
        if (Telemetry.robotVerbosity == RobotTelemetry.HIGH) {
            SmartDashboard.putNumber("Arm Encoder Value", armEncoder.getPosition());
            SmartDashboard.putNumber("Arm Rotation Value", armEncoder.getPosition() * (1 / 2.25));
        }
    }

    public Double getArmAngle()
    {
        return armEncoder.getPosition() * (1/2.25);
        // Find out the conversion factor to properly set as angle in degrees.
        // For now the angle is set as the setpoint value multiple.
    }

    public Trigger limitHit()
    {
        return new Trigger(() -> {
            return topLimit.get();
        });
    }

    public Command stopArm()
    {
        return run(() -> {
            leftMotor.set(0);
            rightMotor.set(0);
        });
    }

    public Trigger armAtAngle(Double bottomDegrees, Double topDegrees) 
    {
        return new Trigger(() -> {
            return this.getArmAngle() <= bottomDegrees && this.getArmAngle() >= topDegrees;
        });
    }

    public Command runArm(double setpoint , double tolerance)
    {
        return run(() -> {
            armController.setReference(setpoint, ControlType.kPosition);
        }).until(this.armAtAngle(setpoint - tolerance/2, setpoint + tolerance/2))
        .andThen(this.stopArm());
    }

    /**
     * Reset the arm encoder to the up position.
     */
    public Command ArmEncoderUp()
    {  // Resets arm encoder to the up position.
        return run(() -> {
            armEncoder.setPosition(82);
        });
    }
}
