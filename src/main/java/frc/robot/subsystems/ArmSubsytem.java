// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsytem extends SubsystemBase {
  // Create the arms motors
  CANSparkMax leftMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);

  // Create the encoder and PID Controller
  SparkPIDController armController;
  RelativeEncoder armEncoder;

  // Create the limit switches
  DigitalInput topLimit = new DigitalInput(0);
  DigitalInput bottomLimit = new DigitalInput(1);

  public ArmSubsytem() {
    // Set motors to brake when not receiving input
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    // Create encoder and PID controller
    armEncoder = leftMotor.getEncoder();
    armController = leftMotor.getPIDController();

    // Enable PID wrapping allowing the motor to move to a set position
    armController.setPositionPIDWrappingEnabled(true);

    // Set the right motor mirror the left motors movements
    rightMotor.follow(leftMotor, true);

    // Setup PID and Feedforward for the armController and flash motor settings
    armController.setP(ManipulatorConstants.armP);
    armController.setFF(ManipulatorConstants.armFF);
    armController.setOutputRange(ManipulatorConstants.armMinOutput, ManipulatorConstants.armMaxOutput);
    leftMotor.burnFlash();
  }

  @Override
  public void periodic() {  // Periodic is called continously

      if (topLimit.get()) { // We are going up and top limit is tripped so set position
          armEncoder.setPosition(82);

      } else if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so set position
          armEncoder.setPosition(0);

      } else {} // If neither do nothing

      // Push telemetry data to the NetworkTables
      SmartDashboard.putNumber("Arm Encoder Value", armEncoder.getPosition());
      SmartDashboard.putNumber("Arm Rotation Value", armEncoder.getPosition() * (1/2.25));
      SmartDashboard.putBoolean("Top Arm Limit", topLimit.get());
      SmartDashboard.putBoolean("Bottom Arm Limit", bottomLimit.get());
  }

  public Command ArmCommand(double speed) {
    return runEnd(() -> {
      if (speed > 0) {  // If speed is greater than zero

        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftMotor.stopMotor();
            armEncoder.setPosition(82);

        } else {  // We are going up but top limit is not tripped so go at commanded speed
            armController.setReference(speed, ControlType.kDutyCycle);

        } } else {  // Else speed is less than zero (going in reverse)

        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftMotor.stopMotor();
            armEncoder.setPosition(0);

        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            armController.setReference(speed, ControlType.kDutyCycle);
        } 
      }
    }, () -> {  // Command has finished, stop motors
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    });
  }

  public Command MoveToSetpoint(double setpoint) {
    return runEnd(() -> {
      if (leftMotor.getAppliedOutput() > 0) { // If speed is greater than zero

        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftMotor.stopMotor();
            armEncoder.setPosition(82);

        } else {  // We are going up but top limit is not tripped so go at commanded speed
            armController.setReference(setpoint, ControlType.kPosition);

        } } else {  // Else speed is less than zero (going in reverse)

        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftMotor.stopMotor();
            armEncoder.setPosition(0);

        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            armController.setReference(setpoint, ControlType.kPosition);
        } 
      }
    }, () -> {  // Command has finished, stop motors
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    });
  }

  public void ArmEncoderUp() {  // Resets arm encoder to the up position
    armEncoder.setPosition(82);
  }
}
