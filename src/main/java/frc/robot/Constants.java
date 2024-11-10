// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public final class Constants 
{
    /**
     * Contant values for operating the robot.
     */
    public static final class OperatorConstants {
        // Joystick Deadbands
        public static final double LEFT_X_DEADBAND  = 0.05;
        public static final double LEFT_Y_DEADBAND  = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.05;
        public static final double TURN_CONSTANT    = 6;
    }

    /**
     * Contant values for the autonomous systems.
     */
    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0); // PID values for X Y movement in Pathplanner.
        public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);  // PID values for rotation in Pathplanner.
    }

    /**
     * Constant values for the swerve systems.
     */
    public static final class SwerveConstants {
        public static final double maxSpeed = Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration.
        public static final double visionSpeedMultiple = .4; // Multiplier used to limit robot speed while tracking note (speed * visionSpeedMultiple).
        public static final double lowSpeedMultiple = .3;   // Multiplier used to limit robot speed while held (speed * lowSpeedMultiple).
        public static final double baseSpeedMultiple = .5;  // Multiplier used to limit robot speed while held (speed * baseSpeedMultiple).
        public static final double highSpeedMultiple = .8;  // Multiplier used to limit robot speed while held (speed * highSpeedMultiple).
        public static final PIDConstants visionTurnPID = new PIDConstants(0.85, 0, 0);  // PID Turn values for vision tracking.
        public static final PIDConstants visionForwardPID = new PIDConstants(1.4, 0, 0);    // PID Forward values for vision tracking.
    }

    /**
     * Constant values for game piece manipulating systems on the robot.
     */
    public static final class ManipulatorConstants {
        public static final int intakeMotorPort = 10; // Brushed intake motor with spark max.
        public static final int shooterMotorPort= 11; // Brushed shooter motor with spark max.
        public static final int leftArmMotorPort = 12; // Brushless arm motor with spark max.
        public static final int rightArmMotorPort = 13; // Brushless arm motor with spark max.

        public static final PIDConstants armPID = new PIDConstants(0.8, 0, 0);  // PID values for arm control.
        public static final double armFF = 1.0; // PID FeedForward value for arm control.
        public static final double armMaxOutput = 0.4;  // Max speed for arm PID control.
        public static final double armMinOutput = -0.4; // Max speed for arm PID control.
    }

    /**
     * Constant values for robot climb system.
     */
    public static final class ClimbConstants {
        public static final int climbMotorPort = 14;  // Brushless climb motor with spark max.
        public static final int climbLimitPort = 2; // Limit switch port.
    }
}
