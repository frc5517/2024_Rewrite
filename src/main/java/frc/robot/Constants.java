// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants 
{
    public static final class OperatorConstants {
        // Joystick Deadbands
        public static final double LEFT_X_DEADBAND  = 0.05;
        public static final double LEFT_Y_DEADBAND  = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.05;
        public static final double TURN_CONSTANT    = 6;
    }

    public static final class AutonConstants {
        
    }

    public static final class SwerveConstants {
        public static final double visionTurnP = 0.85;  // PID Turn Value for vision tracking
        public static final double visionForwardP = 1.4;    // PID Forward Value for vision tracking
    }

    public static final class ManipulatorConstants {
        public static final int intakeMotorPort = 10; // brushed intake motor with spark max
        public static final int shooterMotorPort= 11; // brushed shooter motor with spark max
        public static final int leftArmMotorPort = 12; // brushless arm motor with spark max
        public static final int rightArmMotorPort = 13; // brushless arm motor with spark max

        public static final double armP = 0.8;  // PID value for arm control
        public static final double armFF = 1.0; // PID FeedForward value for arm control
        public static final double armMaxOutput = 0.4;  // Max speed for arm PID control
        public static final double armMinOutput = -0.4; // Max speed for arm PID control
    }

    public static final class ClimbConstants {
        public static final int climbMotorPort = 14;  // Brushless climb motor with spark max
        public static final int climbLimitPort = 2; // Limit switch port
    }
}
