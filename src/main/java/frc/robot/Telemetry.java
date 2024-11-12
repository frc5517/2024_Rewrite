// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Telemetry extends SubsystemBase {
    // Sets the verbosity level of robot telemetry.
    public static RobotTelemetry robotVerbosity = RobotTelemetry.HIGH;

    // Sets the verbosity level of swerve drive telemetry.
    public static TelemetryVerbosity swerveVerbosity = TelemetryVerbosity.HIGH;

    public enum RobotTelemetry {
        /*
         * No telemetry data is sent to dashboard.
         */
        NONE,

        /*
         * Only basic telemetry data is sent to dashboard.
         */
        LOW,

        /*
         * All telemetry data is sent to dashboard.
         */
        HIGH
    }

}
