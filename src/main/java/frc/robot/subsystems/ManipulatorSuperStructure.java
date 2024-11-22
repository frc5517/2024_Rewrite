// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ManipulatorSuperStructure extends SubsystemBase {
    // Initialize the manipulator subsystems.
    ArmSubsystem arm;
    IntakeSubsystem intake;

    /**
     * Super Structure that controls the manipulator for better control.
     */
    public ManipulatorSuperStructure(ArmSubsystem arm, IntakeSubsystem intake) {
        // Setups the manipulator intance for control.
        this.arm = arm;
        this.intake = intake;
    }

    public Command angleArmAndIntake() {
        return run(() -> {
            arm.runArm(2, 1).
            until(arm.armAtAngle(1.5, 2.5)).
            andThen(intake.intake().
            repeatedly().withTimeout(10));
        });
    }

    public Command angleArmAndShootHigh() {
        return run(() -> {
            intake.revShooter().alongWith(arm.runArm(6, .2)).
            until(intake.shooterReady().
            and(arm.armAtAngle(5.8, 6.2))).
            andThen(intake.intake().
            repeatedly().withTimeout(1));
        });
    }

}
