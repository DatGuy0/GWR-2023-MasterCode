// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public static VictorSPX intake = new VictorSPX(6);

  /** Creates a new Intake. */
  public Intake() {
    intake.configFactoryDefault();
  }

  public static void intake(double speed) {
    intake.set(VictorSPXControlMode.PercentOutput, speed * RobotContainer.intakeEntry.getDouble(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
