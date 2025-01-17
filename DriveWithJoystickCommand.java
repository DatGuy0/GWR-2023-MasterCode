// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveWithJoystickCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem drivetrainSubsystem;

  //private JoystickButton intake1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoystickCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Starting joystick drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RobotContainer.joystick.getRawAxis(1);// * Math.abs(RobotContainer.joystick.getRawAxis(1));
    double turningSpeed = RobotContainer.joystick.getRawAxis(4);// * Math.abs(RobotContainer.joystick.getRawAxis(4));
    DrivetrainSubsystem.arcadeDrive(forwardSpeed*RobotContainer.maxSpeed.getDouble(0), turningSpeed*RobotContainer.maxSpeed.getDouble(0));

    // if(RobotContainer.joystick.getRawButtonPressed(4)){
    //   Intake.intake(-0.6);
    // }
    // if(RobotContainer.joystick.getRawButtonPressed(3)){
    //   Intake.intake(-0);
    // }


    /*else{
      Robot.intake.set(0);
    }

    if(RobotContainer.joystick.getRawButtonPressed(6)){
      Robot.intake.set(-0.5);
    }
    else{
      Robot.intake.set(0);
    }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
