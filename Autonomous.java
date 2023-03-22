package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.RobotContainer;

public class Autonomous extends SequentialCommandGroup {
    
    public Autonomous(int choice){
        switch (choice){
            case 1:
                addCommands(
                    //new InstantDown(),
                    new TimeShoot(RobotContainer.shooter, -0.525).withTimeout(1.2).asProxy().andThen(
                        new AutonKick(RobotContainer.kicker, -0.55).withTimeout(1)),
                    new PID_Drive(RobotContainer.getDriveTrainSubsystem(), 9.2));
                break;

            case 2:
                addCommands(
                    new InstantDown(),
                    new TimeShoot(RobotContainer.shooter, -0.525).withTimeout(1.2).asProxy().andThen(
                            new AutonKick(RobotContainer.kicker, -0.55).withTimeout(1)),
                    new InstantUp(),
                    new PID_DriveCurve(RobotContainer.getDriveTrainSubsystem(), 8),
                    new PID_DriveBackCurve(RobotContainer.getDriveTrainSubsystem(), 8),
                    new brakemode(RobotContainer.getDriveTrainSubsystem())
                );
                break;

            case 3:
                addCommands(
                    new InstantDown(),
                    new TimeShoot(RobotContainer.shooter, -0.55).withTimeout(1.2).asProxy().andThen(
                        new AutonKick(RobotContainer.kicker, -0.55).withTimeout(1)),
                    new PID_Drive(RobotContainer.getDriveTrainSubsystem(), 9.2),
                    new Intaker(RobotContainer.intake).withTimeout(1.25).asProxy(),
                    new PID_DriveBack(RobotContainer.getDriveTrainSubsystem(), 9),
                    new TimeShoot(RobotContainer.shooter, -0.50).withTimeout(1.2).asProxy().andThen(
                        new AutonKick(RobotContainer.kicker, -0.55).withTimeout(1)));
                    break;
    }
}
