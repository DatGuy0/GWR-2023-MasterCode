// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //Pneumatics pneumatic = new Pneumatics();

  public static int count = 0;

  //public static DigitalInput photoelectric = new DigitalInput(0);

  //public static CANSparkMax intake = new CANSparkMax(4, MotorType.kBrushless);

  // UsbCamera camera1;
  // UsbCamera camera2;
  // VideoSink server;
  // NetworkTableEntry cameraSelection;
  // AHRS gyro = new AHRS(SPI.Port.kMXP);

  int choice = 0;
  String positionSelection = "";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //PathPlannerServer.startServer(8576);
    m_robotContainer = new RobotContainer();

    // m_robotContainer.getDriveTrainSubsystem();
    // DrivetrainSubsystem.zeroHeading();
    // m_robotContainer.getDriveTrainSubsystem().resetEncoders();
    // DrivetrainSubsystem.setCoastMode();

    // camera1 = CameraServer.startAutomaticCapture(0);
    // camera2 = CameraServer.startAutomaticCapture(1);

    // camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    // cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    RobotContainer.positionChooser.addOption("Red Left", "Red_Left");
    RobotContainer.positionChooser.addOption("Red Middle", "Red_Middle");
    RobotContainer.positionChooser.addOption("Red Right", "Red_Right");
    RobotContainer.positionChooser.addOption("Blue Left", "Blue_Left");
    RobotContainer.positionChooser.addOption("Blue Middle", "Blue_Middle");
    RobotContainer.positionChooser.addOption("Blue Right", "Blue_Right");
    
    RobotContainer.autoOptions.add("Position Chooser", RobotContainer.positionChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser)
      .withSize(2, 1);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //Roll and Yaw seem to be swtiched 
    // RobotContainer.rollEntry.setDouble((double) gyro.getYaw());
    // RobotContainer.pitchEntry.setDouble((double) gyro.getPitch()-90);
    // RobotContainer.yawEntry.setDouble((double) gyro.getRoll());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    m_robotContainer.getDriveTrainSubsystem();
    DrivetrainSubsystem.zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("deploy/pathplanner/generatedJSON/Middle.wpilib.json", new PathConstraints(0.5, 0.25));

    // // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
    // // Or the path can be sampled at a given point in time for custom path following

    // // Sample the state of the path at 1.2 seconds
    // PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

    // // Print the velocity at the sampled time
    // System.out.println(exampleState.velocityMetersPerSecond);
    
    m_robotContainer.getDriveTrainSubsystem();
    DrivetrainSubsystem.zeroHeading();
    m_robotContainer.getDriveTrainSubsystem().resetEncoders();
    DrivetrainSubsystem.setCoastMode();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    switch ((String) RobotContainer.positionChooser.getSelected()){
      case "Red_Left":
        choice = 1;
        break;
      case "Red_Middle":
        choice = 2;
        break;
      case "Red_Right":
        choice = 3;
        break;
      case "Blue_Left":
        choice = 3;
        break;
      case "Blue_Middle":
        choice = 2;
        break;
      case "Blue_Right":
        choice = 1;
        break;
    }

    RobotContainer.chosen.setInteger(choice);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (RobotContainer.joystick.getRawButtonPressed(1)) {
    //   if (cameraNum == 1){
    //     cameraSelection.setString(camera2.getName());
    //     cameraNum = 2;
    //   }
    //   else if (cameraNum == 2){
    //     cameraSelection.setString(camera1.getName());
    //     cameraNum = 1;
    //   }
    // }
  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
