// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.subsystems.Intake;
import frc.subsystems.SwerveDrive;
import frc.util.logging.MotorLogger;
import frc.util.logging.NavXLogger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static SwerveDrive swerveDrive;
  public static Intake intake;
  public static Compressor compressor;
  public static AHRS navX;

  public static OI oi;

  private long time = 0;
  private boolean logged = false;
  private MotorLogger navXLogger;
  private double speed = 0.0;
  private boolean driving = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    swerveDrive = new SwerveDrive(Constants.SWERVE_TUNING, Constants.SWERVE_LOGGING);
    navX = swerveDrive.getNavX();
    this.navXLogger = new MotorLogger(new NavXLogger(navX));
    //swerveDrive.drive(0, 0, 0);

    oi = new OI();

    intake = new Intake();
    intake.retract();
    intake.stopIntaking();

    compressor = new Compressor(RobotMap.PCM_ID);
    compressor.setClosedLoopControl(true);
    compressor.stop();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    swerveDrive.zeroYaw();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(oi.getRightTurningToggle()) {
      swerveDrive.toogleRightTurning();
    } else if(oi.getLeftTurningToggle()) {
      swerveDrive.toogleLeftTurning();
    }
    swerveDrive.drive(oi.getX(), oi.getY(), oi.getRotation());
    if(oi.getIntakeExtensionToggle()) {
      System.out.println("extention toggle");
      intake.toggleExtension();
    }
    if(oi.getIntakeToggle()) {
      intake.toggleIntaking();
    }
    if(oi.getCompressingToggle()) {
      System.out.println("Compressor toggle");
      if(compressor.enabled()) {
        compressor.stop();
      } else {
        compressor.start();
      }
    }
    if(oi.getRawButtonPressed(14)) {
      System.out.println("zero Encoders");
      swerveDrive.zeroEncoders();
    }
    if(oi.getRawButtonPressed(1)) {
      System.out.println("logged");
      swerveDrive.saveLog();
      navXLogger.saveDataToCSV("accel.csv");
    }
    /*if(oi.getRawButtonPressed(1)) {

      swerveDrive.printModuleEncoders((short)0);
    }
    if(oi.getRawButtonPressed(2)) {
      swerveDrive.printModuleEncoders((short)1);
    }
    if(oi.getRawButtonPressed(3)) {
      swerveDrive.printModuleEncoders((short)2);
    }
    if(oi.getRawButtonPressed(4)) {
      swerveDrive.printModuleEncoders((short)3);
    }*/
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    swerveDrive.zeroYaw();
    swerveDrive.zeroDriveEncoders();
    this.time = System.currentTimeMillis();
    this.speed += 0.1;
    this.driving = true;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    this.navXLogger.run();
    long timePassed = System.currentTimeMillis() - this.time;
    if(timePassed < 500) {
      swerveDrive.drive(0, 0, 0);
    } else if(oi.getRawButtonPressed(2)) {
      this.driving = false;
    } else if(this.driving) {
      //swerveDrive.drive(0, 0.3, 0);
      swerveDrive.drive(0, this.speed, 0);
    } else {
      swerveDrive.drive(0,0,0);
    }
    //System.out.println(swerveDrive.avgEncoderValue());
    swerveDrive.log();
  }
}
