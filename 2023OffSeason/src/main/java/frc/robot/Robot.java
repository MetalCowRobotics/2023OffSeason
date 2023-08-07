// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  DriveTrain driveTrain = new DriveTrain();
  XboxController box = new XboxController(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}



  

  @Override
  public void teleopPeriodic() {

  System.out.println(box.getLeftX());
  /*backRight.setVelocity(driveSpeed);
  backRight.setAngle(steerAngle);*/
  driveTrain.setLeftSpeed((Math.pow(box.getLeftY(),3)+Math.pow(box.getRightX(),3))/2);
  driveTrain.setRightSpeed(-(Math.pow(box.getLeftY(),3)-Math.pow(box.getRightX(),3))/2);

  driveTrain.periodic();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void applyOperatorInputs() {}
  
  }
