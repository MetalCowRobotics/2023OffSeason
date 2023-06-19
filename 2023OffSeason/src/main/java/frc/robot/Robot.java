// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController m_Xbox = new XboxController(0);
  SwerveModule frontLeft = new SwerveModule(0,2, 4,5);
  SwerveModule frontRight = new SwerveModule(0,2,10, 11);
  SwerveModule backLeft = new SwerveModule(0,2,7, 8);
  SwerveModule backRight = new SwerveModule(0,2,1, 2);
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
    frontLeft.periodic(m_Xbox.getRightX());
    frontRight.periodic(m_Xbox.getRightX());
    backLeft.periodic(m_Xbox.getRightX());
    backRight.periodic(m_Xbox.getRightX());
    applyOperatorInputs();
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

  private void applyOperatorInputs() {
    //Steering Inputs
    if (m_Xbox.getAButton()) {
      frontLeft.setStateAngle0();
      frontRight.setStateAngle0();
      backLeft.setStateAngle0();
      backRight.setStateAngle0();
    }
    else if (m_Xbox.getBButton()) {
      frontLeft.setStateAngle90();
      frontRight.setStateAngle90();
      backLeft.setStateAngle90();
      backRight.setStateAngle90();
    }
    else if (m_Xbox.getXButton()) {
      frontLeft.setStateAngle180();
      frontRight.setStateAngle180();
      backLeft.setStateAngle180();
      backRight.setStateAngle180();
    }
    else if (m_Xbox.getYButton()) {
      frontLeft.setStateAngle270();
      frontRight.setStateAngle270();
      backLeft.setStateAngle270();
      backRight.setStateAngle270();
    }
    else {
      frontLeft.setStateRStickSteering();
      frontRight.setStateRStickSteering();
      backLeft.setStateRStickSteering();
      backRight.setStateRStickSteering();
    }
    //Driving Inputs
    if (m_Xbox.getLeftBumper()) {
      frontLeft.setStateVelocity1();
    }
    else if (m_Xbox.getRightBumper()) {
      frontLeft.setStateVelocity2();
    }
    else {
      frontLeft.setStateVelocity0();
    }
  }
}