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
  SwerveModule frontLeft = new SwerveModule(1,2,10, 11);
  XboxController m_Xbox = new XboxController(0);
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
    frontLeft.periodic();
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
    if (m_Xbox.getAButton()) {
      frontLeft.setStateAngle0();
    }
    else if (m_Xbox.getBButton()) {
      frontLeft.setStateAngle90();
    }
    else if (m_Xbox.getXButton()) {
      frontLeft.setStateAngle180();
    }
    else if (m_Xbox.getYButton()) {
      frontLeft.setStateAngle270();
    }
    else {
      frontLeft.setStateStopSteering();
    }

    if (m_Xbox.getLeftY() != 0) {
      frontLeft.setStateDriving();
    }
    else {
      frontLeft.setStateStopDriving();
    }
    }
  }
