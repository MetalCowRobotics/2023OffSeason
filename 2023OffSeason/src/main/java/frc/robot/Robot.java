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
    applyOperatorInputs();
    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();
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

    frontLeft.setTargetAngle((Math.toDegrees(Math.atan2(m_Xbox.getRightX(), m_Xbox.getRightY()))) + 180);
    frontRight.setTargetAngle((Math.toDegrees(Math.atan2(m_Xbox.getRightX(), m_Xbox.getRightY()))) + 180);
    backLeft.setTargetAngle((Math.toDegrees(Math.atan2(m_Xbox.getRightX(), m_Xbox.getRightY()))) + 180);
    backRight.setTargetAngle((Math.toDegrees(Math.atan2(m_Xbox.getRightX(), m_Xbox.getRightY()))) + 180);

    frontLeft.setTargetRPM(m_Xbox.getLeftY() * 638);
    frontRight.setTargetRPM(m_Xbox.getLeftY() * 638);
    backLeft.setTargetRPM(m_Xbox.getLeftY() * 638);
    backRight.setTargetRPM(m_Xbox.getLeftY() * 638);

  }
}