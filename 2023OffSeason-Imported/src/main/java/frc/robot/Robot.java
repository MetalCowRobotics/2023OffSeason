// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  XboxController driverController = new XboxController(4);
  TalonSRX fRight = new TalonSRX(1);
  TalonSRX fLeft = new TalonSRX(4);
  TalonSRX bRight = new TalonSRX(2);
  TalonSRX bLeft = new TalonSRX(3);
  AnalogGyro gyro = new AnalogGyro(0);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    gyro.reset();
  }

  @Override
  public void autonomousPeriodic() {

    // fRight.setInverted(true);
    // bRight.setInverted(true);

    System.out.println(gyro.getAngle());

    while(gyro.getAngle()<1){
      fRight.set(TalonSRXControlMode.PercentOutput, 0.2);
      fLeft.set(TalonSRXControlMode.PercentOutput, 0.2);
      bRight.set(TalonSRXControlMode.PercentOutput, 0.2);
      bLeft.set(TalonSRXControlMode.PercentOutput, 0.2);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    applyDriverInputs();

    double drive = driverController.getLeftY();
    double rotate = driverController.getRightX();

    fRight.setInverted(true);
    bRight.setInverted(true);

    //Going Forward + Backward
    fRight.set(TalonSRXControlMode.PercentOutput, drive-rotate/2);
    bRight.set(TalonSRXControlMode.PercentOutput, drive-rotate/2);
    fLeft.set(TalonSRXControlMode.PercentOutput, drive+rotate/2);
    bLeft.set(TalonSRXControlMode.PercentOutput, drive+rotate/2);

    //Strafe
    // fRight.set(TalonSRXControlMode.PercentOutput, -xPos);
    // bRight.set(TalonSRXControlMode.PercentOutput, xPos);
    // fLeft.set(TalonSRXControlMode.PercentOutput, xPos);
    // bLeft.set(TalonSRXControlMode.PercentOutput, -xPos);
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

  private void applyDriverInputs() {
  }
}