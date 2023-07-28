package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase{

    /* Motor Controllers */
    TalonSRX driveMotor1 = new TalonSRX(1);
    TalonSRX driveMotor2 = new TalonSRX(2);
    TalonSRX driveMotor3 = new TalonSRX(3);
    TalonSRX driveMotor4 = new TalonSRX(4);

    /* Variables for Driving */
    double throttleValue;
    double turningValue;
    double speedMultiplier;

    public DriveTrain() {

        /* Sets one side Inverted to Drive in same Direction */
        driveMotor1.setInverted(false);
        driveMotor2.setInverted(false);
        driveMotor3.setInverted(true);
        driveMotor4.setInverted(true);

        /* PIDF Loops for Later Config */
        driveMotor1.selectProfileSlot(0, 0);
            driveMotor1.config_kF(0, 0);
            driveMotor1.config_kP(0, 0);
            driveMotor1.config_kI(0, 0);
            driveMotor1.config_kD(0, 0);

        driveMotor2.selectProfileSlot(0, 0);
            driveMotor2.config_kF(0, 0);
            driveMotor2.config_kP(0, 0);
            driveMotor2.config_kI(0, 0);
            driveMotor2.config_kD(0, 0);

        driveMotor3.selectProfileSlot(0, 0);
            driveMotor3.config_kF(0, 0);
            driveMotor3.config_kP(0, 0);
            driveMotor3.config_kI(0, 0);
            driveMotor3.config_kD(0, 0);

        driveMotor4.selectProfileSlot(0, 0);
            driveMotor4.config_kF(0, 0);
            driveMotor4.config_kP(0, 0);
            driveMotor4.config_kI(0, 0);
            driveMotor4.config_kD(0, 0);
    }

    /* Checks for Crawl or Sprint Modifier, Crawl has most priority (help me come up with a better method name) */
    public void getCrawlOrSprint(double leftTrigger, double rightTrigger) {
        if (leftTrigger > 0.8) {
            /* Crawl */
            speedMultiplier = 0.5;
        }
        else if (rightTrigger > 0.8) {
            /* Sprint */
            speedMultiplier = 1.3;
        }
        else {
            /* Normal */
            speedMultiplier = 1.0;
        }
    }

    /* Gets Joystick Axis Values for Tank Drive */
    public void getJoystickInputs(double leftY, double rightX) {
        throttleValue = leftY;
        turningValue = rightX;
    }

    public void periodic() {

        /* Tank Drive Based Driving */
        driveMotor1.set(TalonSRXControlMode.PercentOutput, (((throttleValue + turningValue) / 2) * 0.6) * speedMultiplier);
        driveMotor2.set(TalonSRXControlMode.PercentOutput, (((throttleValue + turningValue) / 2) * 0.6) * speedMultiplier);
        driveMotor3.set(TalonSRXControlMode.PercentOutput, (((throttleValue - turningValue) / 2) * 0.6) * speedMultiplier);
        driveMotor4.set(TalonSRXControlMode.PercentOutput, (((throttleValue - turningValue) / 2) * 0.6) * speedMultiplier);        driveMotor4.set(TalonSRXControlMode.PercentOutput, ((throttleValue + turningValue) / 2) * 0.6);
    }
}