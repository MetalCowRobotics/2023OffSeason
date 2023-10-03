package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase{

    /* Motor Controllers */
    TalonSRX driveMotor1 = new TalonSRX(1);
    TalonSRX driveMotor2 = new TalonSRX(2);
    TalonSRX driveMotor3 = new TalonSRX(3);
    TalonSRX driveMotor4 = new TalonSRX(4);

    // AnalogPotentiometer ultraSonicSensor = new AnalogPotentiometer(0, 180);
    // PIDController ultraSonicPID = new PIDController(0, 0, 0);

    AnalogPotentiometer UltrasonicSensor= new AnalogPotentiometer(2,  180);
    PIDController PidFinder= new PIDController(0.03,0,0);
    /* Variables for Driving */
    double throttleValue;
    double turningValue;
    double speedMultiplier;

    public DriveTrain() {
        PidFinder.setSetpoint(15);

        /* Sets one side Inverted to Drive in same Direction */
        driveMotor1.setInverted(false);
        driveMotor2.setInverted(true);
        driveMotor3.setInverted(false);
        driveMotor4.setInverted(true);
    }

    /* Gets Joystick Axis Values for Tank Drive */
    public void getJoystickInputs(double leftY, double rightX) {
        throttleValue = leftY;
        turningValue = rightX;
    }

    public void periodic() {

        /* Tank Drive Based Driving */
        
       


    }

    public void getYButton() {
        driveMotor1.set(TalonSRXControlMode.PercentOutput, (((throttleValue + turningValue) / 2)) * speedMultiplier);
        driveMotor2.set(TalonSRXControlMode.PercentOutput, (((throttleValue - turningValue) / 2)) * speedMultiplier);
        driveMotor3.set(TalonSRXControlMode.PercentOutput, (((-throttleValue + turningValue) / 2)) * speedMultiplier);
        driveMotor4.set(TalonSRXControlMode.PercentOutput, (((-throttleValue - turningValue) / 2)) * speedMultiplier);
       }
    

    public void getXButton() {
        driveMotor1.set(TalonSRXControlMode.PercentOutput, -PidFinder.calculate(UltrasonicSensor.get()));
        driveMotor2.set(TalonSRXControlMode.PercentOutput, -PidFinder.calculate(UltrasonicSensor.get()));
        driveMotor3.set(TalonSRXControlMode.PercentOutput, PidFinder.calculate(UltrasonicSensor.get()));
        driveMotor4.set(TalonSRXControlMode.PercentOutput, PidFinder.calculate(UltrasonicSensor.get()));
        SmartDashboard.putNumber("Petentiometer Range", UltrasonicSensor.get());
        SmartDashboard.putNumber("Faster", PidFinder.calculate(UltrasonicSensor .get()));
    }
    public void getBButton(){
        
    }
}
