package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain {

    ADIS16470_IMU gyro = new ADIS16470_IMU();
    PIDController pid = new PIDController(.05, 0, 0);

    Timer time = new Timer();

    double leftSpeed;
    double rightSpeed;

    TalonSRX leftcontroller1 = new TalonSRX(1);
    TalonSRX leftcontroller2 = new TalonSRX(4);
    TalonSRX rightcontroller1 = new TalonSRX(2);
    TalonSRX rightcontroller2 = new TalonSRX(3);

    public void setLeftSpeed(double val){

        leftSpeed = val;
        //leftcontroller1.set(TalonSRXControlMode.PercentOutput, val);
        //leftcontroller2.set(TalonSRXControlMode.PercentOutput, val);

    }
    public void setRightSpeed(double val){

        rightSpeed = val;
        //rightcontroller1.set(TalonSRXControlMode.PercentOutput, val);
        //rightcontroller2.set(TalonSRXControlMode.PercentOutput, val);

    }


public void periodic(){

    double pidstore = -(pid.calculate(gyro.getAngle()));

    /*leftcontroller1.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    leftcontroller2.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    rightcontroller1.set(TalonSRXControlMode.PercentOutput, rightSpeed);
    rightcontroller2.set(TalonSRXControlMode.PercentOutput, rightSpeed);*/

    leftcontroller1.set(TalonSRXControlMode.PercentOutput, pidstore);
    leftcontroller2.set(TalonSRXControlMode.PercentOutput, pidstore);
    rightcontroller1.set(TalonSRXControlMode.PercentOutput, pidstore);
    rightcontroller2.set(TalonSRXControlMode.PercentOutput, pidstore);

    System.out.println(gyro.getAngle());

    System.out.println(time.getFPGATimestamp());
    
}



}

