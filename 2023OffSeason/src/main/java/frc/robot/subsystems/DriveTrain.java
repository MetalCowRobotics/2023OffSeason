package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {

    TalonSRX motor = new TalonSRX(11);
    PIDController pid = new PIDController(0.001, 0, 0);
    double ultraSensor;
    double pidValue;

    public void setSetPoint(){
        pid.setSetpoint(15);
    }

    public void getUltraSonicSensor(double ultraSensor){
        this.ultraSensor = ultraSensor;
    }

    public void runPIDMotor(){
        pidValue = pid.calculate(ultraSensor);
        motor.set(TalonSRXControlMode.PercentOutput, pidValue);
        SmartDashboard.putNumber("Distance", ultraSensor);
        SmartDashboard.putNumber("Motor Percentage", pidValue);
    }

}

