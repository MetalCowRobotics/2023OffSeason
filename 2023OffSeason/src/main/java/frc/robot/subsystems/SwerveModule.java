package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Vector;

import org.opencv.calib3d.StereoBM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;



public class SwerveModule extends SubsystemBase{
    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    double speed;
    double angle;
    PIDController anglePid = new PIDController(0.007, 0, 0);
    

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);
        // steeringMotor.configIntegratedSensorAbsoluteRange(360);
        this.speed = speed;
        this.angle = angle;
        anglePid.enableContinuousInput(0, 360);
        anglePid.setSetpoint(180);
    }

    public double getAngle(){
        double motorAngle = (steeringMotor.getSelectedSensorPosition() % (2048*12.8))*(360/(2048*12.8));
        
        motorAngle %= 360;
		if (motorAngle < 0){
			motorAngle += 360;
		}
		return motorAngle;
    }

    public double getVelocity(){
        return driveMotor.getSelectedSensorVelocity();
    }

    public void periodic(){
        double angleError = anglePid.calculate(getAngle());
        SmartDashboard.putNumber("swerve angle: ", steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("swerve velocity: ", driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("swerve angle (deg): ", (steeringMotor.getSelectedSensorPosition() % (2048*12.8
        ))*(360/(2048*12.8)));
        SmartDashboard.putNumber("swerve velocity (m/s)", (638/60)*(2048 / 6.12));
        if(angleError < -1.0){
            angleError = -1.0;
        }
        else if(angleError > 1.0){
            angleError = 1.0;
        }
        steeringMotor.set(ControlMode.PercentOutput, angleError);
    }
    }


