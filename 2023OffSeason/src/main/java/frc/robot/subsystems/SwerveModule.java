package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Vector;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase{

    public enum SteeringSwerveStateMachine {
        StopSteering,
        Angle0,
        Angle90,
        Angle180,
        Angle270
    }
    SteeringSwerveStateMachine steeringState = SteeringSwerveStateMachine.StopSteering;

    // public enum DriveSwerveStateMachine {
    //     StopDriving,
    //     Driving
    // }
    // DriveSwerveStateMachine drivingState = DriveSwerveStateMachine.StopDriving;

    public void setStateStopSteering() {
		steeringState = SteeringSwerveStateMachine.StopSteering;
	}

    public void setStateAngle0() {
		steeringState = SteeringSwerveStateMachine.Angle0;
	}

    public void setStateAngle90() {
		steeringState = SteeringSwerveStateMachine.Angle90;
	}

    public void setStateAngle180() {
		steeringState = SteeringSwerveStateMachine.Angle180;
	}

    public void setStateAngle270() {
		steeringState = SteeringSwerveStateMachine.Angle270;
	}

    // public void setStateStopDriving() {
    //     drivingState = DriveSwerveStateMachine.StopDriving;
    // }

    // public void setStateDriving() {
    //     drivingState = DriveSwerveStateMachine.Driving;
    // }

    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    double speed;
    double angle;

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);

        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        driveMotor.configNominalOutputForward(0);
		driveMotor.configNominalOutputReverse(0);
		driveMotor.configPeakOutputForward(1);
		driveMotor.configPeakOutputReverse(-1);
        steeringMotor.configNominalOutputForward(0);
		steeringMotor.configNominalOutputReverse(0);
		steeringMotor.configPeakOutputForward(1);
		steeringMotor.configPeakOutputReverse(-1);

        driveMotor.selectProfileSlot(0, 0);
		driveMotor.config_kF(0, 0.02);
		driveMotor.config_kP(0, 0.02);
		driveMotor.config_kI(0, 0);
		driveMotor.config_kD(0, 0);
        steeringMotor.selectProfileSlot(0, 0);
		steeringMotor.config_kF(0, 0.5);
		steeringMotor.config_kP(0, 0.5);
		steeringMotor.config_kI(0, 0);
		steeringMotor.config_kD(0, 0);

        driveMotor.configMotionCruiseVelocity(5000,100);
        driveMotor.configMotionAcceleration(2500, 100);
        driveMotor.configMotionSCurveStrength(0);
        steeringMotor.configMotionCruiseVelocity(30000,100);
        steeringMotor.configMotionAcceleration(20000, 100);
        steeringMotor.configMotionSCurveStrength(0);

        this.speed = speed;
        this.angle = angle;
    }

    public void periodic(){
        double swerveDegreeAngle = (steeringMotor.getSelectedSensorPosition() % (2048 * 12.8)) * (360 / (2048 * 12.8));
        SmartDashboard.putNumber("swerve angle (deg): ", swerveDegreeAngle);
        SmartDashboard.putNumber("swerve angle (ticks): ", steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("swerve velocity: ", driveMotor.getSelectedSensorVelocity());

        //Steering MotionMagic
        if (steeringState.equals(SteeringSwerveStateMachine.StopSteering)) {
            steeringMotor.set(TalonFXControlMode.PercentOutput, 0);
        }
        else if (steeringState.equals(SteeringSwerveStateMachine.Angle0)) {
            double degreesInTicks = (0 * ((2048 * 12.8) / 360));
            steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        }
        else if (steeringState.equals(SteeringSwerveStateMachine.Angle90)) {
            double degreesInTicks = (90 * ((2048 * 12.8) / 360));
            steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        }
        else if (steeringState.equals(SteeringSwerveStateMachine.Angle180)) {
            double degreesInTicks = (180 * ((2048 * 12.8) / 360));
            steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        }
        else if (steeringState.equals(SteeringSwerveStateMachine.Angle270)) {
            double degreesInTicks = (270 * ((2048 * 12.8) / 360));
            steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        }
        
        //Driving MotionMagic
        // if (drivingState.equals(DriveSwerveStateMachine.StopDriving)) {
        //     driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        // }
        // else if (drivingState.equals(DriveSwerveStateMachine.Driving)) {
            driveMotor.set(TalonFXControlMode.PercentOutput, speed);
        // }
    }
}