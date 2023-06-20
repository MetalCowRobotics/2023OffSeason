package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Vector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase{

    public enum SteeringSwerveStateMachine {
        RStickSteering,
        Angle0,
        Angle90,
        Angle180,
        Angle270
    }
    SteeringSwerveStateMachine steeringState = SteeringSwerveStateMachine.RStickSteering;

    public enum DrivingSwerveStateMachine {
        Velocity0,
        Velocity1,
        Velocity2
    }
    DrivingSwerveStateMachine drivingState = DrivingSwerveStateMachine.Velocity0;

    public void setStateRStickSteering() {
		steeringState = SteeringSwerveStateMachine.RStickSteering;
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

    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    double targetRPM;
    double targetAngle;
    double speed;
    double angle;

    public void setTargetAngle(double Angle) {
        targetAngle = (Angle * ((2048 * 12.8) / 360));
    }

    public void setTargetRPM(double RPM) {
        targetRPM = (((RPM * 2048) / 600));
    }

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);
        driveMotor.setNeutralMode(NeutralMode.Coast);

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
		driveMotor.config_kF(0, (1023.0/20660.0));
		driveMotor.config_kP(0, 0.17);
		driveMotor.config_kI(0, 0);
		driveMotor.config_kD(0, 0);

        steeringMotor.selectProfileSlot(0, 0);
		steeringMotor.config_kF(0, 0.5);
		steeringMotor.config_kP(0, 0.5);
		steeringMotor.config_kI(0, 0);
		steeringMotor.config_kD(0, 0);

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
        SmartDashboard.putNumber("swerve velocity (ticks): ", driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("swerve velocity (m/s): ", (driveMotor.getSelectedSensorVelocity() * 600) / 2048);
        // double percentAngleSpeed = angleSpeed * 0.20;

        //Steering
        steeringMotor.set(TalonFXControlMode.MotionMagic, targetAngle);
        // if (steeringState.equals(SteeringSwerveStateMachine.RStickSteering)) {
        //     steeringMotor.set(TalonFXControlMode.PercentOutput, 0);
        // }
        // else if (steeringState.equals(SteeringSwerveStateMachine.Angle0)) {
        //     double degreesInTicks = (0 * ((2048 * 12.8) / 360));
        //     steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        // }
        // else if (steeringState.equals(SteeringSwerveStateMachine.Angle90)) {
        //     double degreesInTicks = (90 * ((2048 * 12.8) / 360));
        //     steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        // }
        // else if (steeringState.equals(SteeringSwerveStateMachine.Angle180)) {
        //     double degreesInTicks = (180 * ((2048 * 12.8) / 360));
        //     steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        // }
        // else if (steeringState.equals(SteeringSwerveStateMachine.Angle270)) {
        //     double degreesInTicks = (270 * ((2048 * 12.8) / 360));
        //     steeringMotor.set(TalonFXControlMode.MotionMagic, degreesInTicks);
        // }

        //Driving
        driveMotor.set(TalonFXControlMode.Velocity, targetRPM);
        // if (drivingState.equals(DrivingSwerveStateMachine.Velocity0)) {
        // }
        // else if (drivingState.equals(DrivingSwerveStateMachine.Velocity1)) {
            // driveMotor.set(TalonFXControlMode.Velocity, 1000);
        // }
        // else if (drivingState.equals(DrivingSwerveStateMachine.Velocity2)) {
            // driveMotor.set(TalonFXControlMode.Velocity, 2500);
        // }
    }
}