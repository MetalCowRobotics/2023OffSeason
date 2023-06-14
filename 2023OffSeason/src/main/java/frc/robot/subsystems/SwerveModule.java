package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Vector;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase{

    public enum SwerveStateMachine {
        Stop,
        Angle0,
        Angle90,
        Angle180,
        Angle270,
        MotionMagic
    }
    SwerveStateMachine state = SwerveStateMachine.Stop;

    public void setStateStop() {
		state = SwerveStateMachine.Stop;
	}

    public void setStateAngle0() {
		state = SwerveStateMachine.Angle0;
	}

    public void setStateAngle90() {
		state = SwerveStateMachine.Angle90;
	}

    public void setStateAngle180() {
		state = SwerveStateMachine.Angle180;
	}

    public void setStateAngle270() {
		state = SwerveStateMachine.Angle270;
	}

    public void setStateMotionMagic() {
        state = SwerveStateMachine.MotionMagic;
    }

    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    PIDController m_SteeringPID = new PIDController(0.02, 0, 0);
    double speed;
    double angle;

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);

        steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        steeringMotor.configNominalOutputForward(0);
		steeringMotor.configNominalOutputReverse(0);
		steeringMotor.configPeakOutputForward(1);
		steeringMotor.configPeakOutputReverse(-1);

        steeringMotor.selectProfileSlot(0, 0);
		steeringMotor.config_kF(0, 0.2);
		steeringMotor.config_kP(0, 0.2);
		steeringMotor.config_kI(0, 0);
		steeringMotor.config_kD(0, 0);

        steeringMotor.configMotionCruiseVelocity(75, 0);
        steeringMotor.configMotionAcceleration(30, 0);
        steeringMotor.configMotionSCurveStrength(0);

        this.speed = speed;
        this.angle = angle;
    }

    public void periodic(){
        double swerveDegreeAngle = (steeringMotor.getSelectedSensorPosition() % (2048 * 12.8)) * (360 / (2048 * 12.8));
        double error = m_SteeringPID.calculate(swerveDegreeAngle);
        SmartDashboard.putNumber("swerve angle (deg): ", swerveDegreeAngle);
        SmartDashboard.putNumber("swerve angle (ticks): ", steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("swerve velocity: ", driveMotor.getSelectedSensorVelocity());

        //Steering PID
        if (state.equals(SwerveStateMachine.Stop)) {
            steeringMotor.set(TalonFXControlMode.PercentOutput, 0);
        }
        else if (state.equals(SwerveStateMachine.Angle0)) {
            m_SteeringPID.setSetpoint(0);
            steeringMotor.set(TalonFXControlMode.PercentOutput, error);
        }
        else if (state.equals(SwerveStateMachine.Angle90)) {
            m_SteeringPID.setSetpoint(90);
            steeringMotor.set(TalonFXControlMode.PercentOutput, error);
        }
        else if (state.equals(SwerveStateMachine.Angle180)) {
            m_SteeringPID.setSetpoint(180);
            steeringMotor.set(TalonFXControlMode.PercentOutput, error);
        }
        else if (state.equals(SwerveStateMachine.Angle270)) {
            m_SteeringPID.setSetpoint(270);
            steeringMotor.set(TalonFXControlMode.PercentOutput, error);
        }
        //Steering Motion Magic
        else if (state.equals(SwerveStateMachine.MotionMagic)) {
            double nintyDegreesInTicks = (90 * ((2048 * 12.8) / 360));
            steeringMotor.set(TalonFXControlMode.MotionMagic, nintyDegreesInTicks);
        }        
    }
}