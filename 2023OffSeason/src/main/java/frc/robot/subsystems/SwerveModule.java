package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Vector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase{

    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    CANCoder absoluteEncoder;
    double targetRPM;
    double targetAngle;
    double speed;
    double angle;
    String moduleName;
    double angleOffSet;

    public void setTargetAngle(double Angle) {
        targetAngle = ((Angle) * ((2048 * 12.8) / 360));
    }

    public void setTargetRPM(double RPM) {
        targetRPM = ((RPM * 2048) / 600);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        steeringMotor.setSelectedSensorPosition((absoluteEncoder.getAbsolutePosition() - angleOffSet) * ((2048 * 12.8) / 360));
    }

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID, int absoluteEncoderCanID, double offSet, String name){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);
        absoluteEncoder = new CANCoder(absoluteEncoderCanID);
        angleOffSet = offSet;
        moduleName = name;

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
		driveMotor.config_kP(0, 0);
		driveMotor.config_kI(0, 0);
		driveMotor.config_kD(0, 0);

        steeringMotor.selectProfileSlot(0, 0);
		steeringMotor.config_kF(0, 0);
		steeringMotor.config_kP(0, 0.12);
		steeringMotor.config_kI(0, 0);
		steeringMotor.config_kD(0, 0);

        this.speed = speed;
        this.angle = angle;

        resetEncoders();
    }

    public void periodic(){
        double swerveDegreeAngle = (steeringMotor.getSelectedSensorPosition() % (2048 * 12.8)) * (360 / (2048 * 12.8));

        SmartDashboard.putNumber("swerve angle (ticks): ", steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("swerve angle (deg): ", swerveDegreeAngle);

        SmartDashboard.putNumber("swerve velocity (ticks): ", driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("swerve velocity (RPM): ", (driveMotor.getSelectedSensorVelocity() * 600) / 2048);

        SmartDashboard.putNumber(moduleName + " Position", absoluteEncoder.getAbsolutePosition() - angleOffSet);

        SmartDashboard.putNumber(moduleName + " Raw Position", absoluteEncoder.getAbsolutePosition());

        //Steering
        steeringMotor.set(TalonFXControlMode.Position, targetAngle);

        //Driving
        driveMotor.set(TalonFXControlMode.Velocity, targetRPM);
    }
}