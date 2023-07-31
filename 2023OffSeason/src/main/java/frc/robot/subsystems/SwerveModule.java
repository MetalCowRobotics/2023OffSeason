package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Vector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.math.Conversions;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();

    TalonFX driveMotor;
    TalonFX angleMotor;
    CANCoder absoluteEncoder;

    double targetRPM;
    double targetAngle;
    String moduleName;
    int moduleNumber;
    double angleOffSet;
    Rotation2d lastAngle;

    private static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward((0.32 / 12), (1.51 / 12), (0.27 / 12));

    public SwerveModule(int driveCanID, int angleCanID, int absoluteEncoderCanID, double angleOffSet, String moduleName, int moduleNumber){

        absoluteEncoder = new CANCoder(absoluteEncoderCanID);
        configAbsoluteEncoder();

        driveMotor = new TalonFX(driveCanID);
        configDriveMotor();

        angleMotor = new TalonFX(angleCanID);
        configAngleMotor();

        this.angleOffSet = angleOffSet;
        this.moduleName = moduleName;

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
            // driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, chosenModule.wheelCircumference, chosenModule.driveGearRatio); 
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            // driveMotor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), chosenModule.angleGearRatio));
        // angleMotor.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), chosenModule.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), chosenModule.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    public void resetToAbsoluteEncoders() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffSet, chosenModule.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), chosenModule.wheelCircumference, chosenModule.driveGearRatio), getAngle()); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), chosenModule.wheelCircumference, chosenModule.driveGearRatio), getAngle());
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();

        SupplyCurrentLimitConfiguration driveMotorSupplyLimit = new SupplyCurrentLimitConfiguration(true,20,30,0.1);
        driveMotor.configSupplyCurrentLimit(driveMotorSupplyLimit);

        // driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		driveMotor.config_kP(0, 0.05);
		driveMotor.config_kI(0, 0);
		driveMotor.config_kD(0, 0);
        driveMotor.config_kF(0, 0);
        // driveMotor.config_kF(0, (1023.0/20660.0));
        driveMotor.selectProfileSlot(0, 0);

        driveMotor.configOpenloopRamp(0.5);
        driveMotor.configClosedloopRamp(0.25);

        driveMotor.setInverted(chosenModule.driveMotorInvert);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();

        SupplyCurrentLimitConfiguration angleMotorSupplyLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
        angleMotor.configSupplyCurrentLimit(angleMotorSupplyLimit);

        // angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		angleMotor.config_kP(0, chosenModule.angleKP);
        // angleMotor.config_kP(0, 0.12);
		angleMotor.config_kI(0, chosenModule.angleKI);
		angleMotor.config_kD(0, chosenModule.angleKD);
        angleMotor.config_kF(0, chosenModule.angleKF);

        angleMotor.setInverted(chosenModule.angleMotorInvert);
        angleMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsoluteEncoders();
    }

    private void configAbsoluteEncoder() {
        absoluteEncoder.configFactoryDefault();

        CANCoderConfiguration absoluteEncoderConfig = new CANCoderConfiguration();
        absoluteEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        absoluteEncoderConfig.sensorDirection = chosenModule.canCoderInvert;
        absoluteEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        absoluteEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        absoluteEncoder.configAllSettings(absoluteEncoderConfig);
    }

    // public void setTargetAngle(double Angle) {
    //     targetAngle = ((Angle) * ((2048 * 12.8) / 360));
    // }

    // public void setTargetRPM(double RPM) {
    //     targetRPM = ((RPM * 2048) / 600);
    // }

    // public void periodic(){
    //     //Steering
    //     angleMotor.set(TalonFXControlMode.Position, targetAngle);

    //     //Driving
    //     driveMotor.set(TalonFXControlMode.Velocity, targetRPM);
    // }
}