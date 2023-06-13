package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Vector;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveModule extends SubsystemBase{
    Vector<Double> targetState = new Vector<Double>();
    Vector<Double> currentState = new Vector<Double>();
    TalonFX driveMotor;
    TalonFX steeringMotor;
    double speed;
    double angle;

    public SwerveModule(double speed, double angle, int driveCanID, int steeringCanID){
        driveMotor = new TalonFX(driveCanID);
        steeringMotor = new TalonFX(steeringCanID);
        // steeringMotor.configIntegratedSensorAbsoluteRange(360);
        this.speed = speed;
        this.angle = angle;
    }

    public void periodic(){
        SmartDashboard.putNumber("swerve angle: ", steeringMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("swerve velocity: ", driveMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("swerve angle (deg): ", (steeringMotor.getSelectedSensorPosition() % (2048*12.8
        ))*(360/(2048*12.8)));
        SmartDashboard.putNumber("swerve velocity (m/s)", (638/60)*(2048 / 6.12));
    }

}
