package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase{

    double Angle;
    double RPM;

    SwerveModule frontLeft = new SwerveModule(0,2,4,5, 6, 91.84, "Front Left");
    SwerveModule frontRight = new SwerveModule(0,2,10, 11,12, 168.22, "Front Right");
    SwerveModule backLeft = new SwerveModule(0,2,7, 8,9, 329.67, "Back Left");
    SwerveModule backRight = new SwerveModule(0,2,1, 2,3, 275.80, "Back Right");

    public void masterSetTargetAngle(double Angle){
        frontLeft.setTargetAngle(Angle);
        frontRight.setTargetAngle(Angle);
        backLeft.setTargetAngle(Angle);
        backRight.setTargetAngle(Angle);
    }

    public void masterSetTargetRPM(double RPM){
        frontLeft.setTargetRPM(RPM);
        frontRight.setTargetRPM(RPM);
        backLeft.setTargetRPM(RPM);
        backRight.setTargetRPM(RPM);
    }

    public void periodic(){
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
    }
}