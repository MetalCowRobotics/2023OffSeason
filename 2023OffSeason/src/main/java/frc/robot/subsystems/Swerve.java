package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveMods;
    public Pigeon2 gyro;

    double Angle;
    double RPM;

    // private final double linearAcceleration = Constants.Swerve.maxSpeed * 1.39 / Constants.Swerve.accelerationTime;
    // private final double angularAcceleration = Constants.Swerve.maxAngularVelocity / Constants.Swerve.accelerationTime;
    // private double speedMultiplier = 1;
    // private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    // private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    // private SlewRateLimiter angleSlewRateLimiter = new SlewRateLimiter(angularAcceleration, -angularAcceleration, 0);

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveMods = new SwerveModule[] {
            new SwerveModule(4, 5, 6, Constants.Swerve.Mod1.angleOffSet, "Front Left", 1),
            new SwerveModule(10, 11, 12, Constants.Swerve.Mod3.angleOffSet, "Front Right", 3),
            new SwerveModule(7, 8, 9, Constants.Swerve.Mod2.angleOffSet, "Back Left", 2),
            new SwerveModule(1, 2, 3, Constants.Swerve.Mod0.angleOffSet, "Back Right", 0)
        };

        // Timer.delay(1.0);
        // resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // double xSpeed = m_xSlewRateLimiter.calculate(translation.getX() * speedMultiplier);
        // double ySpeed = m_ySlewRateLimiter.calculate(translation.getY() * speedMultiplier);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (false) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public void resetModulesToAbsolute(){
    //     for(SwerveModule mod : swerveMods){
    //         mod.resetToAbsoluteEncoders();
    //     }
    // }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        /* Get Position */        
        SmartDashboard.putNumber("tracked x", getPose().getX());
        SmartDashboard.putNumber("tracked y", getPose().getY());

        /* Get Each Module's Angle/Velocity */
        for(SwerveModule mod : swerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleName + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleName + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleName + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}