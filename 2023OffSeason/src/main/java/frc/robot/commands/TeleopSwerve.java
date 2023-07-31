package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    

    // public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    //     this.s_Swerve = s_Swerve;
    //     addRequirements(s_Swerve);

    //     this.translationSup = translationSup;
    //     this.strafeSup = strafeSup;
    //     this.rotationSup = rotationSup;
    //     this.robotCentricSup = robotCentricSup;
    // }

    public void periodic(Swerve m_Swerve, double translationSup, double strafeSup, double rotationSup, boolean robotCentricSup) {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup, Constants.Swerve.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.Swerve.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.Swerve.stickDeadband);

        /* Drive */
        m_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );
        m_Swerve.periodic();
    }
}