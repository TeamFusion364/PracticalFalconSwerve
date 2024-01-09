package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;


public class SwerveCommand extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier dynamicHeadingSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampenSup;
    private PIDController rotationController;
    

    public SwerveCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier dynamicHeadingSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        //TODO: Tune heading PID
        rotationController = new PIDController(Constants.Swerve.HeadingKP, Constants.Swerve.HeadingKI, Constants.Swerve.HeadingKD );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Constants.Swerve.HeadingTolerence);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.dampenSup = dampen;
        this.dynamicHeadingSup = dynamicHeadingSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband, Dampen */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampenSup.getAsBoolean() ? 0.2 : 1);
        //TODO: Add code for dynamic heading- the supplier is a placeholder right now
        double dynamicHeading = dynamicHeadingSup.getAsDouble();
        
     //heading direction state
        switch(States.driveState){
            case forwardHold:

                //heading lock - forward
               rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(0));
               System.out.println("HEADING LOCK");
                break;
            case backwardHold:

                //heading lock - backward
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(180));
                System.out.println("HEADING LOCK");
                break;
            case leftHold:

                //heading lock - left
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(90));
                System.out.println("HEADING LOCK");
                break;
            case rightHold:

                //heading lock - right
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(270));
                System.out.println("HEADING LOCK");
                break;
            case DynamicLock:
        
                //heading lock - dynamic
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(dynamicHeading));
                System.out.println("HEADING LOCK");
                break;
            case standard:
            
                //normal
                rotationVal = rotationVal * Constants.Swerve.maxAngularVelocity;
                break;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}