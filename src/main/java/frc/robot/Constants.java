package frc.robot;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final class Swerve {

    //Canbus can be either "rio" or "canivore".
    public static String canbus = "rio";
    public static int odo_hz = 100;

    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSTalonFXSwerveConstants
        chosenModule = // TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
                COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    public static final double trackWidth =
        Units.inchesToMeters(30.0); // TODO: This must be tuned to specific robot
    public static final double wheelBase =
        Units.inchesToMeters(30.0); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 40;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.2;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Heading PID Values */
    public static final double HeadingKP = 4;
    public static final double HeadingKI = 0.0;
    public static final double HeadingKD = 0;
    public static final double HeadingTolerence = 0;

    /* Drive Motor Characterization Values from SysID */
    public static final double driveKS = (0.32); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(184.595);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(343.818);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(355.449);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(184.513);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final
  class AutoConstants { // TODO: The below constants are used for pathplanner and must be tuned per robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants translationPID = new PIDConstants(0, 0, 0);
    public static final PIDConstants rotationPID = new PIDConstants(0, 0, 0);

    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    public static final ModuleConfig moduleConfig = new ModuleConfig(
              (Constants.Swerve.chosenModule.wheelDiameter / 2),
              (Constants.Swerve.maxSpeed),
              Constants.AutoConstants.WHEEL_COF,
              DCMotor.getKrakenX60(1)
                  .withReduction(Constants.Swerve.chosenModule.driveGearRatio),
              40,
              1);

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class Vision {
    public static final String leftLLName = "left";
    public static final String rightLLName = "right";
  }

  public static final class PoseEstimator {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    // public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.3, 0.3,
    // Units.degreesToRadians(0.01));
    // public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.045, 0.045,
    // Units.degreesToRadians(0.01));
  }

}
