package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class Mod {
  private final ModIO io;
  private final ModIOInputsAutoLogged inputs = new ModIOInputsAutoLogged();
  public final int moduleNumber;
  private final SwerveModuleConstants moduleConstants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private Rotation2d angleOffset;

  public Mod(int moduleNumber, SwerveModuleConstants moduleConstants, ModIO io) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    this.io = io;
    this.moduleConstants = null;

     driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(moduleNumber) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(moduleNumber) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(moduleNumber) + ".",
            AlertType.kError);
   }

   public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(moduleNumber), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * (Constants.Swerve.chosenModule.wheelDiameter / 2);
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }
  

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState.optimize(getState().angle);
    desiredState.cosineScale(getState().angle);
    //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    io.setTurnPosition(desiredState.angle);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      //mDriveMotor.setControl(driveDutyCycle);
      io.setDriveOpenLoop(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
    } else {
      io.setDriveVelocity(Conversions.MPSToRPS(
        desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference));
    }
  }

  public Rotation2d getCANcoder() {
    //return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    return inputs.turnAbsolutePosition;
  }

  public void resetToAbsolute() {
    //double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    double absolutePosition = inputs.turnAbsolutePosition.getRotations() - angleOffset.getRotations();
    //mAngleMotor.setPosition(absolutePosition);
    io.setTurnPosition(new Rotation2d(absolutePosition));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            inputs.driveVelocityRPS, Constants.Swerve.wheelCircumference),
            inputs.turnPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
          inputs.drivePositionRotations, Constants.Swerve.wheelCircumference),
          inputs.turnPosition);
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }
}
