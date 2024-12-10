package frc.robot.subsystems.Swerve;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;
import frc.robot.Constants;
import frc.robot.Robot;

//TalonFX Swerve module IO implementation for 2x Talon motors and 1 Cancoder on Phoenix 6

public class ModIOTalonFX implements ModIO {
    private final SwerveModuleConstants constants;

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder cancoder;

    /* drive motor control requests */
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);

    //Timestamp from phoenix odometry thread
    private final Queue<Double> timestampQueue;

    //Drive motor inputs
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    //Angle motor inputs
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    //Debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModIOTalonFX(SwerveModuleConstants constants) {
        this.constants = constants;

        driveMotor =  new TalonFX(constants.driveMotorID, Constants.Swerve.canbus);
        driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setPosition(0.0);
    
        angleMotor =  new TalonFX(constants.angleMotorID, Constants.Swerve.canbus);
        angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);

        cancoder =  new CANcoder(constants.cancoderID, Constants.Swerve.canbus);
        cancoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveMotor.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        // Create turn status signals
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = angleMotor.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(angleMotor.getPosition());
        turnVelocity = angleMotor.getVelocity();
        turnAppliedVolts = angleMotor.getMotorVoltage();
        turnCurrent = angleMotor.getStatorCurrent();

         // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.Swerve.odo_hz, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
          50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, angleMotor);
    }

    @Override
    public void updateInputs(ModIOInputs inputs) {
        //Refresh signals
        var driveStatus = 
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
        var turnStatus = 
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        //Drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
        inputs.drivePositionRotations = drivePosition.getValueAsDouble();

        //turn Inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

        // Update odometry inputs
        inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
        inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
      driveMotor.setControl(dutyCycleRequest.withOutput(output));
    }

    @Override
    public void setDriveVoltage(double output) {
      driveMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
    }

    @Override
    public void setTurnOpenLoop(double output) {
        angleMotor.setControl(voltageRequest.withOutput(output));
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        angleMotor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }


}

