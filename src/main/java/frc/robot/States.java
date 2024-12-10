package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class States {

   //Default state setter for simulation
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static enum DriveStates {
    //Heading lock states for Drivetrain
    standard,
    leftHold,
    rightHold,
    forwardHold,
    backwardHold,
    DynamicLock
  }

  public static enum AlignedStates {
    //Alignment states for auto-alignment
    aligned,
    unAligned,
    normal
  }

  public static DriveStates driveState = DriveStates.standard;
  public static AlignedStates alignedState = AlignedStates.normal;
}
