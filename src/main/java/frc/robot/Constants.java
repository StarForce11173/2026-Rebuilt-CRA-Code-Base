// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// The imports needed to run drive system
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


// The imports that were originally on the file, but are not needed for the drive system and can be added back in when those systems are implemented

// import java.util.HashMap;
// import java.util.Optional;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import edu.wpi.first.math.geometry.Translation3d;
// import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final Vision USING_VISION = Vision.NO_VISION;
  public static final boolean UPDATE_HEADING_FROM_VISION = true;  //if false heading is only from gyro
  public static final boolean GET_INITIAL_POSE_FROM_AUTO_ROUTINE = false;  
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);//left/right
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);//front/back
    // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),//front left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//front right
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//back left
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));//back right

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 9;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 3 ;

    public static final boolean kGyroReversed = false;
  }

  public static final class IntakeConstants
  {
    // SPARK MAX CAN ID
    public static final int kIntakeMotorCanId = 10;

    public static final double kIntakeMotorCurrentLimit = 40;

    public static final double kIntakeInSpeed = 1;
    public static final double kIntakeOutSpeed = -1;
  }

  public static final class ModuleConstants
  {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
  }

  public static final class OperatorConstants
  {
    // USB port on the Driver Station that the controllers are plugged into
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants
  {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants
  {
    public static final double kFreeSpeedRpm = 5676;
  }

  // public static class SensationConstants
  // {
  //   public static final int enter = 3;
  //   public static final int hopperBack = 1;
  //   public static final int hopperFront = 0;
  //   public static final int exit = 2;
  // }

  // public static class Conveyor
  // {
  //   public static final int MOTOR_CAN_ID = 15;
  //   public static final int CURRENT_LIMIT = 30;
  // }

  // public class Climber
  // {
  //   public static final int MOTOR = 14;

  //   public static final double RADIANS_PER_REVOLUTION = (Math.PI * 2) / 125;

  //   public static final double P = 0;
  //   public static final double I = 0;
  //   public static final double D = 0;

  //   public static final double KS = 0;
  //   public static final double KG = 3.96;
  //   public static final double KV = 2.44;
  //   public static final double KA = 0.14;

  //   public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
  //           .p(P)
  //           .i(I)
  //           .d(D);
  // }
  
  public enum Vision {
    NO_VISION,
    PHOTON_VISION,
    LIMELIGHT_VISION
  }

  // public static class Positions {
  //   private static final HashMap<String, Pose2d> startingPoseOfAuto;

  //   static {
  //     startingPoseOfAuto = new HashMap<String, Pose2d>();
  //     startingPoseOfAuto.put("StandardLeft", new Pose2d(7.2, 7.5, Rotation2d.fromDegrees(-90))); 
  //     startingPoseOfAuto.put("StandardRight", new Pose2d(7.2, 0.5, Rotation2d.fromDegrees(90)));
  //     startingPoseOfAuto.put("StandardCenter", new Pose2d(7.165, 4, Rotation2d.fromDegrees(180)));
  //   }

  //   public static Optional<Pose2d> getPositionForRobot(String autoName) {
  //     Pose2d startingPose = startingPoseOfAuto.get(autoName);
  //     if (startingPose == null) {
  //       return Optional.empty();
  //     }
  //     else {
  //       return Optional.of(startingPose);
  //     }
        
  //   }       
  // }
}