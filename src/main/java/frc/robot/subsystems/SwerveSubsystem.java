package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private SwerveModule[] modules;
    private SwerveDriveOdometry odometry;
    private SwerveDriveKinematics kinematics;
    
    private final Pigeon2 gyro;

    private double yawOffset = 0;
    private double pitchOffset = 0;

    private boolean isPracticeRobot;

    private final Field2d m_field = new Field2d();

    public SwerveSubsystem() {
        gyro = new Pigeon2( Constants.DriveConstants.gyroPort, "CANivore" );
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setYaw(0, 1000);
                zeroHeading(0.0);
            } catch (Exception e) {
            }
        }).start();
        modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
        kinematics = new SwerveDriveKinematics( 
            DriveConstants.frontLeftModuleOffset,
            DriveConstants.frontRightModuleOffset,
            DriveConstants.backLeftModuleOffset,
            DriveConstants.backRightModuleOffset 
        );
        odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getPositions());
        
        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            DriveConstants.pathFollowerConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        SmartDashboard.putData("Field", m_field);
    }

    public void zeroHeading(double heading) {
        //gyro.setAccumZAngle(0); //.setFusedHeading(0);
        yawOffset = gyro.getYaw() + heading;
        System.out.println("zeroHeading: heading=" + heading + ", offset=" + yawOffset);
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        odometry.resetPosition(getRotation2d(),new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()},new Pose2d(0,0,getRotation2d()));
    }

    public double getHeading() {
        //TODO: not sure if the Pigeon2 getYaw returning -368 to 368 is OK here (should it be 0...360)?
        //return Math.IEEEremainder( gyro.getYaw() - yawOffset, 360 );
        return gyro.getYaw() - yawOffset;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }
    

  @Override
    public void periodic() {
        odometry.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()});
        m_field.setRobotPose(odometry.getPoseMeters());
      
        Logger.getInstance().recordOutput("MyPose2d", m_field.getRobotPose());

        // SwerveModuleState
        Logger.getInstance().recordOutput("MySwerveModuleStates",
             frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState() );
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
        for (int i = 0; i < modules.length; i++) {
          modules[i].setTargetState(targetStates[i]);
        }
    }
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
          states[i] = modules[i].getState();
        }
        return states;
    }

    public void setRampRate(double rampSeconds){
        frontLeft.setRampRate(rampSeconds);
        frontRight.setRampRate(rampSeconds);
        backLeft.setRampRate(rampSeconds);
        backRight.setRampRate(rampSeconds);
    }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

}