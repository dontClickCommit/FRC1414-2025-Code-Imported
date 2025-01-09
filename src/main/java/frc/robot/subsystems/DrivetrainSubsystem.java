package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.TalonFXSwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private static DrivetrainSubsystem instance;

    /*
     * Initialize swerve modules using TalonFXSwerveModule.
     */
    private final TalonFXSwerveModule frontLeftSwerve = new TalonFXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final TalonFXSwerveModule frontRightSwerve = new TalonFXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final TalonFXSwerveModule rearLeftSwerve = new TalonFXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final TalonFXSwerveModule rearRightSwerve = new TalonFXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    /*
     * Initialize gyroscope (if using a Pigeon2 or similar).
     */
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanID);

    /*
     * Initialize odometry.
     */
    SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getHeading(), getSwerveModulePositions(), new Pose2d(),
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)),
            VecBuilder.fill(0.75, 0.75, 99999999));

    /*
     * Visualization field.
     */
    private Field2d field = new Field2d();

    public DrivetrainSubsystem() {
        SmartDashboard.putData("Field", field);
    }

    public static synchronized DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, AutoConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
                getCurrentPose().getRotation());
        driveRobotRelative(robotRelative);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftSwerve.getState(),
                frontRightSwerve.getState(),
                rearLeftSwerve.getState(),
                rearRightSwerve.getState()
        };
    }

    public void drive(Transform2d transform, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(transform.getX(), transform.getY(),
                                transform.getRotation().getRadians(),
                                getCurrentPose().getRotation())
                        : new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        setModuleStates(swerveModuleStates);
    }

    public void lock() {
        SwerveModuleState[] lockStates = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
        setModuleStates(lockStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        frontLeftSwerve.setDesiredState(desiredStates[0]);
        frontRightSwerve.setDesiredState(desiredStates[1]);
        rearLeftSwerve.setDesiredState(desiredStates[2]);
        rearRightSwerve.setDesiredState(desiredStates[3]);
    }

    public Pose2d getCurrentPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftSwerve.getPosition(),
                frontRightSwerve.getPosition(),
                rearLeftSwerve.getPosition(),
                rearRightSwerve.getPosition()
        };
    }

    public void resetEncoders() {
        frontLeftSwerve.resetEncoders();
        frontRightSwerve.resetEncoders();
        rearLeftSwerve.resetEncoders();
        rearRightSwerve.resetEncoders();
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getSwerveModulePositions());
        field.setRobotPose(odometry.getEstimatedPosition());
        SmartDashboard.putNumberArray("Pose XY",
                new Double[] { odometry.getEstimatedPosition().getX(), odometry.getEstimatedPosition().getY() });
    }
}
