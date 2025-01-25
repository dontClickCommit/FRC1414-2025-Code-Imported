package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.ModuleConstants;

public class TalonFXSwerveModule {
    private final TalonFX m_drivingMotor;
    private final TalonFX m_turningMotor;

    private final CANcoder m_turningEncoder;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public TalonFXSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double chassisAngularOffset) {
        m_drivingMotor = new TalonFX(drivingCANId);
        m_turningMotor = new TalonFX(turningCANId);
        m_turningEncoder = new CANcoder(turningEncoderCANId);

        m_chassisAngularOffset = chassisAngularOffset;

        // Configure motors
        configureMotor(m_drivingMotor, true);
        configureMotor(m_turningMotor, false);

        // Configure turning encoder

        // Set desired state to the current position
        m_desiredState = new SwerveModuleState(0.0, new Rotation2d(m_turningEncoder.getAbsolutePosition().getValue()));
    }

    private void configureMotor(TalonFX motor, boolean isDrivingMotor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.configFactoryDefault();

        if (isDrivingMotor) {
            // Configure driving motor settings using constants
            motor.config_kP(0, ModuleConstants.kDrivingP);
            motor.config_kI(0, ModuleConstants.kDrivingI);
            motor.config_kD(0, ModuleConstants.kDrivingD);
            motor.config_kF(0, ModuleConstants.kDrivingFF);
            motor.configContinuousCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            motor.setNeutralMode(ModuleConstants.kDrivingMotorIdleMode);
        } else {
            // Configure turning motor settings using constants
            motor.config_kP(0, ModuleConstants.kTurningP);
            motor.config_kI(0, ModuleConstants.kTurningI);
            motor.config_kD(0, ModuleConstants.kTurningD);
            motor.config_kF(0, ModuleConstants.kTurningFF);
            motor.configContinuousCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            motor.setNeutralMode(ModuleConstants.kTurningMotorIdleMode);
        }

        motor.configAllSettings(config);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingMotor.getSelectedSensorVelocity(), 
            new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()) - m_chassisAngularOffset)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingMotor.getSelectedSensorPosition(), 
            new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()) - m_chassisAngularOffset)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState, 
            new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()))
        );

        double driveVelocity = optimizedState.speedMetersPerSecond * ModuleConstants.kDriveEncoderVelocityConversionFactor;
        double turnPosition = optimizedState.angle.getRadians();

        m_drivingMotor.set(ControlMode.Velocity, driveVelocity);
        m_turningMotor.set(ControlMode.Position, turnPosition);

        m_desiredState = optimizedState;
    }

    public void resetEncoders() {
        m_drivingMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }
}
