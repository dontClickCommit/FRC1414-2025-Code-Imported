package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import frc.robot.Constants.ModuleConstants;

public class MK4iSwerveModule {
    private final TalonFX m_drivingMotor;
    private final TalonFX m_turningMotor;

    private final CANCoder m_drivingEncoder;
    private final CANCoder m_turningEncoder;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MK4iSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingMotor = new TalonFX(drivingCANId);
        m_turningMotor = new TalonFX(turningCANId);

        TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
        TalonFXConfiguration turningConfig = new TalonFXConfiguration();

        // Restore factory defaults
        m_drivingMotor.configFactoryDefault();
        m_turningMotor.configFactoryDefault();

        m_drivingEncoder = new CANCoder(drivingCANId);
        m_turningEncoder = new CANCoder(turningCANId);

        // Configure TalonFX motor settings
        m_drivingMotor.configAllSettings(drivingConfig);
        m_turningMotor.configAllSettings(turningConfig);

        // Set appropriate conversion factors for MK4i modules
        m_drivingEncoder.configSensorInitializationTimeout(10);
        m_turningEncoder.configSensorInitializationTimeout(10);

        // Set PID values (tune these as needed for your Kraken motors and MK4i configuration).
        m_drivingMotor.config_kP(0, ModuleConstants.kDrivingP);
        m_drivingMotor.config_kI(0, ModuleConstants.kDrivingI);
        m_drivingMotor.config_kD(0, ModuleConstants.kDrivingD);
        m_drivingMotor.config_kF(0, ModuleConstants.kDrivingFF);

        m_turningMotor.config_kP(0, ModuleConstants.kTurningP);
        m_turningMotor.config_kI(0, ModuleConstants.kTurningI);
        m_turningMotor.config_kD(0, ModuleConstants.kTurningD);
        m_turningMotor.config_kF(0, ModuleConstants.kTurningFF);

        // Set current limits
        m_drivingMotor.configContinuousCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningMotor.configContinuousCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Set idle modes
        m_drivingMotor.setNeutralMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningMotor.setNeutralMode(ModuleConstants.kTurningMotorIdleMode);

        // Reset encoders
        m_drivingMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingMotor.getSelectedSensorPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition()));

        m_drivingMotor.set(ControlMode.Velocity, optimizedDesiredState.speedMetersPerSecond);
        m_turningMotor.set(ControlMode.Position, optimizedDesiredState.angle.getRadians());

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_drivingMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }
}
