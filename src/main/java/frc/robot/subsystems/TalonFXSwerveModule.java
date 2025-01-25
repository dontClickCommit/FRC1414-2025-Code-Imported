package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        motor.getConfigurator().apply(new TalonFXConfiguration());;

        if (isDrivingMotor) {
            var motorConfigs = new Slot0Configs();
            motorConfigs.kP = ModuleConstants.kDrivingP; 
            motorConfigs.kI = ModuleConstants.kDrivingI; 
            motorConfigs.kD = ModuleConstants.kDrivingD; 

            motor.getConfigurator().apply(motorConfigs);

            // APPLY LIMITS IN TUNER X - motor.configContinuousCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            motor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            var motorConfigs = new Slot0Configs();
            motorConfigs.kP = ModuleConstants.kTurningP; 
            motorConfigs.kI = ModuleConstants.kTurningI; 
            motorConfigs.kD = ModuleConstants.kTurningD; 

            motor.getConfigurator().apply(motorConfigs);

            // THE LIMITS NEED TO BE APPLIED IN TUNER X - motor.configContinuousCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            motor.setNeutralMode(NeutralModeValue.Brake);
        }

        // motor.configAllSettings(config); - THIS MIGHT CAUSE PROBLEMS IF SO LMK
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingMotor.getEncoder().getVelocity(), 
            new Rotation2d(Math.toRadians(m_turningEncoder.getAbsolutePosition()) - m_chassisAngularOffset)
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_drivingMotor.getEncoder().getVelocity(), 
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

        m_drivingMotor.set(driveVelocity);
        m_turningMotor.set(turnPosition);

        m_desiredState = optimizedState;
    }

    public void resetEncoders() {
        m_drivingMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }
}
