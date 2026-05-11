package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turret;
    private final TalonFXConfigurator m_turretConfig;

    private final CANcoder m_CANCoder;
    private final CANcoderConfigurator m_CANCoderConfig;

    private PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private double m_setpoint = 0.0;
    private double m_lastSetpoint = getPosition();

    private InvertedValue m_invertedValue;
    private SensorDirectionValue m_sensorDirection;
    private double m_offset;

    private String m_side;
    private boolean m_isLeft;

    private DoubleSupplier m_robotSpeed;

    public TurretSubsystem(boolean isLeft, DoubleSupplier robotSpeed) {
        m_side = isLeft ? "Left" : "Right";
        m_isLeft = isLeft;

        m_robotSpeed = robotSpeed;

        m_turret = new TalonFX(isLeft ? TurretConstants.kLeftTurretID : TurretConstants.kRightTurretID);
        m_turretConfig = m_turret.getConfigurator();

        m_CANCoder = new CANcoder(isLeft ? TurretConstants.kLeftCANCoderID : TurretConstants.kRightCANCoderID);
        m_CANCoderConfig = m_CANCoder.getConfigurator();

        m_invertedValue = m_isLeft ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_sensorDirection = m_isLeft ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        m_offset = m_isLeft ? TurretConstants.kRotorOffsetLeft : TurretConstants.kRotorOffsetRight;

        CANCoderConfigs();
        motorConfigs();
    }

    public double getPosition() {
        return m_turret.getPosition().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return m_turret.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_turret.getSupplyCurrent().getValueAsDouble();
    }

    public boolean isLeft() {
        return m_isLeft;
    }

    public void setPosition(double position) {
        m_setpoint = position;

    }

    public void stop() {
        m_turret.stopMotor();
    }

    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position));
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    private void CANCoderConfigs() {
        m_CANCoderConfig.apply(new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(TurretConstants.kDiscontinuityPoint)
            .withMagnetOffset(TurretConstants.kMagnetOffset)
            .withSensorDirection(m_sensorDirection));
    }

    private void motorConfigs() {
        m_turretConfig.apply(new MotorOutputConfigs()
            .withInverted(m_invertedValue));

        m_turretConfig.apply(new Slot0Configs()
            .withKP(TurretConstants.kP)
            .withKI(TurretConstants.kI)
            .withKD(TurretConstants.kD));

        m_turretConfig.apply(new FeedbackConfigs()
            .withFusedCANcoder(m_CANCoder)
            .withFeedbackRotorOffset(m_offset)
            .withSensorToMechanismRatio(TurretConstants.kGearRatio));

        m_turretConfig.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TurretConstants.kMaxAngle)
            .withReverseSoftLimitThreshold(TurretConstants.kMinAngle)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitEnable(true));

        m_turretConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kTurretStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kTurretSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
    }

    @Override
    public void periodic() {
        m_turret.setControl(m_pvReq.withPosition(m_setpoint)
                                .withEnableFOC(true)
                                    .withFeedForward(Voltage.ofBaseUnits(
                                        (m_robotSpeed.getAsDouble() * 1.1 / (2.0 * Math.PI)) * TurretConstants.kV, Volts)));
        
        m_lastSetpoint = m_setpoint;

        SmartDashboard.putNumber(m_side + " Turret Position", getPosition());
        SmartDashboard.putNumber(m_side + " Turret Stator Current", getStatorCurrent());
        SmartDashboard.putNumber(m_side + " Turret Supply Current", getSupplyCurrent());
        SmartDashboard.putNumber(m_side + " Turret Setpoint", m_setpoint);
        SmartDashboard.putNumber(m_side + " Turret Last Setpoint", m_lastSetpoint);
    }

    public double wrapAngle(double req) {
        double pose = req - Math.floor(req);
        if (pose >= 0.5) pose -= 1.0;
        return pose;
    }

    public double chooseSetpoint(double wrappedSetpoint, double lastSetpoint) {
        double best = Double.NaN;
        double bestAbsErr = Double.POSITIVE_INFINITY;

        for (int k = -1; k <= 1; k++) {
            double candidate = wrappedSetpoint + k;

            if (candidate < m_minAngle || candidate > m_maxAngle)
                continue;
            double err = candidate - lastSetpoint;
            double absErr = Math.abs(err);

            if (absErr < bestAbsErr) {
                bestAbsErr = absErr;
                best = candidate;
            }
        }

        if (Double.isNaN(best)) {
            best = Math.max(m_minAngle, Math.min(m_maxAngle, wrappedSetpoint));
        }

        return best;
    }
}
