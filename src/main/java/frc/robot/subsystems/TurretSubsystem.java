package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(CANBusConstants.kTurretID, CANBusConstants.kCANBus);
    private final TalonFXConfigurator m_turretConfig = m_motor.getConfigurator();

    private final CANcoder m_CANCoder = new CANcoder(CANBusConstants.CANCoder.kTurretID);
    private final CANcoderConfigurator m_CANCoderConfig = m_CANCoder.getConfigurator();

    private PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private double m_setpoint = 0.0;
    private double m_lastSetpoint = getPosition();

    private DoubleSupplier m_robotSpeed;

    public TurretSubsystem(DoubleSupplier robotSpeed) {
        CANCoderConfigs();
        motorConfigs();
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public void setPosition(double position) {
        m_setpoint = position;
    }

    public void setRadians(double radians) {
        m_setpoint = radians - Math.floor(radians);
        if (m_setpoint >= 0.5) m_setpoint -= 1.0;

        double bestSetpoint = Double.NaN;
        double bestPoseErr = Double.POSITIVE_INFINITY;

        for (int a = -1; a <= 1; a++) {
            m_setpoint += a;

            if (m_setpoint < TurretConstants.kMinAngle 
                || m_setpoint > TurretConstants.kMaxAngle)
                    continue;

            double poseErr = Math.abs(m_setpoint - m_lastSetpoint);
            
            if (poseErr < bestPoseErr) {
                bestPoseErr = poseErr;
                bestSetpoint = m_setpoint;
            }
        }

        if (Double.isNaN(m_setpoint)) 
            m_setpoint = Math.max(TurretConstants.kMinAngle, Math.min(TurretConstants.kMaxAngle, m_setpoint));
        
        m_setpoint = bestSetpoint;
    }

    public void stop() {
        m_motor.stopMotor();
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
            .withSensorDirection(TurretConstants.kSensorDirectionValue));
    }

    private void motorConfigs() {
        m_turretConfig.apply(new MotorOutputConfigs()
            .withInverted(TurretConstants.kInvertedValue));

        m_turretConfig.apply(new Slot0Configs()
            .withKP(TurretConstants.kP)
            .withKI(TurretConstants.kI)
            .withKD(TurretConstants.kD));

        m_turretConfig.apply(new FeedbackConfigs()
            .withFusedCANcoder(m_CANCoder)
            .withFeedbackRotorOffset(TurretConstants.kRotorOffset)
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
        m_motor.setControl(m_pvReq.withPosition(m_setpoint)
                                .withEnableFOC(true)
                                    .withFeedForward(Voltage.ofBaseUnits(
                                        (m_robotSpeed.getAsDouble() * 1.1 / (2.0 * Math.PI)) * TurretConstants.kV, Volts)));
        
        m_lastSetpoint = m_setpoint;

        SmartDashboard.putNumber("Turret Position", getPosition());
        SmartDashboard.putNumber("Turret Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Turret Supply Current", getSupplyCurrent());
        SmartDashboard.putNumber("Turret Setpoint", m_setpoint);
        SmartDashboard.putNumber("Turret Last Setpoint", m_lastSetpoint);
    }
}