package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final TalonFXConfigurator m_cfg;

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);

    private double m_setpoint = 0.0;

    private InvertedValue m_invertedValue;

    private boolean m_isLeft;
    private String m_side;

    public KickerSubsystem(boolean isLeft) {
        m_isLeft = isLeft;
        m_side = isLeft ? "Left" : "Right";

        m_motor = new TalonFX(isLeft ? KickerConstants.kLeftKickerID : KickerConstants.kRightKickerID);
        m_cfg = m_motor.getConfigurator();

        m_invertedValue = m_isLeft ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        motorConfigs();
    }

    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    public void setVelocity(double velocity) {
        m_setpoint = velocity;
        m_motor.setControl(m_vvReq.withVelocity(m_setpoint));
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public Command runKickerCmd(double velocity) {
        return runEnd(()-> setVelocity(velocity), ()-> stop());
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    public Command setVelocityCmd(double velocity) {
        return runOnce(()-> setVelocity(velocity));
    }

    private void motorConfigs() {
        m_cfg.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CurrentLimitConstants.kKickerSupplyLimit)
            .withStatorCurrentLimit(CurrentLimitConstants.kKickerStatorLimit)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true));

        m_cfg.apply(new Slot0Configs()
            .withKP(KickerConstants.kP)
            .withKI(KickerConstants.kI)
            .withKD(KickerConstants.kD));

        m_cfg.apply(new MotorOutputConfigs().withInverted(m_invertedValue));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(m_side + " Kicker Velocity", getVelocity());
        SmartDashboard.putNumber(m_side + " Kicker Setpoint", m_setpoint);
        SmartDashboard.putNumber(m_side + " Kicker Stator Current", getStatorCurrent());
        SmartDashboard.putNumber(m_side + " Kicker Supply Current", getSupplyCurrent());
    }
}
