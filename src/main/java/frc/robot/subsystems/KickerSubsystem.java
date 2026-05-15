package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.KickerConstants;

public class KickerSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(CANBusConstants.kKickerID, CANBusConstants.kCANBus);
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);

    private double m_setpoint = 0.0;

    public KickerSubsystem() {
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

    public Command runKickerCmd() {
        return runEnd(()-> setVelocity(KickerConstants.kDefaultVelocity), ()-> stop());
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

        m_cfg.apply(new MotorOutputConfigs()
            .withInverted(KickerConstants.kInvertedValue));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Velocity", getVelocity());
        SmartDashboard.putNumber("Kicker Setpoint", m_setpoint);
        SmartDashboard.putNumber("Kicker Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Kicker Supply Current", getSupplyCurrent());
    }
}
