package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(CANBusConstants.kIndexerID, CANBusConstants.kCANBus);
    private final TalonFXConfigurator m_cfg = m_motor.getConfigurator();

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);

    private double m_setpoint = 0.0;

    public IndexerSubsystem() {
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

    public Command runIndexerCmd() {
        return runEnd(()-> setVelocity(IndexerConstants.kDefaultVelocity), ()-> stop());
    }

    public Command stopCmd() {
        return runOnce(()-> stop());
    }

    public Command setVelocityCmd(double velocity) {
        return runOnce(()-> setVelocity(velocity));
    }

    private void motorConfigs() {
        m_cfg.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(CurrentLimitConstants.kIndexerSupplyLimit)
            .withStatorCurrentLimit(CurrentLimitConstants.kIndexerStatorLimit)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true));

        m_cfg.apply(new Slot0Configs()
            .withKP(IndexerConstants.kP)
            .withKI(IndexerConstants.kI)
            .withKD(IndexerConstants.kD));

        m_cfg.apply(new MotorOutputConfigs()
            .withInverted(IndexerConstants.kInvertedValue));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer Velocity", getVelocity());
        SmartDashboard.putNumber("Indexer Setpoint", m_setpoint);
        SmartDashboard.putNumber("Indexer Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Indexer Supply Current", getSupplyCurrent());
    }
}