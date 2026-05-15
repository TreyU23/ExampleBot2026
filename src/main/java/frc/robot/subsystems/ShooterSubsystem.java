package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase { 
    private final TalonFX m_shooter = new TalonFX(CANBusConstants.kShooterID, CANBusConstants.kCANBus);
    private final TalonFX m_hood = new TalonFX(CANBusConstants.kShooterHoodID, CANBusConstants.kCANBus);

    private final TalonFXConfigurator m_shooterConfig = m_shooter.getConfigurator();
    private final TalonFXConfigurator m_hoodConfig = m_hood.getConfigurator();

    private final VelocityVoltage m_vvReq = new VelocityVoltage(0.0);
    private final PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private double m_setPosition = 0.0;
    private double m_setVelocity = 0.0;

    private final BooleanSupplier m_allowUp;

    public ShooterSubsystem(BooleanSupplier allowUp) {
        m_allowUp = allowUp;

        shooterConfigs();
        hoodConfigs();
    }

    public double getPosition() {
        return m_hood.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_shooter.getVelocity().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return m_shooter.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return m_shooter.getSupplyCurrent().getValueAsDouble();
    }

    public void setPosition(double pose) {
        if (m_allowUp.getAsBoolean()) {
            m_setPosition = pose/ShooterConstants.Hood.kGearRatio;
        } else {
            m_setPosition = 0.0;
        }

        m_hood.setControl(m_pvReq.withPosition(m_setPosition));
    }

    public void setVelocity(double velocity) {
        m_setVelocity = velocity;
        m_shooter.setControl(m_vvReq.withVelocity(m_setVelocity));
    }

    public void stopShooter() {
        m_shooter.stopMotor();
    }

    public void stopHood() {
        m_hood.stopMotor();
    }

    public Command stopCmd() {
        return runOnce(()-> {
            stopShooter();
            stopHood();
        });
    }

    public Command setShooterCmd(double pose, double velocity) {
        return runEnd(()-> {
            setPosition(pose);
            setVelocity(velocity);
        }, ()-> {
            stopShooter();
            stopHood();
        });
    }

    public boolean isAtPosition() {
        return Math.abs(getPosition() - m_setPosition) < ShooterConstants.Hood.kPostionTolerance;
    }

    private void shooterConfigs() {
        m_shooterConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kShooterStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kShooterSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
        
        m_shooterConfig.apply(new Slot0Configs()
            .withKP(ShooterConstants.Shooter.kP)
            .withKI(ShooterConstants.Shooter.kI)
            .withKD(ShooterConstants.Shooter.kD));

        m_shooterConfig.apply(new MotorOutputConfigs()
            .withInverted(ShooterConstants.Shooter.kInvertedValue));
    }

    private void hoodConfigs() {
        m_hoodConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kShooterHoodStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kShooterHoodSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
        
        m_hoodConfig.apply(new Slot0Configs()
            .withKP(ShooterConstants.Hood.kP)
            .withKI(ShooterConstants.Hood.kI)
            .withKD(ShooterConstants.Hood.kD));

        m_hoodConfig.apply(new MotorOutputConfigs()
            .withInverted(ShooterConstants.Hood.kInvertedValue));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());
        SmartDashboard.putNumber("Hood Position", getPosition());
        SmartDashboard.putBoolean("Hood At Position", isAtPosition());
        SmartDashboard.putNumber("Shooter Set Velocity", m_setVelocity);
        SmartDashboard.putNumber("Hood Set Position", m_setPosition);
        SmartDashboard.putNumber("Shooter Stator Current", getStatorCurrent());
        SmartDashboard.putNumber("Shooter Supply Current", getSupplyCurrent());
    }
}
