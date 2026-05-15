package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANBusConstants;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.DriverConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_roller = new TalonFX(CANBusConstants.kIntakeRollerID, CANBusConstants.kCANBus);
    private final TalonFX m_arm = new TalonFX(CANBusConstants.kIntakeArmID, CANBusConstants.kCANBus);
    private final CANcoder m_CANCoder = new CANcoder(CANBusConstants.CANCoder.kIntakeID);

    private final TalonFXConfigurator m_rollerConfig = m_roller.getConfigurator();
    private final TalonFXConfigurator m_armConfig = m_arm.getConfigurator();
    private final CANcoderConfigurator m_CANCoderConfig = m_CANCoder.getConfigurator();

    private final PositionVoltage m_pvReq = new PositionVoltage(0.0);

    private static double m_setVoltage = 0.0;
    private static double m_setPosition = 0.0;
    
    public IntakeSubsystem() {
        configureRoller();
        configureArm();
    }

    public double getVelocity() {
        return m_roller.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return m_arm.getPosition().getValueAsDouble();
    }

    public void setVoltage(double volts) {
        m_setVoltage = volts;
        m_roller.setVoltage(m_setVoltage);
    }

    public void setPosition(double pose) {
        m_setPosition = pose; 

        if (m_setPosition > IntakeConstants.Arm.kMaxPosition)
                m_setPosition = IntakeConstants.Arm.kMaxPosition;

        else if (m_setPosition < IntakeConstants.Arm.kMinPosition)
                m_setPosition = IntakeConstants.Arm.kMinPosition;

        m_setPosition /= IntakeConstants.Arm.kGearRatio;

        m_arm.setControl(m_pvReq.withPosition(m_setPosition));
    }

    public void dynamicVoltage(DoubleSupplier vx) {
        m_setVoltage = ((IntakeConstants.Roller.kMaxVoltage - IntakeConstants.Roller.kMinVoltage)
                            /DriverConstants.kMaxSpeed) * (vx.getAsDouble()) + IntakeConstants.Roller.kMinVoltage;

        if (m_setVoltage > IntakeConstants.Roller.kMaxVoltage) 
                m_setVoltage = IntakeConstants.Roller.kMaxVoltage;

        if (m_setVoltage < IntakeConstants.Roller.kMinVoltage) 
                m_setVoltage = IntakeConstants.Roller.kMinVoltage;

        setVoltage(m_setVoltage);
    }

    public void stopRoller() {
        m_roller.stopMotor();
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public Command setVoltageCmd(double volts, double vx) {
        return runOnce(()-> setVoltage(volts));
    }

    public Command setPositionCmd(double pose) {
        return runOnce(()-> setPosition(pose));
    }

    public Command stopCmd() {
        return runOnce(()-> {
            stopRoller();
            stopArm();
        });
    }

    public Command runIntakeCmd(DoubleSupplier vx) {
        return runEnd(()-> {
            dynamicVoltage(vx);
            setPosition(IntakeConstants.Arm.kIntakePosition);
        }, ()-> {
            stopRoller();
            stopArm();
        });
    }

    private void configureRoller() {
        m_rollerConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeRollerStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeRollerSupplyLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true));

        m_roller.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureArm() {
        m_armConfig.apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeArmStatorLimit)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeArmSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));

        m_armConfig.apply(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(IntakeConstants.Arm.kMinPosition)
            .withReverseSoftLimitThreshold(IntakeConstants.Arm.kMaxPosition)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));

        m_armConfig.apply(new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(IntakeConstants.Arm.kP)
            .withKI(IntakeConstants.Arm.kI)
            .withKD(IntakeConstants.Arm.kD)
            .withKS(IntakeConstants.Arm.kS)
            .withKG(IntakeConstants.Arm.kG));

        m_armConfig.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(IntakeConstants.Arm.kGearRatio)
            .withRemoteCANcoder(m_CANCoder));

        m_arm.setNeutralMode(NeutralModeValue.Brake);

        m_CANCoderConfig.apply(new MagnetSensorConfigs()
            .withMagnetOffset(IntakeConstants.kMagnetOffset));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Velocity", getVelocity());
        SmartDashboard.putNumber("Intake Roller Set Voltage", m_setVoltage);
        SmartDashboard.putNumber("Intake Roller Stator", m_roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller Supply", m_roller.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Intake Arm Position", Units.rotationsToDegrees(getPosition()));
        SmartDashboard.putNumber("Intake Arm Setpoint", m_setPosition);
        SmartDashboard.putNumber("Intake Arm Stator", m_arm.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Arm Supply", m_arm.getSupplyCurrent().getValueAsDouble());
    }
}
