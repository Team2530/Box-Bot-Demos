package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants.KrakenConstants;


public class KrakenSubsystem extends SubsystemBase {
    private final TalonFX m_Kraken;

    private AngularVelocity targetVelocity;

    public KrakenSubsystem() {
        m_Kraken = new TalonFX(KrakenConstants.CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.withSlot0(
                    new Slot0Configs()
                        .withKP(KrakenConstants.kP)
                        .withKI(KrakenConstants.kI)
                        .withKD(KrakenConstants.kD)
                        .withKS(KrakenConstants.kS)
                        .withKV(KrakenConstants.kV)
                        .withKA(KrakenConstants.kA)
                ).withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withSupplyCurrentLimit(30)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimitEnable(true)
                );

        m_Kraken.getConfigurator().apply(config);

        targetVelocity = RotationsPerSecond.of(0);
    }

    @Override
    public void periodic() {
        AngularVelocity setVelocity;
        setVelocity = targetVelocity;

        m_Kraken.setControl(
            new VelocityVoltage(
                setVelocity.in(RotationsPerSecond)
            ).withUpdateFreqHz(20)
        );

        SmartDashboard.putNumber(
            "Kraken/targetVelocity", 
            targetVelocity.in(RotationsPerSecond)
        );
        SmartDashboard.putNumber(
            "Kraken/velocity",
            m_Kraken.getVelocity().getValueAsDouble()
        );
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = RotationsPerSecond.of(velocity);
    }


}
