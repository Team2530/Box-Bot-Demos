package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.RobotContainer;
import frc.robot.Constants.VortexConstants;;

public class VortexSubsystem extends SubsystemBase {

    private final SparkFlex m_Vortex;
    private final SparkClosedLoopController vortexController; 

    private AngularVelocity targetVelocity;

    public VortexSubsystem() {
        m_Vortex = new SparkFlex(
            VortexConstants.CAN_ID,
            MotorType.kBrushless
        );

        vortexController = m_Vortex.getClosedLoopController();

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .closedLoop
                .pid(VortexConstants.kP, VortexConstants.kI, VortexConstants.kD)
            .feedForward
                .sva(VortexConstants.kS, VortexConstants.kV, VortexConstants.kA);

        config.smartCurrentLimit(80);
            
        
        m_Vortex.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );




    }

    @Override
    public void periodic() {
        AngularVelocity setVelocity;
        setVelocity = targetVelocity;

        vortexController.setSetpoint(setVelocity.in(RadiansPerSecond), ControlType.kVelocity);

        SmartDashboard.putNumber(
            "Vortex/targetVelocity",
            setVelocity.in(RadiansPerSecond)
        );
        SmartDashboard.putNumber(
            "Vortex/velocity",
            m_Vortex.getEncoder().getVelocity()
        );

    }


    public void setTargetVelocity(double velocity) {
        targetVelocity = RotationsPerSecond.of(velocity);
    }

}