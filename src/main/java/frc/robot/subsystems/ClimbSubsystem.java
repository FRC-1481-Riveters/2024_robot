package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax m_motor = new CANSparkMax(ClimbConstants.CLIMB_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_motorFollower = new CANSparkMax(ClimbConstants.CLIMB_MOTOR_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless );
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private PIDController pid = new PIDController(0.13, 0, 0.005);
    private double m_setpoint;
    private boolean m_pidEnabled;

    public ClimbSubsystem() 
    {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(80, 50);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_encoder.setPosition(0);
        m_motorFollower.restoreFactoryDefaults();
        m_motorFollower.setInverted(true);
        m_motorFollower.setSmartCurrentLimit(80, 50);
        m_motorFollower.setIdleMode(IdleMode.kBrake);
//        m_motorFollower.follow(m_motor,false);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Climb/Position", 0.0 );
        Logger.recordOutput("Climb/Output", 0.0 );
        Logger.recordOutput("Climb/Setpoint", 0.0 );
        Logger.recordOutput("Climb/JogOutput", 0.0 );
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        double position;
        double pidCalculate;
        double output;
        position = m_encoder.getPosition();

        // This method will be called once per scheduler run
        Logger.recordOutput("Climb/Position", position );

        if( m_pidEnabled == true )
        {
            pidCalculate = pid.calculate( position, m_setpoint);
            output = MathUtil.clamp( pidCalculate, -0.4, 0.4);
            m_motor.set(output);
            m_motorFollower.set(output);
            Logger.recordOutput("Climb/Output", position );
        }
    }

    public void setClimb( double targetPosition )
    {
        m_pidEnabled = true;
        m_setpoint = targetPosition;

        Logger.recordOutput("Climb/Setpoint", m_setpoint );
        Logger.recordOutput("Climb/JogOutput", 0.0 );
    }

    public void setClimbJog( double percentOutput )
    {
        m_pidEnabled = false;
        m_motor.set(percentOutput);
        m_motorFollower.set(percentOutput);
        Logger.recordOutput("Climb/Setpoint", 0.0 );
        Logger.recordOutput("Climb/JogOutput", percentOutput );
    }
}
