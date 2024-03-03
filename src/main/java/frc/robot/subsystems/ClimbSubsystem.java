package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

    private CANSparkMax m_climbMotor = new CANSparkMax(ClimbConstants.CLIMB_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_climbMotorFollower = new CANSparkMax(ClimbConstants.CLIMB_MOTOR_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless );
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_climbMotor.getEncoder();
    private PIDController pid = new PIDController(0.13, 0, 0.005);
    private double m_setpoint;
    private boolean m_pidEnabled;

    public ClimbSubsystem() 
    {
        m_climbMotor.restoreFactoryDefaults();
        m_climbMotor.setInverted(false);
        m_climbMotor.setSmartCurrentLimit(80, 50);
        m_climbMotor.setIdleMode(IdleMode.kBrake);
        m_encoder.setPosition(0);
        m_climbMotorFollower.restoreFactoryDefaults();
        m_climbMotorFollower.setInverted(false);
        m_climbMotorFollower.setSmartCurrentLimit(80, 50);
        m_climbMotorFollower.setIdleMode(IdleMode.kBrake);
//        m_climbMotorFollower.follow(m_climbMotor,false);
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
            m_climbMotor.set(output);
            m_climbMotorFollower.set(output);
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
        m_climbMotor.set(percentOutput);
        m_climbMotorFollower.set(percentOutput);
        Logger.recordOutput("Climb/Setpoint", 0.0 );
        Logger.recordOutput("Climb/JogOutput", percentOutput );
    }
}
