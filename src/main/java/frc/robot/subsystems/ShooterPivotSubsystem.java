package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.sensors.CANCoder;

import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase 
{
    private CANSparkMax m_motor = new CANSparkMax(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private CANCoder m_CANCoder = new CANCoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);
    private PIDController pid = new PIDController(
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KP,
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KI,
                                        ShooterPivotConstants.SHOOTER_PIVOT_0_KD
                                    );
    private double m_Setpoint;
    private double m_output;
    private double m_position;
    private double m_tolerance;
    private double m_expectedAngle;
    private boolean m_atSetpoint;
    private int m_atSetpointDebounceCounter;
    private boolean m_pid;


    public ShooterPivotSubsystem() 
    {
        m_CANCoder.setPosition(m_CANCoder.getAbsolutePosition());

        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(30, 20);
        m_motor.setIdleMode(IdleMode.kBrake);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("ShooterPivot/Setpoint", 0.0 );
        Logger.recordOutput("ShooterPivot/Position", 0.0);
        Logger.recordOutput("ShooterPivot/Output", 0.0);
        Logger.recordOutput("ShooterPivot/AtSetpoint", false );
    }

    public void setShooterPivot( double angle )
    {
        double sensorSetpoint;

        m_tolerance = 2.5;
        m_Setpoint = angle;
        m_pid = true;
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );

        System.out.println("setShooterPivot " + angle + ", current angle=" + m_position);
    }

    public void setShooterPivotJog( double speed )
    {
        m_pid = false;
        m_output = speed;
        m_Setpoint = 0;
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );
        System.out.println("setShooterPivotJog " + m_output );
    }

    public boolean atSetpoint()
    {        
        return m_atSetpoint;
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        double pidCalculate;
  
        m_position = m_CANCoder.getAbsolutePosition();
        if( m_position > 300 )
        {
            m_position = m_position - 360;
        }

        if( m_pid == true )
        {
          pidCalculate = pid.calculate( m_position, m_Setpoint);
          m_output = MathUtil.clamp( pidCalculate, -0.3, 0.3);
        }
        if( (m_position > ShooterPivotConstants.SHOOTER_PIVOT_MAX) ||
            (m_position < ShooterPivotConstants.SHOOTER_PIVOT_MIN) )
        {
            m_output = 0;
        }
    
        m_motor.set( m_output );
        Logger.recordOutput("ShooterPivot/Output", m_output);

        Logger.recordOutput("ShooterPivot/Position", m_position);
        if( Math.abs( m_position - m_expectedAngle ) > m_tolerance )
        {
            m_atSetpoint = false;
            m_atSetpointDebounceCounter = 0;
            Logger.recordOutput("ShooterPivot/AtSetpoint", m_atSetpoint );
        }
        else if( m_atSetpointDebounceCounter < 7 )
        {
            m_atSetpointDebounceCounter++;
            if( m_atSetpointDebounceCounter == 7 )
            {
                m_atSetpoint = true;
                Logger.recordOutput("ShooterPivot/AtSetpoint", m_atSetpoint );
            }
        }
    }

    public double getPosition(){
        return m_position;
    }

}
