package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase
{
    private double m_shooterIntendedSpeed;
    private CANSparkMax m_shooterMotorTop = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_shooterMotorBottom = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_shooterMotorTop.getEncoder();
    private SparkPIDController m_pidController = m_shooterMotorTop.getPIDController();
    DigitalInput m_shooterBeamBreak = new DigitalInput(2);
    
    public ShooterSubsystem()
    {
        m_shooterMotorTop.restoreFactoryDefaults();
        m_shooterMotorTop.setInverted(false);
        m_shooterMotorTop.setSmartCurrentLimit(50, 50);
        m_shooterMotorTop.setIdleMode(IdleMode.kCoast);
        m_shooterMotorBottom.restoreFactoryDefaults();
        m_shooterMotorBottom.setInverted(false);
        m_shooterMotorBottom.setSmartCurrentLimit(50, 50);
        m_shooterMotorBottom.setIdleMode(IdleMode.kCoast);
        m_shooterMotorBottom.follow(m_shooterMotorTop,true);
        m_pidController.setP(0.00005);
        m_pidController.setI(0.000000060);
        m_pidController.setD(0.0001);
        m_pidController.setFF(0.000145); //0.00018

        setShooterSpeed(0.0);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Shooter/Setpoint", 0.0 );
        Logger.recordOutput("Shooter/Speed", 0.0 );
        Logger.recordOutput("Shooter/AtSpeed", false );
        Logger.recordOutput("Shooter/BeamBreak", false );
        Logger.recordOutput("Shooter/Jog", 0.0 );
    }

    @Override
    public void periodic() 
    {
        // This method will be called once per scheduler run
        Logger.recordOutput("Shooter/Speed", getSpeed() );
        Logger.recordOutput("Shooter/BeamBreak", isLightCurtainBlocked() );
    }

    public void setShooterSpeed (double rpm)
    {
        System.out.println("setShooterSpeed " + rpm);
        Logger.recordOutput("Shooter/Setpoint", rpm );
        Logger.recordOutput("Shooter/Jog", 0.0 );

        m_shooterIntendedSpeed = rpm;
        if (m_shooterIntendedSpeed > 10.0) 
        {
            // Spark MAX PID
            m_pidController.setReference(rpm, ControlType.kVelocity);
        }
        else 
        {
            // Turn shooter off
            m_shooterMotorTop.set(0.0);
        }
        
    }

    public void setShooterJog (double output){
        System.out.println("setShooterJog " + output);

        m_shooterIntendedSpeed = 0;
        m_shooterMotorTop.set(output);
        Logger.recordOutput("Shooter/Setpoint", 0.0);
        Logger.recordOutput("Shooter/Jog", output );
   }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }
    
    public boolean isAtSpeed() 
    {
        boolean retval;
        if( m_shooterIntendedSpeed == 0.0 )
            retval = true;
        else if (Math.abs(
            (getSpeed() - m_shooterIntendedSpeed) / m_shooterIntendedSpeed) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) 
        {
            // return TRUE if the shooter is at the right speed
            retval = true;
        }
        else 
        {
            retval = false;
        }
        Logger.recordOutput("Shooter/AtSpeed", retval );
        return retval;
    }
    
    public boolean isLightCurtainBlocked()
    {
        // reverse logic for blocked
        if ( m_shooterBeamBreak.get() == true )
        {
            return false;
        }
        else
        {
            return true;
        }
    }    
}
