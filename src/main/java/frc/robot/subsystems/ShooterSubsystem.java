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
    private double m_intendedSpeed;
    private CANSparkMax m_bottomMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM_FORWARD, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_bottomMotorFollower = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM_BACK, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_topMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_FORWARD, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_topMotorFollower = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_BACK, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_bottomMotor.getEncoder();
    private SparkPIDController m_pid = m_bottomMotor.getPIDController();
    private SparkPIDController m_topPid = m_topMotor.getPIDController();

    DigitalInput m_beamBreak = new DigitalInput(2);
    
    public ShooterSubsystem()
    {
        m_bottomMotor.restoreFactoryDefaults();
        m_bottomMotor.setInverted(true);
        m_bottomMotor.setSmartCurrentLimit(80, 50);
        m_bottomMotor.setIdleMode(IdleMode.kCoast);

        m_topMotor.restoreFactoryDefaults();
        m_topMotor.setInverted(true);
        m_topMotor.setSmartCurrentLimit(80, 50);
        m_topMotor.setIdleMode(IdleMode.kCoast);

        m_bottomMotorFollower.restoreFactoryDefaults();
        m_bottomMotorFollower.setInverted(true);
        m_bottomMotorFollower.setSmartCurrentLimit(80, 50);
        m_bottomMotorFollower.setIdleMode(IdleMode.kCoast);
        m_bottomMotorFollower.follow(m_bottomMotor,false);

        m_topMotorFollower.restoreFactoryDefaults();
        m_topMotorFollower.setInverted(true);
        m_topMotorFollower.setSmartCurrentLimit(80, 50);
        m_topMotorFollower.setIdleMode(IdleMode.kCoast);
        m_topMotorFollower.follow(m_topMotor,false);

        m_pid.setP(0.00005);
        m_pid.setI(0.000000060);
        m_pid.setD(0.0001);
        m_pid.setFF(0.000145); //0.00018

        m_topPid.setP(0.00005);
        m_topPid.setI(0.000000060);
        m_topPid.setD(0.0001);
        m_topPid.setFF(0.000145); //0.00018


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

        m_intendedSpeed = rpm;
        if (m_intendedSpeed > 10.0) 
        {
            // Spark MAX PID
            m_pid.setReference(rpm, ControlType.kVelocity);
            m_topPid.setReference(rpm, ControlType.kVelocity);
        }
        else 
        {
            // Turn shooter off
            m_bottomMotor.set(0.0);
            m_topMotor.set(0.0);
        }
        
    }

    public void setShooterJog (double output){
        System.out.println("setShooterJog " + output);

        m_intendedSpeed = 0;
        m_bottomMotor.set(output);
        m_topMotor.set(output);
        Logger.recordOutput("Shooter/Setpoint", 0.0);
        Logger.recordOutput("Shooter/Jog", output );
   }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }
    
    public boolean isAtSpeed() 
    {
        boolean retval;
        if( m_intendedSpeed == 0.0 )
            retval = true;
        else if (Math.abs(
            (getSpeed() - m_intendedSpeed) / m_intendedSpeed) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) 
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
        if ( m_beamBreak.get() == true )
        {
            return false;
        }
        else
        {
            return true;
        }
    }    
}
