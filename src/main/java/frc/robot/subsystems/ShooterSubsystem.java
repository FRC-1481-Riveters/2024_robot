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
    private CANSparkMax m_motor = new CANSparkMax(ShooterConstants.SHOOTER_OUTER_MOTOR_LEFT, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_motorFollower = new CANSparkMax(ShooterConstants.SHOOTER_OUTER_MOTOR_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_outerMotor = new CANSparkMax(ShooterConstants.SHOOTER_OUTER_MOTOR_LEFT, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_outerMotorFollower = new CANSparkMax(ShooterConstants.SHOOTER_OUTER_MOTOR_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private SparkPIDController m_pid = m_motor.getPIDController();
    private SparkPIDController m_outerPid = m_outerMotor.getPIDController();

    DigitalInput m_beamBreak = new DigitalInput(2);
    
    public ShooterSubsystem()
    {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(50, 50);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_outerMotor.restoreFactoryDefaults();
        m_outerMotor.setInverted(false);
        m_outerMotor.setSmartCurrentLimit(50, 50);
        m_outerMotor.setIdleMode(IdleMode.kBrake);



        m_motorFollower.restoreFactoryDefaults();
        m_motorFollower.setInverted(false);
        m_motorFollower.setSmartCurrentLimit(50, 50);
        m_motorFollower.setIdleMode(IdleMode.kBrake);
        m_motorFollower.follow(m_motor,true);

        m_outerMotorFollower.restoreFactoryDefaults();
        m_outerMotorFollower.setInverted(false);
        m_outerMotorFollower.setSmartCurrentLimit(50, 50);
        m_outerMotorFollower.setIdleMode(IdleMode.kBrake);
        m_outerMotorFollower.follow(m_outerMotor,true);

        m_pid.setP(0.00005);
        m_pid.setI(0.000000060);
        m_pid.setD(0.0001);
        m_pid.setFF(0.000145); //0.00018

        m_outerPid.setP(0.00005);
        m_outerPid.setI(0.000000060);
        m_outerPid.setD(0.0001);
        m_outerPid.setFF(0.000145); //0.00018


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
            m_outerPid.setReference(rpm, ControlType.kVelocity);
        }
        else 
        {
            // Turn shooter off
            m_motor.set(0.0);
            m_outerMotor.set(0.0);
        }
        
    }

    public void setShooterJog (double output){
        System.out.println("setShooterJog " + output);

        m_intendedSpeed = 0;
        m_motor.set(output);
        m_outerMotor.set(output);
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
