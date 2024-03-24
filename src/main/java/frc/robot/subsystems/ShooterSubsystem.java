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
    private CANSparkMax m_bottomBackMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM_BACK, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax m_topMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_FORWARD, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_topBackMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP_BACK, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_bottomMotor.getEncoder();
    private SparkPIDController m_bottomPid = m_bottomMotor.getPIDController();
    private SparkPIDController m_topPid = m_topMotor.getPIDController();
    private SparkPIDController m_bottomBackPid = m_bottomBackMotor.getPIDController();
    private SparkPIDController m_topBackPid = m_topBackMotor.getPIDController();

    DigitalInput m_beamBreak = new DigitalInput(2);
    
    public ShooterSubsystem()
    {
        m_bottomMotor.restoreFactoryDefaults();
        m_bottomMotor.setInverted(true);
        m_bottomMotor.setSmartCurrentLimit(80, 50);
        m_bottomMotor.setIdleMode(IdleMode.kCoast);
        m_bottomPid.setP(0.00005);
        m_bottomPid.setI(0.000000060);
        m_bottomPid.setD(0.0001);
        m_bottomPid.setFF(0.000145); //0.00018
        m_bottomMotor.burnFlash();

        m_topMotor.restoreFactoryDefaults();
        m_topMotor.setInverted(true);
        m_topMotor.setSmartCurrentLimit(80, 50);
        m_topMotor.setIdleMode(IdleMode.kCoast);
        m_topPid.setP(0.00005);
        m_topPid.setI(0.000000060);
        m_topPid.setD(0.0001);
        m_topPid.setFF(0.000145); //0.00018
        m_topMotor.burnFlash();

        m_bottomBackMotor.restoreFactoryDefaults();
        m_bottomBackMotor.setInverted(true);
        m_bottomBackMotor.setSmartCurrentLimit(80, 50);
        m_bottomBackMotor.setIdleMode(IdleMode.kCoast);
        m_bottomBackPid.setP(0.00005);
        m_bottomBackPid.setI(0.000000060);
        m_bottomBackPid.setD(0.0001);
        m_bottomBackPid.setFF(0.000145); //0.00018
        m_bottomBackMotor.burnFlash();

        m_topBackMotor.restoreFactoryDefaults();
        m_topBackMotor.setInverted(true);
        m_topBackMotor.setSmartCurrentLimit(80, 50);
        m_topBackMotor.setIdleMode(IdleMode.kCoast);
        m_topBackPid.setP(0.00005);
        m_topBackPid.setI(0.000000060);
        m_topBackPid.setD(0.0001);
        m_topBackPid.setFF(0.000145); //0.00018
        m_topBackMotor.burnFlash();

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
            m_bottomPid.setReference(rpm, ControlType.kVelocity);
            m_topPid.setReference(rpm, ControlType.kVelocity);
            m_bottomBackPid.setReference(rpm, ControlType.kVelocity);
            m_topBackPid.setReference(rpm, ControlType.kVelocity);
        }
        else 
        {
            // Turn shooter off
            m_bottomMotor.set(0.0);
            m_topMotor.set(0.0);
            m_bottomBackMotor.set(0.0);
            m_topBackMotor.set(0.0);
        }
        
    }

    public void setShooterJog (double output){
        System.out.println("setShooterJog " + output);

        m_intendedSpeed = 0;
        m_bottomMotor.set(output);
        m_topMotor.set(output);
        m_bottomBackMotor.set(output);
        m_topBackMotor.set(output);
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
