package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

    private TalonSRX m_motor;
    private TalonSRX m_motorFollower;
    private PIDController pid = new PIDController(0.030, 0.012, 0.001);
    private CANCoder m_CANCoder = new CANCoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);
    private double m_Setpoint;
    private double m_output;
    private boolean m_pidEnable;
    private boolean m_initialBangBang;

    public ShooterPivotSubsystem() 
    {
        m_motor = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR);
        m_motor.configFactoryDefault();
        // Set peak current
        m_motor.setInverted(false);
        m_motor.configPeakCurrentLimit(20);
        m_motor.configPeakCurrentDuration(500);
        m_motor.configContinuousCurrentLimit(20);
        m_motor.enableCurrentLimit(true);
        m_motor.setNeutralMode(NeutralMode.Brake);

        m_motorFollower = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR_FOLLOWER);
        m_motorFollower.configFactoryDefault();
        // Set peak current
        m_motor.setInverted(false);
        m_motorFollower.configPeakCurrentLimit(20);
        m_motorFollower.configPeakCurrentDuration(500);
        m_motorFollower.configContinuousCurrentLimit(20);
        m_motorFollower.enableCurrentLimit(true);
        m_motorFollower.setNeutralMode(NeutralMode.Brake);
        m_motorFollower.setInverted(InvertType.FollowMaster);
        m_motorFollower.follow(m_motor);
        m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_motor.configRemoteFeedbackFilter(m_CANCoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_motor.setSensorPhase(false);
        /* 
         * TRY AND FIX THESE SOMEDAY BUT THEY ARE NOT USED FOR NOW
         * SHOOTER PIVOT USES BANG-BANG CONTROL AND THEN WPILIB PID INSTEAD
         
            // Set Motion Magic gains in slot0
            m_motor.selectProfileSlot(0, 0);
            m_motor.config_kF(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KF);
            m_motor.config_kP(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KP);
            m_motor.config_kI(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KI);
            m_motor.config_kD(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KD);
            // Set Motion Magic gains in slot0
            m_motor.selectProfileSlot(1, 0);
            m_motor.config_kF(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KF);
            m_motor.config_kP(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KP);
            m_motor.config_kI(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KI);
            m_motor.config_kD(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KD);
            // Set acceleration and cruise velocity
            m_motor.configMotionCruiseVelocity(ShooterPivotConstants.SHOOTER_PIVOT_CRUISE );
            m_motor.configMotionAcceleration(ShooterPivotConstants.SHOOTER_PIVOT_ACCELERATION );
        */
        m_motor.configPeakOutputForward(0.6);
        m_motor.configPeakOutputReverse(-0.6);
        // Set extend motion limits
        m_motor.configForwardSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MAX*(4096/360));
        m_motor.configForwardSoftLimitEnable(true);
        m_motor.configReverseSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MIN*(4096/360));
        m_motor.configReverseSoftLimitEnable(true);

        m_CANCoder.setPosition(m_CANCoder.getAbsolutePosition());

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("ShooterPivot/Setpoint", 0.0 );
        Logger.recordOutput("ShooterPivot/Position", 0.0);
        Logger.recordOutput("ShooterPivot/Output", 0.0);
        Logger.recordOutput("ShooterPivot/AtSetpoint", false );
    }

    public void setShooterPivot( double angle )
    {
        m_pidEnable = true;
        if( angle > ShooterPivotConstants.SHOOTER_PIVOT_HIGH )
        {
            // different PID settings for amp
            pid.setPID( 0.015, 0, 0);
        }
        else
        {
            // PID settings for regular shooting positions
            pid.setPID( 0.024, 0.16, 0.0013 );
            pid.setTolerance(0.5);
            pid.setIZone(1);
        }
        pid.reset();
        m_initialBangBang = true;
        m_Setpoint = angle;
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );

        System.out.println("setShooterPivot " + angle + ", current angle=" + getShooterPivot());
    }

    public void setShooterPivotJog( double speed )
    {
        m_pidEnable = false;
        m_output = speed;
        m_motor.set(ControlMode.PercentOutput, m_output);
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );
        System.out.println("setShooterPivotJog " + m_output );
    }

    public double getShooterPivot ()
    {
        return (m_CANCoder.getAbsolutePosition());
    }

    public boolean atSetpoint(){
        boolean retval;
        double tolerance;

        if( m_Setpoint > ShooterPivotConstants.SHOOTER_PIVOT_HIGH )
            tolerance = 5;
        else
            tolerance = 0.5;

        if( Math.abs( getShooterPivot() - m_Setpoint ) < tolerance )
            retval = true;
        else   
            retval = false;
        Logger.recordOutput("ShooterPivot/AtSetpoint", retval );
        return retval;
    }

    @Override
    public void periodic() 
    {
        double position;
        double pidCalculate;

        // This method will be called once per scheduler run

        position = m_CANCoder.getAbsolutePosition();

        Logger.recordOutput("ShooterPivot/Position", position);
        m_motorFollower.follow(m_motor);   // Recommended by CTRE in case follower loses power


        if( m_pidEnable == true )
        {
            // Instead of starting with PID, just enable the motor output
            // until it gets close to the final position, and then switch to PID
            if( m_initialBangBang == true )
            {
                if( m_Setpoint > ShooterPivotConstants.SHOOTER_PIVOT_HIGH )
                    m_output = 0.5; // going to amp
                else
                    m_output = 0.26;  // regular shot
                if( position > (m_Setpoint - 2.0) )
                {
                    // once the pivot is close to its position, turn off bang-bang control
                    m_initialBangBang = false;
                }
            }
            else
            {
                // after bang-bang is done, switch to PID
                pidCalculate = pid.calculate( position, m_Setpoint);
                m_output = MathUtil.clamp( pidCalculate, -0.6, 0.6);
                if( position < ShooterPivotConstants.SHOOTER_PIVOT_HIGH )
                    m_output += 0.07;  // feed forward
            }
            m_motor.set(ControlMode.PercentOutput, m_output);
        }
        Logger.recordOutput("ShooterPivot/Output", m_output);
    }

}
