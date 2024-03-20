package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

    private TalonSRX m_motor;
    private TalonSRX m_motorFollower;
    private CANCoder m_CANCoder = new CANCoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);
    private double m_Setpoint;
    private double m_output;
    private double m_position;

    public ShooterPivotSubsystem() 
    {
        m_CANCoder.setPosition(m_CANCoder.getAbsolutePosition());
        m_motor = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR);
        m_motor.configFactoryDefault();
        // Set peak current
        m_motor.configPeakCurrentLimit(20);
        m_motor.configPeakCurrentDuration(500);
        m_motor.configContinuousCurrentLimit(20);
        m_motor.enableCurrentLimit(true);
        m_motor.setNeutralMode(NeutralMode.Coast);

        // Set peak current
        m_motor.setInverted(true);
        m_motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_motor.configRemoteFeedbackFilter(m_CANCoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_motor.setSensorPhase(true);
        //Set Motion Magic gains in slot0
        m_motor.selectProfileSlot(0, 0);
        m_motor.config_kF(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KF);
        m_motor.config_kP(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KP);
        m_motor.config_kI(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KI);
        m_motor.config_kD(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KD);
        // Set acceleration and cruise velocity
        m_motor.configMotionCruiseVelocity(ShooterPivotConstants.SHOOTER_PIVOT_CRUISE );
        m_motor.configMotionAcceleration(ShooterPivotConstants.SHOOTER_PIVOT_ACCELERATION );
        m_motor.configPeakOutputForward(0.9);
        m_motor.configPeakOutputReverse(-0.9);
        // Set extend motion limits
        m_motor.configForwardSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MAX*(4096/360));
        m_motor.configForwardSoftLimitEnable(true);
        m_motor.configReverseSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MIN*(4096/360));
        m_motor.configReverseSoftLimitEnable(true);


        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("ShooterPivot/Setpoint", 0.0 );
        Logger.recordOutput("ShooterPivot/Position", 0.0);
        Logger.recordOutput("ShooterPivot/Output", 0.0);
        Logger.recordOutput("ShooterPivot/AtSetpoint", false );
    }

    public void setShooterPivot( double angle )
    {
        double sensorSetpoint;
        m_Setpoint = angle;
        sensorSetpoint = angle * (4096/360);
        m_motor.set(ControlMode.MotionMagic, sensorSetpoint);
        Logger.recordOutput("ShooterPivot/Setpoint", m_Setpoint );

        System.out.println("setShooterPivot " + angle + ", current angle=" + getShooterPivot());
    }

    public void setShooterPivotJog( double speed )
    {
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
        
        if( Math.abs( getShooterPivot() - m_Setpoint ) < ShooterPivotConstants.SHOOTER_PIVOT_TOLERANCE )
            retval = true;
        else   
            retval = false;
        Logger.recordOutput("ShooterPivot/AtSetpoint", retval );
        return retval;
    }

    @Override
    public void periodic() 
    {
        
        double pidCalculate;

        // This method will be called once per scheduler run

        m_position = m_CANCoder.getAbsolutePosition();

        Logger.recordOutput("ShooterPivot/Position", m_position);
    }

    public double getPosition(){
        return m_position;
    }

}
