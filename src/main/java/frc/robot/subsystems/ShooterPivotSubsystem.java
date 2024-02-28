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

    private TalonSRX m_shooterPivotMotor;
    private TalonSRX m_shooterPivotMotorFollower;
    private PIDController pid = new PIDController(0.030, 0.012, 0.001);
    private CANCoder m_shooterPivotCANCoder = new CANCoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);
    private double m_shooterPivotSetpoint;
    private double m_output;
    private boolean m_pidEnable;
    private boolean m_initialBangBang;

    public ShooterPivotSubsystem() 
    {
        m_shooterPivotMotor = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR);
        m_shooterPivotMotor.configFactoryDefault();
        // Set peak current
        m_shooterPivotMotor.setInverted(false);
        m_shooterPivotMotor.configPeakCurrentLimit(20);
        m_shooterPivotMotor.configPeakCurrentDuration(500);
        m_shooterPivotMotor.configContinuousCurrentLimit(20);
        m_shooterPivotMotor.enableCurrentLimit(true);
        m_shooterPivotMotor.setNeutralMode(NeutralMode.Brake);

        m_shooterPivotMotorFollower = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR_FOLLOWER);
        m_shooterPivotMotorFollower.configFactoryDefault();
        // Set peak current
        m_shooterPivotMotor.setInverted(false);
        m_shooterPivotMotorFollower.configPeakCurrentLimit(20);
        m_shooterPivotMotorFollower.configPeakCurrentDuration(500);
        m_shooterPivotMotorFollower.configContinuousCurrentLimit(20);
        m_shooterPivotMotorFollower.enableCurrentLimit(true);
        m_shooterPivotMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_shooterPivotMotorFollower.setInverted(InvertType.FollowMaster);
        m_shooterPivotMotorFollower.follow(m_shooterPivotMotor);
        m_shooterPivotMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_shooterPivotMotor.configRemoteFeedbackFilter(m_shooterPivotCANCoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_shooterPivotMotor.setSensorPhase(false);
        // Set Motion Magic gains in slot0
        m_shooterPivotMotor.selectProfileSlot(0, 0);
        m_shooterPivotMotor.config_kF(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KF);
        m_shooterPivotMotor.config_kP(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KP);
        m_shooterPivotMotor.config_kI(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KI);
        m_shooterPivotMotor.config_kD(0, ShooterPivotConstants.SHOOTER_PIVOT_0_KD);
        // Set Motion Magic gains in slot0
        m_shooterPivotMotor.selectProfileSlot(1, 0);
        m_shooterPivotMotor.config_kF(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KF);
        m_shooterPivotMotor.config_kP(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KP);
        m_shooterPivotMotor.config_kI(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KI);
        m_shooterPivotMotor.config_kD(1, ShooterPivotConstants.SHOOTER_PIVOT_1_KD);
        // Set acceleration and cruise velocity
        m_shooterPivotMotor.configMotionCruiseVelocity(ShooterPivotConstants.SHOOTER_PIVOT_CRUISE );
        m_shooterPivotMotor.configMotionAcceleration(ShooterPivotConstants.SHOOTER_PIVOT_ACCELERATION );
        m_shooterPivotMotor.configPeakOutputForward(0.6);
        m_shooterPivotMotor.configPeakOutputReverse(-0.6);
        // Set extend motion limits
        m_shooterPivotMotor.configForwardSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MAX*(4096/360));
        m_shooterPivotMotor.configForwardSoftLimitEnable(true);
        m_shooterPivotMotor.configReverseSoftLimitThreshold(ShooterPivotConstants.SHOOTER_PIVOT_MIN*(4096/360));
        m_shooterPivotMotor.configReverseSoftLimitEnable(true);

        m_shooterPivotCANCoder.setPosition(m_shooterPivotCANCoder.getAbsolutePosition());

    }

    public void setShooterPivot( double angle )
    {
        m_pidEnable = true;
        if( angle > 40 )
        {
            pid.setPID( 0.016, 0, 0);
        }
        else
        {
            pid.setPID( 0.05, 0.08, 0.001 );
        }
        pid.reset();
        m_initialBangBang = true;
        m_shooterPivotSetpoint = angle;
        Logger.recordOutput("ShooterPivot/Setpoint", m_shooterPivotSetpoint );

        System.out.println("setShooterPivot " + angle + ", current angle=" + getShooterPivot());
    }

    public void setShooterPivotJog( double speed )
    {
        m_pidEnable = false;
        m_output = speed;
        m_shooterPivotMotor.set(ControlMode.PercentOutput, m_output);
        System.out.println("setShooterPivotJog " + m_output );
    }

    public double getShooterPivot ()
    {
        return (m_shooterPivotCANCoder.getAbsolutePosition());
    }

    public boolean atSetpoint(){
        boolean retval;

        if( Math.abs( getShooterPivot() - m_shooterPivotSetpoint ) < 0.5 )
            retval = true;
        else   
            retval = false;
        Logger.recordOutput("ShooterPivot/AtSetpoint", retval );
        return retval;
    }

    public void shooterPivotDisable()
    {
        m_shooterPivotMotor.set(ControlMode.PercentOutput, 0);
        System.out.println("shooterPivotDisable current angle=" + getShooterPivot());
    }

    @Override
    public void periodic() 
    {
        double position;
        double pidCalculate;

        // This method will be called once per scheduler run

        position = m_shooterPivotCANCoder.getAbsolutePosition();

        Logger.recordOutput("ShooterPivot/Position", position);
        m_shooterPivotMotorFollower.follow(m_shooterPivotMotor);

        if( m_pidEnable == true )
        {
            if( m_initialBangBang == true )
            {
                m_output = 0.23;
                if( (position + 0.5) > m_shooterPivotSetpoint )
                {
                    m_initialBangBang = false;
                }
            }
            else
            {
                pidCalculate = pid.calculate( position, m_shooterPivotSetpoint);
                m_output = MathUtil.clamp( pidCalculate, -0.6, 0.6);
                if( position < 40 )
                    m_output += 0.15;
            }
            m_shooterPivotMotor.set(ControlMode.PercentOutput, m_output);
        }
        Logger.recordOutput("ShooterPivot/Output", m_output);
    }

}

