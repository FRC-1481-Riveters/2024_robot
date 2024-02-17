package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

    private TalonSRX m_shooterPivotMotor;
    private TalonSRX m_shooterPivotMotorFollower;
    private PIDController pidShooterPivot = new PIDController(0.007, 0, 0);
    private CANcoder m_shooterPivotCANcoder = new CANcoder(ShooterPivotConstants.SHOOTER_PIVOT_CANCODER);
    private boolean m_shooterPivotPid;
    private double m_shooterPivotSetpoint;

    public ShooterPivotSubsystem() 
    {
        m_shooterPivotMotor = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR);
        m_shooterPivotMotor.configFactoryDefault();
        // Set peak current
        m_shooterPivotMotor.setInverted(true);
        m_shooterPivotMotor.configPeakCurrentLimit(8);
        m_shooterPivotMotor.configPeakCurrentDuration(500);
        m_shooterPivotMotor.configContinuousCurrentLimit(8);
        m_shooterPivotMotor.enableCurrentLimit(true);
        m_shooterPivotMotor.setNeutralMode(NeutralMode.Brake);

        m_shooterPivotMotorFollower = new TalonSRX(ShooterPivotConstants.SHOOTER_PIVOT_MOTOR_FOLLOWER);
        m_shooterPivotMotorFollower.configFactoryDefault();
        // Set peak current
        m_shooterPivotMotorFollower.configPeakCurrentLimit(8);
        m_shooterPivotMotorFollower.configPeakCurrentDuration(500);
        m_shooterPivotMotorFollower.configContinuousCurrentLimit(8);
        m_shooterPivotMotorFollower.enableCurrentLimit(true);
        m_shooterPivotMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_shooterPivotMotorFollower.setInverted(false);
        m_shooterPivotMotorFollower.follow(m_shooterPivotMotor);

        //PID Shooter Angle Stuff
        pidShooterPivot.setTolerance(2, 10);
    }

    public void setShooterPivot( double angle )
    {
        pidShooterPivot.reset();
        m_shooterPivotSetpoint = angle;
        Logger.recordOutput("ShooterPivotSet", angle );
        System.out.println("setShooterPivot " + angle + ", current angle=" + getShooterPivot());
        m_shooterPivotPid = true;

    }

    public void setShooterPivotJog( double speed )
    {
        m_shooterPivotMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getShooterPivot ()
    {
        return (m_shooterPivotCANcoder.getAbsolutePosition().getValue())*360;
    }

    public boolean atSetpoint(){
        return pidShooterPivot.atSetpoint();
    }

    public void shooterPivotDisable()
    {
        m_shooterPivotMotor.set(ControlMode.PercentOutput, 0);
        m_shooterPivotPid = false;
        System.out.println("shooterPivotDisable current angle=" + getShooterPivot());
    }

    @Override
    public void periodic() 
    {
        double angle;
        double pidCalculate;

        if( m_shooterPivotPid == true )
        {
            angle = getShooterPivot();
            pidCalculate = pidShooterPivot.calculate( angle, m_shooterPivotSetpoint);
            m_shooterPivotMotor.set( 
                ControlMode.PercentOutput, 
                MathUtil.clamp( pidCalculate, -0.4, 0.4)
            );
            Logger.recordOutput("ShooterPivotAngle", angle );
        }
    }

}

