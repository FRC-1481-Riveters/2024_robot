package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeRollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private TalonSRX m_intakeAngleMotor;
    private TalonSRX m_intakeAngleMotorFollower;
    private NetworkTableEntry intakeSpeedEntry;
    private SparkRelativeEncoder m_intakeRollerEncoder = (SparkRelativeEncoder) m_intakeRollerMotor.getEncoder();
    private PIDController pidIntakeAngle = new PIDController(0.007, 0, 0);
    private CANcoder m_intakeAngleCANcoder = new CANcoder(IntakeConstants.INTAKE_ANGLE_CANCODER);
    private boolean m_intakeAnglePid;
    private double m_intakeAngleSetpoint;

    public IntakeSubsystem() 
    {
        m_intakeRollerMotor.restoreFactoryDefaults();
        m_intakeRollerMotor.setInverted(false);
        m_intakeRollerMotor.setSmartCurrentLimit(80, 30);
        m_intakeRollerMotor.setIdleMode(IdleMode.kCoast);

        m_intakeAngleMotor = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR);
        m_intakeAngleMotor.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotor.setInverted(true);
        m_intakeAngleMotor.configPeakCurrentLimit(8, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configContinuousCurrentLimit(8, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.enableCurrentLimit(true);
        m_intakeAngleMotor.setNeutralMode(NeutralMode.Brake);

        m_intakeAngleMotorFollower = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR_FOLLOWER);
        m_intakeAngleMotorFollower.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotorFollower.configPeakCurrentLimit(8, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configContinuousCurrentLimit(8, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.enableCurrentLimit(true);
        m_intakeAngleMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_intakeAngleMotorFollower.setInverted(false);
        m_intakeAngleMotorFollower.follow(m_intakeAngleMotor);

        intakeSpeedEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Intake Roller Speed");

        //PID Intake Angle Stuff
        pidIntakeAngle.setTolerance(2, 10);
    }

    public void setIntakeRoller( double minus_one_to_one )
    {
        m_intakeRollerMotor.set(minus_one_to_one);
        Logger.recordOutput("IntakeRollerSet", minus_one_to_one );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeAngle( double angle )
    {
        pidIntakeAngle.reset();
        m_intakeAngleSetpoint = angle;
        Logger.recordOutput("IntakeAngleSet", angle );
        System.out.println("setIntakeAngle " + angle + ", current angle=" + getIntakeAngle());
        m_intakeAnglePid = true;

    }

    public double getIntakeAngle ()
    {
        return (m_intakeAngleCANcoder.getAbsolutePosition().getValue())*360;
    }

    public boolean atSetpoint(){
        return pidIntakeAngle.atSetpoint();
    }

    public void intakeAngleDisable()
    {
        m_intakeAngleMotor.set(ControlMode.PercentOutput, 0);
        m_intakeAnglePid = false;
        System.out.println("intakeAngleDisable current angle=" + getIntakeAngle());
    }

    @Override
    public void periodic() 
    {
        double rpm;
        double angle;
        double pidCalculate;
        rpm = m_intakeRollerEncoder.getVelocity();

        // This method will be called once per scheduler run
        intakeSpeedEntry.setDouble( rpm );
        Logger.recordOutput("IntakeRollerRPM", rpm );

        if( m_intakeAnglePid == true )
        {
            angle = getIntakeAngle();
            pidCalculate = pidIntakeAngle.calculate( angle, m_intakeAngleSetpoint);
            m_intakeAngleMotor.set( 
                ControlMode.PercentOutput, 
                MathUtil.clamp( pidCalculate, -0.4, 0.4)
            );
        }
    }

}

