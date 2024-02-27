package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DigitalInput;


import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeRollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private TalonSRX m_intakeAngleMotor;
    private TalonSRX m_intakeAngleMotorFollower;
    private NetworkTableEntry intakeSpeedEntry;
    private SparkRelativeEncoder m_intakeRollerEncoder = (SparkRelativeEncoder) m_intakeRollerMotor.getEncoder();
    private CANCoder m_intakeAngleCANcoder = new CANCoder(IntakeConstants.INTAKE_ANGLE_CANCODER);
    private boolean m_intakeAnglePid;
    private double m_intakeAngleSetpoint;
    private DigitalInput m_intakeBeamBreakShooter = new DigitalInput(0);
    private DigitalInput m_intakeBeamBreakLoaded = new DigitalInput(1);

    public IntakeSubsystem() 
    {
        m_intakeRollerMotor.restoreFactoryDefaults();
        m_intakeRollerMotor.setInverted(false);
        m_intakeRollerMotor.setSmartCurrentLimit(80, 30);
        m_intakeRollerMotor.setIdleMode(IdleMode.kCoast);
        m_intakeAngleCANcoder.setPosition(m_intakeAngleCANcoder.getAbsolutePosition());

        m_intakeAngleMotor = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR);
        m_intakeAngleMotor.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotor.setInverted(true);
        m_intakeAngleMotor.configPeakCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configContinuousCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.enableCurrentLimit(true);
        m_intakeAngleMotor.setNeutralMode(NeutralMode.Coast);

        m_intakeAngleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_intakeAngleMotor.configRemoteFeedbackFilter(m_intakeAngleCANcoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_intakeAngleMotor.setSensorPhase(true);
        // Set Motion Magic gains in slot0
        m_intakeAngleMotor.selectProfileSlot(0, 0);
        m_intakeAngleMotor.config_kF(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KF);
        m_intakeAngleMotor.config_kP(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KP);
        m_intakeAngleMotor.config_kI(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KI);
        m_intakeAngleMotor.config_kD(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KD);
        // Set acceleration and cruise velocity
        m_intakeAngleMotor.configMotionCruiseVelocity(IntakeConstants.INTAKE_ANGLE_MOTOR_CRUISE );
        m_intakeAngleMotor.configMotionAcceleration(IntakeConstants.INTAKE_ANGLE_MOTOR_ACCELERATION );
        // Set extend motion limits
        m_intakeAngleMotor.configForwardSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MAX*(4096/360));
        m_intakeAngleMotor.configForwardSoftLimitEnable(true);
        m_intakeAngleMotor.configReverseSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MIN*(4096/360));
        m_intakeAngleMotor.configReverseSoftLimitEnable(true);


        m_intakeAngleMotorFollower = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR_FOLLOWER);
        m_intakeAngleMotorFollower.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotorFollower.configPeakCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configContinuousCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.enableCurrentLimit(true);
        m_intakeAngleMotorFollower.setNeutralMode(NeutralMode.Coast);
        m_intakeAngleMotorFollower.setInverted(false);
        m_intakeAngleMotorFollower.follow(m_intakeAngleMotor);

        intakeSpeedEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Intake Roller Speed");
    }

    public void setIntakeRoller( double minus_one_to_one )
    {
        m_intakeRollerMotor.set(minus_one_to_one);
        Logger.recordOutput("IntakeRollerSet", minus_one_to_one );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeAngle( double angle )
    {
        double sensorSetpoint;
        sensorSetpoint = angle * (4096/360);
        m_intakeAngleSetpoint = angle;
        Logger.recordOutput("Intake/AngleSet", angle );
        System.out.println("setIntakeAngle " + angle + ", current angle=" + getIntakeAngle());
        m_intakeAngleMotor.set(ControlMode.MotionMagic, sensorSetpoint );
    }

    public double getIntakeAngle ()
    {
        return (m_intakeAngleCANcoder.getAbsolutePosition());
    }

    public boolean atSetpoint(){
        double intended, current;
        intended = m_intakeAngleSetpoint;
        current = m_intakeAngleCANcoder.getAbsolutePosition();
        if( Math.abs(intended - current ) < IntakeConstants.INTAKE_ANGLE_TOLERANCE )
        {
            return true;
        }
        else
        {
            return false;
        }
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
        Logger.recordOutput("Intake/RollerRPM", rpm );
        m_intakeAngleMotorFollower.follow(m_intakeAngleMotor);

        Logger.recordOutput("Intake/AnglePosition", m_intakeAngleCANcoder.getAbsolutePosition());
        Logger.recordOutput("Intake/Output", m_intakeAngleMotor.getMotorOutputPercent());
        Logger.recordOutput("Intake/FollowOutput", m_intakeAngleMotorFollower.getMotorOutputPercent());
        Logger.recordOutput("Intake/Current", m_intakeAngleMotor.getStatorCurrent());
        Logger.recordOutput("Intake/SupplyA", m_intakeAngleMotor.getSupplyCurrent());
        Logger.recordOutput("Intake/Temp", m_intakeAngleMotor.getTemperature());
        Logger.recordOutput("Intake/FollowCurrent", m_intakeAngleMotorFollower.getStatorCurrent());
        Logger.recordOutput("Intake/FollowTemp", m_intakeAngleMotorFollower.getTemperature());
        Logger.recordOutput("Intake/FollowSupplyA", m_intakeAngleMotorFollower.getSupplyCurrent());
        Logger.recordOutput("Intake/BeamBreakShooter", !m_intakeBeamBreakShooter.get() );
        Logger.recordOutput("Intake/BeamBreakLoaded", !m_intakeBeamBreakLoaded.get() );
    }

    public boolean isIntakeBeamBreakLoaded()
    {
        if( m_intakeBeamBreakLoaded.get() )
            return false;
        else
            return true;
    }
}

