package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.DigitalInput;


import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_rollerMotor = new CANSparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private TalonSRX m_angleMotor;
    private TalonSRX m_angleMotorFollower;
    private SparkRelativeEncoder m_rollerEncoder = (SparkRelativeEncoder) m_rollerMotor.getEncoder();
    private CANCoder m_angleCANcoder = new CANCoder(IntakeConstants.INTAKE_ANGLE_CANCODER);
    private double m_angleSetpoint;
    private DigitalInput m_BeamBreakShooter = new DigitalInput(0);
    private DigitalInput m_BeamBreakLoaded = new DigitalInput(1);
    private boolean m_BeamBreakLoadedPrevious;
    private RobotContainer m_robotContainer;

    public IntakeSubsystem( RobotContainer robotContainer ) 
    {
        m_robotContainer = robotContainer;
        m_rollerMotor.restoreFactoryDefaults();
        m_rollerMotor.setInverted(false);
        m_rollerMotor.setSmartCurrentLimit(80, 30);
        m_rollerMotor.setIdleMode(IdleMode.kCoast);
        m_angleCANcoder.setPosition(m_angleCANcoder.getAbsolutePosition());

        m_angleMotor = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR);
        m_angleMotor.configFactoryDefault();
        // Set peak current
        m_angleMotor.setInverted(true);
        m_angleMotor.configPeakCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.configContinuousCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotor.enableCurrentLimit(true);
        m_angleMotor.setNeutralMode(NeutralMode.Coast);

        m_angleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_angleMotor.configRemoteFeedbackFilter(m_angleCANcoder, 0);
        // Configure Talon  SRX output and sensor direction
        m_angleMotor.setSensorPhase(true);
        // Set Motion Magic gains in slot0
        m_angleMotor.selectProfileSlot(0, 0);
        m_angleMotor.config_kF(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KF);
        m_angleMotor.config_kP(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KP);
        m_angleMotor.config_kI(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KI);
        m_angleMotor.config_kD(0, IntakeConstants.INTAKE_ANGLE_MOTOR_KD);
        // Set acceleration and cruise velocity
        m_angleMotor.configMotionCruiseVelocity(IntakeConstants.INTAKE_ANGLE_MOTOR_CRUISE );
        m_angleMotor.configMotionAcceleration(IntakeConstants.INTAKE_ANGLE_MOTOR_ACCELERATION );
        // Set extend motion limits
        m_angleMotor.configForwardSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MAX*(4096/360));
        m_angleMotor.configForwardSoftLimitEnable(true);
        m_angleMotor.configReverseSoftLimitThreshold(IntakeConstants.INTAKE_ANGLE_MOTOR_MIN*(4096/360));
        m_angleMotor.configReverseSoftLimitEnable(true);


        m_angleMotorFollower = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR_FOLLOWER);
        m_angleMotorFollower.configFactoryDefault();
        // Set peak current
        m_angleMotorFollower.configPeakCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.configContinuousCurrentLimit(15, IntakeConstants.TALON_TIMEOUT_MS);
        m_angleMotorFollower.enableCurrentLimit(true);
        m_angleMotorFollower.setNeutralMode(NeutralMode.Coast);
        m_angleMotorFollower.setInverted(false);
        m_angleMotorFollower.follow(m_angleMotor);

        // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
        Logger.recordOutput("Intake/RollerSet", 0.0 );
        Logger.recordOutput("Intake/RollerRPM", 0.0 );
        Logger.recordOutput("Intake/AngleSet", 0.0 );
        Logger.recordOutput("Intake/AnglePosition", 0.0 );
        Logger.recordOutput("Intake/Output", 0.0 );
        Logger.recordOutput("Intake/BeamBreakShooter", false );
        Logger.recordOutput("Intake/BeamBreakLoaded", false );
    }

    public void setIntakeRoller( double minus_one_to_one )
    {
        m_rollerMotor.set(minus_one_to_one);
        Logger.recordOutput("Intake/RollerSet", minus_one_to_one );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeAngle( double angle )
    {
        double sensorSetpoint;
        sensorSetpoint = angle * (4096/360);
        m_angleSetpoint = angle;
        m_angleMotor.set(ControlMode.MotionMagic, sensorSetpoint );
        if( angle > (0.9 * IntakeConstants.INTAKE_FLOOR_PICKUP) )
           m_robotContainer.setBling(0, 0, 0);
        Logger.recordOutput("Intake/AngleSet", angle );
        System.out.println("setIntakeAngle " + angle + ", current angle=" + getIntakeAngle());
    }

    public double getIntakeAngle ()
    {
        return (m_angleCANcoder.getAbsolutePosition());
    }

    public boolean atSetpoint(){
        double intended, current;
        intended = m_angleSetpoint;
        current = m_angleCANcoder.getAbsolutePosition();
        if( Math.abs(intended - current ) < IntakeConstants.INTAKE_ANGLE_TOLERANCE )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void intakeAngleDisable(boolean stopped)
    {
        m_angleMotor.set(ControlMode.PercentOutput, 0);
        System.out.println("intakeAngleDisable current angle=" + getIntakeAngle());
    }

    @Override
    public void periodic() 
    {
        double rpm;
        rpm = m_rollerEncoder.getVelocity();
        boolean m_BeamBreakLoadedNew;

        m_BeamBreakLoadedNew = !m_BeamBreakLoaded.get();
        if (m_BeamBreakLoadedPrevious != m_BeamBreakLoadedNew){
            if (m_BeamBreakLoadedNew == true){
                m_robotContainer.setBling(255, 25, 0 );
            }
            else{
                m_robotContainer.setRosie();
            }
        m_BeamBreakLoadedPrevious = m_BeamBreakLoadedNew;
        }

        // This method will be called once per scheduler run
        Logger.recordOutput("Intake/RollerRPM", rpm );
        m_angleMotorFollower.follow(m_angleMotor);  // Recommended by CTRE in case follower loses power

        Logger.recordOutput("Intake/AnglePosition", m_angleCANcoder.getAbsolutePosition());
        Logger.recordOutput("Intake/Output", m_angleMotor.getMotorOutputPercent());
        Logger.recordOutput("Intake/BeamBreakShooter", !m_BeamBreakShooter.get() );
        Logger.recordOutput("Intake/BeamBreakLoaded", m_BeamBreakLoadedNew );
    }

    public boolean isIntakeBeamBreakLoaded()
    {
        if( m_BeamBreakLoaded.get() )
            return false;
        else
            return true;
    }
}
