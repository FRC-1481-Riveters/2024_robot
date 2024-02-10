package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private TalonSRX m_intakeRollerMotor;
    private TalonSRX m_intakeAngleMotor;
    private TalonSRX m_intakeAngleMotorFollower;
    private NetworkTableEntry intakeSpeedEntry;

    public IntakeSubsystem() 
    {
        m_intakeRollerMotor = new TalonSRX(IntakeConstants.INTAKE_ROLLER_MOTOR);
        m_intakeRollerMotor.configFactoryDefault();
        // Set peak current
        m_intakeRollerMotor.configPeakCurrentLimit(30, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeRollerMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeRollerMotor.configContinuousCurrentLimit(25, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeRollerMotor.enableCurrentLimit(true);
        m_intakeRollerMotor.setNeutralMode(NeutralMode.Coast);

        m_intakeAngleMotor = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR);
        m_intakeAngleMotor.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotor.configPeakCurrentLimit(5, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.configContinuousCurrentLimit(5, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotor.enableCurrentLimit(true);
        m_intakeAngleMotor.setNeutralMode(NeutralMode.Brake);

        m_intakeAngleMotorFollower = new TalonSRX(IntakeConstants.INTAKE_ANGLE_MOTOR_FOLLOWER);
        m_intakeAngleMotorFollower.configFactoryDefault();
        // Set peak current
        m_intakeAngleMotorFollower.configPeakCurrentLimit(5, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configPeakCurrentDuration(500, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.configContinuousCurrentLimit(5, IntakeConstants.TALON_TIMEOUT_MS);
        m_intakeAngleMotorFollower.enableCurrentLimit(true);
        m_intakeAngleMotorFollower.setNeutralMode(NeutralMode.Brake);
        m_intakeAngleMotorFollower.setInverted(true);
        m_intakeAngleMotorFollower.follow(m_intakeAngleMotor);

        intakeSpeedEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Intake Speed");
        m_intakeRollerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void setIntakeRoller( double minus_one_to_one )
    {
        double output;
        output = minus_one_to_one;

        m_intakeRollerMotor.set(ControlMode.PercentOutput, output);
        Logger.getInstance().recordOutput("IntakeRollerSet", output );
        System.out.println("setIntakeRoller " + minus_one_to_one);
    }

    public void setIntakeAngle( double minus_one_to_one )
    {
        double output;
        output = minus_one_to_one;

        m_intakeAngleMotor.set(ControlMode.PercentOutput, output);
        Logger.getInstance().recordOutput("IntakeAngleSet", output );
        System.out.println("setIntakeAngle " + minus_one_to_one);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      intakeSpeedEntry.setDouble( m_intakeRollerMotor.getSelectedSensorVelocity() );      
    }

}

