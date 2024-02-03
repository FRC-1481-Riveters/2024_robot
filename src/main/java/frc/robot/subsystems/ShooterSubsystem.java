package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private double m_shooterIntendedSpeed = 10000.0;
    private CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR, MotorType.kBrushless);
    private CANSparkMax m_shooterMotorFollower = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_FOLLOWER, MotorType.kBrushless);
    private SparkMaxRelativeEncoder m_encoder = (SparkMaxRelativeEncoder) m_shooterMotor.getEncoder();
    //TalonSRX m_kickerMotor = new TalonSRX(KICKER_MOTOR);
    //private SparkMaxPIDController m_pidController = m_shooterMotor.getPIDController();
    private NetworkTableEntry shooterKp;
    private NetworkTableEntry shooterKi;
    private NetworkTableEntry shooterKd;
    private NetworkTableEntry shooterKf;
    private NetworkTableEntry shooterOutputEntry;
    private NetworkTableEntry shooterSpeedEntry;
    private NetworkTableEntry shooterSetpointEntry;
    DigitalInput m_shooterBeamBreak = new DigitalInput(0);
    private ShuffleboardTab tab;
    private NetworkTableEntry lightSensor;
    
    public ShooterSubsystem(){
      /*m_kickerMotor.configFactoryDefault();
      m_kickerMotor.setInverted(true);
      m_kickerMotor.setNeutralMode(NeutralMode.Brake);
      m_kickerMotor.configPeakCurrentLimit(20, 5000);
      m_kickerMotor.configPeakCurrentDuration(200, 5000);
      m_kickerMotor.configContinuousCurrentLimit(15, 5000);
      m_kickerMotor.enableCurrentLimit(true);*/
      m_shooterMotor.restoreFactoryDefaults();
      //m_shooterMotor.setInverted(true);
      m_shooterMotor.setSmartCurrentLimit(50, 50);
      m_shooterMotor.setIdleMode(IdleMode.kCoast);
      //m_pidController.setP(0.0008);
      //m_pidController.setI(0.000000060);
      //m_pidController.setD(0.0001);
      //m_pidController.setFF(0.000145); //0.00018

      shooterKp = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kP");
      shooterKi = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kI");
      shooterKd = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kD");
      shooterKf = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter kF");
      shooterOutputEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Output");
      shooterSpeedEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Speed");
      shooterSetpointEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Setpoint");

      //shooterKp.setDouble(m_pidController.getP());
      //shooterKi.setDouble(m_pidController.getI());
      //shooterKd.setDouble(m_pidController.getD());
      //shooterKf.setDouble(m_pidController.getFF());

      shooterOutputEntry.setDouble(0);
      shooterSpeedEntry.setDouble(0);
      shooterSetpointEntry.setDouble(0);

      setShooterSpeed(0.0);

      //FIXME: ugh why doesn't this work sometimes
      //m_kickerMotor.setInverted(true);
      //m_shooterMotor.setInverted(true);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //shooterOutputEntry.setDouble( m_shooterMotor.getBusVoltage() );
      //shooterSpeedEntry.setDouble( getSpeed() );
    }

    public void setShooterSpeed (double RPM){
        m_shooterMotor.set(RPM);
        m_shooterMotorFollower.set(RPM);
        /*m_shooterIntendedSpeed = RPM;
        shooterSetpointEntry.setDouble(m_shooterIntendedSpeed);
        if (m_shooterIntendedSpeed > 10.0) {
          m_pidController.setReference(RPM, ControlType.kVelocity);
        } else {
          m_shooterMotor.set(0.0);
        }*/
        
    }

    public void setKickerSpeed(double value){
        //m_kickerMotor.set(ControlMode.PercentOutput, value);
    }
    public double getSpeed() {
        return m_encoder.getVelocity();
    }
    
    public boolean isAtSpeed() {
      if (Math.abs(
          (getSpeed() - m_shooterIntendedSpeed) / m_shooterIntendedSpeed) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) {
        return true;
      } else {
        return false;
      }
    }
    
    public boolean isLightCurtainBlocked(){
      if (m_shooterBeamBreak.get() == true){
        return false;
      }
      else{
        return true;
      }
    }    
}
