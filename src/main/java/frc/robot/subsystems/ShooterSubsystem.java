package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase{
    private double m_shooterIntendedSpeed;
    private CANSparkMax m_shooterMotorTop = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TOP, CANSparkLowLevel.MotorType.kBrushless );
    private CANSparkMax m_shooterMotorBottom = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_shooterMotorTop.getEncoder();
    private SparkPIDController m_pidController = m_shooterMotorTop.getPIDController();
    private NetworkTableEntry shooterOutputEntry;
    private NetworkTableEntry shooterSpeedEntry;
    private NetworkTableEntry shooterSetpointEntry;
    DigitalInput m_shooterBeamBreak = new DigitalInput(2);
    private ShuffleboardTab tab;
    private NetworkTableEntry lightSensor;
    
    public ShooterSubsystem(){
      m_shooterMotorTop.restoreFactoryDefaults();
      m_shooterMotorTop.setInverted(false);
      m_shooterMotorTop.setSmartCurrentLimit(50, 50);
      m_shooterMotorTop.setIdleMode(IdleMode.kCoast);
      m_shooterMotorBottom.restoreFactoryDefaults();
      m_shooterMotorBottom.setInverted(false);
      m_shooterMotorBottom.setSmartCurrentLimit(50, 50);
      m_shooterMotorBottom.setIdleMode(IdleMode.kCoast);
      m_shooterMotorBottom.follow(m_shooterMotorTop,true);
      m_pidController.setP(0.00005);
      m_pidController.setI(0.000000060);
      m_pidController.setD(0.0001);
      m_pidController.setFF(0.000145); //0.00018

      shooterOutputEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Output");
      shooterSpeedEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Speed");
      shooterSetpointEntry = NetworkTableInstance.getDefault().getTable("SmartDashboard").getEntry("Shooter Setpoint");

      shooterOutputEntry.setDouble(0);
      shooterSpeedEntry.setDouble(0);
      shooterSetpointEntry.setDouble(0);

      setShooterSpeed(0.0);

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double speed;
      Logger.recordOutput("Shooter/Speed", getSpeed() );
      Logger.recordOutput("Shooter/BeamBreak", isLightCurtainBlocked() );
    }

    public void setShooterSpeed (double rpm){
        System.out.println("setShooterSpeed " + rpm);
        Logger.recordOutput("Shooter/Setpoint", rpm );
        Logger.recordOutput("Shooter/Jog", 0 );


        m_shooterIntendedSpeed = rpm;
        shooterSetpointEntry.setDouble(m_shooterIntendedSpeed);
        if (m_shooterIntendedSpeed > 10.0) {
          m_pidController.setReference(rpm, ControlType.kVelocity);
        } else {
          m_shooterMotorTop.set(0.0);
        }
        
    }

    public void setShooterJog (double speed){
        System.out.println("setShooterJog " + speed);

        m_shooterIntendedSpeed = 0;
        m_shooterMotorTop.set(speed);
        Logger.recordOutput("Shooter/Setpoint", 0);
        Logger.recordOutput("Shooter/Jog", speed );
   }

    public double getSpeed() {
        return m_encoder.getVelocity();
    }
    
    public boolean isAtSpeed() {
      boolean retval;
      if (Math.abs(
          (getSpeed() - m_shooterIntendedSpeed) / m_shooterIntendedSpeed) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) {
        retval = true;
      } else {
        retval = false;
      }
      Logger.recordOutput("Shooter/AtSpeed", retval );
      return retval;
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
