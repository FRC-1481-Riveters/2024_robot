package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
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

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax m_elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_elevatorMotor.getEncoder();
    private PIDController pidElevator = new PIDController(0.007, 0, 0);
    private boolean m_elevatorPid;
    private double m_elevatorSetpoint;
    
    public ElevatorSubsystem(){
      m_elevatorMotor.restoreFactoryDefaults();
      m_elevatorMotor.setInverted(true);
      m_elevatorMotor.setSmartCurrentLimit(50, 30);
      m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double position;
      double pidCalculate;
      position = m_encoder.getPosition();

      // This method will be called once per scheduler run
      Logger.recordOutput("ElevatorPosition", position );

      if( m_elevatorPid == true )
      {
        pidCalculate = pidElevator.calculate( position, m_elevatorSetpoint);
        m_elevatorMotor.set(MathUtil.clamp( pidCalculate, -0.4, 0.4));
      }
    }

    public void setElevatorJog( double speed )
    {
      m_elevatorMotor.set(speed);
    }

    public void setElevatorPosition (double position){
        System.out.println("setElevatorPosition " + position);

        m_elevatorSetpoint = position;        
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }
    
    public boolean isAtPosition() {
      if (Math.abs((getPosition() - m_elevatorSetpoint)) <= ElevatorConstants.ELEVATOR_POSITION_TOLERANCE) {
        return true;
      } else {
        return false;
      }
    }

    public void elevatorDisable()
    {
        m_elevatorMotor.set(0);
        m_elevatorPid = false;
        System.out.println("elevatorDisable current angle=" + getPosition());
    }
}
