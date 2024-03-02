package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import static frc.robot.Constants.*;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax m_elevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_elevatorMotor.getEncoder();
    private PIDController pidElevator = new PIDController(0.13, 0, 0.005);
    private DigitalInput m_elevatorDownBeamBreak = new DigitalInput(3);

    private boolean m_elevatorPid;
    private double m_elevatorSetpoint;
    private int m_positionStable;

    public ElevatorSubsystem(){
      m_elevatorMotor.restoreFactoryDefaults();
      m_elevatorMotor.setInverted(true);
      m_elevatorMotor.setSmartCurrentLimit(20, 20);
      m_elevatorMotor.setIdleMode(IdleMode.kBrake);
      m_encoder.setPosition(0);
      m_elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElevatorConstants.ELEVATOR_AMP_MAX);
      m_elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double position;
      double pidCalculate;
      position = m_encoder.getPosition();

      // This method will be called once per scheduler run
      Logger.recordOutput("Elevator/Position", position );
      Logger.recordOutput("Elevator/BeamBreak", m_elevatorDownBeamBreak.get() );

      if( m_elevatorPid == true )
      {
        pidCalculate = pidElevator.calculate( position, m_elevatorSetpoint);
        m_elevatorMotor.set(MathUtil.clamp( pidCalculate, -0.4, 0.4));
      }

      // If the elevator is all the way down, zero the encoder
      if( m_elevatorDownBeamBreak.get() == false )
      {
        m_encoder.setPosition(0);
      }
    }

    public void setElevatorJog( double speed )
    {
      m_elevatorPid = false;
      m_elevatorMotor.set(speed);
      System.out.println("setElevatorJog " + speed );
    }

    public void setElevatorPosition (double position){
        System.out.println("setElevatorPosition " + position);

        m_elevatorSetpoint = position;    
        m_elevatorPid = true;
        m_positionStable = 0;
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }
    
    public boolean isAtPosition() {
      boolean retval;
      if (Math.abs((getPosition() - m_elevatorSetpoint)) <= ElevatorConstants.ELEVATOR_POSITION_TOLERANCE) 
      {
        m_positionStable++;
        if (m_positionStable >= 4)
        {
          retval = true; 
        }
        else
        {
          retval = false;
        } 
      } 
      else
      {
        m_positionStable = 0;
        retval = false;
      }
      Logger.recordOutput("Elevator/AtPosition", retval );
      return retval;
    }

    public void elevatorDisable()
    {
        m_elevatorMotor.set(0);
        m_elevatorPid = false;
        System.out.println("elevatorDisable current angle=" + getPosition());
    }
}
