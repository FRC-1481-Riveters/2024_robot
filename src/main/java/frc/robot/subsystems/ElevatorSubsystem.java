package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR, CANSparkLowLevel.MotorType.kBrushless );
    private SparkRelativeEncoder m_encoder = (SparkRelativeEncoder) m_motor.getEncoder();
    private PIDController pidElevator = new PIDController(0.20, 0.090, 0.005);
    private DigitalInput m_beamBreak = new DigitalInput(3);

    private boolean m_pid;
    private double m_setpoint;
    public boolean m_beamBreakState;
    private double m_position;
    private boolean m_atPosition;

    public ElevatorSubsystem(){
      m_motor.restoreFactoryDefaults();
      m_motor.setInverted(true);
      m_motor.setSmartCurrentLimit(30, 30);
      m_motor.setIdleMode(IdleMode.kBrake);
      m_encoder.setPosition(0);
      m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElevatorConstants.ELEVATOR_MAX);
      m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      // Create an initial log entry so they all show up in AdvantageScope without having to enable anything
      Logger.recordOutput("Elevator/Position", 0.0 );
      Logger.recordOutput("Elevator/Setpoint", 0.0);
      Logger.recordOutput("Elevator/Output", 0.0 );
      Logger.recordOutput("Elevator/BeamBreak", false );
      Logger.recordOutput("Elevator/AtPosition", false );

      pidElevator.setIZone(1.0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double pidCalculate;
      double output;
      m_position = m_encoder.getPosition();

      // This method will be called once per scheduler run
      Logger.recordOutput("Elevator/BeamBreak", m_beamBreak.get() );

      // If the elevator is all the way down, zero the encoder
      if( m_beamBreak.get() == false )
      {
        m_position = 0;
        m_encoder.setPosition(m_position);
        m_beamBreakState = true;
      }
      else{
        m_beamBreakState = false;
      }

      m_atPosition = false;

      if( m_pid == true )
      {
        pidCalculate = pidElevator.calculate( m_position, m_setpoint);
        output = MathUtil.clamp( pidCalculate, -0.50, 0.50);
        m_motor.set( output );
        Logger.recordOutput("Elevator/Output", output);
        Logger.recordOutput("Elevator/Current", m_motor.getOutputCurrent());
        if (Math.abs((m_position - m_setpoint)) <= ElevatorConstants.ELEVATOR_POSITION_TOLERANCE)
        {
           m_atPosition = true;
        }
      }

      Logger.recordOutput("Elevator/Position", m_position );
      Logger.recordOutput("Elevator/AtPosition", m_atPosition );
    } // end of method

    public void setElevatorJog( double speed )
    {
      m_pid = false;
      m_motor.set(speed);
      Logger.recordOutput("Elevator/Output", speed);
      System.out.println("setElevatorJog " + speed );
    }

    public void setElevatorPosition (double position){
        System.out.println("setElevatorPosition " + position);

        m_setpoint = position;    
        m_pid = true;
        pidElevator.reset();
        Logger.recordOutput("Elevator/Setpoint", m_setpoint);
    }

    public double getPosition() {
        return m_position;
    }
    
    public boolean isAboveIntake()
    {
      boolean retval;
      if( m_position < ElevatorConstants.ELEVATOR_ABOVE_BUMP )
        retval = true;
      else
        retval = false;
      return retval;
    }

    public boolean isAtPosition() 
    {
      return m_atPosition;
    }

    public void elevatorDisable()
    {
        m_motor.set(0.0);
        m_pid = false;
        System.out.println("elevatorDisable current position=" + getPosition());
        Logger.recordOutput("Elevator/Output", 0.0);
    }

    public void zeroEncoder()
    {
        m_position = 0;
        m_encoder.setPosition(m_position);
    }
}
