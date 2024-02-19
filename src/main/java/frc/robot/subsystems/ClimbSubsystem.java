package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;

import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

    private TalonSRX m_climbMotor;

    public ClimbSubsystem() 
    {
     /*
         m_climbMotor = new TalonSRX(ClimbConstants.CLIMB_MOTOR);
          
         m_climbMotor.configFactoryDefault();
         // Configure Talon  SRX output and sensor direction
         m_climbMotor.setSensorPhase(false);
         // Set peak current
         m_climbMotor.configPeakCurrentLimit(20, ClimbConstants.TALON_TIMEOUT_MS);
         m_climbMotor.configPeakCurrentDuration(500, ClimbConstants.TALON_TIMEOUT_MS);
         m_climbMotor.configContinuousCurrentLimit(15, ClimbConstants.TALON_TIMEOUT_MS);
         m_climbMotor.enableCurrentLimit(true);
         m_climbMotor.setNeutralMode(NeutralMode.Brake);
         */
    }

    public void setClimb( double minus_one_to_one )
    {
         double output;
         output = minus_one_to_one;

         //m_climbMotor.set(ControlMode.PercentOutput, output);
         Logger.getInstance().recordOutput("ClimbMotor", output );
    }

}
