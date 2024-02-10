package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.lang.model.util.ElementScanner14;

import org.w3c.dom.css.RGBColor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;


import frc.robot.commands.SwerveJoystickCmd;

public class RobotContainer 
{
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();


    public final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kOperatorControllerPort);

    private boolean isPracticeRobot;

    private Field2d m_field;

    double driveDivider = Constants.DriveConstants.DRIVE_DIVIDER_NORMAL;

    double m_dCreep=0;

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    public RobotContainer() 
    {
        DigitalInput input;
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        
        input = new DigitalInput(9);
        isPracticeRobot = !input.get();
        input.close();

        //configureAutonomousCommands();
    
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> getDriverMoveFwdBack(),
                () -> getDriverMoveLeftRight(),
                () -> getDriverRotate(),
                () -> !driverJoystick.getHID().getBackButton() ));

        configureButtonBindings();

        // Create and push Field2d to SmartDashboard.
        m_field = new Field2d();
        SmartDashboard.putData(m_field);

        // FIXME MUST NOT BE ENABLED WITH FMS!!!
        // FIXME DISABLE THIS BEFORE COMPETITION!
        //PathPlannerServer.startServer(5811); // 5811 = port number. adjust this according to your needs
    }

    private void setBling( int red, int green, int blue )
    {
        int i;
        for( i=0; i<m_ledBuffer.getLength(); i++ )
        {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private double getDriverMoveFwdBack()
    {
        // Handle creeping forward if the driver is pressing D-pad up
        double pos;
        if( m_dCreep != 0 )
            // Use a fixed value to creep forward
            pos = m_dCreep;
        else
            // Use the joystick axis
            pos = driverJoystick.getRawAxis(OIConstants.kDriverYAxis) / driveDivider;
        return pos;
    }

    private double getDriverMoveLeftRight()
    {
        double pos;
        if( m_dCreep != 0 )
            pos = 0;
        else
            pos = driverJoystick.getRawAxis(OIConstants.kDriverXAxis) / driveDivider;
        return pos;
    }

    private double getDriverRotate()
    {
        double pos;
        if( m_dCreep != 0 )
            pos = 0;
        else
            pos = -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis) / driveDivider;
        return pos;
    }

    private void DriveDividerSet( double divider )
    {
        driveDivider = divider;
    }
    
    public void setCreep( double value )
    {
        m_dCreep = value;
        AllianceStationID station;

        station = DriverStation.getRawAllianceStation();

        if( (station == AllianceStationID.Blue1) || 
            (station == AllianceStationID.Blue2) ||
            (station == AllianceStationID.Blue3) ) {
            m_dCreep = m_dCreep * 1.10;
            //juice blue side a little higher
        }
           
        System.out.println("setCreep " + m_dCreep);
    }

    private void configureButtonBindings() 
    {
        Trigger aButton = driverJoystick.a();
        aButton.onTrue( Commands.runOnce( () -> swerveSubsystem.zeroHeading(0.0) ) );

        Trigger driverLeftTrigger = driverJoystick.leftTrigger( 0.7 );
        driverLeftTrigger
            .onFalse(Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_NORMAL )))
            .onTrue( Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_TURBO )));

        Trigger operatorIntakeDeployTrigger = operatorJoystick.y();
        operatorIntakeDeployTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( 0.2 ), intakeSubsystem));

        Trigger operatorIntakeRetractTrigger = operatorJoystick.a();
        operatorIntakeRetractTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( -0.2 ), intakeSubsystem));

        Trigger operatorIntakeWheelsInTrigger = operatorJoystick.leftBumper();
        operatorIntakeWheelsInTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 1.0 ), intakeSubsystem));

        Trigger operatorIntakeWheelsOutTrigger = operatorJoystick.rightBumper();
        operatorIntakeWheelsOutTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( -1.0 ), intakeSubsystem));

        Trigger operatorLeftTrigger = operatorJoystick.leftTrigger( 0.7 );
        operatorLeftTrigger
            .onFalse(Commands.runOnce( ()-> climbSubsystem.setClimb( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> climbSubsystem.setClimb( 0.5 ), climbSubsystem));
        
       //Medium
        Trigger operatorDPadUp = operatorJoystick.povUp();
       operatorDPadUp
            .onTrue(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0.6), shooterSubsystem))
            .onFalse(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));

        //Low
        Trigger operatorDPadLeft = operatorJoystick.povLeft();
        operatorDPadLeft
            .onTrue(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0.4), shooterSubsystem))
            .onFalse(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));

        //High
        Trigger operatorDPadRight = operatorJoystick.povRight();
        operatorDPadRight
            .onTrue(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(1), shooterSubsystem))
            .onFalse(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem));
    }

 }
