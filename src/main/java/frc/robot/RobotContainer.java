package frc.robot;

import java.util.List;

import org.w3c.dom.css.RGBColor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;


public class RobotContainer 
{
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterPivotSubsystem shooterPivotSubsystem = new ShooterPivotSubsystem();

    public final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    public final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kOperatorControllerPort);

    SendableChooser<Command> m_autoChooser;


    private Field2d m_field;

    double driveDivider = Constants.DriveConstants.DRIVE_DIVIDER_NORMAL;

    double m_dCreep=0;

    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    public RobotContainer() 
    {
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
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> getDriverMoveFwdBack(),
                () -> getDriverMoveLeftRight(),
                () -> getDriverRotate(),
                () -> !driverJoystick.getHID().getBackButton() ));

        configureButtonBindings();

        // A chooser for autonomous commands
        // Add a button to run the example auto to SmartDashboard
        //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
        m_autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData( "Auto Mode", m_autoChooser );
        // Register named pathplanner commands
        //NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        //NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        //NamedCommands.registerCommand("print hello", Commands.print("hello"));

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
        Trigger aButton = driverJoystick.start();
        aButton
            .onTrue( Commands.runOnce( () -> swerveSubsystem.zeroHeading(0.0) ) );

        Trigger driverLeftTrigger = driverJoystick.leftTrigger( 0.7 );
        driverLeftTrigger
            .onFalse(Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_NORMAL )))
            .onTrue( Commands.runOnce( ()-> DriveDividerSet( Constants.DriveConstants.DRIVE_DIVIDER_TURBO )));
        
        Trigger driverShootTrigger = driverJoystick.a();
        driverShootTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 1 ), intakeSubsystem));

        Trigger operatorIntakeDeployTrigger = operatorJoystick.y();
        operatorIntakeDeployTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.intakeAngleDisable(), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle(IntakeConstants.INTAKE_FLOOR_PICKUP ), intakeSubsystem));

        Trigger operatorIntakeRetractTrigger = operatorJoystick.a();
        operatorIntakeRetractTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.intakeAngleDisable(), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeAngle( IntakeConstants.INTAKE_ANGLE_STOWED ), intakeSubsystem));


        Trigger operatorIntakeWheelsInTrigger = operatorJoystick.leftBumper();
        operatorIntakeWheelsInTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( -1 ), intakeSubsystem));

        Trigger operatorIntakeWheelsOutTrigger = operatorJoystick.rightBumper();
        operatorIntakeWheelsOutTrigger
            .onFalse(Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 0 ), intakeSubsystem))
            .onTrue( Commands.runOnce( ()-> intakeSubsystem.setIntakeRoller( 1 ), intakeSubsystem));

        Trigger operatorLeftTrigger = operatorJoystick.leftTrigger( 0.7 );
        operatorLeftTrigger
            .onFalse(Commands.runOnce( ()-> climbSubsystem.setClimb( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> climbSubsystem.setClimb( 0.5 ), climbSubsystem));

        Trigger operatorRightJoystickAxisUp = operatorJoystick.axisGreaterThan(5, 0.7 );
        operatorRightJoystickAxisUp
            .onFalse(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 0.5 ), climbSubsystem));
        
        Trigger operatorRightJoystickAxisDown = operatorJoystick.axisLessThan(5, -0.7 );
        operatorRightJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog( -0.5 ), climbSubsystem));
        
        Trigger operatorLeftJoystickAxisUp = operatorJoystick.axisGreaterThan(1, 0.7 );
        operatorLeftJoystickAxisUp 
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0.25 ), climbSubsystem));
        
        Trigger operatorLeftJoystickAxisDown = operatorJoystick.axisLessThan(1, -0.7 );
        operatorLeftJoystickAxisDown
            .onFalse(Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( 0 ), climbSubsystem))
            .onTrue( Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog( -0.25 ), climbSubsystem));
   
        Trigger operatorRightJoystickAxisLeft = operatorJoystick.axisLessThan(4, -0.7 );
        operatorRightJoystickAxisLeft
            .onFalse( Commands.runOnce( ()->shooterSubsystem.setShooterJog(0), shooterSubsystem))
            .onTrue( Commands.runOnce( ()->shooterSubsystem.setShooterJog(1), shooterSubsystem));
        
        Trigger operatorRightJoystickAxisRight = operatorJoystick.axisGreaterThan(4, 0.7 );
        operatorRightJoystickAxisRight
            .onFalse( Commands.runOnce( ()->shooterSubsystem.setShooterJog(0), shooterSubsystem))
            .onTrue( Commands.runOnce( ()->shooterSubsystem.setShooterJog(1), shooterSubsystem));
        
        //Podium
        Trigger operatorDPadUp = operatorJoystick.povUp();
        operatorDPadUp
            .onFalse(
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
            .alongWith (
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0), elevatorSubsystem)
                )
            )      
            .onTrue(
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_PODIUM), shooterSubsystem)
                .alongWith(
                    Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_PODIUM)),
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_PODIUM), elevatorSubsystem))
                    );

        //Close
        Trigger operatorDPadLeft = operatorJoystick.povLeft();
        operatorDPadLeft
         .onFalse(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
            .alongWith (
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0), elevatorSubsystem)
                )
            )      

        .onTrue(
            Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_SPEAKER), shooterSubsystem)
                 .alongWith(
                    Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_SPEAKER)),
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_SPEAKER), elevatorSubsystem))
                    );
            

        //Wing
        Trigger operatorDPadRight = operatorJoystick.povRight();
        operatorDPadRight
            .onFalse(
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
                .alongWith (
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0), elevatorSubsystem)
                )
            )                     
            .onTrue(
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_WING), shooterSubsystem)
                .alongWith(
                    Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_WING)),
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_WING), elevatorSubsystem))
                    )
         ;
        
        //Amp
        Trigger operatorDPadDown = operatorJoystick.povDown();
        operatorDPadDown
            .onFalse(Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(0), shooterSubsystem)
            .alongWith (
                Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivotJog(0), shooterPivotSubsystem),
                Commands.runOnce( ()-> elevatorSubsystem.setElevatorJog(0), elevatorSubsystem)
                )
            )      
            .onTrue(
                Commands.runOnce( ()-> shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_SPEED_AMP), shooterSubsystem)
                 .alongWith(
                    Commands.runOnce( ()-> shooterPivotSubsystem.setShooterPivot(ShooterPivotConstants.SHOOTER_PIVOT_AMP)),
                    Commands.runOnce( ()-> elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_AMP), elevatorSubsystem))
                );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        return m_autoChooser.getSelected();
    }

 }
