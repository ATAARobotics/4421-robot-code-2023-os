package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {

    // The initial position of the robot relative to the field. This is measured
    // from the left-hand corner of the field closest to the driver, from the
    // driver's perspective

    public Translation2d initialPosition = new Translation2d(0, 0);

    // Create hardware objects
    private Pigeon pigeon = new Pigeon();
    private final OI joysticks = new OI();

    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    // Auto Stuff
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private double swerveSpeed = Constants.SLOW_MAXIMUM_SPEED;
    private double swerveSpeedRot = Constants.SLOW_MAXIMUM_ROTATIONAL_SPEED;


    public RobotContainer(Alliance alliance) {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        
        m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "rio", alliance);
        // new AprilTagLimelight(m_swerveDriveSubsystem.getOdometry(), m_swerveDriveSubsystem);
     
        m_swerveDriveSubsystem.setBrakes(true);
        // Timer.delay(10);
        m_swerveDriveSubsystem.setDefaultCommand(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                        joysticks::getYVelocity,
                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                        this::getSwerveSpeedRot));
        

        // Testing Autos
        // autoChooser.addOption("Square", new Square(m_swerveDriveSubsystem));
        //autoChooser.addOption("Test", new Test(m_swerveDriveSubsystem, m_intakeSubsystem));

        // autoChooser.addOption("SquareWithRot", new SquareWithRot(m_swerveDriveSubsystem));
        // autoChooser.addOption("SquareWithOtherRot", new SquareWithOtherRot(m_swerveDriveSubsystem));


        SmartDashboard.putData("Auto Chooser", autoChooser);

        LiveWindow.disableAllTelemetry();

        configureBindings();
    }


    private void configureBindings() {

        

        //joysticks.ResetOdo.onTrue(new InstantCommand(m_swerveDriveSubsystem::resetPosition));
        // joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
        // .onFalse(new InstantCommand(m_telescopingSubsystem::stop));

        // joysticks.SlideLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0.1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));
        // joysticks.SlideRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -0.1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));
        // joysticks.RotateLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -0.1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));
        // joysticks.RotateRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
        //                 () -> -0.1,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));   
        joysticks.Forward.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -1,
                                () -> 0,
                                () -> 0, this::getSwerveSpeed,
                                this::getSwerveSpeedRot))
                        .onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                joysticks::getYVelocity,
                                joysticks::getRotationVelocity, this::getSwerveSpeed,
                                this::getSwerveSpeedRot));
        joysticks.RecordOffset.onTrue(new InstantCommand(m_swerveDriveSubsystem::setOffsets, m_swerveDriveSubsystem));
        //.onFalse(new InstantCommand(() -> {swerveSpeed=Constants.SLOW_MAXIMUM_SPEED; swerveSpeedRot=Constants.SLOW_MAXIMUM_ROTATIONAL_SPEED;}));

    }

    public OI getOI() {
        return joysticks;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return m_swerveDriveSubsystem;
    }

   

    public SendableChooser<Command> getAutonomousChooser() {
        return autoChooser;
    }
    public double getSwerveSpeed(){
        return swerveSpeed;
    }
    public double getSwerveSpeedRot(){
        return swerveSpeedRot;
    }
}
