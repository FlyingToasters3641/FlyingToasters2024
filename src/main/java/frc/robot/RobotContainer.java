// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LauncherCommands;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotSystem;
import frc.robot.subsystems.RobotSystem.SystemState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleComp;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFXComp;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOTalonFXComp;

public class RobotContainer {
    //Subsystems Intitalization
    private final RobotSystem m_robotSystem;
    public final DriveSubsystem m_robotDrive;
    private final Intake m_intake;
    private final Launcher m_launcher;
    private final Elevator m_elevator;
    
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public final Limelight m_Limelight = new Limelight();
    
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
  
    public RobotContainer() {
        //Hardware or SIM? Switch the case in Constants class
        switch (Constants.currentMode) {
        case REAL:
        // Real robot, instantiate hardware IO implementations
        m_robotDrive =
            new DriveSubsystem(
                new GyroIOPigeon2(true),
                new SwerveModuleComp(0),
                new SwerveModuleComp(1),
                new SwerveModuleComp(2),
                new SwerveModuleComp(3));
        m_intake = new Intake(new IntakeIOTalonFXComp());   
        m_launcher = new Launcher(new LauncherIOTalonFXComp());
        m_elevator = new Elevator(new ElevatorIOTalonFX());    
        m_robotSystem = new RobotSystem(m_launcher, m_intake, m_elevator, m_Limelight, m_robotDrive); 
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_robotDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_intake = new Intake(new IntakeIO() {});
        m_launcher = new Launcher(new LauncherIO() {});
        m_elevator = new Elevator(new ElevatorIO() {});
        m_robotSystem = new RobotSystem(m_launcher, m_intake, m_elevator, m_Limelight, m_robotDrive); 
        break;

      default:
        // Replayed robot, disable IO implementations
        m_robotDrive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_launcher = new Launcher(new LauncherIO() {});
        m_elevator = new Elevator(new ElevatorIO() {});
        m_robotSystem = new RobotSystem(m_launcher, m_intake, m_elevator, m_Limelight, m_robotDrive);
        break;
    }

    //NamedCommands - used in pathplanner
    NamedCommands.registerCommand("Shoot", LauncherCommands.autoShootNote(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("FastShoot", Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.SHOOT)));
    NamedCommands.registerCommand("Intake", IntakeCommands.intake(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("Front Intake", IntakeCommands.frontIntake(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("Rear Intake", IntakeCommands.rearIntakeNote(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("Aim", Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.AIM)));
    NamedCommands.registerCommand("ShootAndIntake", Commands.runOnce(() ->  m_robotSystem.setGoalState(SystemState.INTAKE_AND_SHOOT)));
    NamedCommands.registerCommand("Outtake", Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.FRONT_OUTTAKE)));
    NamedCommands.registerCommand("AutoAim", LauncherCommands.AutoAutoAim(m_launcher, m_Limelight, m_robotDrive));
    NamedCommands.registerCommand("AimDrive", DriveCommands.AutoAutoAim(m_robotDrive, m_Limelight));
    NamedCommands.registerCommand("SubwoofAim", Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.SUBWOOF_AIM)));
    NamedCommands.registerCommand("SubwoofShoot", LauncherCommands.autoShootSubwoofer(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("Shoot8", LauncherCommands.autoShoot8(m_launcher, m_intake, m_robotSystem));
    NamedCommands.registerCommand("Aim8", Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.AIM_8)));

    //AutoRoutines 
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    
    autoChooser.addDefaultOption("4 Piece Left-Closer", new PathPlannerAuto("SweepingDemon"));
    autoChooser.addOption("4 Piece Left-Far", new PathPlannerAuto("CenterDemon"));
    autoChooser.addOption("6 Piece Middle", new PathPlannerAuto("MiddleCenterDemon"));
    autoChooser.addOption("4 Piece Top-Close", new PathPlannerAuto("TopDemon"));
    autoChooser.addOption("Electro", new PathPlannerAuto("Electro"));
    autoChooser.addOption("2 Piece Fast Center", new PathPlannerAuto("StrykeDemon"));
    autoChooser.addOption("2 Piece Fast Center - Middle Note", new PathPlannerAuto("StrykeDemonV2"));
    autoChooser.addOption("6 Piece Top", new PathPlannerAuto("TopGodDemon"));
    autoChooser.addOption("3 Piece Far Side", new PathPlannerAuto("FarDemon"));

    configureButtonBindings();
    }

    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      m_robotDrive.setDefaultCommand(
              DriveCommands.joystickDrive(
                      m_robotDrive,
                      () -> -m_driverController.getLeftY(),
                      () -> -m_driverController.getLeftX(),
                      () -> -m_driverController.getRightX(),
                      m_Limelight));

      //AutoAim - While holding A, drive will orient itself automatically to the speaker
      m_driverController.a().onTrue(Commands.runOnce(m_robotDrive::setAimGoal)).onFalse(Commands.runOnce(m_robotDrive::clearAimGoal));
      //SubwooferShot - Will toggle a fast shot while the robot is up against the subwoofer
      m_driverController.b().toggleOnTrue(new ConditionalCommand(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.SUBWOOF_AIM)), Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.SUBWOOF_SHOOT)).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE))), ()-> m_robotSystem.getGoalState() != SystemState.SUBWOOF_AIM));
      //Shoot - Hold to start aiming, let go to release the shot
      m_driverController.rightTrigger().onTrue(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.AIM))).onFalse(new ShootNote(m_robotSystem).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE))));
      //Lob - Robot will orient itself and lob occurding to the field pose estimator
      m_driverController.rightBumper().onTrue(LauncherCommands.Lob(m_launcher, m_robotSystem, m_Limelight, m_robotDrive)).onFalse(LauncherCommands.EndLob(m_launcher, m_robotSystem, m_Limelight, m_robotDrive));
      //Intake - Hold to run both intakes, release to stop intake
      m_driverController.leftTrigger().whileTrue(IntakeCommands.intake(m_launcher, m_intake, m_robotSystem)).onFalse(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE)));
      //Outtake - Outtakes the note out to the rear
      m_driverController.leftBumper().whileTrue(IntakeCommands.rearOutakeNote(m_launcher, m_intake, m_robotSystem)).onFalse(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE)));
      //AmplifierScore - Press to allow the climber to deposit a note into the amplifier. Robot will return to base state once the action is finished
      m_driverController.y().toggleOnTrue(new ConditionalCommand(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.AMP_AIM)),Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.AMP_SCORE)).andThen(new WaitCommand(0.75)).andThen(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE))), ()-> m_robotSystem.getGoalState() != SystemState.AMP_AIM ));
      //Climb - Hold to extend climb, let go to release the climer and lock the climbing mechanism 
      m_driverController.x().toggleOnTrue(new ConditionalCommand(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.CLIMB_EXTEND)), ElevatorCommands.climb(m_elevator, m_robotSystem), () -> m_robotSystem.getGoalState() != SystemState.CLIMB_EXTEND));
      //Resets the robot's degree in field pose estimation
      m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setPose(new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d())),m_robotDrive)
                .ignoringDisable(true));
     //Front Outtake - Outtakes the note out to the front if it gets stuck
     m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.FRONT_OUTTAKE))).onFalse(Commands.runOnce(() -> m_robotSystem.setGoalState(SystemState.IDLE)));

        
  }   
  public Command getAutonomousCommand() {
      return autoChooser.get();
  }

}
