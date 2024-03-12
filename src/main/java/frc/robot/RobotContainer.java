package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.util.ControllerIO;

/**
 * This class is where the bulk of the robot should be declared. The structure
 * of the robot (including subsystems, commands, and trigger mappings) should be
 * declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Arm arm;
	// private final Climber climber;
	private final Gyrometer gyro;
	private final Intake intake;
	// private final KMeans kmeans;
	// private final Odometry odometry;
	private final Shooter shooter;
	private final Swerve swerve;
	// private final Vision vision;
	private final AutoChooser autoChooser;

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
	private static final CommandXboxController driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private static final CommandXboxController operatorController = new CommandXboxController(
			OIConstants.kOperatorControllerPort);

	// Camera
	public final UsbCamera camera = CameraServer.startAutomaticCapture(0);

	public RobotContainer() {
		// intake camera limitations
		camera.setFPS(30);
		camera.setResolution(320, 240);

		// Create subsystems with real or simulated hardware depending on current mode
		switch (Constants.currentMode) {
			case REAL -> {
				arm = new Arm(new ArmIOSparkMax());
				// climber = new Climber(new ClimberIOSparkMax());
				intake = new Intake(new IntakeIOSparkMax());
				shooter = new Shooter(new ShooterIOSparkMax(), arm);
			}
			case SIM -> {
				arm = new Arm(new ArmIOSim());
				// climber = new Climber(new ClimberIOSim());
				intake = new Intake(new IntakeIOSim());
				shooter = new Shooter(new ShooterIOSim(), arm);
			}
			default -> {
				arm = new Arm(new ArmIO() {
				});
				// climber = new Climber(new ClimberIO() {});
				intake = new Intake(new IntakeIO() {
				});
				shooter = new Shooter(new ShooterIO() {
				}, arm);
			}
		}
		// kmeans = new KMeans();
		// vision = new Vision(kmeans);
		swerve = new Swerve();
		gyro = new Gyrometer(swerve);
		// odometry = new Odometry(vision, gyro);

		// Create AutoChooser
		autoChooser = new AutoChooser();

		// Configure default commands for driving and arm movement
		swerve.setDefaultCommand(new SwerveTeleOp(swerve, gyro,
				() -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
				() -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
				() -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
				() -> OIConstants.controllerTypeDriver == ControllerType.JOYSTICK
						? driverJoystick.getRawButton(ControllerIO.getTrigger())
						: true,
				() -> driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value),
				() -> driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));

		arm.setDefaultCommand(new MoveArm(arm, () -> operatorJoystick.getRawAxis(ControllerIO.getLeftY())));

		// Configure button bindings
		configureBindings();

		// Post webcam feed to Shuffleboard
		Shuffleboard.getTab("Main").add("Camera", camera).withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4)
				.withPosition(5, 0);

		// Configure the PathPlanner auto-builder
		AutoBuilder.configureHolonomic(gyro::getPose, gyro::resetPose, swerve::getRobotRelativeSpeeds,
				swerve::setModuleStates, DriveConstants.kHolonomicConfig, () -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == Alliance.Red;
					}
					return false;
				}, swerve);
	}

	public void configureBindings() {
		// Button bindings
		operatorController.leftBumper().whileTrue(new Pickup(intake));
		operatorController.leftTrigger(0.5).whileTrue(new Drop(intake));
		operatorController.rightTrigger().whileTrue(new Shoot(shooter, intake));

		operatorController.back().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SOURCE)));
		operatorController.start().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERSTAGE)));
		operatorController.a().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.ROOMBA)));
		operatorController.b().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.AMP)));
		operatorController.x().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.TAXI)));
		operatorController.y().onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERFRONT)));

		operatorController.povUp().or(operatorController.povDown()).or(operatorController.povLeft())
				.or(operatorController.povRight()).onTrue(new InstantCommand(() -> intake.toggleSensorDisabled()));

		// Overrides
		driverController.back()
				.onTrue(new InstantCommand(() -> Lights.getInstance().hasNote = !Lights.getInstance().hasNote));
		driverController.start().onTrue(new InstantCommand(() -> gyro.reset()));

		// Register PathPlanner named commands
		NamedCommands.registerCommand("AutoShoot", new AutoShoot(shooter, intake));
		NamedCommands.registerCommand("Pickup", new AutoIntake(intake));
		NamedCommands.registerCommand("PickupTimed", new AutoIntakeTimed(intake));
		NamedCommands.registerCommand("TaxiPosition", new AutoSetArmMode(arm, ArmMode.TAXI, 0.1));
		NamedCommands.registerCommand("AmpPosition", new AutoSetArmMode(arm, ArmMode.AMP, 0.1));
		NamedCommands.registerCommand("IntakePosition", new AutoSetArmMode(arm, ArmMode.ROOMBA, 0.05));
		NamedCommands.registerCommand("SpeakerFrontPosition", new AutoSetArmMode(arm, ArmMode.SPEAKERFRONT, 0.2));
		NamedCommands.registerCommand("SpeakerAnglePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERANGLE, 0.2));
		NamedCommands.registerCommand("SpeakerStagePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERSTAGE, 0.2));
	}

	public Command getAutonomousCommand() {
		if (autoChooser.getAutoCommand().equals("Do Nothing")) {
			return null;
		}

		return new PathPlannerAuto(autoChooser.getAutoCommand());
	}

	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Joystick getOperatorJoystick() {
		return operatorJoystick;
	}
}
