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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autos.AutoSetArmMode;
import frc.robot.commands.autos.AutoShoot;
import frc.robot.commands.autos.AutoIntake;
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
	private final KMeans kmeans;
	// private final Odometry odometry;
	private final Shooter shooter;
	private final Swerve swerve;
	// private final Vision vision;
	private final AutoChooser autoChooser;

	// Joysticks
	private static final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
	private static final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
	// private static final CommandXboxController operatorController = new
	// CommandXboxController(OIConstants.kOperatorControllerPort);

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

		kmeans = new KMeans();
		// vision = new Vision(kmeans);
		swerve = new Swerve();
		gyro = new Gyrometer(swerve);
		// odometry = new Odometry(vision, gyro);

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
				.withPosition(3, 0);

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

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate
	 */
	public void configureBindings() {
		new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Pickup(intake));
		new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value)
				.whileTrue(new Shoot(shooter, intake));

		new JoystickButton(operatorJoystick, XboxController.Button.kY.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERFRONT)));
		new POVButton(operatorJoystick, 0).onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERANGLE)));
		new JoystickButton(operatorJoystick, XboxController.Button.kStart.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.SPEAKERSTAGE)));
		new JoystickButton(operatorJoystick, XboxController.Button.kB.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.TAXI)));
		new JoystickButton(operatorJoystick, XboxController.Button.kA.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.ROOMBA)));
		new JoystickButton(operatorJoystick, XboxController.Button.kX.value)
				.onTrue(new InstantCommand(() -> arm.setArmMode(ArmMode.AMP)));
		// new JoystickButton(operatorJoystick, XboxController.Button.kY.value)
		// .whileTrue(arm.routine().dynamic(Direction.kForward));
		// new JoystickButton(operatorJoystick, XboxController.Button.kB.value)
		// .whileTrue(arm.routine().quasistatic(Direction.kForward));
		// new JoystickButton(operatorJoystick, XboxController.Button.kA.value)
		// .whileTrue(arm.routine().dynamic(Direction.kReverse));
		// new JoystickButton(operatorJoystick, XboxController.Button.kX.value)
		// .whileTrue(arm.routine().quasistatic(Direction.kReverse));
		// new JoystickButton(operatorJoystick,
		// XboxController.Button.kStart.value).onTrue(new InstantCommand(() ->
		// arm.setArmMode(ArmMode.TRAP)));

		new JoystickButton(operatorJoystick, XboxController.Button.kBack.value).whileTrue(new Drop(intake));

		// Overrides
		new JoystickButton(driverJoystick, XboxController.Button.kStart.value)
				.onTrue(new InstantCommand(() -> gyro.reset()));
		new JoystickButton(driverJoystick, XboxController.Button.kBack.value)
				.onTrue(new InstantCommand(() -> Lights.getInstance().hasNote = !Lights.getInstance().hasNote));

		NamedCommands.registerCommand("AutoShoot", new AutoShoot(shooter, intake));
		NamedCommands.registerCommand("Pickup", new AutoIntake(intake));
		NamedCommands.registerCommand("TaxiPosition", new AutoSetArmMode(arm, ArmMode.TAXI));
		NamedCommands.registerCommand("AmpPosition", new AutoSetArmMode(arm, ArmMode.AMP));
		NamedCommands.registerCommand("IntakePosition", new AutoSetArmMode(arm, ArmMode.ROOMBA));
		NamedCommands.registerCommand("SpeakerFrontPosition", new AutoSetArmMode(arm, ArmMode.SPEAKERFRONT));
		NamedCommands.registerCommand("SpeakerAnglePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERANGLE));
		NamedCommands.registerCommand("SpeakerStagePosition", new AutoSetArmMode(arm, ArmMode.SPEAKERSTAGE));

		// new JoystickButton(operatorJoystick, XboxController.Button.kStart.value)
		// .whileTrue(new Climb(climber, "extend"));
		// new JoystickButton(operatorJoystick, XboxController.Button.kBack.value)
		// .whileTrue(new Climb(climber, "retract"));

		/*
		 * Will switch to this soon to simplify button constructors
		 *
		 * operatorController.leftTrigger().onTrue(new Command());
		 * operatorController.leftTrigger().whileTrue(new Command());
		 *
		 * operatorController.a().onTrue(new Command());
		 * operatorController.rightTrigger().whileTrue(new Command());
		 *
		 * operatorController.povUp().onTrue(new Command());
		 */
	}

	public Command getAutonomousCommand() {
		return new PathPlannerAuto(autoChooser.getAutoCommand());
	}

	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Joystick getOperatorJoystick() {
		return operatorJoystick;
	}
}
