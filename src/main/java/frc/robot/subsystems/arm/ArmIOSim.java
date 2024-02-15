package frc.robot.subsystems.arm;

import static frc.robot.Constants.GainsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
	private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(2), ArmConstants.kArmReduction,
			ArmConstants.kArmJKgMetersSquared, ArmConstants.kArmLength, ArmConstants.kMinArmAngle,
			ArmConstants.kMaxArmAngle, true, Math.PI / 2);
	private final Constraints armConstraints = new Constraints(ArmConstants.kMaxSpeed.get(),
			ArmConstants.kMaxAcceleration.get());
	private final ProfiledPIDController armPIDController = new ProfiledPIDController(armGains.kP(), armGains.kI(),
			armGains.kD(), armConstraints);
	private ArmFeedforward armFF = new ArmFeedforward(armGains.ffkS(), armGains.ffkG(), armGains.ffkV(),
			armGains.ffkA());

	Mechanism2d arm = new Mechanism2d(2, 2);
	MechanismRoot2d armRoot = arm.getRoot("Arm Root", 1, 0.5);
	MechanismLigament2d superstructureMech = armRoot.append(new MechanismLigament2d("Superstructure", 0.2, 90));
	MechanismLigament2d armMech = superstructureMech.append(new MechanismLigament2d("Arm", 0.58, 90));
	MechanismLigament2d intakeMech = armMech.append(new MechanismLigament2d("Intake", 0.36, 64.0));
	MechanismLigament2d shooterMech = armMech.append(new MechanismLigament2d("Shooter", 0.11, 244.0));

	private double appliedVoltage = 0.0;

	public ArmIOSim() {
		System.out.println("[Init] Creating ArmIOSim");

		armPIDController.disableContinuousInput();
		armPIDController.setTolerance(ArmConstants.kArmPIDTolerance.get());

		resetPIDControllers();
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		armSim.update(0.02);
		inputs.velocityRadPerSec = getPositionRads();
		inputs.positionRads = armSim.getAngleRads();
		inputs.appliedVoltage = appliedVoltage;
		inputs.currentAmps = new double[]{armSim.getCurrentDrawAmps()};
	}

	@Override
	public void update() {
		// Update Mechanism2D simulation stuff
		armMech.setAngle(getPositionRads());

		Logger.recordOutput("ArmMechanism", arm);
		SmartDashboard.putData("Mechanism", arm);
	}

	// Returns the current position of the arm in radians
	@Override
	public double getPositionRads() {
		return armSim.getAngleRads();
	}

	// Calculates the voltage to run the motors at
	@Override
	public void setPosition(double setpoint) {
		double volts = (armPIDController.calculate(getPositionRads(), setpoint)
			+ armFF.calculate(getPositionRads(), setpoint));

		setInputVoltage(volts);
	}

	// Sets the input voltage for a motor
	@Override
	public void setInputVoltage(double volts) {
		double appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
		armSim.setInputVoltage(appliedVoltage);
	}

	// Checks if the arm is past the from limit (could hit ground/front of robot)
	@Override
	public boolean armPastFrontLimit() {
		return armSim.hasHitLowerLimit();
	}

	// Checks if the arm is past the back limit (could hit back/tip over robot)
	@Override
	public boolean armPastBackLimit() {
		return armSim.hasHitUpperLimit();
	}

	// Check if the arm can be moved
	@Override
	public boolean armWithinRange() {
		return !armPastFrontLimit() && !armPastBackLimit();
	}

	// Gets the preset angle in radians for the arm
	@Override
	public void getPreset() {
		// what function would i use in sim mode?
	}

	@Override
	public void stop() {
		setInputVoltage(0.0);
	}

	public void setGains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
		armPIDController.setPID(kP, kI, kD);
		armFF = new ArmFeedforward(ffkS, ffkG, ffkV, ffkA);
	}

	@Override
	public void resetAbsoluteEncoders() {
		// don't need to do because simulation measures angle accurately
	}

	@Override
	public void resetPIDControllers() {
		armPIDController.reset(getPositionRads());
	}
}
