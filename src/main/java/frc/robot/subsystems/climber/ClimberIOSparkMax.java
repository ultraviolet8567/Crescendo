// package frc.robot.subsystems.climber;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.CAN;
// import frc.robot.Constants.ClimberConstants;

// public class ClimberIOSparkMax implements ClimberIO {
// public final CANSparkMax rightClimberMotor, leftClimberMotor;
// public final RelativeEncoder leftRelativeEncoder, rightRelativeEncoder;
// public final DutyCycleEncoder leftAbsoluteEncoder, rightAbsoluteEncoder;

// public ClimberIOSparkMax() {
// System.out.println("[Init] Creating ClimberIOSparkMax");

// leftClimberMotor = new CANSparkMax(CAN.kLeftClimberPort,
// MotorType.kBrushless);
// leftClimberMotor.enableVoltageCompensation(12.0);
// leftClimberMotor.setSmartCurrentLimit(40);
// leftClimberMotor.setIdleMode(IdleMode.kBrake);

// leftRelativeEncoder = leftClimberMotor.getEncoder();
// leftRelativeEncoder.setPositionConversionFactor(1.0 /
// ClimberConstants.kClimberPositionReduction * 2 * Math.PI);
// leftRelativeEncoder.setVelocityConversionFactor(1.0 /
// ClimberConstants.kClimberVelocityReduction * 2 * Math.PI);

// leftAbsoluteEncoder = new
// DutyCycleEncoder(ClimberConstants.kLeftClimberAbsoluteEncoderPort);

// rightClimberMotor = new CANSparkMax(CAN.kRightClimberPort,
// MotorType.kBrushless);
// rightClimberMotor.enableVoltageCompensation(12.0);
// rightClimberMotor.setSmartCurrentLimit(40);
// rightClimberMotor.setIdleMode(IdleMode.kBrake);

// rightRelativeEncoder = rightClimberMotor.getEncoder();
// rightRelativeEncoder
// .setPositionConversionFactor(1.0 / ClimberConstants.kClimberPositionReduction
// * 2 * Math.PI);
// rightRelativeEncoder
// .setVelocityConversionFactor(1.0 / ClimberConstants.kClimberVelocityReduction
// * 2 * Math.PI);

// rightAbsoluteEncoder = new
// DutyCycleEncoder(ClimberConstants.kLeftClimberAbsoluteEncoderPort);
// }

// @Override
// public void updateInputs(ClimberIOInputs inputs) {
// inputs.leftVelocityRadPerSec = leftRelativeEncoder.getVelocity();
// inputs.leftPositionRads = leftRelativeEncoder.getPosition();
// inputs.leftAppliedVoltage = leftClimberMotor.getAppliedOutput() *
// leftClimberMotor.getBusVoltage();
// inputs.leftCurrentAmps = new double[]{leftClimberMotor.getOutputCurrent()};
// inputs.leftTempCelsius = new
// double[]{leftClimberMotor.getMotorTemperature()};

// inputs.rightVelocityRadPerSec = rightRelativeEncoder.getVelocity();
// inputs.rightPositionRads = rightRelativeEncoder.getPosition();
// inputs.rightAppliedVoltage = rightClimberMotor.getAppliedOutput() *
// rightClimberMotor.getBusVoltage();
// inputs.rightCurrentAmps = new double[]{rightClimberMotor.getOutputCurrent()};
// inputs.rightTempCelsius = new
// double[]{rightClimberMotor.getMotorTemperature()};
// }

// @Override
// public void setInputVoltage(double leftVolts, double rightVolts) {
// setLeftInputVoltage(leftVolts);
// setRightInputVoltage(rightVolts);
// }

// @Override
// public void setLeftInputVoltage(double volts) {
// leftClimberMotor.setVoltage(volts);
// }

// @Override
// public void setRightInputVoltage(double volts) {
// rightClimberMotor.setVoltage(volts);
// }

// @Override
// public double getLeftPositionRads() {
// double position = leftAbsoluteEncoder.getAbsolutePosition();
// position *= 2 * Math.PI;
// position += ClimberConstants.kLeftClimberEncoderOffset;
// return position;
// }

// @Override
// public double getRightPositionRads() {
// double position = rightAbsoluteEncoder.getAbsolutePosition();
// position *= 2 * Math.PI;
// position += ClimberConstants.kRightClimberEncoderOffset;
// return position;
// }

// @Override
// public boolean leftClimberPastFrontLimit() {
// return getLeftPositionRads() < ArmConstants.kMinArmAngle;
// }

// @Override
// public boolean leftClimberPastBackLimit() {
// return getLeftPositionRads() > ArmConstants.kMaxArmAngle;
// }

// @Override
// public boolean rightClimberPastFrontLimit() {
// return getRightPositionRads() < ArmConstants.kMinArmAngle;
// }

// @Override
// public boolean rightClimberPastBackLimit() {
// return getRightPositionRads() > ArmConstants.kMaxArmAngle;
// }

// @Override
// public void resetAbsoluteEncoders() {
// leftAbsoluteEncoder.reset();
// rightAbsoluteEncoder.reset();
// }

// @Override
// public void setBrakeMode(boolean brake) {
// leftClimberMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
// rightClimberMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
// }

// @Override
// public void stop() {
// stopLeft();
// stopRight();
// }

// @Override
// public void stopLeft() {
// setLeftInputVoltage(0.0);
// }

// @Override
// public void stopRight() {
// setRightInputVoltage(0.0);
// }
// }
