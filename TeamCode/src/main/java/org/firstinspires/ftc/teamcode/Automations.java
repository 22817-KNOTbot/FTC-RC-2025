package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoRotatingClaw;

import org.firstinspires.ftc.teamcode.util.GamepadStorage;

public class Automations {
	public State automationState;
	private Modes mode;
	private SamplePurpose samplePurpose;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private ElapsedTime timer;
	private FtcDashboard dashboard;
	public TelemetryPacket telemetryPacket;

	private Gamepad gamepad1;
	private Gamepad gamepad2;

	// Hardware mapping
	private Intake intake;
	private Slides slides;
	private CV4B cv4b;
	private Claw claw;

	private AutoRotatingClaw autoRotatingClaw;

	// Misc
	private boolean intakeFirstMoving = false;

	public enum State {
		ABORT,
		IDLE,
		// Intake
		INTAKE_READY,
		INTAKE_GRABBING,
		INTAKE_GRABBED,
		INTAKE_RELEASE,
		// Transfer (Variant NO_TRANSFER found below)
		TRANSFER,
		TRANSFER_WAIT,
		TRANSFERRING,
		TRANSFERRED,
		DEPOSIT_EXTENDING,
		DEPOSIT_EXTENDED,
		DEPOSITED,
		// NO_TRANSFER for intake but no transfer (specimens)
		NO_TRANSFER,
		SAMPLE_LOADED,
		SAMPLE_EJECT_WAIT,
		SAMPLE_EJECTED,
		// Specimens
		SPECIMEN_GRABBING,
		SPECIMEN_GRABBED,
		SPECIMEN_HANGING,
		SPECIMEN_HUNG,
		// Ascent
		ASCEND_LOW_EXTENDING,
		ASCEND_LOW_EXTENDED,
		ASCEND_LOW_RETRACTING,
		ASCENDED,

		// Mode switching
		SAMPLE_TO_SPECIMEN,
		SPECIMEN_TO_SAMPLE
	}

	public enum Modes {
		SAMPLE,
		SPECIMEN
	}

	public enum Alliance {
		RED,
		BLUE
	}

	public enum Basket {
		LOW,
		HIGH
	}

	public enum SamplePurpose {
		SAMPLE,
		SPECIMEN
	}

	public Automations(HardwareMap hardwareMap, Modes mode) {
		this(hardwareMap, mode, false);
	}

	public Automations(HardwareMap hardwareMap, Modes mode, boolean DEBUG) {
		this.automationState = State.IDLE;
		this.hardwareMap = hardwareMap;
		this.mode = mode;
		this.DEBUG = DEBUG;
		this.timer = new ElapsedTime();
		this.dashboard = FtcDashboard.getInstance();
		this.telemetryPacket = new TelemetryPacket();

		if (GamepadStorage.gamepad1 != null) this.gamepad1 = GamepadStorage.gamepad1;
		if (GamepadStorage.gamepad2 != null) this.gamepad2 = GamepadStorage.gamepad2;

		intake = new Intake(hardwareMap, false);
		slides = new Slides(hardwareMap, false);
		cv4b = new CV4B(hardwareMap);
		claw = new Claw(hardwareMap);

		autoRotatingClaw = new AutoRotatingClaw(hardwareMap, DEBUG);
	}

	public void updateDashboardTelemetry() {
		if (DEBUG) {
			telemetryPacket.put("Intake Bucket", Intake.intakePosition);
			telemetryPacket.put("Intake Slides", Intake.slidePosition);
			telemetryPacket.put("Intake Slides Value", intake.getSlidePosition());
			telemetryPacket.put("CV4B", CV4B.position);
		}

		dashboard.sendTelemetryPacket(telemetryPacket);
		telemetryPacket = new TelemetryPacket();
	}

	public void abort() {
		intake.abort();
		slides.setPower(0);

		automationState = State.IDLE;
	}

	public void retract() {
		slides.setPosition(Slides.Positions.RETRACTED);
		claw.setPosition(Claw.Positions.OPEN);
		if (mode == Modes.SAMPLE) {
			intake.setPosition(Intake.Positions.TRANSFER);
			cv4b.setPosition(CV4B.Positions.TRANSFER);
		} else if (mode == Modes.SPECIMEN) {
			intake.setPosition(Intake.Positions.RETRACTED);
			cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		}
		intake.setWristRotation(0);
	}

	public void intakeInit(SamplePurpose samplePurpose) {
		this.samplePurpose = samplePurpose;
		intake.setPosition(Intake.Positions.PRE_INTAKE);
		intake.setWristRotation(0);
		intake.openClaw();
		intakeFirstMoving = true;
		timer.reset();

		automationState = State.INTAKE_READY;
	}
	
	public void intakePosition(double x, double y, double heading) {
		double wristTarget = 0;
		double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
		double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

		if (x == 0 && y == 0) {
			autoRotatingClaw.process();
			double angle = autoRotatingClaw.getRotation();
			// double modifiedAngle = angle-Math.pow(angle, 0.3);
			wristTarget = angle * Intake.WRIST_VALUE_PER_DEG;
		} else if (rotatedX == 0) {
			wristTarget = 0;
		} else if (rotatedY == 0) {
			wristTarget = 90 * Intake.WRIST_VALUE_PER_DEG;
		} else {		
			// This formula clips the angle to the range -90deg to 90.
			// Values outside this range are clipped to their opposite (ex: 135deg becomes -45deg)
			double clippedAngle = ((Math.toDegrees(Math.atan2(rotatedX * (rotatedY / Math.abs(rotatedY)), Math.abs(rotatedY))) + 180) % 360) - 180;
			wristTarget = clippedAngle * Intake.WRIST_VALUE_PER_DEG;
		}

		intake.setWristRotation(wristTarget);
		if (DEBUG) {
			telemetryPacket.put("Wrist Rotation", wristTarget);
		}
	}

	public void intakeGrab() {
		intake.setIntakePosition(Intake.Positions.INTAKE);

		timer.reset();

		automationState = State.INTAKE_GRABBING;
	}

	public void intakeGrabbing() {
		if (timer.time() < 0.5) return;
		intake.closeClaw();
		if (timer.time() < 0.8) return;

		automationState = State.INTAKE_GRABBED;
	}
	
	public void intakeGrabbed(Alliance alliance, boolean yellowAllowed, boolean forceIntake) {
		if (samplePurpose == SamplePurpose.SPECIMEN) yellowAllowed = false;
		// Check sample colour
		if (DEBUG) {
			telemetryPacket.put("Color", String.format("%d|%d|%d", intake.getRed(), intake.getGreen(), intake.getBlue()));
		}

		if (
		forceIntake ||
		(intake.getDistance(DistanceUnit.MM) <= 20 &&
		yellowAllowed ? (
			intake.checkSample(Intake.SampleColours.YELLOW)
		) : false) || (alliance == Alliance.RED ? (
			intake.checkSample(Intake.SampleColours.RED)
		) : (
			intake.checkSample(Intake.SampleColours.BLUE)
		))) {
			// If our alliance, transfer
			automationState = (samplePurpose == SamplePurpose.SAMPLE) ? State.TRANSFER : State.NO_TRANSFER;
		} else {
			// If other alliance, dump
			automationState = State.INTAKE_RELEASE;
		}
	}

	public void intakeRelease() {
		intake.openClaw();
		intake.setIntakePosition(Intake.Positions.PRE_INTAKE);
		
		automationState = State.INTAKE_READY;
	}

	public void transferInit() {		
		// Retract intake slides
		intake.setPosition(Intake.Positions.POST_INTAKE);
		intake.setWristRotation(0);
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		claw.setPosition(Claw.Positions.OPEN);
		vibrateControllers();
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (!intake.isSlideBusyFast()) {
			intake.setPosition(Intake.Positions.TRANSFER);
			intake.setSlidePosition(-100);
			timer.reset();

			automationState = State.TRANSFERRING;
		}
	}

	public void transferring() {
		if (timer.time() > 1) {
			if (timer.time() < 1.2) {
				claw.setPosition(Claw.Positions.CLOSED);
			} else {
				intake.openClaw();
				intake.setPosition(Intake.Positions.TRANSFER);
				vibrateControllers();

				automationState = State.TRANSFERRED;
			}
		}	
	}

	public void depositInit(Basket basket) {
		claw.setPosition(Claw.Positions.CLOSED);
		// Extend linear slide
		slides.setPosition(basket == Basket.HIGH ? Slides.Positions.HIGH_BASKET : Slides.Positions.LOW_BASKET);
		timer.reset();

		automationState = State.DEPOSIT_EXTENDING;
	}

	public void depositExtending() {
		if (timer.time() < 0.7) return;

		cv4b.setPosition(CV4B.Positions.DEPOSIT);

		automationState = State.DEPOSIT_EXTENDED;
	}

	public void depositSample() {
		claw.setPosition(Claw.Positions.OPEN);

		automationState = State.DEPOSITED;
	}

	public void resetDeposit() {
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		slides.setPosition(Slides.Positions.RETRACTED);

		automationState = State.IDLE;
	}

	// Retract arms without transferring. Designed for samples to be given to HP
	public void noTransfer() {
		intake.setPosition(Intake.Positions.POST_INTAKE);
		intake.setWristRotation(0);

		automationState = State.SAMPLE_LOADED;
	}

	public void sampleEject() {
		intake.setSlidePosition(Intake.Positions.INTAKE);
		timer.reset();
		
		automationState = State.SAMPLE_EJECT_WAIT;
	}

	public void sampleEjectWait() {
		if (timer.time() < 0.3) return;
		intake.openClaw();
		automationState = State.SAMPLE_EJECTED;
	}

	public void resetSampleEject() {
		intake.setPosition(Intake.Positions.TRANSFER);

		automationState = State.IDLE;
	}

	public void grabSpecimen() {
		claw.setPosition(Claw.Positions.CLOSED);
		timer.reset();

		automationState = State.SPECIMEN_GRABBING;
	}

	public void grabSpecimenWait() {
		if (timer.time() < 0.5) return;
		cv4b.setPosition(CV4B.Positions.SPECIMEN_HANG);
		slides.setPosition(Slides.Positions.HIGH_CHAMBER_PREHANG);

		automationState = State.SPECIMEN_GRABBED;
	}

	public void hangSpecimen() {
		slides.setPosition(Slides.Positions.HIGH_CHAMBER_HANG);
		timer.reset();

		automationState = State.SPECIMEN_HANGING;
	}

	public void hangSpecimenWait() {
		if (timer.time() > 0.5) {
			claw.setPosition(Claw.Positions.OPEN);
			vibrateControllers();

			automationState = State.SPECIMEN_HUNG;
		}
	}

	public void resetSpecimen() {
		slides.setPosition(Slides.Positions.RETRACTED);
		cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);

		automationState = State.IDLE;
	}

	public void ascendInit() {
		// Extend linear slides
		slides.setPosition(Slides.Positions.ASCEND_TWO_PRE);
		timer.reset();

		automationState = State.ASCEND_LOW_EXTENDING;
	}

	public void ascendLowExtending() {
		if (timer.time() > 0.5) {
			automationState = State.ASCEND_LOW_EXTENDED;
		}
	}

	public void ascendLowRetract() {
		slides.setPosition(Slides.Positions.ASCEND_TWO_RETRACT);

		// TODO: Change state and put proper code
		// automationState = State.ASCEND_HIGH_EXTENDING;
		automationState = State.ASCENDED;
	}

	/*
	 * Mode handling
	 */
	public Modes getMode() {
		return mode;
	}

	public void setMode(Modes targetMode) {
		if (targetMode == Modes.SAMPLE && mode == Modes.SPECIMEN) {
			timer.reset();
			specimenToSample();
			automationState = State.SPECIMEN_TO_SAMPLE;
		} else if (targetMode == Modes.SPECIMEN && mode == Modes.SAMPLE) {
			timer.reset();
			sampleToSpecimen();
			automationState = State.SAMPLE_TO_SPECIMEN;
		}
	}

	public void specimenToSample() {
		if (timer.time() < 0.3) {
			cv4b.setPosition(CV4B.Positions.DEPOSIT);
		} else if (timer.time() < 0.6) {
			intake.setIntakePosition(Intake.Positions.TRANSFER);
		} else if (timer.time() < 1) {
			cv4b.setPosition(CV4B.Positions.TRANSFER);
		} else {
			mode = Modes.SAMPLE;
			automationState = State.IDLE;
		}
	}

	public void sampleToSpecimen() {
		if (timer.time() < 0.3) {
			cv4b.setPosition(CV4B.Positions.DEPOSIT);
		} else if (timer.time() < 0.6) {
			intake.setIntakePosition(Intake.Positions.RETRACTED);
		} else if (timer.time() < 1) {
			cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		} else {
			mode = Modes.SPECIMEN;
			automationState = State.IDLE;
		}
	}

	/*
	 * Util functions
	 */
	public int getSlideLeftPosition() {
		return slides.getSlideLeftPosition();
	}

	public int getSlideRightPosition() {
		return slides.getSlideRightPosition();
	}

	public boolean getSlideBusy() {
		return slides.isSlideBusy();
	}

	public void setSlidesPower(double power) {
		slides.setPower(power);
	}

	public boolean colourSensorResponding() {
		return intake.colourSensorResponding();
	}

	public void vibrateControllers() {
		vibrateControllers(100);
	}

	public void vibrateControllers(int durationMs) {
		if (gamepad1 != null) gamepad1.rumble(1, 1, durationMs);
		if (gamepad2 != null) gamepad2.rumble(1, 1, durationMs);
	}
}