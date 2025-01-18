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

	// Misc
	private boolean intakeFirstMoving = false;

	public enum State {
		ABORT,
		IDLE,
		// Intake
		INTAKE_WAIT,
		INTAKE_FILLED,
		INTAKE_DUMPING,
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
		// Specimens
		SPECIMEN_INIT_WAIT,
		SPECIMEN_GRAB_READY,
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
		this.mode = mode;
		this.hardwareMap = hardwareMap;
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
	}

	public void updateDashboardTelemetry() {
		if (DEBUG) {
			telemetryPacket.put("Intake Bucket", Intake.bucketPosition);
			telemetryPacket.put("Intake Slides", Intake.slidePosition);
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
		if (mode == Modes.SAMPLE) {
			intake.setPosition(Intake.Positions.TRANSFER);
			cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		} else if (mode == Modes.SPECIMEN) {
			intake.setPosition(Intake.Positions.RETRACTED);
			cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		}
	}

	public void intakeInit(SamplePurpose samplePurpose) {
		this.samplePurpose = samplePurpose;
		intake.setSlidePosition(Intake.Positions.INTAKE);
		intake.setPower(0.6);
		intakeFirstMoving = true;
		timer.reset();

		automationState = State.INTAKE_WAIT;
	}

	public void intakeWait() {
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) <= 50) || intake.isTouched()) {
			vibrateControllers();
			automationState = State.INTAKE_FILLED;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", intake.getDistance(DistanceUnit.MM));
			telemetryPacket.put("Touched", intake.isTouched());
		}
	}

	public void intakePosition(double input, boolean extend, boolean retract) {
		if (timer.time() < 0.5) return;
		final double up = Intake.BUCKET_INTAKE_HIGH;
		final double down = Intake.BUCKET_INTAKE_LOW;
		intake.setBucketPosition(down + input * (up - down));

		if (intakeFirstMoving) {
			if (!intake.isSlideBusy()) {
				intakeFirstMoving = false;
			}
			return;
		}
		if (extend != retract) {
			intake.setSlidePosition(extend ? Intake.SLIDE_POSITION_MAX : Intake.SLIDE_POSITION_MIN);
		} else {
			intake.setSlidePosition(intake.getSlidePosition());
		}
	}

	public void setIntakePower(double power) {
		intake.setPower(power);
	}

	public void intakeFilled(Alliance alliance, boolean yellowAllowed) {
		if (samplePurpose == SamplePurpose.SPECIMEN) yellowAllowed = false;
		// Check sample colour
		if (DEBUG) {
			telemetryPacket.put("Color", String.format("%d|%d|%d", intake.getRed(), intake.getGreen(), intake.getBlue()));
		}

		if ((yellowAllowed ? (
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
			automationState = State.INTAKE_DUMPING;
		}
	}

	public void intakeDumping() {
		intake.setPower(-0.5);
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) > 95) && !intake.isTouched()) {
			intake.setPower(0.6);
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", intake.getDistance(DistanceUnit.MM));
			telemetryPacket.put("Touched", intake.isTouched());
		}
	}

	public void transferInit() {		
		// Retract intake slides
		intake.setPosition(Intake.Positions.TRANSFER);
		intake.setPower(0);
		cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		claw.setPosition(Claw.Positions.OPEN);
		timer.reset();
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (timer.time() > 1 && intake.getSlidePosition() < 110) {
			cv4b.setPosition(CV4B.Positions.TRANSFER);
			timer.reset();

			automationState = State.TRANSFERRING;
		}
	}

	public void transferring() {
		if (timer.time() > 0.5 && timer.time() < 0.8) {
			claw.setPosition(Claw.Positions.CLOSED);
		} else if (timer.time() > 0.8) {
			intake.setPosition(Intake.Positions.POST_TRANSFER);
			vibrateControllers();

			automationState = State.TRANSFERRED;
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
		if (timer.time() < 0.5) return;

		cv4b.setPosition(CV4B.Positions.DEPOSIT);

		automationState = State.DEPOSIT_EXTENDED;
	}

	public void depositSample() {
		claw.setPosition(Claw.Positions.OPEN);

		automationState = State.DEPOSITED;
	}

	public void resetDeposit() {
		cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		slides.setPosition(Slides.Positions.RETRACTED);

		automationState = State.IDLE;
	}

	// Retract arms without transferring. Designed for samples to be given to HP
	public void noTransfer() {
		// TODO: Don't retract
		intake.setSlidePosition(Intake.Positions.TRANSFER);

		automationState = State.SAMPLE_LOADED;
	}

	public void sampleEjectInit() {
		intake.setPower(-0.5);
		automationState = State.SAMPLE_EJECT_WAIT;
	}

	public void sampleEject() {
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) > 95) && !intake.isTouched()) {
			intake.setPower(0);
			automationState = State.IDLE;
		}
	}

	public void specimenInit() {
		cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		claw.setPosition(Claw.Positions.OPEN);

		timer.reset();

		automationState = State.SPECIMEN_INIT_WAIT;
	}

	public void specimenInitWait() {
		if (timer.time() < 0.5) return;
		vibrateControllers();

		automationState = State.SPECIMEN_GRAB_READY;
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
			intake.setBucketPosition(Intake.Positions.TRANSFER);
		} else if (timer.time() < 1) {
			cv4b.setPosition(CV4B.Positions.PRE_TRANSFER);
		} else {
			mode = Modes.SAMPLE;
			automationState = State.IDLE;
		}
	}

	public void sampleToSpecimen() {
		if (timer.time() < 0.3) {
			cv4b.setPosition(CV4B.Positions.DEPOSIT);
		} else if (timer.time() < 0.6) {
			intake.setBucketPosition(Intake.Positions.RETRACTED);
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