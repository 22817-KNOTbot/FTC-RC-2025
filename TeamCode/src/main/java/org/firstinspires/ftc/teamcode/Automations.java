package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

public class Automations {
	public State automationState;
	private SamplePurpose samplePurpose;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private ElapsedTime timer;
	private FtcDashboard dashboard;
	public TelemetryPacket telemetryPacket;

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

	public Automations(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public Automations(HardwareMap inputHardwareMap, boolean DEBUG) {
		this.automationState = State.IDLE;
		this.hardwareMap = inputHardwareMap;
		this.DEBUG = DEBUG;
		this.timer = new ElapsedTime();
		this.dashboard = FtcDashboard.getInstance();
		this.telemetryPacket = new TelemetryPacket();

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
		intake.setSlidePosition(Intake.Positions.TRANSFER);
		cv4b.setPosition(CV4B.Positions.TRANSFER);
	}

	public void intakeInit(SamplePurpose samplePurpose) {
		this.samplePurpose = samplePurpose;
		intake.setPosition(Intake.Positions.INTAKE);
		intake.setPower(0.5);
		intakeFirstMoving = true;

		automationState = State.INTAKE_WAIT;
	}
	
	public void intakeWait() {
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) <= 50) /* && intake.isTouched() */) {
			automationState = State.INTAKE_FILLED;
		}	
		if (DEBUG) {
			telemetryPacket.put("Distance", intake.getDistance(DistanceUnit.MM));
		}
	}

	public void intakePosition(double input, boolean extend, boolean retract) {
		final double up = Intake.BUCKET_INTAKE_HIGH;
		final double down = Intake.BUCKET_INTAKE_LOW;
		intake.setBucketPosition(down + input*(up-down));

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
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) > 30) /* && !intake.isTouched() */) {
			intake.setPower(0.5);
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", intake.getDistance(DistanceUnit.MM));
		}
	}

	public void transferInit() {		
		// Retract intake slides
		intake.setPosition(Intake.Positions.TRANSFER);
		intake.setPower(0);
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		claw.setPosition(Claw.Positions.OPEN);
		timer.reset();
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (timer.time() > 1.5 && !intake.isSlideBusy()) {
			claw.setPosition(Claw.Positions.CLOSED);
			if (timer.time() > 1.7) {

				// Note: originally TRANSFERRING - may need to change back
				automationState = State.TRANSFERRED;
			}
		}
	}

	// TODO: Verify transferring is not needed, then remove it. Never called currently
	public void transferring() {
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) > 75) /* && !intake.isTouched() */) {
			intake.setPower(0);

			automationState = State.TRANSFERRED;
		}
	}

	public void depositInit(Basket basket) {
		// Extend linear slide
		slides.setPosition(basket == Basket.HIGH ? Slides.Positions.HIGH_BASKET : Slides.Positions.LOW_BASKET);
		cv4b.setPosition(CV4B.Positions.DEPOSIT);

		automationState = State.DEPOSIT_EXTENDING;
	}

	public void depositExtending() {
		if (!slides.slideIsBusy()) {
			automationState = State.DEPOSIT_EXTENDED;
		}
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
		intake.setSlidePosition(Intake.Positions.TRANSFER);

		automationState = State.SAMPLE_LOADED;
	}

	public void sampleEjectInit() {
		intake.setPower(-0.5);
		automationState = State.SAMPLE_EJECT_WAIT;
	}

	public void sampleEject() {
		if ((!colourSensorResponding() || intake.getDistance(DistanceUnit.MM) > 75) /* && !intake.isTouched() */) {
			intake.setPower(0);
			automationState = State.IDLE;
		}
	}
	
	public void specimenInit() {
		cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		claw.setPosition(Claw.Positions.OPEN);

		automationState = State.SPECIMEN_INIT_WAIT;
	}

	public void specimenInitWait() {
		if (timer.time() < 2) return;
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

		automationState = State.SPECIMEN_HANGING;
	}

	public void hangSpecimenWait() {
		if (!slides.slideIsBusy()) {
			claw.setPosition(Claw.Positions.OPEN);

			automationState = State.SPECIMEN_HUNG;
		}
	}

	public void resetSpecimen() {
		claw.setPosition(Claw.Positions.CLOSED);
		slides.setPosition(Slides.Positions.RETRACTED);
		cv4b.setPosition(CV4B.Positions.TRANSFER);

		automationState = State.IDLE;
	}
	
	public void ascendInit() {
		// Extend linear slides		
		slides.setPosition(Slides.Positions.ASCEND_PRE);

		automationState = State.ASCEND_LOW_EXTENDING;
	}
	
	public void ascendLowExtending() {
		if (!slides.slideIsBusy()) {
			automationState = State.ASCEND_LOW_EXTENDED;
		}
	}

	public void ascendLowRetract() {
		slides.setPosition(Slides.Positions.ASCEND_RETRACT);
	
		// TODO: Change state and put proper code
		// automationState = State.ASCEND_HIGH_EXTENDING;
		automationState = State.IDLE;
	}

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
}