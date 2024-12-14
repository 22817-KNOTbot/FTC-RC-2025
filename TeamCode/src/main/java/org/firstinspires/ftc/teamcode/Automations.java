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

import org.firstinspires.ftc.teamcode.subsystems.CV4B;

public class Automations {
	public State automationState;
	private SamplePurpose samplePurpose;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private ElapsedTime timer;
	private FtcDashboard dashboard;
	public TelemetryPacket telemetryPacket;

	// Hardware mapping
	public CV4B cv4b;
	private DcMotor intakeSlides;
	private DcMotor scoopMotor;
	private Servo flipServo;
	private ColorRangeSensor colourRangeSensor;
	private Servo clawServo;
	public DcMotor slideMotorLeft;
	public DcMotor slideMotorRight;

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

		cv4b = new CV4B(hardwareMap);
		intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeSlides.setTargetPosition(0);
		intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		scoopMotor.setDirection(DcMotor.Direction.REVERSE);
		scoopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Bucket: 0.76 = transfer, 0.887 = intake, 0.78 = post-transfer, 0.81 = ascent, 0.84 = beginning of spec auto
		flipServo = hardwareMap.get(Servo.class, "flipServo");

		colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		// Claw: 0 = open, 1 = closed
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.setDirection(Servo.Direction.REVERSE);
		clawServo.scaleRange(0, 0.01);

		slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotorLeft.setTargetPosition(0);
		slideMotorRight.setTargetPosition(0);
		slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void updateDashboardTelemetry() {
		dashboard.sendTelemetryPacket(telemetryPacket);
		telemetryPacket = new TelemetryPacket();
	}

	public void abort() {
		scoopMotor.setPower(0);
		slideMotorLeft.setPower(0);
		slideMotorRight.setPower(0);
		intakeSlides.setPower(0);

		automationState = State.IDLE;
	}

	public void intakeInit(SamplePurpose samplePurpose) {
		this.samplePurpose = samplePurpose;
		// Extend intake slides
		setIntakeSlidePosition(1000);

		if (DEBUG) {
			telemetryPacket.put("intakeSlides", intakeSlides.getCurrentPosition());
		}
		// Start intake motor
		scoopMotor.setPower(0.5);

		automationState = State.INTAKE_WAIT;
	}
	
	public void intakeWait() {
		// Do nothing until distance <15mm
		if (colourRangeSensor.getDistance(DistanceUnit.MM) <= 50) {
			automationState = State.INTAKE_FILLED;
		}	
		if (DEBUG) {
			telemetryPacket.put("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
		}
	}

	public void intakePosition(double input, boolean extend, boolean retract) {
		final double up = 0.8;
		final double down = 0.875;
		flipServo.setPosition(down + input*(up-down));

		if (extend == retract || (500 >= intakeSlides.getCurrentPosition() && intakeSlides.getCurrentPosition() >= 1000)) {
			if (!intakeSlides.isBusy()) {
				setIntakeSlidePosition(intakeSlides.getCurrentPosition());
			}
		} else {
			setIntakeSlidePosition(intakeSlides.getCurrentPosition() + (extend ? 100 : -100));
		}
	}

	public void manualEject() {
		scoopMotor.setPower(-0.5);
	}

	public void stopManualEject() {
		scoopMotor.setPower(0.5);
	}

	public void intakeFilled(Alliance alliance, boolean yellowAllowed) {
		if (samplePurpose == SamplePurpose.SPECIMEN) yellowAllowed = false;
		// Check sample colour
		int red = colourRangeSensor.red();
		int green = colourRangeSensor.green();
		int blue = colourRangeSensor.blue();
		if (DEBUG) {
			telemetryPacket.put("Color", String.format("%d|%d|%d", red, green, blue));
		}

		if ((yellowAllowed ? (
			// Yellow check (if yellow is allowed)
			(red / blue > 2.5) && (green / blue > 3)
		) : false) || (alliance == Alliance.RED ? (
			// Red check
			(red / green > 1.6) && (red / blue > 2)
		) : (
			// Blue check
			(blue / red > 3.5) && (blue / green > 1.2)
		))) {
			// If our alliance, transfer
			// scoopMotor.setPower(0);
			automationState = (samplePurpose == SamplePurpose.SAMPLE) ? State.TRANSFER : State.NO_TRANSFER;
		} else {
			// If other alliance, dump
			automationState = State.INTAKE_DUMPING;
		}
	}

	public void intakeDumping() {
		// reverse intake motor until proximity > 15mm
		scoopMotor.setPower(-0.5);
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 20) {
			scoopMotor.setPower(0.5);
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
		}
	}

	public void transferInit() {		
		// Retract intake slides
		setIntakeSlidePosition(600);
		flipServo.setPosition(0.76);
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		timer.reset();
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (timer.time() > 1.5 && !intakeSlides.isBusy()) {
			scoopMotor.setPower(-0.5);

			automationState = State.TRANSFERRING;
		}
	}

	public void transferring() {
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 75) {
			scoopMotor.setPower(0);
			flipServo.setPosition(0.875);

			automationState = State.TRANSFERRED;
		}
	}

	public void changeTransferSpeed(double power) {
		scoopMotor.setPower(power);
	}

	public void depositInit(Basket basket) {
		// Extend linear slide
		setSlidePosition(basket == Basket.HIGH ? 4100 : 2000); // May be 4100 for high basket
		cv4b.setPosition(CV4B.Positions.PRE_DEPOSIT);
		clawServo.setPosition(1);

		timer.reset();

		automationState = State.DEPOSIT_EXTENDING;
	}

	public void depositExtending() {
		if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
			automationState = State.DEPOSIT_EXTENDED;
		}
	}

	public void depositSample() {
		cv4b.setPosition(CV4B.Positions.DUMP);
		
		automationState = State.DEPOSITED;
	}

	public void resetDeposit() {
		cv4b.setPosition(CV4B.Positions.TRANSFER);
		setSlidePosition(0);

		automationState = State.IDLE;
	}

	// Retract arms without transferring. Designed for samples to be given to HP
	public void noTransfer() {
		setIntakeSlidePosition(600);

		automationState = State.SAMPLE_LOADED;
	}

	public void sampleEjectInit() {
		scoopMotor.setPower(-0.5);
		automationState = State.SAMPLE_EJECT_WAIT;
	}

	public void sampleEject() {
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 75) {
			scoopMotor.setPower(0);
			automationState = State.IDLE;
		}
	}
	
	public void specimenInit() {
		cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);

		timer.reset();

		automationState = State.SPECIMEN_INIT_WAIT;
	}

	public void specimenInitWait() {
		if (timer.time() < 2) return;
		clawServo.setPosition(0);
		automationState = State.SPECIMEN_GRAB_READY;
	}

	public void grabSpecimen() {
		clawServo.setPosition(1);
		setSlidePosition(2100);

		automationState = State.SPECIMEN_GRABBED;
	}

	public void hangSpecimen() {
		setSlidePosition(1500);
		cv4b.setPosition(CV4B.Positions.SPECIMEN_HANG);

		automationState = State.SPECIMEN_HANGING;
	}

	public void hangSpecimenWait() {
		if (!slideMotorLeft.isBusy() && !slideMotorRight.isBusy()) {
			clawServo.setPosition(0);
			setSlidePosition(0);
			cv4b.setPosition(CV4B.Positions.SPECIMEN_GRAB);
		}
	}

	public void resetSpecimen() {
		clawServo.setPosition(1);
		setSlidePosition(0);
		cv4b.setPosition(CV4B.Positions.TRANSFER);

		automationState = State.IDLE;
	}
	
	public void ascendInit() {
		automationState = State.ASCEND_LOW_EXTENDING;
	}
	
	public void ascendLowExtending() {
		// Extend linear slides		
		setSlidePosition(3600);
	
		automationState = State.ASCEND_LOW_EXTENDED;
	}

	public void ascendLowRetract() {
		setSlidePosition(2600);
	
		// TODO: Change state and put proper code
		// automationState = State.ASCEND_HIGH_EXTENDING;
		automationState = State.IDLE;
	}

	// Util functions
	public void setSlidePosition(int targetPos) {
		// Max 4100
		slideMotorLeft.setPower(1);
		slideMotorRight.setPower(1);
		slideMotorLeft.setTargetPosition(targetPos);
		slideMotorRight.setTargetPosition(targetPos);

		if (DEBUG) {
			telemetryPacket.put("Slide 1", slideMotorLeft.getCurrentPosition());
			telemetryPacket.put("Slide 2", slideMotorRight.getCurrentPosition());
		}
	}

	public void setIntakeSlidePosition(int targetPos) {
		// Max ___
		intakeSlides.setPower(1);
		intakeSlides.setTargetPosition(targetPos);

		if (DEBUG) {
			telemetryPacket.put("Intake slides", intakeSlides.getCurrentPosition());
		}
	}

	public void setCV4BPosition(CV4B.Positions position) {
		cv4b.setPosition(position);
	}
}