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

public class Automations {
	public State automationState;
	private SamplePurpose samplePurpose;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private ElapsedTime timer;
	private FtcDashboard dashboard;
	public TelemetryPacket telemetryPacket;

	// Hardware mapping
	private Servo cv4bLeftServo;
	private Servo cv4bRightServo;
	private Servo cv4bCoaxialServo;
	private Servo dropArmServo1;
	private Servo dropArmServo2;
	private DcMotor scoopMotor;
	private ColorRangeSensor colourRangeSensor;
	private Servo clawServo;
	public DcMotor slideMotor1;
	public DcMotor slideMotor2;

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
		TRANSFERRED,
		DEPOSIT_EXTENDING,
		DEPOSIT_EXTENDED,
		DEPOSITED,
		// NO_TRANSFER for intake but no transfer (specimens)
		NO_TRANSFER,
		SAMPLE_LOADED,
		SAMPLE_EJECT_WAIT,
		SAMPLE_EJECTED,
		SPECIMEN_GRAB_READY,
		SPECIMEN_GRABBED,
		SPECIMEN_HUNG,
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

		// cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
		// cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
		// cv4bLeftServo.setDirection(Servo.Direction.REVERSE);
		// cv4bRightServo.setDirection(Servo.Direction.FORWARD);
		// cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
		// cv4bCoaxialServo.scaleRange(0, 0.6);
		// Dropdown arms: 0 = down, 1 = up
		dropArmServo1 = hardwareMap.get(Servo.class, "dropArmServoLeft");
		dropArmServo2 = hardwareMap.get(Servo.class, "dropArmServoRight");
		dropArmServo1.setDirection(Servo.Direction.REVERSE);
		dropArmServo2.setDirection(Servo.Direction.FORWARD);
		dropArmServo1.scaleRange(0.45, 1);
		dropArmServo2.scaleRange(0, 0.55);
		// scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		// scoopMotor.setDirection(DcMotor.Direction.REVERSE);
		colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
		// Claw: 0 = open, 1 = closed
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.setDirection(Servo.Direction.REVERSE);
		clawServo.scaleRange(0.7, 0.85);
		slideMotor1 = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotor2 = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotor2.setDirection(DcMotor.Direction.REVERSE);
		slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotor1.setTargetPosition(0);
		slideMotor2.setTargetPosition(0);
		slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void updateDashboardTelemetry() {
		dashboard.sendTelemetryPacket(telemetryPacket);
		telemetryPacket = new TelemetryPacket();
	}

	public void abort() {
		scoopMotor.setPower(0);
		slideMotor1.setPower(0);
		slideMotor2.setPower(0);

		automationState = State.IDLE;
	}

	public void intakeInit(SamplePurpose samplePurpose) {
		this.samplePurpose = samplePurpose;
		// Lower dropdown arm
		dropArmServo1.setPosition(0);
		dropArmServo2.setPosition(0);
		if (DEBUG) {
			telemetryPacket.put("DropArmL", dropArmServo1.getPosition());
			telemetryPacket.put("DropArmR", dropArmServo2.getPosition());
		}
		// Start intake motor
		scoopMotor.setPower(0.5);

		automationState = State.INTAKE_WAIT;
	}
	
	public void intakeWait() {
		// Do nothing until distance <10mm
		if (colourRangeSensor.getDistance(DistanceUnit.MM) < 10) {
			automationState = State.INTAKE_FILLED;
		}	
		if (DEBUG) {
			telemetryPacket.put("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
		}
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
			scoopMotor.setPower(0);
			automationState = samplePurpose == SamplePurpose.SAMPLE ? State.TRANSFER : State.NO_TRANSFER;
		} else {
			// If other alliance, dump
			automationState = State.INTAKE_DUMPING;
		}
	}

	public void intakeDumping() {
		// reverse intake motor until proximity > 50mm
		scoopMotor.setPower(-0.5);
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 50) {
			scoopMotor.setPower(0.5);
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
		}
	}

	public void transferInit() {		
		// Rotate dropdown arm motor up
		dropArmServo1.setPosition(1);
		dropArmServo2.setPosition(1);
		// setCV4BPosition(0, 0);

		timer.reset();
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (timer.time() > 2) {
			// setCV4BPosition(0, 0.2);

			automationState = State.TRANSFERRED;
		} else if (timer.time() > 1) {
			scoopMotor.setPower(-0.5);
		}
	}

	public void depositInit(Basket basket) {
		// Extend linear slide
		setSlidePosition(basket == Basket.HIGH ? 4200 : 2000);
		// setCV4BPosition(0.7, 0.3);

		automationState = State.DEPOSIT_EXTENDING;
	}

	public void depositExtending() {
		if (!slideMotor1.isBusy()) {
			automationState = State.DEPOSIT_EXTENDED;
		}
	}

	public void depositSample() {
		// setCV4BPosition(0.7, 1);
		
		automationState = State.DEPOSITED;
	}

	public void resetDeposit() {
		// setCV4BPosition(0, 0);
		setSlidePosition(0);

		automationState = State.IDLE;
	}

	// Retract arms without transferring. Designed for samples to be given to HP
	public void noTransfer() {
		dropArmServo1.setPosition(1);
		dropArmServo2.setPosition(1);

		automationState = State.SAMPLE_LOADED;
	}

	public void sampleEjectInit() {
		dropArmServo1.setPosition(0);
		dropArmServo2.setPosition(0);

		timer.reset();

		automationState = State.SAMPLE_EJECT_WAIT;
	}

	public void sampleEject() {
		if (timer.time() > 1) {
			if (colourRangeSensor.getDistance(DistanceUnit.MM) < 50) {
				scoopMotor.setPower(-0.5);
			} else {
				scoopMotor.setPower(0);
				automationState = State.SAMPLE_EJECTED;
			}
		}
	}

	public void resetEject() {
		dropArmServo1.setPosition(1);
		dropArmServo2.setPosition(1);

		automationState = State.IDLE;
	}

	public void specimenInit() {
		// setCV4BPosition(0.8, 0.5);
		clawServo.setPosition(0);

		automationState = State.SPECIMEN_GRAB_READY;
	}

	public void grabSpecimen() {
		clawServo.setPosition(1);
		setSlidePosition(3000);

		automationState = State.SPECIMEN_GRABBED;
	}

	public void hangSpecimen() {
		setSlidePosition(0);
		clawServo.setPosition(0);

		automationState = State.SPECIMEN_HUNG;
	}

	public void resetSpecimen() {
		// setCV4BPosition(0, 0);
		clawServo.setPosition(1);

		automationState = State.IDLE;
	}
	
	public void ascendInit() {
		automationState = State.ASCEND_LOW_EXTENDING;
	}
	
	public void ascendLowExtending() {
		// Extend linear slides		
		setSlidePosition(2200);
	
		automationState = State.ASCEND_LOW_EXTENDED;
	}

	public void ascendLowRetract() {
		setSlidePosition(0);
	
		// TODO: Change state and put proper code
		// automationState = State.ASCEND_HIGH_EXTENDING;
		automationState = State.IDLE;
	}

	// Util functions
	public void setCV4BPosition(float v4bRot, float coaxialRot) {
		// 0 = in, 1 = out
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}

	public void setSlidePosition(int targetPos) {
		// Max 4200
		slideMotor1.setPower(1);
		slideMotor2.setPower(1);
		slideMotor1.setTargetPosition(targetPos);
		slideMotor2.setTargetPosition(targetPos);

		if (DEBUG) {
			telemetryPacket.put("Slide 1", slideMotor1.getCurrentPosition());
			telemetryPacket.put("Slide 2", slideMotor2.getCurrentPosition());
		}
	}
}