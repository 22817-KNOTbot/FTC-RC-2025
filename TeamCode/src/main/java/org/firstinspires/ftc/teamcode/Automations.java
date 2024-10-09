package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

public class Automations {
	public static State automationState;
	private HardwareMap hardwareMap;
	private boolean DEBUG = false;
	private ElapsedTime servoTime;
	private TelemetryPacket telemetryPacket;
	private FtcDashboard dashboard;

	// Hardware mapping
	private Servo wristLeftServo;
	private Servo wristRightServo;
	private Servo dropArmServo1;
	private Servo dropArmServo2;
	private CRServo scoopServo;
	private ColorRangeSensor colourRangeSensor;
	private Servo clawServo;
	public DcMotor slideMotor1;
	public DcMotor slideMotor2;

	public enum State {
		ABORT,
		IDLE,
		INTAKE_WAIT,
		INTAKE_FILLED,
		INTAKE_DUMPING,
		TRANSFER,
		TRANSFER_WAIT,
		DEPOSIT_EXTENDING,
		DEPOSIT_EXTENDED,
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

	public Automations(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public Automations(HardwareMap inputHardwareMap, boolean DEBUG) {
		Automations.automationState = State.IDLE;
		this.hardwareMap = inputHardwareMap;
		this.DEBUG = DEBUG;
		this.servoTime = new ElapsedTime();
		this.telemetryPacket = new TelemetryPacket();
		this.dashboard = FtcDashboard.getInstance();

		// wristLeftServo = inputHardwareMap.get(Servo.class, "wristLeftServo");
		// wristRightServo = inputHardwareMap.get(Servo.class, "wristRightServo");
		dropArmServo1 = inputHardwareMap.get(Servo.class, "dropArmServoLeft");
		dropArmServo2 = inputHardwareMap.get(Servo.class, "dropArmServoRight");
		dropArmServo1.setDirection(Servo.Direction.REVERSE);
		dropArmServo2.setDirection(Servo.Direction.FORWARD);
		dropArmServo1.scaleRange(0.45, 1);
		dropArmServo2.scaleRange(0, 0.55);
		scoopServo = inputHardwareMap.get(CRServo.class, "scoopServo");
		scoopServo.setDirection(DcMotor.Direction.REVERSE);
		colourRangeSensor = inputHardwareMap.get(ColorRangeSensor.class, "colorSensor");
		clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.setDirection(Servo.Direction.REVERSE);
		clawServo.scaleRange(0.7, 0.85);
		slideMotor1 = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotor2 = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotor1.setTargetPosition(0);
		slideMotor2.setTargetPosition(0);
		slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void updateDashboardTelemetry() {
		dashboard.sendTelemetryPacket(telemetryPacket);
	}

	public void abort() {
		scoopServo.setPower(0);
		slideMotor1.setPower(0);
		slideMotor2.setPower(0);

		automationState = State.IDLE;
	}

	public void intakeInit() {
		// Lower dropdown arm
		dropArmServo1.setPosition(0);
		dropArmServo2.setPosition(0);
		if (DEBUG) {
			telemetryPacket.put("DropArmL", dropArmServo1.getPosition());
			telemetryPacket.put("DropArmR", dropArmServo2.getPosition());
		}
		// Start intake motor
		scoopServo.setPower(0.5);

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
			scoopServo.setPower(0);
			automationState = State.TRANSFER;
		} else {
			// If other alliance, dump
			automationState = State.INTAKE_DUMPING;
		}
	}

	public void intakeDumping() {
		// reverse intake motor until proximity > 50mm
		scoopServo.setPower(-0.5);
		if (colourRangeSensor.getDistance(DistanceUnit.MM) > 50) {
			scoopServo.setPower(0.5);
			automationState = State.INTAKE_WAIT;
		}
		if (DEBUG) {
			telemetryPacket.put("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
		}

	}

	public void transferInit() {
		
		// Rotate dropdown arm motor -_ deg
		dropArmServo1.setPosition(1);
		dropArmServo2.setPosition(1);
		servoTime.reset();

		// # Transfer
		// wristMove range: 60-240
		clawServo.setPosition(0);
		// wristRotate(-180);
		// wristMove(240);
		
		automationState = State.TRANSFER_WAIT;
	}

	public void transferWait() {
		if (servoTime.time() < 2) {return;}
		clawServo.setPosition(1);
		// wristMove(60);
		// automationState = State.IDLE;
		
		automationState = State.IDLE;
	}

	public void depositInit(Basket basket) {
		// Extend linear slide
		setSlidePosition(basket == Basket.HIGH ? 4200 : 2000);
		// TODO: uncomment after wrist added
		// wristRotate(180);

		automationState = State.DEPOSIT_EXTENDING;
	}

	public void depositExtending() {
		if (!slideMotor1.isBusy()) {
			automationState = State.DEPOSIT_EXTENDED;
		}
	}

	public void depositSample() {
		clawServo.setPosition(0);
		
		automationState = State.IDLE;
	}

	public void grabSpecimen() {
		// TODO: Everything
		// Find out what system it will be
		automationState = State.IDLE;
	}

	public void hangSpecimen() {
		// TODO: Everything
		// Find out what system it will be
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
	public void setSlidePosition(int targetPos) {
		slideMotor1.setPower(1);
		slideMotor2.setPower(1);
		slideMotor1.setTargetPosition(targetPos);
		slideMotor2.setTargetPosition(-targetPos);

		slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		if (DEBUG) {
			telemetryPacket.put("Slide 1", slideMotor1.getCurrentPosition());
			telemetryPacket.put("Slide 2", slideMotor2.getCurrentPosition());
		}
	}

	public void wristMove(int deg) {
		wristLeftServo.setDirection(Servo.Direction.REVERSE);

		wristLeftServo.setPosition(deg/300);
		wristRightServo.setPosition(deg/300);

		if (DEBUG) {
			telemetryPacket.put("Wrist", String.format("%.2f|.2f", wristLeftServo.getPosition(), wristRightServo.getPosition()));
		}
	}

	public void wristRotate(int deg) {
		wristLeftServo.setDirection(Servo.Direction.FORWARD);
		wristLeftServo.setDirection(Servo.Direction.REVERSE);

		wristLeftServo.setPosition(deg/300);
		wristRightServo.setPosition(deg/300);
	}
}