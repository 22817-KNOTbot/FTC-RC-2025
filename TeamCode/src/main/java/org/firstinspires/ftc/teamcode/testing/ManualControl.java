package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Manual Control", group="Debug")
public class ManualControl extends LinearOpMode {
	public static double BUCKET_POSITION = 0.93;
	public static double SCOOP_POWER = 0;
	public static Cv4bPosition CV4B_POSITION = Cv4bPosition.BASE;
	public static double POSITION_V4B = 0;
	public static double POSITION_COAX = 0;
	public static int SLIDE_POSITION = 0;
	public static int INTAKE_POSITION = 0;
	public static double CLAW_POSITION = 1;
	private Servo cv4bLeftServo;
	private Servo cv4bRightServo;
	private Servo cv4bCoaxialServo;
	public DcMotor slideMotorLeft;
	public DcMotor slideMotorRight;

	/*
	 * Control
	 * scoopMotor
	 * rightFront
	 * slideMotorRIght
	 * rightBack
	 * 
	 * cv4bRightServo
	 * .
	 * cv4bCoaxialServo
	 * 
	 * Expansion
	 * 
	 */

	private enum Cv4bPosition {
		BASE,
		TRANSFER,
		PRE_DEPOSIT,
		DUMP,
		SPECIMEN_GRAB,
		MANUAL
	}

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		// Bucket: 0 = up, 1 = down
		Servo flipServo = hardwareMap.get(Servo.class, "flipServo");
		// flipServo.scaleRange(0.85, 0.93);
		DcMotor scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		scoopMotor.setDirection(DcMotor.Direction.REVERSE);

		cv4bLeftServo = hardwareMap.get(Servo.class, "cv4bLeftServo");
		cv4bRightServo = hardwareMap.get(Servo.class, "cv4bRightServo");
		cv4bLeftServo.setDirection(Servo.Direction.FORWARD);
		cv4bRightServo.setDirection(Servo.Direction.REVERSE);
		cv4bCoaxialServo = hardwareMap.get(Servo.class, "cv4bCoaxialServo");
		cv4bCoaxialServo.setDirection(Servo.Direction.REVERSE);

		slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");
		slideMotorLeft.setPower(1);
		slideMotorRight.setPower(1);
		slideMotorLeft.setTargetPosition(0);
		slideMotorRight.setTargetPosition(0);
		slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		DcMotor intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		intakeSlides.setPower(1);
		intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		intakeSlides.setTargetPosition(0);
		intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Claw: 0 = open, 1 = closed
		Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.setDirection(Servo.Direction.REVERSE);
		clawServo.scaleRange(0, 0.2);

		// ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		waitForStart();

		while (opModeIsActive()) {
			flipServo.setPosition(BUCKET_POSITION);
			scoopMotor.setPower(SCOOP_POWER);
			setCV4BPosition(CV4B_POSITION);
			setSlidePosition(SLIDE_POSITION);
			intakeSlides.setTargetPosition(INTAKE_POSITION);
			clawServo.setPosition(CLAW_POSITION);

			// int red = colourRangeSensor.red();
			// int green = colourRangeSensor.green();
			// int blue = colourRangeSensor.blue();
	
			// if ((red / blue > 2.5) && (green / blue > 3)) {
			// 	telemetry.addData("Colour", "YELLOW");
			// } else if ((red / green > 1.6) && (red / blue > 2)) {
			// 	telemetry.addData("Colour", "RED");
			// } else if ((blue / red > 3.5) && (blue / green > 1.2)) {
			// 	telemetry.addData("Colour", "BLUE");
			// }
			// telemetry.addData("Distance", colourRangeSensor.getDistance(DistanceUnit.MM));
			telemetry.addData("Slide 1", slideMotorLeft.getCurrentPosition());
			telemetry.addData("Slide 2", slideMotorRight.getCurrentPosition());
			telemetry.update();

		}
	}

	public void setCV4BPosition(Cv4bPosition position) {
		switch (position) {
			case BASE:
				setCV4BPosition(0.14, 0.2);
				break;
			case TRANSFER:
				setCV4BPosition(0.3, 0.1);
				break;
			case PRE_DEPOSIT:
				setCV4BPosition(0.68, 0.35);
				break;
			case DUMP:
				setCV4BPosition(0.68, 0.8);
				break;
			case SPECIMEN_GRAB:
				setCV4BPosition(0.78, 0.5);
				break;
			case MANUAL:
				setCV4BPosition(POSITION_V4B, POSITION_COAX);
				break;
		}
	}

	public void setCV4BPosition(double v4bRot, double coaxialRot) {
		cv4bLeftServo.setPosition(v4bRot);
		cv4bRightServo.setPosition(v4bRot);
		cv4bCoaxialServo.setPosition(coaxialRot);
	}

	public void setSlidePosition(int targetPos) {
		// Max 4200
		slideMotorLeft.setTargetPosition(targetPos);
		slideMotorRight.setTargetPosition(targetPos);
	}
}