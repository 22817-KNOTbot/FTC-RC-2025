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

import org.firstinspires.ftc.teamcode.subsystems.CV4B;

@Config
@TeleOp(name="Manual Control", group="Debug")
public class ManualControl extends LinearOpMode {
	public static double BUCKET_POSITION = 0.875;
	public static double SCOOP_POWER = 0;
	public static CV4B.Positions CV4B_POSITION = CV4B.Positions.TRANSFER;
	public static boolean MANUAL_CV4B = false;
	public static double POSITION_V4B = 0;
	public static double POSITION_COAX = 0;
	public static int SLIDE_POSITION = 0;
	public static double SLIDE_POWER = 1;
	public static int INTAKE_POSITION = 0;
	public static double INTAKE_SLIDE_POWER = 1;
	public static double CLAW_POSITION = 1;
	public static boolean CV4B_ENABLED = true;
	private CV4B cv4b;
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

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		// Bucket: 0.75 = transfer, 0.879 = intake, 0.78 = post-transfer
		// Servo flipServo = hardwareMap.get(Servo.class, "flipServo");
		// DcMotor scoopMotor = hardwareMap.get(DcMotor.class, "scoopMotor");
		// scoopMotor.setDirection(DcMotor.Direction.REVERSE);
		// scoopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		cv4b = new CV4B(hardwareMap);

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
		// 3500

		// DcMotor intakeSlides = hardwareMap.get(DcMotor.class, "intakeSlides");
		// intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		// intakeSlides.setTargetPosition(0);
		// intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// Transfer: 500

		// Claw: 0 = open, 1 = closed
		Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
		clawServo.setDirection(Servo.Direction.REVERSE);
		clawServo.scaleRange(0, 0.01);

		// ColorRangeSensor colourRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

		waitForStart();

		while (opModeIsActive()) {
			// flipServo.setPosition(BUCKET_POSITION);
			// scoopMotor.setPower(SCOOP_POWER);
			if (CV4B_ENABLED) {
				if (MANUAL_CV4B) {
					cv4b.setPosition(POSITION_V4B, POSITION_COAX);
				} else {
					cv4b.setPosition(CV4B_POSITION);
				}
			} else {
				cv4b.setDrivePWM(CV4B_ENABLED);
				cv4b.setCoaxialPWM(CV4B_ENABLED);
			}
			slideMotorLeft.setPower(SLIDE_POWER);
			slideMotorRight.setPower(SLIDE_POWER);
			setSlidePosition(SLIDE_POSITION);
			// intakeSlides.setPower(INTAKE_SLIDE_POWER);
			// intakeSlides.setTargetPosition(INTAKE_POSITION);
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

	public void setSlidePosition(int targetPos) {
		// Max 4200
		slideMotorLeft.setTargetPosition(targetPos);
		slideMotorRight.setTargetPosition(targetPos);
	}
}