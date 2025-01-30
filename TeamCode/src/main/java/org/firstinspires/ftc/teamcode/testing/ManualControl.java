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

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name="Manual Control", group="Debug")
public class ManualControl extends LinearOpMode {
	public static Claw.Positions CLAW_POSITION = Claw.Positions.CLOSED;
	public static boolean CV4B_ENABLED = true;
	public static CV4B.Positions CV4B_POSITION = CV4B.Positions.TRANSFER;
	public static boolean MANUAL_CV4B = false;
	public static double POSITION_V4B = 0;
	public static double POSITION_COAX = 0;
	public static Slides.Positions SLIDE_POSITION = Slides.Positions.RETRACTED;
	public static int SLIDE_POSITION_MANUAL = -1;
	public static double SLIDE_POWER = 1;
	public static Intake.Positions BUCKET_POSITION = Intake.Positions.INTAKE;
	public static double BUCKET_POSITION_MANUAL_DRIVE = -1;
	public static double BUCKET_POSITION_MANUAL_COAX = -1;
	public static double SCOOP_POWER = 0;
	public static Intake.Positions INTAKE_POSITION = Intake.Positions.TRANSFER;
	public static int INTAKE_POSITION_MANUAL = -1;
	public static double INTAKE_SLIDE_POWER = 1;

	private Intake intake;
	private Slides slides;
	private CV4B cv4b;
	private Claw claw;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		intake = new Intake(hardwareMap, false);
		slides = new Slides(hardwareMap, false);
		cv4b = new CV4B(hardwareMap);
		claw = new Claw(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
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

			slides.setPower(SLIDE_POWER);
			if (SLIDE_POSITION_MANUAL == -1) {
				slides.setPosition(SLIDE_POSITION);
			} else {
				slides.setPosition(SLIDE_POSITION_MANUAL);
			}

			if (BUCKET_POSITION_MANUAL_DRIVE == -1 || BUCKET_POSITION_MANUAL_COAX == -1) {
				intake.setBucketPosition(BUCKET_POSITION);
			} else {
				intake.setBucketPosition(BUCKET_POSITION_MANUAL_DRIVE, BUCKET_POSITION_MANUAL_COAX);
			}
			intake.setPower(SCOOP_POWER);
			intake.setSlidePower(INTAKE_SLIDE_POWER);
			if (INTAKE_POSITION_MANUAL == -1) {
				intake.setSlidePosition(INTAKE_POSITION);
			} else {
				intake.setSlidePosition(INTAKE_POSITION_MANUAL);
			}

			claw.setPosition(CLAW_POSITION);

			if (intake.colourSensorResponding()) {
				int red = intake.getRed();
				int green = intake.getGreen();
				int blue = intake.getBlue();
		
				if ((red / blue > 2.5) && (green / blue > 3)) {
					telemetry.addData("Colour", "YELLOW");
				} else if ((red / green > 1.6) && (red / blue > 2)) {
					telemetry.addData("Colour", "RED");
				} else if ((blue / red > 3.5) && (blue / green > 1.2)) {
					telemetry.addData("Colour", "BLUE");
				}
				telemetry.addData("Distance", intake.getDistance(DistanceUnit.MM));
			}
			telemetry.addData("Intake Touch", intake.isTouched());
			telemetry.addData("Slide 1", slides.getSlideLeftPosition());
			telemetry.addData("Slide 2", slides.getSlideRightPosition());
			telemetry.update();

		}
	}
}