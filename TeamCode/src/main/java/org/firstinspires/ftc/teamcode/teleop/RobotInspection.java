package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.CV4B;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name="Robot Inspection", group="Debug")
public class RobotInspection extends LinearOpMode {
	public static CV4B.Positions CV4B_POSITION = CV4B.Positions.DEPOSIT;
	public static Slides.Positions SLIDE_POSITION = Slides.Positions.HIGH_BASKET;
	public static int INTAKE_POSITION = Intake.SLIDE_POSITION_MAX;
	private static boolean extended = false;
	private CV4B cv4b;
	private Intake intake;
	private Slides slides;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		cv4b = new CV4B(hardwareMap);
		intake = new Intake(hardwareMap, false);
		slides = new Slides(hardwareMap, false);

		waitForStart();

		while (opModeIsActive()) {
			if (!extended) {
				cv4b.setPosition(CV4B.Positions.TRANSFER);
				slides.setPosition(Slides.Positions.RETRACTED);
				intake.setSlidePosition(0);
				intake.setIntakePosition(Intake.Positions.TRANSFER);
			} else {
				cv4b.setPosition(CV4B_POSITION);
				slides.setPosition(SLIDE_POSITION);
				intake.setSlidePosition(INTAKE_POSITION);
				intake.setIntakePosition(Intake.Positions.TRANSFER);
			}

			if (gamepad1.left_bumper) {
				extended = false;
			} else if (gamepad1.right_bumper) {
				extended = true;
			}
		}
	}
}