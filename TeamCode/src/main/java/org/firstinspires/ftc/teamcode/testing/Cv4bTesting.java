package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.CV4B;

@Config
@TeleOp(name="CV4B testing", group="Debug")
public class Cv4bTesting extends LinearOpMode {
	public static CV4B.Positions POSITION = CV4B.Positions.BASE;
	public static boolean MANUAL = false;
	public static double POSITION_V4B = 0;
	public static double POSITION_COAX = 0;
	public CV4B cv4b;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		cv4b = new CV4B(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			if (MANUAL) {
				cv4b.setPosition(POSITION_V4B, POSITION_COAX);
			} else {
				cv4b.setPosition(POSITION);
			}
		}
	}
}