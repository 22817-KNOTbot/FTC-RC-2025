package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo testing", group="Debug")
public class ServoTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static String SERVONAME = "testServo"; 

	@Override
	public void runOpMode() {
		Servo testServo = hardwareMap.get(Servo.class, SERVONAME);
		// testServo.setDirection(Servo.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			testServo.setPosition(POSITION);
			// 250-210
			// open close

			telemetry.addData("Position", POSITION);
			telemetry.update();

		}
	}
}