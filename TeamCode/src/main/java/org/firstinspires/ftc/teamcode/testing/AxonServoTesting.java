package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;

import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
// @TeleOp(name="Servo testing", group="Debug")
public class AxonServoTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static String SERVONAME = "testServo"; 

	@Override
	public void runOpMode() {
		AxonServo testServo = new AxonServo(hardwareMap.get(Servo.class, SERVONAME));
		// testServo.setDirection(Servo.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			testServo.update();
			testServo.setPosition(POSITION);

			telemetry.addData("Position", POSITION);
			telemetry.update();

		}
	}
}