package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ManualHardwareManager;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
// @TeleOp(name="Servo testing", group="Debug")
public class ServoTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static int SERVO_PORT = -1;
	public static String SERVO_NAME = "testServo"; 

	@Override
	public void runOpMode() {
		Servo testServo;
		if (SERVO_PORT < 0) {
			testServo = hardwareMap.get(Servo.class, SERVO_NAME);
		} else {
			int address = SERVO_PORT >= 6 ? 2 : 173;
			testServo = ManualHardwareManager.getServo(hardwareMap, SERVO_PORT % 6, address);
		}

		// testServo.setDirection(Servo.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			testServo.setPosition(POSITION);

			telemetry.addData("Position", POSITION);
			telemetry.update();

		}
	}
}