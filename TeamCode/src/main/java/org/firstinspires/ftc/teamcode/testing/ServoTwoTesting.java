package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
// @TeleOp(name="Servo two testing", group="Debug")
@Disabled
public class ServoTwoTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static double POSITION_TWO = 0;
	public static String SERVONAME = "testServo"; 
	public static String SERVONAME_TWO = "turretYawServo1"; 

	@Override
	public void runOpMode() {
		Servo testServo = hardwareMap.get(Servo.class, SERVONAME);
		Servo testServo2 = hardwareMap.get(Servo.class, SERVONAME_TWO);
		// testServo.setDirection(Servo.Direction.REVERSE);

		waitForStart();

		while (opModeIsActive()) {
			testServo.setPosition(POSITION);
			testServo2.setPosition(POSITION_TWO);
			// 250-210
			// open close

			telemetry.addData("Position", POSITION);
			telemetry.addData("Position 2", POSITION_TWO);
			telemetry.update();

		}
	}
}