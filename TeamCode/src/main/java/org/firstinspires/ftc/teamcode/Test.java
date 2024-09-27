package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo testing", group="Debug")
public class Test extends LinearOpMode {
	public static double POSITION = 0;

	@Override
	public void runOpMode() {
		Servo dropArmServo = hardwareMap.get(Servo.class, "dropArmServo");

		waitForStart();

		while (opModeIsActive()) {
			dropArmServo.setPosition(POSITION/300);

			telemetry.addData("Position", POSITION/300);
			telemetry.update();

		}
	}
}