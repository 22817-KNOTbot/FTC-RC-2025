package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Motor testing", group="Debug")
public class MotorTesting extends LinearOpMode {
	public static double POWER = 0;
	public static String MOTOR_NAME = "testMotor"; 

	@Override
	public void runOpMode() {
		DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);

		waitForStart();

		while (opModeIsActive()) {
			motor.setPower(POWER);

			telemetry.addData("Power", POWER);
			telemetry.update();

		}
	}
}