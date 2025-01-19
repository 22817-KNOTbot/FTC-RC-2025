package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
// @TeleOp(name="Linear slide testing", group="Debug")
public class SlideTesting extends LinearOpMode {
	public static String MOTOR1 = "slideMotorLeft";
	public static String MOTOR2 = "slideMotorRight";
	public static float POWER = 0;
	public static int TARGET = 0;
	// 4200

	@Override
	public void runOpMode() {
		DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR1);
		DcMotor motor2 = hardwareMap.get(DcMotor.class, MOTOR2);
		motor.setPower(POWER);
		motor2.setPower(POWER);
		motor.setTargetPosition(0);
		motor2.setTargetPosition(0);
		motor.setDirection(DcMotor.Direction.REVERSE);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		waitForStart();

		while (opModeIsActive()) {
			motor.setPower(POWER);
			motor2.setPower(POWER);
	
			motor.setTargetPosition(TARGET);
			motor2.setTargetPosition(TARGET);

			telemetry.addData("Position", TARGET);
			telemetry.addData("Current Position", motor.getCurrentPosition());
			telemetry.addData("Current Position", motor2.getCurrentPosition());
			telemetry.update();
		}
	}
}