package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Linear slide testing", group="Debug")
public class SlideTesting extends LinearOpMode {
	public static int max_position = 0;
	// 4500

	@Override
	public void runOpMode() {
		DcMotor motor = hardwareMap.get(DcMotor.class, "slideMotorRight");
		DcMotor motor2 = hardwareMap.get(DcMotor.class, "slideMotorLeft");
		motor.setPower(1);
		motor2.setPower(1);
		motor.setTargetPosition(0);
		motor2.setTargetPosition(0);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		waitForStart();

		while (opModeIsActive()) {
			int target = (int) (gamepad1.left_trigger * max_position);
			motor.setTargetPosition(target);
			motor2.setTargetPosition(-target);

			telemetry.addData("Position", target);
			telemetry.addData("Current Position", motor.getCurrentPosition());
			telemetry.addData("Current Position", motor2.getCurrentPosition());
			telemetry.update();

		}
	}
}