package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="Manual slide control", group="Debug")
// Design for control of intake slides
// Set power = 0 to manually get encoder limits
// Set power = 1 to set position
public class ManualSlideControl extends LinearOpMode {
	public static String MOTOR1 = "intakeSlides";
	public static double POWER = 0;
	public static int TARGET = 0;
	public static ZERO_POWER ZEROPOWER = ZERO_POWER.FLOAT;

	private enum ZERO_POWER {
		FLOAT,
		BRAKE
	}

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR1);
		motor.setPower(POWER);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setTargetPosition(0);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setZeroPowerBehavior(ZEROPOWER == ZERO_POWER.FLOAT ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);

		waitForStart();

		while (opModeIsActive()) {
			// Max: 1800
			motor.setPower(POWER);
			motor.setTargetPosition(TARGET);
			telemetry.addData("Slide position", motor.getCurrentPosition());
			telemetry.update();
		}
	}
}