package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class MotorThreeTesting extends LinearOpMode {
	public static double POWER_1 = 0;
	public static double POWER_2 = 0;
	public static double POWER_3 = 0;
	public static String MOTOR_NAME_1 = "testMotor";
	public static String MOTOR_NAME_2 = "testMotor2";
	public static String MOTOR_NAME_3 = "testMotor3";

	@Override
	public void runOpMode() {
		DcMotor motor1 = hardwareMap.get(DcMotor.class, MOTOR_NAME_1);
		DcMotor motor2 = hardwareMap.get(DcMotor.class, MOTOR_NAME_2);
		DcMotor motor3 = hardwareMap.get(DcMotor.class, MOTOR_NAME_3);
		motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		waitForStart();

		while (opModeIsActive()) {
			motor1.setPower(POWER_1);
			motor2.setPower(POWER_2);
			motor3.setPower(POWER_3);

			telemetry.addData("Power 1", POWER_1);
			telemetry.addData("Power 2", POWER_2);
			telemetry.addData("Power 3", POWER_3);
			telemetry.update();

		}
	}
}