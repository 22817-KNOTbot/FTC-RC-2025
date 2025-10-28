package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
// @TeleOp(name="Motor 2 testing", group="Debug")
public class MotorTwoTesting extends LinearOpMode {
	public static double POWER_1 = 0; // +0.7
	public static double POWER_2 = 0; // -0.7
	public static String MOTOR_NAME_1 = "testMotor";
	public static String MOTOR_NAME_2 = "testMotor2";

	@Override
	public void runOpMode() {
		DcMotor motor1 = hardwareMap.get(DcMotor.class, MOTOR_NAME_1);
		DcMotor motor2 = hardwareMap.get(DcMotor.class, MOTOR_NAME_2);
		motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		waitForStart();

		while (opModeIsActive()) {
			motor1.setPower(POWER_1);
			motor2.setPower(POWER_2);

			telemetry.addData("Power 1", POWER_1);
			telemetry.addData("Power 2", POWER_2);
			telemetry.update();

		}
	}
}