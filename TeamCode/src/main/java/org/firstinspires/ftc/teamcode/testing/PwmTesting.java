package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
// import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

@Config
@TeleOp(name="PWM Servo testing", group="Debug")
public class PwmTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static int MIN_PWM = 600;
	public static int MAX_PWM = 2400;
	public static String SERVONAME = "testServo"; 

	@Override
	public void runOpMode() {
		ServoImplEx testServo = hardwareMap.get(ServoImplEx.class, SERVONAME);
		testServo.setPwmRange(new ServoImplEx.PwmRange(MIN_PWM, MAX_PWM));

		waitForStart();

		while (opModeIsActive()) {
			testServo.setPosition(POSITION);
			
			telemetry.addData("Position", POSITION);
			telemetry.update();

		}
	}
}