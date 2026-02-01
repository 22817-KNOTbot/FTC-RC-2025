package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
// import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

@Disabled
@Config
// @TeleOp(name="PWM Servo testing", group="Debug")
public class PwmTesting extends LinearOpMode {
	public static double POSITION = 0;
	public static int MIN_PWM = 500;
	public static int MAX_PWM = 2500;
	public static String SERVONAME = "testServo"; 
	public static String ANALOG = "analog"; 

	@Override
	public void runOpMode() {
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcTelemetry(telemetry);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		ServoImplEx testServo = hardwareMap.get(ServoImplEx.class, SERVONAME);
		testServo.setPwmRange(new ServoImplEx.PwmRange(MIN_PWM, MAX_PWM));
		AnalogInput analog = hardwareMap.get(AnalogInput.class, ANALOG);

		waitForStart();

		while (opModeIsActive()) {
			testServo.setPosition(POSITION);
			
			telemetryManager.addData("Position", POSITION);
			telemetryManager.addData("Analog", analog.getVoltage());
			telemetryManager.update();

		}
	}
}