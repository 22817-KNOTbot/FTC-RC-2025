package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoAlign.AlignmentDirection;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.ControlTheory;

@Configurable
public class AutoAlignTesting extends LinearOpMode {
	public static Integer aprilTagId = null;

	private Turret turret;
	private ControlTheory.PID PIDController;

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
		turret = new Turret(hardwareMap);
		PIDController = new ControlTheory.PID(Turret.Kp, Turret.Ki, Turret.Kd, true);

		boolean previousDebug = Vision.DEBUG;
		Vision.DEBUG = true;

		Vision visionProcessor = new Vision(hardwareMap, aprilTagId);

		waitForStart();

		while (opModeIsActive()) {
			PIDController.setKp(Turret.Kp);
			PIDController.setKi(Turret.Ki);
			PIDController.setKd(Turret.Kd);
			AlignmentDirection direction = visionProcessor.getAlignmentDirection();
			if (direction.directionKnown) {
				turret.rotateTurret(PIDController.calculate(direction.x, 0));
			} else {
				PIDController.resetIntegral();
				PIDController.resetLastError();
			}
			telemetry.addLine(direction.toString());
			visionProcessor.showTelemetry(telemetry);
			telemetry.update();
		}

		visionProcessor.close();
		Vision.DEBUG = previousDebug;
	}
}
