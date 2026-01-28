package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@Config
// @TeleOp(name="Turret testing", group="Debug")
public class TurretTesting extends LinearOpMode {
	public static Mode mode = Mode.ABSOLUTE;
	public static double rotation = Turret.BASE_ROTATION;
	public static double rotateVector = 0;

	private Turret turret;

	public enum Mode {
		ABSOLUTE,
		RELATIVE
	}

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		turret = new Turret(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			if (mode == Mode.ABSOLUTE) {
				turret.setRotation(rotation);
			} else if (mode == Mode.RELATIVE) {
				turret.rotateTurret(rotateVector);
				rotateVector = 0;
			}

			telemetry.addData("Rotation", Turret.getTargetRotation());
			telemetry.addData("Rotation", turret.getRotation());
			telemetry.update();
		}
	}
}