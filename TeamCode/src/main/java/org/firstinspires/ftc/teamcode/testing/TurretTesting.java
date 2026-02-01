package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

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
		TelemetryManager telemetryManager = new TelemetryManager();
		telemetryManager.setFtcFastTelemetry(this);
		telemetryManager.setDashboardInstance(FtcDashboard.getInstance());
		telemetryManager.setPanelsTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());

		turret = new Turret(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			turret.update();
			if (mode == Mode.ABSOLUTE) {
				turret.setRotation(rotation);
			} else if (mode == Mode.RELATIVE) {
				turret.rotateTurret(rotateVector);
				rotateVector = 0;
			}

			telemetryManager.addData("Rotation", Turret.getTargetRotation());
			telemetryManager.addData("Analog Rotation", turret.getRotation());
			telemetryManager.update();
		}
	}
}