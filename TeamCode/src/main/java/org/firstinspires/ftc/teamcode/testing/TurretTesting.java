package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
// @TeleOp(name="Turret testing", group="Debug")
public class TurretTesting extends LinearOpMode {
	public static Mode mode = Mode.ABSOLUTE;
	public static Turret.Position position = Turret.Position.BASE;
	public static double rotation = Turret.BASE_ROTATION;
	public static double pitch = Turret.BASE_PITCH;
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
			if (position == Turret.Position.BASE) {
				turret.setPosition(position);
			} else if (position == Turret.Position.MANUAL) {
				if (mode == Mode.ABSOLUTE) {
					turret.setRotation(rotation);
					turret.setPitch(pitch);
				} else if (mode == Mode.RELATIVE) {
					turret.rotateTurret(rotateVector);
					rotateVector = 0;
				}
			}

			telemetry.addData("Position", Turret.getPosition());
			telemetry.addData("Rotation", Turret.getRotation());
			telemetry.addData("Pitch", Turret.getPitch());
			telemetry.update();

		}
	}
}