package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;

import org.firstinspires.ftc.teamcode.subsystems.Storage;

@Configurable
// @TeleOp(name="Storage testing", group="Debug")
public class StorageTesting extends LinearOpMode {
	public static Command command = Command.NONE;
	public static Colour desiredArtifact = Colour.PURPLE;
	public static boolean resetEncoder = true;

	private Storage storage;

	public enum Command {
		NONE,
		INTAKE,
		STORE,
		RELEASE,
		SORT
	}

	public enum GateMode {
		MANUAL,
		AUTO
	}

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		storage = new Storage(hardwareMap, true);

		waitForStart();

		while (opModeIsActive()) {
			Object output = null;
			switch (command) {
				case NONE:
					break;
				case INTAKE:
					output = storage.intake();
					break;
				case STORE:
					output = storage.intake();
					command = Command.NONE;
					break;
				case RELEASE:
					storage.release();
					command = Command.NONE;
					break;
				case SORT:
					output = storage.turnToArtifact(desiredArtifact);
					command = Command.NONE;
			}
			telemetry.addData("Command output", output == null ? "null" : output);

			telemetry.addLine("=== Stored Artifacts ===");
			telemetry.addData("Front Left Artifact", storage.getIntakeArtifact());
			telemetry.addData("Front Right Artifact", storage.getBackLeftArtifact());
			telemetry.addData("Back Artifact", storage.getBackRightArtifact());

			telemetry.addLine("=== Colour/Range ===");
			telemetry.addData("Loaded (back)", storage.isArtifactLoaded());
			telemetry.addData("Colour", storage.getArtifactColour());
			telemetry.addData("Red", storage.getRed());
			telemetry.addData("Green", storage.getGreen());
			telemetry.addData("Blue", storage.getBlue());

		}
	}
}