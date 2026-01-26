package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

@Configurable
@Config
// @TeleOp(name="Storage testing", group="Debug")
public class StorageTesting extends LinearOpMode {
	public static Command command = Command.NONE;
	public static Colour desiredArtifact = Colour.PURPLE;
	public static boolean manualDirectionMode = false;
	public static Storage.TurnDirection manualTurnDirection = Storage.TurnDirection.NONE;
	public static boolean resetEncoders = true;

	private Storage storage;

	public enum Command {
		NONE,
		INTAKE,
		TURN_TO,
		RELEASE,
		FINISH_RELEASE,
	}

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		storage = new Storage(hardwareMap, resetEncoders);

		waitForStart();

		while (opModeIsActive()) {
			Object output = null;
			if (!manualDirectionMode) {
				switch (command) {
					case NONE:
						break;
					case INTAKE:
						output = storage.intake();
						if (output.equals(true)) command = Command.NONE;
						break;
					case TURN_TO:
						output = storage.turnToArtifact(desiredArtifact, true);
					case RELEASE:
						output = storage.transferStart();
						command = Command.NONE;
						break;
					case FINISH_RELEASE:
						storage.transferFinish();
						command = Command.NONE;
				}
			} else {
				if (manualTurnDirection == Storage.TurnDirection.CW) {
					storage.storageTurnCW();
				} else if (manualTurnDirection == Storage.TurnDirection.CCW) {
					storage.storageTurnCCW();
				}
				manualTurnDirection = Storage.TurnDirection.NONE;
			}
			telemetry.addData("Command output", output == null ? "null" : output);

			telemetry.addLine("=== Stored Artifacts ===");
			telemetry.addData("Storage Full", storage.storageFull());
			telemetry.addData("Active Artifact", storage.getActiveArtifact());
			telemetry.addData("Back Left Artifact", storage.getBackLeftArtifact());
			telemetry.addData("Back Right Artifact", storage.getBackRightArtifact());

			telemetry.addLine("=== Colour/Range ===");
			telemetry.addData("Loaded (active)", storage.isArtifactLoaded());
			telemetry.addData("Colour", storage.getArtifactColour());
			telemetry.addData("Red", storage.getRed());
			telemetry.addData("Green", storage.getGreen());
			telemetry.addData("Blue", storage.getBlue());

		}
	}
}