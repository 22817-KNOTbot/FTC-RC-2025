package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.subsystems.Storage;

@Configurable
// @TeleOp(name="Storage testing", group="Debug")
public class StorageTesting extends LinearOpMode {
	public static GateMode gateMode = GateMode.AUTO;
	public static Command command = Command.NONE;
	public static boolean frontLeftGateOpen = false;
	public static boolean frontRightGateOpen = false;
	public static boolean backLeftGateOpen = false;
	public static boolean backRightGateOpen = false;

	private Storage storage;

	public enum Command {
		NONE,
		INTAKE,
		STORE,
		RELEASE_LEFT,
		RELEASE_RIGHT
	}

	public enum GateMode {
		MANUAL,
		AUTO
	}

	@Override
	public void runOpMode() {
		telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

		storage = new Storage(hardwareMap);

		waitForStart();

		while (opModeIsActive()) {
			if (gateMode == GateMode.AUTO) {
				Object output = null;
				switch (command) {
					case NONE:
						break;
					case INTAKE:
						output = storage.intake();
						break;
					case STORE:
						output = storage.storeArtifact();
						command = Command.NONE;
						break;
					case RELEASE_LEFT:
						storage.releaseLeft();
						command = Command.NONE;
						break;
					case RELEASE_RIGHT:
						storage.releaseRight();
						command = Command.NONE;
						break;
				}
				telemetry.addData("Command output", output == null ? "null" : output);
			} else if (gateMode == GateMode.MANUAL) {
				if (frontLeftGateOpen) {
					storage.getFrontLeftGate().open();
				} else {
					storage.getFrontLeftGate().close();
				}
				if (frontRightGateOpen) {
					storage.getFrontRightGate().open();
				} else {
					storage.getFrontRightGate().close();
				}
				if (backLeftGateOpen) {
					storage.getBackLeftGate().open();
				} else {
					storage.getBackLeftGate().close();
				}
				if (backRightGateOpen) {
					storage.getBackRightGate().open();
				} else {
					storage.getBackRightGate().close();
				}
			}

			telemetry.addLine("=== Stored Artifacts ===");
			telemetry.addData("Front Left Artifact", storage.getFrontLeftArtifact());
			telemetry.addData("Front Right Artifact", storage.getFrontRightArtifact());
			telemetry.addData("Back Artifact", storage.getBackArtifact());

			telemetry.addLine("=== Colour/Range ===");
			telemetry.addData("Loaded (back)", storage.isArtifactLoaded());
			telemetry.addData("Colour", storage.getArtifactColour());
			telemetry.addData("Red", storage.getRed());
			telemetry.addData("Green", storage.getGreen());
			telemetry.addData("Blue", storage.getBlue());
			
			telemetry.addLine("=== Gates ===");
			telemetry.addData("frontLeftGate", storage.getFrontLeftGate().isOpen());
			telemetry.addData("frontRightGate", storage.getFrontRightGate().isOpen());
			telemetry.addData("backLeftGate", storage.getBackLeftGate().isOpen());
			telemetry.addData("backRightGate", storage.getBackRightGate().isOpen());
			telemetry.update();

		}
	}
}