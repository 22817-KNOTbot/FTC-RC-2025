package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;

@Configurable
public class Storage {
	public static double distance_threshold_mm = 5;

	private static Colour frontLeftArtifact = null;
	private static Colour frontRightArtifact = null;
	private static Colour backArtifact = null;

	private Gate frontLeftGate;
	private Gate frontRightGate;
	private Gate backLeftGate;
	private Gate backRightGate;
	private ColorRangeSensor colourSensor;

	private boolean previouslyLoaded;

	public Storage(HardwareMap hardwareMap) {
		frontLeftGate = new Gate(hardwareMap.get(Servo.class, "frontLeftGateServo"));
		frontRightGate = new Gate(hardwareMap.get(Servo.class, "frontRightGateServo"));
		backLeftGate = new Gate(hardwareMap.get(Servo.class, "backLeftGateServo"));
		backRightGate = new Gate(hardwareMap.get(Servo.class, "backRightGateServo"));

		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");
	}

	public enum Chamber {
		LEFT,
		RIGHT
	}

	@Configurable
	public static class Gate {
		public static double gate_open = 0;
		public static double gate_closed = 0;

		private Servo servo;
		private boolean open = false;

		protected Gate(Servo servo) {
			this.servo = servo;
			open = servo.getPosition() == gate_open;
		}

		public void open() {
			servo.setPosition(gate_open);
			open = true;
		}

		public void close() {
			servo.setPosition(gate_closed);
			open = false;
		}

		public boolean isOpen() {
			return open;
		}
	}

	public void abort() {
		// Currently does nothing
		// Exists for future use
	}

	/*
	 * Getter methods
	 */

	public static Colour getFrontLeftArtifact() {
		return frontLeftArtifact;
	}

	public static Colour getFrontRightArtifact() {
		return frontRightArtifact;
	}

	public static Colour getBackArtifact() {
		return backArtifact;
	}

	/*
	 * Storage
	 */

	public boolean intake() {
		backLeftGate.close();
		backRightGate.close();
		if (isArtifactLoaded()) {
			backArtifact = getArtifactColour();
			return true;
		}
		return false;
	}

	// Returns the chamber if successfully stored
	// Returns null if both slots are full
	public Chamber storeArtifact() {
		if (frontLeftArtifact == null) {
			backLeftGate.open();
			frontLeftGate.close();
			frontLeftArtifact = backArtifact;
			backArtifact = null;
			return Chamber.LEFT;
		} else if (frontRightArtifact == null) {
			backRightGate.open();
			frontRightGate.close();
			frontRightArtifact = backArtifact;
			backArtifact = null;
			return Chamber.RIGHT;
		}
		return null;
	}

	public boolean releaseLeft() {
		if (frontLeftArtifact == null)
			return false;
		
		frontLeftGate.open();
		frontLeftArtifact = null;
		return true;
	}

	public boolean releaseRight() {
		if (frontRightArtifact == null)
			return false;
		
		frontRightGate.open();
		frontRightArtifact = null;
		return true;
	}

	/*
	 * Colour/range sensor
	 */

	public boolean isArtifactLoaded() {
		return colourSensor.getDistance(DistanceUnit.MM) < distance_threshold_mm;
	}

	// Returns null if unknown
	public Colour getArtifactColour() {
		int red = getRed();
		int green = getGreen();
		int blue = getBlue();
		Colour colour = null;
		if (colourSensorResponding()) {
			// TODO: Update checks after testing
			if (red > green && green < blue && blue > red) {
				colour = Colour.PURPLE;
			} else if (red < green && green > blue && blue > red) {
				colour = Colour.GREEN;
			}
		}
		return colour;
	}

	public int getRed() {
		return colourSensor.red();
	}

	public int getGreen() {
		return colourSensor.green();
	}

	public int getBlue() {
		return colourSensor.blue();
	}

	public boolean colourSensorResponding() {
		// Try to find better way to detect disconnect
		return !(colourSensor == null || (getRed() == 0 && getGreen() == 0 && getBlue() == 0));
	}
}
