package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;

@Configurable
public class Storage {

	private DcMotor storageMotor = hardwareMap.get(DcMotor.class, "storageMotor");

	public static double distance_threshold_mm = 5;
	public static int motorPos = 1;
	public static double positionGap = 5; //arbitary mumber, will change with further testing
	public static Colour artifactStored[] = {null, null, null};

	private static int numOfArtifacts = 0;

	private ColorRangeSensor colourSensor;

	private boolean previouslyLoaded;

	public Storage(HardwareMap hardwareMap) {

		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");
	}

	@Configurable
	public void abort() {
		// Currently does nothing
		// Exists for future use
	}

	/*
	 * Getter methods
	 */
	
	public static Colour getFirstArtifact() {
		return artifactStored[0];
	}
	public static Colour getSecondArtifact() {
		return artifactStored[1];
	}
	public static Colour getThirdArtifact() {
		return artifactStored[2];
	}

	
	/*
	 * Storage
	 */

	public boolean intake() {
		if (isArtifactLoaded()) {
			artifactStored.set(numOfArtifacts, getArtifactColour());
			numOfArtifacts += 1;
			storageMotor.setTargetPosition(getCurrentPosition() + positionGap);
			return true;
		}
		return false;
	}

	// Returns the storage if successfully stored
	// Returns null if storage is full
	public boolean storageFull() {
		if (artifactStored.contains(null)) {
			return true;
		} else {
			return false;
		}
	}


	public boolean release() {
		if (getThirdArtifact() != null) {
			artifactStored.set(numOfArtifacts, null);
			numOfArtifacts -= 1;
		} else if (getSecondArtifact() != null) {
			artifactStored.set(numOfArtifacts, null);
			numOfArtifacts -= 1;
		} else if (getFirstArtifact() != null) {
			artifactStored.set(numOfArtifacts, null);
			numOfArtifacts -= 1;
		} else {
			return null;
		}
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
