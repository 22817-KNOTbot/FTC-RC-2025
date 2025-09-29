package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;

import java.util.ArrayList;

@Configurable
public class Storage {

	private DcMotor storageMotor;
	private Servo transfer;
	private DcMotor transferMotor;

	public static double distance_threshold_mm = 5;
	public static double transferPos = 0.5;	//arbitary mumber, will change with further testing
	public static int motorPos = 1;
	public static int positionGap = 5; //arbitary mumber, will change with further testing
	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>();
	

	private static int numOfArtifacts = 0;

	private ColorRangeSensor colourSensor;

	private boolean previouslyLoaded;

	public Storage(HardwareMap hardwareMap) {
		artifactStored.add(null);
		artifactStored.add(null);
		artifactStored.add(null);

		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");
		storageMotor = hardwareMap.get(DcMotor.class, "storageMotor");
		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		transfer = hardwareMap.get(Servo.class, "transferServo");
		transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

	}

	public void abort() {
		// Currently does nothing
		// Exists for future use
	}

	/*
	 * Getter methods
	 */
	
	public static Colour getFirstArtifact() {
		return artifactStored.get(0);
	}
	public static Colour getSecondArtifact() {
		return artifactStored.get(1);
	}
	public static Colour getThirdArtifact() {
		return artifactStored.get(2);
	}

	
	/*
	 * Storage
	 */

	public boolean intake() {
		if (isArtifactLoaded()) {
			artifactStored.set(numOfArtifacts, getArtifactColour());
			numOfArtifacts += 1;
			storageTurnCW();
			return true;
		}
		return false;
	}

	//Storage Turn Clockwise
	public void storageTurnCW() {
		storageMotor.setTargetPosition(storageMotor.getCurrentPosition() + positionGap);
	}

	//Storage Turn Counter-clockwise
		public void storageTurnCCW() {
		storageMotor.setTargetPosition(storageMotor.getCurrentPosition() - positionGap);
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
			transfer.setPosition(transferPos);
			transferMotor.setPower(1);
			finishRelease();
			return true;
		} else if (getSecondArtifact() != null) {
			artifactStored.set(numOfArtifacts, null);
			numOfArtifacts -= 1;
			transfer.setPosition(transferPos);
			transferMotor.setPower(1);
			finishRelease();
			return true;
		} else if (getFirstArtifact() != null) {
			artifactStored.set(numOfArtifacts, null);
			numOfArtifacts -= 1;
			transfer.setPosition(transferPos);
			transferMotor.setPower(1);
			finishRelease();
			return true;
		} else {
			return false;
		}
	}

	public void sortArtifacts(Colour desiredArtifact) {
		if (numOfArtifacts >= 0) {
			if (artifactStored.get(numOfArtifacts - 1) == desiredArtifact) {
				release();
			}
		}
	}

	//USE THIS AFTER sortArtifacts() IMMEDEITLY TO HAVE A ARTIFACT IN THE INTAKE POSITION
	public void sortArtifactsTurn(Colour desiredArtifact) {
		if (numOfArtifacts >= 0) {
			if (artifactStored.get(numOfArtifacts - 1) == desiredArtifact) {
				storageTurnCCW();
			} else if (artifactStored.get(numOfArtifacts - 2) == desiredArtifact) {
				storageTurnCCW();
			} else {
				storageTurnCW();
			}
		}
	}

	public void finishRelease() {
		//wait
			transfer.setPosition(0);
			transferMotor.setPower(0);
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
