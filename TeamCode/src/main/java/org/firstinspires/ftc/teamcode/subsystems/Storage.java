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
import java.util.Arrays;

@Configurable
public class Storage {
	public static double distance_threshold_mm = 5;
	public static int positionInterval = 5; // arbitary number, will change with further testing
	public static double transferMotorPower = 1;
	public static int transferInterval = 5; // arbitary number, will change with further testing

	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	private static int numOfArtifacts = 0;

	private DcMotor storageMotor;
	private DcMotor transferMotor;
	private ColorRangeSensor colourSensor;

	public enum TurnDirection {
		AVAILABLE, // AVAILABLE is equal to NONE as in no turn is performed.
		NONE, // But, in AVAILABLE, the Artifact is already in the intake slot.
		CW,
		CCW
	}

	public Storage(HardwareMap hardwareMap, boolean resetEncoder) {
		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");

		storageMotor = hardwareMap.get(DcMotor.class, "storageMotor");
		storageMotor.setTargetPosition(0);
		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
		transferMotor.setTargetPosition(0);
		transferMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		if (resetEncoder) {
			storageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
	}

	public void abort() {
		transferMotor.setPower(0);
	}

	/*
	 * Getter methods
	 */

	public static Colour getActiveArtifact() {
		return artifactStored.get(0);
	}

	public static Colour getBackLeftArtifact() {
		return artifactStored.get(1);
	}

	public static Colour getBackRightArtifact() {
		return artifactStored.get(2);
	}

	/*
	 * Storage
	 */

	public static void resetArtifacts () {
		numOfArtifacts = 0;
		artifactStored.set(0, null);
		artifactStored.set(1, null);
		artifactStored.set(2, null);
	}

	public boolean intake() {
		turnToArtifact(null);
		Colour colour = getArtifactColour();
		if (isArtifactLoaded() && !storageFull() && colour != null) {
			artifactStored.set(0, colour);
			numOfArtifacts += 1;
			return true;
		}
		return false;
	}

	public void storageMotorEnable(boolean enabled){
		if (enabled){
			storageMotor.setPower(1);
		} else {
			storageMotor.setPower(0);
		}
	}

	public void storageTurnCW() {
		storageMotor.setPower(1);
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() + positionInterval);
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageTurnCCW() {
		storageMotor.setPower(1);
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() - positionInterval);
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public boolean storageFull() {
		return !artifactStored.contains(null);
	}

	public TurnDirection turnToArtifact(Colour desiredArtifact) {
		if (numOfArtifacts > 0) {
			if (getActiveArtifact() == desiredArtifact) {
				return TurnDirection.AVAILABLE;
			} else if (getBackLeftArtifact() == desiredArtifact) {
				storageTurnCCW();
				return TurnDirection.CCW;
			} else if (getBackRightArtifact() == desiredArtifact) {
				storageTurnCW();
				return TurnDirection.CW;
			} else {
				return TurnDirection.NONE;
			}
		} else {
			return TurnDirection.NONE;
		}
	}

	public boolean release() {
		if (getActiveArtifact() != null) {
			artifactStored.set(0, null);
			numOfArtifacts -= 1;
			transferMotor.setPower(transferMotorPower);
			transferMotor.setTargetPosition(transferMotor.getTargetPosition() + transferInterval);
			return true;
		} else {
			return false;
		}
	}

	public void finishRelease() {
		// Wait
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
