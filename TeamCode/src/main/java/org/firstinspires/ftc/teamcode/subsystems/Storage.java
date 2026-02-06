package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.teleop.Automations.StorageState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Storage {
	public static double distance_threshold_mm = 30;
	public static int positionInterval = 175;
	public static double storageMotorPower = 0.3;
	public static double storageTolerance = 20;
	public static double transferMotorCWPower = 0.5;
	public static double transferMotorCCWPower = 1;
	
	private static int numOfArtifacts = 0;
	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	private static int currentTargetSlotPosition = 0;
	private IntakeState intakeState = IntakeState.IDLE;
	private TransferState transferState = TransferState.IDLE;
	private boolean transferInit = false;

	public static int colourCacheTimeMs = 10;
	private NormalizedRGBA cachedColoursActive;
	private NormalizedRGBA cachedColoursBackLeft;
	private NormalizedRGBA cachedColoursBackRight;
	private long lastActiveColourUpdateTime;
	private long lastBackLeftColourUpdateTime;
	private long lastBackRightColourUpdateTime;

	private DcMotorEx storageMotor;
	private ColorRangeSensor colourSensorActive;
	private ColorRangeSensor colourSensorBackRight;
	private ColorRangeSensor colourSensorBackLeft;
	private ElapsedTime timer;

	private NormalizedRGBA cachedColours;
	private long lastColourUpdateTime;

	public enum IntakeState {
		IDLE, 
		INTAKING,
		RESET
	}

	public enum TransferState {
		IDLE,
		WAITING_VELOCITY,
		TURNING, 
		RESET
	}
	
	public enum TurnDirection {
		AVAILABLE, // AVAILABLE is equal to NONE as in no turn is performed.
		NONE, // But, in AVAILABLE, the Artifact is already in the intake slot.
		CW,
		CCW
	}


	public Storage(HardwareMap hardwareMap, boolean resetEncoder) {
		colourSensorActive = hardwareMap.get(ColorRangeSensor.class, "colourSensorActive");
		colourSensorBackLeft = hardwareMap.get(ColorRangeSensor.class, "colourSensorBackLeft");
		colourSensorBackRight = hardwareMap.get(ColorRangeSensor.class, "colourSensorBackRight");
		storageMotor = hardwareMap.get(DcMotorEx.class, "storageMotor");
		storageMotor.setTargetPosition(currentTargetSlotPosition);

		if (resetEncoder) {
			storageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			numOfArtifacts = 0;
			artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
			currentTargetSlotPosition = 0;
		}

		storageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		storageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);		
		storageMotor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
				new PIDCoefficients(6, 0, 0));

		timer = new ElapsedTime();
	}

	public void start() {
	}

	public void abort() {
		transferState = TransferState.IDLE;
		transferInit = false;
	}

	/*
	 * Getter methods
	 */
	public static void setArtifactsStored(Colour[] colours) {
		artifactStored = new ArrayList<Colour>(Arrays.asList(colours[0], colours[1], colours[2]));
	}

	public static Colour getActiveArtifact() {
		return artifactStored.get(0);
	}

	public static Colour getBackLeftArtifact() {
		return artifactStored.get(1);
	}

	public static Colour getBackRightArtifact() {
		return artifactStored.get(2);
	}

	public static int getNumOfArtifacts() {
		numOfArtifacts = 0;
		Iterator<Colour> artifactIterator = artifactStored.iterator(); 
		while (artifactIterator.hasNext()) {
			if (artifactIterator.next() != null) {
				numOfArtifacts++;
			}
		}
		return numOfArtifacts;
	}

	public int getStoragePosition() {
		return storageMotor.getCurrentPosition();
	}

	public IntakeState getIntakeState() {
		return intakeState;
	}

	public TransferState getTransferState() {
		return transferState;
	}

	public boolean getTransferInit() {
		return transferInit;
	}

	

	/*
	 * Storage
	 */

	public boolean intakeUpdate() {
		updateStorageArtifacts();
		if (storageFull()) {
			timer.reset();
			return true;
		}
		return false;
	}

	public void storageMotorEnable(boolean enabled){
		if (enabled){
			storageMotor.setPower(0.4);
		} else {
			storageMotor.setPower(0);
		}
	}

	public boolean updateStorageArtifacts() {
		if (storageMotor.getCurrentPosition() % 128 < storageTolerance) {
			clearStorageMemory();
			artifactStored.set(0, getActiveArtifact());
			artifactStored.set(1, getBackLeftArtifact());
			artifactStored.set(2, getBackRightArtifact());
			numOfArtifacts = getNumOfArtifacts();
			return true;
		}
		return false;
	}

	public void storageTurnCW() {
		storageMotor.setPower(transferMotorCWPower);
		storageMotor.setTargetPosition(currentTargetSlotPosition + positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageDoubleTurnCW() {
		storageMotor.setPower(transferMotorCWPower);
		storageMotor.setTargetPosition(currentTargetSlotPosition + 2*positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public void storageTurnCCW() {
		storageMotor.setPower(transferMotorCCWPower);
		storageMotor.setTargetPosition(currentTargetSlotPosition - positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public void storageDoubleTurnCCW() {
		storageMotor.setPower(transferMotorCCWPower);
		storageMotor.setTargetPosition(currentTargetSlotPosition - 2*positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageFullTurnCCW() {
		storageMotor.setPower(transferMotorCCWPower);
		storageMotor.setTargetPosition(currentTargetSlotPosition - 3*positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
	}

	public boolean storageFull() {
		return !artifactStored.contains(null);
	}

	public boolean storageEmpty() {
		for (int i = 0; i < artifactStored.size(); i++) {
			if (artifactStored.get(i) != null) {
				return false;
			}
		}
		return true;
	}

	public void turnToDirection(TurnDirection direction) {
		switch (direction) {
			case CCW:
				storageTurnCCW();
				break;

			case CW:
				storageTurnCW();
				break;

			default:
				break;
		}
	}

	public TurnDirection turnToArtifact(Colour desiredArtifact) {
		return turnToArtifact(desiredArtifact, true);
	}

	public TurnDirection turnToArtifact(Colour desiredArtifact, boolean move) {
		if (numOfArtifacts > 0) {
			if (getActiveArtifact() == desiredArtifact) {
				return TurnDirection.AVAILABLE;
			} else if (getBackRightArtifact() == desiredArtifact) {
				if (move) {
					storageTurnCW();
				}
				return TurnDirection.CCW;
			} else if (getBackLeftArtifact() == desiredArtifact) {
				if (move) {
					storageDoubleTurnCW();
				}
				return TurnDirection.CW;
			} else {
				return TurnDirection.NONE;
			}
		} else {
			return TurnDirection.NONE;
		}
	}

	public TurnDirection turnToAnyArtifact() {
		if (numOfArtifacts > 0) {
			if (getActiveArtifact() != null) {
				return TurnDirection.AVAILABLE;
			} else if (getBackRightArtifact() != null) {
				storageTurnCW();
				return TurnDirection.CW;
			} else if (getBackLeftArtifact() != null) {
				storageDoubleTurnCW();
				return TurnDirection.CCW;
			} else {
				return TurnDirection.NONE;
			} 
		} else {
			return TurnDirection.NONE;
		}
	}

	// Will return NONE if invalid input or sequence not possible
	// Only works for sequences of 3
	public TurnDirection turnToArtifactSequence(Colour[] desiredSequence) {
		if (desiredSequence.length < 3 || !storageFull()) {
			return TurnDirection.NONE;
		}

		for (int i = 0; i < 3; i++) {
			int startingIndex = (3 - i) % 3;
			if (
				artifactStored.get(startingIndex) == desiredSequence[0]
				&& artifactStored.get((startingIndex + 1) % 3) == desiredSequence[1]
				&& artifactStored.get((startingIndex + 2) % 3) == desiredSequence[2]
			) {
				switch (i) {
					case 0:
						return TurnDirection.AVAILABLE;
					case 1:
						storageTurnCW();
						return TurnDirection.CW;
					case 2:
						storageDoubleTurnCW();
						return TurnDirection.CCW;
				}
			}
		}

		return TurnDirection.NONE;
	}

	public boolean transferStart(boolean force) {
		if (getActiveArtifact() != null || force) {
			numOfArtifacts -= 3;
			artifactStored.set(0, null);
			transferState = TransferState.WAITING_VELOCITY;
			return true;
		} else {
			return false;
		}
	}

	public boolean transferStart() {
		return transferStart(false);
	}

	public void transferUpdate(boolean start) {
		switch (transferState) {
			case WAITING_VELOCITY:
				if (start && !isMotorBusy()){
					storageFullTurnCCW();
					timer.reset();
					transferState = TransferState.TURNING;
				}
			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 1) {
					timer.reset();
					transferFinish();
					transferState = TransferState.RESET;
				}				
				break;
			
			default:
				break;
		}
	}

	public void transferFinish() {
		transferState = TransferState.IDLE;
		transferInit = false;
		turnToNeutralPosition();
	}

	public void turnToNeutralPosition() {
		float positionPercentage = (storageMotor.getCurrentPosition() % 3*positionInterval)/(3*positionInterval);
		if (positionPercentage == 0) {
			return;
		} else if (positionPercentage < 0.5) {
			storageDoubleTurnCW();
			return;
		} else {
			storageTurnCW();
			return;
		}
	}

	public boolean isMotorBusy() {
		return storageMotor.isBusy();
	}

	public void clearStorageMemory() {
		numOfArtifacts = 0;
		artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	}

	public List<Colour> getArtifactsStored() {
		return new ArrayList<>(artifactStored);
	}

	/*
	 * Colour/range sensor
	 */

	public boolean isArtifactLoaded() {
		return colourSensorActive.getDistance(DistanceUnit.MM) < distance_threshold_mm;
	}

	// Returns null if unknown
	public Colour getActiveArtifactColour() {
		double red = getRedActive();
		double green = getGreenActive();
		double blue = getBlueActive();
		Colour colour = null;
		if (colourSensorActiveResponding()) {
			if (red < green && green < blue && blue > red) {
				colour = Colour.PURPLE;
			} else if (red < green && green > blue && blue > red && green < 3500) {
				colour = Colour.GREEN;
			}
		}
		return colour;
	}

	public Colour getBackLeftArtifactColour() {
		double red = getRedBackLeft();
		double green = getGreenBackLeft();
		double blue = getBlueBackLeft();
		Colour colour = null;
		if (colourSensorBackLeftResponding()) {
			if (red < green && green < blue && blue > red) {
				colour = Colour.PURPLE;
			} else if (red < green && green > blue && blue > red && green < 3500) {
				colour = Colour.GREEN;
			}
		}
		return colour;
	}

	public Colour getBackRightArtifactColour() {
		double red = getRedBackRight();
		double green = getGreenBackRight();
		double blue = getBlueBackRight();
		Colour colour = null;
		if (colourSensorBackRightResponding()) {
			if (red < green && green < blue && blue > red) {
				colour = Colour.PURPLE;
			} else if (red * 2 < green && green > blue && blue > red && green < 3500) {
				colour = Colour.GREEN;
			}
		}
		return colour;
	}

	public NormalizedRGBA getColoursActive() {
		if (System.currentTimeMillis() - lastActiveColourUpdateTime > colourCacheTimeMs) {
			cachedColoursActive = colourSensorActive.getNormalizedColors();
			lastActiveColourUpdateTime = System.currentTimeMillis();
		}
		return cachedColoursActive;
	}

	public NormalizedRGBA getColoursBackLeft() {
		if (System.currentTimeMillis() - lastBackLeftColourUpdateTime > colourCacheTimeMs) {
			cachedColoursBackLeft = colourSensorBackLeft.getNormalizedColors();
			lastBackLeftColourUpdateTime = System.currentTimeMillis();
		}
		return cachedColoursBackLeft;
	}

	public NormalizedRGBA getColoursBackRight() {
		if (System.currentTimeMillis() - lastBackRightColourUpdateTime > colourCacheTimeMs) {
			cachedColoursBackRight = colourSensorBackRight.getNormalizedColors();
			lastBackRightColourUpdateTime = System.currentTimeMillis();
		}
		return cachedColoursBackRight;
	}

	public double getRedActive() {
		return getColoursActive().red;
	}

	public double getGreenActive() {
		return getColoursActive().green;
	}

	public double getBlueActive() {
		return getColoursActive().blue;
	}

	public double getRedBackLeft() {
		return getColoursBackLeft().red;
	}

	public double getGreenBackLeft() {
		return getColoursBackLeft().green;
	}

	public double getBlueBackLeft() {
		return getColoursBackLeft().blue;
	}

	public double getRedBackRight() {
		return getColoursBackRight().red;
	}

	public double getGreenBackRight() {
		return getColoursBackRight().green;
	}

	public double getBlueBackRight() {
		return getColoursBackRight().blue;
	}

	public boolean colourSensorActiveResponding() {
		// Try to find better way to detect disconnect
		return !(colourSensorActive == null || (getRedActive() == 0 && getGreenActive() == 0 && getBlueActive() == 0));
	}

	public boolean colourSensorBackRightResponding() {
		// Try to find better way to detect disconnect
		return !(colourSensorBackRight == null || (getRedBackRight() == 0 && getGreenBackRight() == 0 && getBlueBackRight() == 0));
	}

	public boolean colourSensorBackLeftResponding() {
		// Try to find better way to detect disconnect
		return !(colourSensorBackLeft == null || (getRedBackLeft() == 0 && getGreenBackLeft() == 0 && getBlueBackLeft() == 0));
	}

	public boolean colourSensorsResponding() {
		return (colourSensorActiveResponding() && colourSensorBackLeftResponding() && colourSensorBackRightResponding());
	}

	public void showTelemetry(TelemetryManager telemetry) {
		// telemetry.addData("Storage", artifactStored);
		telemetry.addData("Artifact Loaded", isArtifactLoaded());
		telemetry.addData("Active Artifact Colour", getActiveArtifactColour());
		telemetry.addData("BackLeft Artifact Colour", getBackLeftArtifactColour());
		telemetry.addData("BackRight Artifact Colour", getBackRightArtifactColour());
		// telemetry.addData("Spindexer Power", storageMotor.getPower());
		telemetry.addData("Spindexer Position", storageMotor.getCurrentPosition());
		telemetry.addData("Spindexer Target", storageMotor.getTargetPosition());
	}
}
