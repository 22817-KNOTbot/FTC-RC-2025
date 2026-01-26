package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.scoring.Artifact.Colour;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Storage {
	public static double distance_threshold_mm = 90;
	public static int positionInterval = 128;
	public static double storageMotorPower = 0.3;
	public static double storageTolerance = 20;
	public static double transferMotorPower = 0.4;
	
	private static int numOfArtifacts = 0;
	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	private static Iterator<Colour> artifactStore = artifactStored.iterator();
	private static int currentTargetSlotPosition = 0;
	private IntakeState intakeState = IntakeState.IDLE;
	private TransferState transferState = TransferState.IDLE;
	private boolean transferInit = false;

	private DcMotorEx storageMotor;
	private ColorRangeSensor colourSensor;
	private ElapsedTime timer;

	public enum IntakeState {
		IDLE, 
		INTAKING,
		RESET
	}

	public enum TransferState {
		IDLE,
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
		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");

		storageMotor = hardwareMap.get(DcMotorEx.class, "storageMotor");
		storageMotor.setTargetPosition(0);

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
		while (artifactStore.hasNext()) {
			if (artifactStore.next() != null) {
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

	public boolean intake() {
		turnToArtifact(null, true);
		Colour colour = getArtifactColour();
		if (isArtifactLoaded() && colour != null) {
			artifactStored.set(0, colour);
			numOfArtifacts += 1;
			timer.reset();
			intakeState = IntakeState.INTAKING;
			return true;
		}
		return false;
	}

	public void intakeUpdate() {
		switch (intakeState) {

			case INTAKING:
				if (getNumOfArtifacts() == 3) {
					timer.reset();
					intakeState = IntakeState.RESET;
				}
				break;

			default:
				break;
		}
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
			artifactStored.set(0, getArtifactColour());
			//get the colour for the other slots
			numOfArtifacts = getNumOfArtifacts();
		}
		return false;
	}

	public void storageTurnCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition + positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageDoubleTurnCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition + 2*positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public void storageTurnCCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition - positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public void storageDoubleTurnCCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition - 2*positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
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
			} else if (getBackLeftArtifact() == desiredArtifact) {
				if (move) {
					storageDoubleTurnCW();
				}
				return TurnDirection.CCW;
			} else if (getBackRightArtifact() == desiredArtifact) {
				if (move) {
					storageTurnCW();
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

	public boolean transferStart(boolean force) {
		if (getActiveArtifact() != null || force) {
			if (getActiveArtifact() != null) {
				numOfArtifacts -= 1;
			}
			artifactStored.set(0, null);
			storageTurnCCW();
			timer.reset();
			transferState = TransferState.TURNING;
			return true;
		} else {
			return false;
		}
	}

	public boolean transferStart() {
		return transferStart(false);
	}

	public void transferUpdate() {
		switch (transferState) {
			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 2) {
					timer.reset();
					transferState = TransferState.RESET;
				}				
				break;
			
			default:
				break;
		}
	}

	public void transferFinish() {
		// transferRamp.setPosition(transferRampInPosition);
		if (storageEmpty()) {
			// gateDown();
		}
		transferState = TransferState.IDLE;
		transferInit = false;
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
		return colourSensor.getDistance(DistanceUnit.MM) < distance_threshold_mm;
	}

	// Returns null if unknown
	public Colour getArtifactColour() {
		int red = getRed();
		int green = getGreen();
		int blue = getBlue();
		Colour colour = null;
		if (colourSensorResponding()) {
			if (red < green && green < blue && blue > red) {
				colour = Colour.PURPLE;
			} else if (red < green && green > blue && blue > red && green < 3500) {
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

	public void showTelemetry(TelemetryManager telemetry) {
		// telemetry.addData("Storage", artifactStored);
		telemetry.addData("Artifact Loaded", isArtifactLoaded());
		telemetry.addData("Artifact Colour", getArtifactColour());
		// telemetry.addData("Spindexer Power", storageMotor.getPower());
		telemetry.addData("Spindexer Position", storageMotor.getCurrentPosition());
		telemetry.addData("Spindexer Target", storageMotor.getTargetPosition());
	}
}
