package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
import java.util.List;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Storage {
	public static double distance_threshold_mm = 30;
	public static int positionInterval = 128;
	public static double transferMotorPower = 0.7;
	public static double intakeGateUpPosition = 0.318;
	public static double intakeGateDownPosition = 0.355;
	public static double intakeGateTurnPosition = 0.33;
	public static double transferRampOutPosition = 0.527;
	public static double transferRampInPosition = 0.465;
	
	private static int numOfArtifacts = 0;
	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	private static int currentTargetSlotPosition = 0;
	private IntakeState intakeState = IntakeState.IDLE;
	private TransferState transferState = TransferState.IDLE;
	private boolean transferInit = false;
	private TransferMode transferMode = TransferMode.NORMAL;

	private DcMotorEx storageMotor;
	private ColorRangeSensor colourSensor;
	private Servo intakeGate;
	private Servo transferRamp;
	private ElapsedTime timer;

	public enum IntakeState {
		IDLE, 
		GATE_UP, 
		TURNING,
		GATE_DOWN,
		RESET
	}

	public enum TransferState {
		IDLE,
		RAMP_OUT,
		TURNING_HALF, 
		TURNING, 
		RESET
	}
	
	public enum TurnDirection {
		AVAILABLE, // AVAILABLE is equal to NONE as in no turn is performed.
		NONE, // But, in AVAILABLE, the Artifact is already in the intake slot.
		CW,
		CCW
	}

	public enum TransferMode {
		NORMAL,
		FULL_SPIN
	}

	public Storage(HardwareMap hardwareMap, boolean resetEncoder) {
		colourSensor = hardwareMap.get(ColorRangeSensor.class, "colourSensor");

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

		intakeGate = hardwareMap.get(Servo.class, "gateServo");

		transferRamp = hardwareMap.get(Servo.class, "transferRampServo");

		timer = new ElapsedTime();
	}

	public void start() {
		transferRamp.setPosition(transferRampInPosition);
	}

	public void abort() {
		gateUp();
		transferRamp.setPosition(transferRampInPosition);
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

	public IntakeState getIntakeState() {
		return intakeState;
	}

	public TransferState getTransferState() {
		return transferState;
	}

	public boolean getTransferInit() {
		return transferInit;
	}

	public TransferMode getTransferMode() {
		return transferMode;
	}

	public void setTransferMode(TransferMode transferMode) {
		this.transferMode = transferMode;
	}

	/*
	 * Storage
	 */

	public boolean intake() {
		turnToArtifact(null, true);
		gateDown();
		Colour colour = getArtifactColour();
		if (isArtifactLoaded() && colour != null) {
			artifactStored.set(0, colour);
			numOfArtifacts += 1;
			if (storageFull()) {
				gateUp();
				intakeState = IntakeState.RESET;
			} else {
				timer.reset();
				intakeState = IntakeState.GATE_UP;
			}
			return true;
		}
		return false;
	}

	public void intakeUpdate() {
		switch (intakeState) {
			case GATE_UP:
				if (timer.time() >= 0.4) {
					storageTurnCCW();
					intakeState = IntakeState.TURNING;
					// if (timer.time() >= 0.6) {
					// 	storageTurnCCW();
					// 	intakeState = IntakeState.TURNING;
					// } else {
					// 	intakeGate.setPosition(intakeGateTurnPosition);
					// }
				}
				break;

			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 8) {
					gateDown();
					timer.reset();
					intakeState = IntakeState.RESET;
				}
				break;

			case GATE_DOWN:
				if (timer.time() >= 0.3) {
					timer.reset();
					intakeState = IntakeState.RESET;
				}
				break;

			default:
				break;
		}
	}

	public void gateUp() {
		intakeGate.setPosition(intakeGateUpPosition);
	}

	public void gateDown() {
		intakeGate.setPosition(intakeGateDownPosition);
	}

	public void storageMotorEnable(boolean enabled){
		if (enabled){
			storageMotor.setPower(0.4);
		} else {
			storageMotor.setPower(0);
		}
	}

	public void storageTurnCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition + positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageTurnCCW() {
		storageMotor.setTargetPosition(currentTargetSlotPosition - positionInterval);
		currentTargetSlotPosition = storageMotor.getTargetPosition();
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackLeftArtifact());
		artifactStored.set(1, getBackRightArtifact());
		artifactStored.set(2, intakeArtifact);
	}

	public void storageHalfTurnCW(boolean update) {
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() + (int) (positionInterval/2));
		if (update) {
			Colour intakeArtifact = getActiveArtifact();
			artifactStored.set(0, getBackLeftArtifact());
			artifactStored.set(1, getBackRightArtifact());
			artifactStored.set(2, intakeArtifact);
		}
	}

	public void storageHalfTurnCCW(boolean update) {
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() - (int) (positionInterval/2));
		if (update) {
			Colour intakeArtifact = getActiveArtifact();
			artifactStored.set(0, getBackLeftArtifact());
			artifactStored.set(1, getBackRightArtifact());
			artifactStored.set(2, intakeArtifact);
		}
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
					storageTurnCCW();
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
			} else if (getBackRightArtifact() != null) {
				storageTurnCCW();
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
			if (
				artifactStored.get(i) == desiredSequence[0]
				&& artifactStored.get((i + 2) % 3) == desiredSequence[1]
				&& artifactStored.get((i + 1) % 3) == desiredSequence[2]
			) {
				switch (i) {
					case 0:
						return TurnDirection.AVAILABLE;
					case 1:
						storageTurnCCW();
						return TurnDirection.CCW;
					case 2:
						storageTurnCW();
						return TurnDirection.CW;
				}
			}
		}

		return TurnDirection.NONE;
	}

	public boolean transferInit(boolean force) {
		if (getActiveArtifact() != null || force) {
			gateUp();
			if (getActiveArtifact() != null) {
				numOfArtifacts -= 1;
			}
			artifactStored.set(0, null);
			if (transferMode == TransferMode.FULL_SPIN) {
				artifactStored.set(1, null);
				artifactStored.set(2, null);
			}
			storageMotor.setTargetPosition(currentTargetSlotPosition - ((int) positionInterval / 4));
			transferRamp.setPosition(transferRampOutPosition);
			timer.reset();
			transferState = TransferState.RAMP_OUT;
			transferInit = true;
			return true;
		} else {
			return false;
		}
	}

	public boolean transferInit() {
		return transferInit(false);
	}

	public boolean transferStart(boolean force) {
		if (getActiveArtifact() != null || force) {
			if (getActiveArtifact() != null) {
				numOfArtifacts -= 1;
			}
			artifactStored.set(0, null);
			storageMotor.setTargetPosition(currentTargetSlotPosition - ((int) positionInterval / 4));
			timer.reset();
			transferState = TransferState.TURNING_HALF;
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
			case RAMP_OUT:
				if (timer.time() >= 0.5) {
					switch (transferMode) {
						case NORMAL:
							storageMotor.setTargetPosition(currentTargetSlotPosition + (int) (positionInterval * 0.75));
							timer.reset();
							transferState = TransferState.TURNING_HALF;
							break;
						case FULL_SPIN:
							storageTurnCW();
							storageTurnCW();
							storageTurnCW();
							timer.reset();
							transferState = TransferState.TURNING;
							break;
					}
				}
				break;
		
			case TURNING_HALF:
				if (timer.time() >= 0.2) {
					storageTurnCW();
					transferState = TransferState.TURNING;
				}
				break;
			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 3) {
					timer.reset();
					transferState = TransferState.RESET;
				}				
				break;
			
			default:
				break;
		}
	}

	public void transferFinish() {
		transferRamp.setPosition(transferRampInPosition);
		if (storageEmpty()) {
			gateDown();
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
		return colourSensor.getDistance(DistanceUnit.MM) < distance_threshold_mm && colourSensorResponding();
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
			} else if (red * 2 < green && green > blue && blue > red && green < 3500) {
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
