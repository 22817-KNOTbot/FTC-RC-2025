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
import org.firstinspires.ftc.teamcode.scoring.Artifact.Pattern;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

import java.util.ArrayList;
import java.util.Arrays;

import com.acmerobotics.dashboard.config.Config;

@Configurable
@Config
public class Storage {
	public static double distance_threshold_mm = 90;
	public static int positionInterval = 128;
	public static double transferMotorPower = 0.4;
	public static double intakeGateUpPosition = 0.318;
	public static double intakeGateDownPosition = 0.355;
	public static double intakeGateTurnPosition = 0.33;
	public static double transferRampOutPosition = 0.533;
	public static double transferRampInPosition = 0.47;
	
	private static int numOfArtifacts = 0;
	private static ArrayList<Colour> artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
	private IntakeState intakeState = IntakeState.IDLE;
	private TransferState transferState = TransferState.IDLE;
	private boolean transferInit = false;

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
			artifactStored = new ArrayList<Colour>(Arrays.asList(null, null, null));
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
		gateDown();
		Colour colour = getArtifactColour();
		if (isArtifactLoaded() && colour != null) {
			if (storageFull()) {
				intakeGate.setPosition(intakeGateUpPosition);
				intakeState = IntakeState.RESET;
			} else {
				intakeGate.setPosition(intakeGateTurnPosition);
				numOfArtifacts += 1;
				timer.reset();
				intakeState = IntakeState.GATE_UP;
			}
			artifactStored.set(0, colour);
			return true;
		}
		return false;
	}

	public void intakeUpdate() {
		switch (intakeState) {
			case GATE_UP:
				if (timer.time() >= 0.5) {
					storageTurnCCW();
					intakeState = IntakeState.TURNING;
				}
				break;

			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 10) {
					gateDown();
					timer.reset();
					intakeState = IntakeState.GATE_DOWN;
				}
				break;

			case GATE_DOWN:
				if (timer.time() >= 0.5) {
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
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() + positionInterval);
		Colour intakeArtifact = getActiveArtifact();
		artifactStored.set(0, getBackRightArtifact());
		artifactStored.set(2, getBackLeftArtifact());
		artifactStored.set(1, intakeArtifact);
	}

	public void storageTurnCCW() {
		storageMotor.setTargetPosition(storageMotor.getTargetPosition() - positionInterval);
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

	public boolean transferInit() {
		if (getActiveArtifact() != null) {
			gateUp();
			artifactStored.set(0, null);
			numOfArtifacts -= 1;
			transferRamp.setPosition(transferRampOutPosition);
			timer.reset();
			transferState = TransferState.RAMP_OUT;
			transferInit = true;
			return true;
		} else {
			return false;
		}
	}

	public boolean transferStart() {
		if (getActiveArtifact() != null) {
			artifactStored.set(0, null);
			numOfArtifacts -= 1;
			storageTurnCW();
			timer.reset();
			transferState = TransferState.TURNING;
			return true;
		} else {
			return false;
		}
	}

	public void transferUpdate() {
		switch (transferState) {
			case RAMP_OUT:
				if (timer.time() >= 0.5) {
					storageTurnCW();
					// timer.reset();
					transferState = TransferState.TURNING;
				}
				break;
		
			case TURNING:
				if (Math.abs(storageMotor.getCurrentPosition() - storageMotor.getTargetPosition()) < 10) {
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
		gateDown();
		transferState = TransferState.IDLE;
		transferInit = false;
	}

	public boolean isMotorBusy() {
		return storageMotor.isBusy();
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
		telemetry.addData("Storage", artifactStored);
		telemetry.addData("Artifact Loaded", isArtifactLoaded());
		telemetry.addData("Artifact Colour", getArtifactColour());
		// telemetry.addData("Spindexer Power", storageMotor.getPower());
		telemetry.addData("Spindexer Position", storageMotor.getCurrentPosition());
		telemetry.addData("Spindexer Target", storageMotor.getTargetPosition());
	}
}
