package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Configurable
public class Transfer {
	public static double power = 1;
	public static double loaded_empty_ms = 500;

	private DcMotor transferMotor;
	private DigitalChannel transferSensor;

	private ElapsedTime transferLoadedTimer;

	public Transfer(HardwareMap hardwareMap) {
		transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
		transferMotor.setDirection(DcMotor.Direction.REVERSE);
		transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		transferSensor = hardwareMap.get(DigitalChannel.class, "transferSensor");
		transferSensor.setMode(DigitalChannel.Mode.INPUT);
	}

	public void enable(boolean enable) {
		if (enable) {
			transferMotor.setPower(power);
		} else {
			transferMotor.setPower(0);
		}
	}

	public void enableReversed(boolean enable) {
		if (enable) {
			transferMotor.setPower(-power);
		} else {
			transferMotor.setPower(0);
		}
	}

	public void setPower(float pow) {
		transferMotor.setPower(pow);
	}

	public boolean getLoaded() {
		return transferSensor.getState();
	}

	public void transferUpdate() {
		if (getLoaded()) {
			if (transferLoadedTimer == null) {
				transferLoadedTimer = new ElapsedTime();
			}
		} else {
			transferLoadedTimer = null;
		}
	}

	public boolean isFinishedTransferring() {
		return transferLoadedTimer != null && transferLoadedTimer.milliseconds() >= loaded_empty_ms;
	}
}
