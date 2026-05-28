package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pidfs;
import org.firstinspires.ftc.teamcode.util.ControlTheory.Pidfs.PidfsCoefficients;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

public class PidMotor implements DcMotorEx {
	private DcMotorEx motor;
	public PidfsCoefficients pidfsCoefficients;

	private boolean positionControllerActive;
	private Pidfs positionController;
	private double referencePosition = 0;
	private double maxPower = 0;

	public PidMotor(DcMotorEx motor, PidfsCoefficients pidfsCoefficients) {
		this.motor = motor;
		this.pidfsCoefficients = pidfsCoefficients;

		if (pidfsCoefficients == null) {
			pidfsCoefficients = new PidfsCoefficients(0, 0, 0, 0, 0);
		}
		positionController = new Pidfs(pidfsCoefficients);
	}

	public Double update() {
		if (getMode() == RunMode.RUN_TO_POSITION) {
			double currentPosition = getCurrentPosition();
			double power = positionController.calculate(referencePosition, currentPosition);
			setPower(Range.clip(power, -maxPower, maxPower));
			return power;
		}
		return null;
	}

	public void setPidfsCoefficients(PidfsCoefficients pidfsCoefficients) {
		this.pidfsCoefficients = pidfsCoefficients;
		positionController.setCoefficients(pidfsCoefficients);
	}

	public PidfsCoefficients getPidfsCoefficients() {
		return pidfsCoefficients;
	}

	/*
	 * Motor methods overrides
	 */

	@Override
	public void setPower(double power) {
		maxPower = Math.min(Math.abs(power), 1);
		motor.setPower(power);
	}

	// Fake being in RTP mode but actually use RWE to use custom PIDF controller
	@Override
	public void setMode(RunMode mode) {
		if (mode == RunMode.RUN_TO_POSITION) {
			positionControllerActive = true;
			mode = RunMode.RUN_WITHOUT_ENCODER;
		} else {
			positionControllerActive = false;
		}
		motor.setMode(mode);
	}

	@Override
	public RunMode getMode() {
		return positionControllerActive ? RunMode.RUN_TO_POSITION : motor.getMode();
	}

	@Override
	public void setTargetPosition(int position) {
		referencePosition = position;
		motor.setTargetPosition(position);
		positionController.resetIntegral();
		positionController.resetLastError();
	}

	/**
	 * @deprecated Use {@link #getPidfsCoefficients()}
	 */
	@Override
	@Deprecated
	public PIDCoefficients getPIDCoefficients(RunMode mode) {
		if (mode == RunMode.RUN_TO_POSITION) {
			return new PIDCoefficients(pidfsCoefficients.Kp, pidfsCoefficients.Ki, pidfsCoefficients.Kd);
		} else {
			return motor.getPIDCoefficients(mode);
		}
	}

	/**
	 * @deprecated Use {@link #getPidfsCoefficients()}
	 */
	@Override
	@Deprecated
	public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
		if (mode == RunMode.RUN_TO_POSITION) {
			return new PIDFCoefficients(pidfsCoefficients.Kp, pidfsCoefficients.Ki, pidfsCoefficients.Kd,
					pidfsCoefficients.Kv);
		} else {
			return motor.getPIDFCoefficients(mode);
		}
	}

	/**
	 * @deprecated Use {@link #setPidfsCoefficients(PidfsCoefficients)}
	 */
	@Override
	@Deprecated
	public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
		if (mode == RunMode.RUN_TO_POSITION) {
			pidfsCoefficients = new PidfsCoefficients(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, 0, 0);
		}
		motor.setPIDCoefficients(mode, pidCoefficients);
	}

	/**
	 * @deprecated Use {@link #setPidfsCoefficients(PidfsCoefficients)}
	 */
	@Override
	@Deprecated
	public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients)
			throws UnsupportedOperationException {
		if (mode == RunMode.RUN_TO_POSITION) {
			pidfsCoefficients = new PidfsCoefficients(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d,
					pidfCoefficients.f, 0);
		}
		motor.setPIDFCoefficients(mode, pidfCoefficients);
	}

	/**
	 * @deprecated Use {@link #setPidfsCoefficients(PidfsCoefficients)}
	 */
	@Override
	@Deprecated
	public void setPositionPIDFCoefficients(double p) {
		pidfsCoefficients = new PidfsCoefficients(p, 0, 0, 0, 0);
		motor.setPositionPIDFCoefficients(p);
	}

	/*
	 * Motor methods passthrough
	 */

	@Override
	public DcMotorController getController() {
		return motor.getController();
	}

	@Override
	public int getCurrentPosition() {
		return motor.getCurrentPosition();
	}

	@Override
	public MotorConfigurationType getMotorType() {
		return motor.getMotorType();
	}

	@Override
	public int getPortNumber() {
		return motor.getPortNumber();
	}

	@Override
	public boolean getPowerFloat() {
		return motor.getPowerFloat();
	}

	@Override
	public int getTargetPosition() {
		return motor.getTargetPosition();
	}

	@Override
	public ZeroPowerBehavior getZeroPowerBehavior() {
		return motor.getZeroPowerBehavior();
	}

	@Override
	public boolean isBusy() {
		return motor.isBusy();
	}

	@Override
	public void setMotorType(MotorConfigurationType arg0) {
		motor.setMotorType(arg0);
	}

	@Override
	@Deprecated
	public void setPowerFloat() {
		motor.setPowerFloat();
	}

	@Override
	public void setZeroPowerBehavior(ZeroPowerBehavior arg0) {
		motor.setZeroPowerBehavior(arg0);
	}

	@Override
	public Direction getDirection() {
		return motor.getDirection();
	}

	@Override
	public double getPower() {
		return motor.getPower();
	}

	@Override
	public void setDirection(Direction arg0) {
		motor.setDirection(arg0);
	}

	@Override
	public void close() {
		motor.close();
	}

	@Override
	public String getConnectionInfo() {
		return motor.getConnectionInfo();
	}

	@Override
	public String getDeviceName() {
		return motor.getDeviceName();
	}

	@Override
	public Manufacturer getManufacturer() {
		return motor.getManufacturer();
	}

	@Override
	public int getVersion() {
		return motor.getVersion();
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
		motor.resetDeviceConfigurationForOpMode();
	}

	@Override
	public double getCurrent(CurrentUnit arg0) {
		return motor.getCurrent(arg0);
	}

	@Override
	public double getCurrentAlert(CurrentUnit arg0) {
		return motor.getCurrentAlert(arg0);
	}

	@Override
	public int getTargetPositionTolerance() {
		return motor.getTargetPositionTolerance();
	}

	@Override
	public double getVelocity() {
		return motor.getVelocity();
	}

	@Override
	public double getVelocity(AngleUnit arg0) {
		return motor.getVelocity(arg0);
	}

	@Override
	public boolean isMotorEnabled() {
		return motor.isMotorEnabled();
	}

	@Override
	public boolean isOverCurrent() {
		return motor.isOverCurrent();
	}

	@Override
	public void setCurrentAlert(double arg0, CurrentUnit arg1) {
		motor.setCurrentAlert(arg0, arg1);
	}

	@Override
	public void setMotorDisable() {
		motor.setMotorDisable();
	}

	@Override
	public void setMotorEnable() {
		motor.setMotorEnable();
	}

	@Override
	public void setTargetPositionTolerance(int arg0) {
		motor.setTargetPositionTolerance(arg0);
	}

	@Override
	public void setVelocity(double arg0) {
		motor.setVelocity(arg0);
	}

	@Override
	public void setVelocity(double arg0, AngleUnit arg1) {
		motor.setVelocity(arg0, arg1);
	}

	@Override
	public void setVelocityPIDFCoefficients(double arg0, double arg1, double arg2, double arg3) {
		motor.setVelocityPIDFCoefficients(arg0, arg1, arg2, arg3);
	}
}
