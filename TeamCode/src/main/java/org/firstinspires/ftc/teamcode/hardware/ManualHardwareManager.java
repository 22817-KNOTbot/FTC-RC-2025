package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.lang.annotation.Annotation;

public class ManualHardwareManager {
	public static DcMotor getMotor(HardwareMap hardwareMap, int port, int moduleAddress) {
		// From HardwareFactory.java
		LynxDcMotorController controller = null;
		try {
			for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
				if (module.getModuleAddress() != moduleAddress)
					continue;

				controller = new LynxDcMotorController(hardwareMap.appContext, module);
			}
		}  catch (RobotCoreException | InterruptedException e) {
			throw new RuntimeException(e);
		}

		assert controller != null;

		// USB Scan Manager.java uses null manager?
		DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

		// Using GoBILDA 5203 Motor Configuration
		MotorConfigurationType motorConfigurationType = new MotorConfigurationType();
		motorConfigurationType.setTicksPerRev(505.3169);
		motorConfigurationType.setGearing(99.5);
		motorConfigurationType.setMaxRPM(60);
		motorConfigurationType.setOrientation(Rotation.CCW);

		DcMotor m = deviceMgr.createDcMotorEx(controller, port, motorConfigurationType, motorConfigurationType.getName());

		// Since it is not automatically enabled, we manually enable it
		MotorCommands.enableMotor(m);

		return m;
	}

	public static Servo getServo(HardwareMap hardwareMap, int port, int moduleAddress) {
		// From HardwareFactory.java
		LynxServoController controller = null;
		try {
			for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
				if (module.getModuleAddress() != moduleAddress)
					continue;

				controller = new LynxServoController(hardwareMap.appContext, module);
			}
		}  catch (RobotCoreException | InterruptedException e) {
			throw new RuntimeException(e);
		}

		assert controller != null;

		// USB Scan Manager.java uses null manager?
		DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

		// Using default servo configuration
		ServoConfigurationType servoConfigurationType = new ServoConfigurationType();

		ServoType servoAnnotation = new ServoType() {
			@Override
			public Class<? extends Annotation> annotationType() {
				return ServoType.class;
			}

			@NonNull
			@Override
			public ServoFlavor flavor() {
				return ServoFlavor.STANDARD;
			}

			@Override
			public double usPulseLower() {
				return PwmControl.PwmRange.usPulseLowerDefault;
			}

			@Override
			public double usPulseUpper() {
				return PwmControl.PwmRange.usPulseUpperDefault;
			}

			@Override
			public double usPulseFrameRate() {
				return PwmControl.PwmRange.usFrameDefault;
			}

			@Override
			public String xmlTag() {
				return "";
			}
		};

		servoConfigurationType.processAnnotation(servoAnnotation);

		Servo s = deviceMgr.createServoEx(controller, port, servoConfigurationType.getName(), servoConfigurationType);

		// Since it is not automatically enabled, we manually enable it
		ServoCommands.enableServo(s);

		return s;
	}

	public static CRServo getCRServo(HardwareMap hardwareMap, int port, boolean isParent) {
		// From HardwareFactory.java
		LynxServoController controller = null;
		try {
			for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
				if (module.isParent() != isParent)
					continue;

				controller = new LynxServoController(hardwareMap.appContext, module);
			}
		}  catch (RobotCoreException | InterruptedException e) {
			throw new RuntimeException(e);
		}

		assert controller != null;

		// USB Scan Manager.java uses null manager?
		DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

		// Using GoBILDA 5203 Motor Configuration
		ServoConfigurationType servoConfigurationType = new ServoConfigurationType();

		CRServo s = deviceMgr.createCRServoEx(controller, port, servoConfigurationType.getName(), servoConfigurationType);

		// Since it is not automatically enabled, we manually enable it
		ServoCommands.enableCRServo(s);

		return s;
	}

	public static class MotorCommands {
		public static void disableMotor(DcMotor motor) {
			((DcMotorControllerEx)motor.getController()).setMotorDisable(motor.getPortNumber());
		}

		public static void enableMotor(DcMotor motor) {
			((DcMotorControllerEx)motor.getController()).setMotorEnable(motor.getPortNumber());
		}
	}

	public static class ServoCommands {
		public static void disableServo(Servo servo) {
			((ServoControllerEx) servo.getController()).setServoPwmDisable(servo.getPortNumber());
		}

		public static void enableServo(Servo servo) {
			((ServoControllerEx) servo.getController()).setServoPwmEnable(servo.getPortNumber());
		}

		public static void disableCRServo(CRServo crServo) {
			((ServoControllerEx) crServo.getController()).setServoPwmDisable(crServo.getPortNumber());
		}

		public static void enableCRServo(CRServo crServo) {
			((ServoControllerEx) crServo.getController()).setServoPwmEnable(crServo.getPortNumber());
		}
	}
}