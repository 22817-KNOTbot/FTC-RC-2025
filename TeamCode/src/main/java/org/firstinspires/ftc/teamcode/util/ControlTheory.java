package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ControlTheory {
	public static class Pid {
		private PidCoefficients coefficients;

		private ElapsedTime timer = new ElapsedTime();
		private double integralSum = 0;
		private Double lastError = null;

		private double reference;
		private double current;
		private double error;
		private double difference;
		private double derivative;
		private double output;
		
		public Pid(PidCoefficients coefficients) {
			this.coefficients = coefficients;
			timer.reset();
		}

		public Pid(double Kp, double Ki, double Kd) {
			this.coefficients = new PidCoefficients(Kp, Ki, Kd);
			timer.reset();
		}

		public static class PidCoefficients {
			public double Kp;
			public double Ki;
			public double Kd;

			public PidCoefficients(double Kp, double Ki, double Kd) {
				this.Kp = Kp;
				this.Ki = Ki;
				this.Kd = Kd;
			}
		}

		public double calculate(double reference, double current) {
			error = reference - current;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			output = (coefficients.Kp * error) +
					(coefficients.Ki * integralSum) +
					(coefficients.Kd * derivative);

			lastError = error;
			timer.reset();

			return output;
		}

		public void showTelemetry(TelemetryManager telemetry) {
				telemetry.addData("Error", error);
				telemetry.addData("ref", reference);
				telemetry.addData("cur", current);
				telemetry.addData("lastError", lastError);
				telemetry.addData("calculated difference", difference);
		}

		public void reset() {
			resetIntegral();
			resetLastError();
		}

		public void resetIntegral() {
			integralSum = 0;
		}

		public void resetLastError() {
			lastError = null;
		}

		public void setCoefficients(PidCoefficients coefficients) {
			this.coefficients = coefficients;
		}

		public void setKp(double Kp) {
			this.coefficients.Kp = Kp;
		}

		public void setKi(double Ki) {
			this.coefficients.Ki = Ki;
		}

		public void setKd(double Kd) {
			this.coefficients.Kd = Kd;
		}
	}

	public static class Pidf {
		private PidfCoefficients coefficients;

		private ElapsedTime timer = new ElapsedTime();
		private double integralSum = 0;
		private Double lastError = null;

		private double reference;
		private double current;
		private double error;
		private double difference;
		private double derivative;
		private double output;

		public Pidf(PidfCoefficients coefficients) {
			this.coefficients = coefficients;
			timer.reset();
		}

		public Pidf(double Kp, double Ki, double Kd, double Kv) {
			this.coefficients = new PidfCoefficients(Kp, Ki, Kd, Kv);
			timer.reset();
		}

		public static class PidfCoefficients {
			public double Kp;
			public double Ki;
			public double Kd;
			public double Kv;

			public PidfCoefficients(double Kp, double Ki, double Kd, double Kv) {
				this.Kp = Kp;
				this.Ki = Ki;
				this.Kd = Kd;
				this.Kv = Kv;
			}
		}

		public double calculate(double reference, double current) {
			error = reference - current;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			output = (coefficients.Kp * error) +
					(coefficients.Ki * integralSum) +
					(coefficients.Kd * derivative) +
					(coefficients.Kv * reference);

			lastError = error;
			timer.reset();

			return output;
		}

		public void showTelemetry(TelemetryManager telemetry) {
				telemetry.addData("Error", error);
				telemetry.addData("ref", reference);
				telemetry.addData("cur", current);
				telemetry.addData("lastError", lastError);
				telemetry.addData("calculated difference", difference);
		}

		public void resetIntegral() {
			integralSum = 0;
		}

		public void resetLastError() {
			lastError = null;
		}

		public void setCoefficients(PidfCoefficients coefficients) {
			this.coefficients = coefficients;
		}

		public void setKp(double Kp) {
			this.coefficients.Kp = Kp;
		}

		public void setKi(double Ki) {
			this.coefficients.Ki = Ki;
		}

		public void setKd(double Kd) {
			this.coefficients.Kd = Kd;
		}

		public void setKv(double Kv) {
			this.coefficients.Kv = Kv;
		}
	}

	public static class Pidfs {
		private PidfsCoefficients coefficients;

		private ElapsedTime timer = new ElapsedTime();
		private double integralSum = 0;
		private Double lastError = null;

		private double reference;
		private double current;
		private double error;
		private double difference;
		private double derivative;
		private double output;

		public Pidfs(PidfsCoefficients coefficients) {
			this.coefficients = coefficients;
			timer.reset();
		}

		public Pidfs(double Kp, double Ki, double Kd, double Kv, double Ks) {
			this.coefficients = new PidfsCoefficients(Kp, Ki, Kd, Kv, Ks);
			timer.reset();
		}

		public static class PidfsCoefficients {
			public double Kp;
			public double Ki;
			public double Kd;
			public double Kv;
			public double Ks;

			public PidfsCoefficients(double Kp, double Ki, double Kd, double Kv, double Ks) {
				this.Kp = Kp;
				this.Ki = Ki;
				this.Kd = Kd;
				this.Kv = Kv;
				this.Ks = Ks;
			}
		}

		public double calculate(double reference, double current) {
			error = reference - current;
			if (lastError != null) {
				difference = error - lastError;
			} else {
				difference = 0;
			}
			derivative = difference / timer.seconds();
			integralSum = integralSum + (error * timer.seconds());

			output = (coefficients.Kp * error) +
					(coefficients.Ki * integralSum) +
					(coefficients.Kd * derivative) +
					(coefficients.Kv * reference)
					+ (coefficients.Ks);
			lastError = error;
			timer.reset();

			return output;
		}

		public void showTelemetry(TelemetryManager telemetry) {
				telemetry.addData("Error", error);
				telemetry.addData("ref", reference);
				telemetry.addData("cur", current);
				telemetry.addData("lastError", lastError);
				telemetry.addData("calculated difference", difference);
		}

		public void resetIntegral() {
			integralSum = 0;
		}

		public void resetLastError() {
			lastError = null;
		}

		public void setCoefficients(PidfsCoefficients coefficients) {
			this.coefficients = coefficients;
		}

		public void setKp(double Kp) {
			this.coefficients.Kp = Kp;
		}

		public void setKi(double Ki) {
			this.coefficients.Ki = Ki;
		}

		public void setKd(double Kd) {
			this.coefficients.Kd = Kd;
		}

		public void setKv(double Kv) {
			this.coefficients.Kv = Kv;
		}

		public void setKs(double Ks) {
			this.coefficients.Ks = Ks;
		}
	}
}
