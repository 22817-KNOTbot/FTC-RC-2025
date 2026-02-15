package org.firstinspires.ftc.teamcode.util.FateWeaver;

import java.util.List;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import gay.zharel.fateweaver.schemas.CustomStructSchema;
import gay.zharel.fateweaver.schemas.DoubleSchema;

public class CustomSchemas {
	public static CustomStructSchema<Pose> getPoseSchema() {
		return new CustomStructSchema<Pose>(
			"Pose",
			List.of("x", "y", "heading"),
			List.of(DoubleSchema.INSTANCE, DoubleSchema.INSTANCE, DoubleSchema.INSTANCE),
			pose -> List.of(pose.getX(), pose.getY(), pose.getHeading())
		);
	}

	public static CustomStructSchema<Vector> getVectorSchema() {
		return new CustomStructSchema<Vector>(
			"Vector",
			List.of("x", "y", "magnitude", "heading"),
			List.of(DoubleSchema.INSTANCE, DoubleSchema.INSTANCE, DoubleSchema.INSTANCE, DoubleSchema.INSTANCE),
			vector -> List.of(vector.getXComponent(), vector.getYComponent(), vector.getMagnitude(), vector.getTheta())
		);
	}
}
