// File to transfer data between OpModes
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Automations;
 
@Config
public class OpModeStorage {
    // Pose for transfer between Auto and TeleOp
	public static double x = Double.NEGATIVE_INFINITY;
	public static double y = Double.NEGATIVE_INFINITY;
	public static double heading = Double.NEGATIVE_INFINITY;
	// Mode of robot
	public static Automations.Modes mode;
}
