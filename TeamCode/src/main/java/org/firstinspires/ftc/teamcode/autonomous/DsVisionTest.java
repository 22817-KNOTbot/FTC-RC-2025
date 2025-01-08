package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Other imports as applicable

@TeleOp
//@TeleOp(name="Name", group="Group") // Both paramters are optional, but must have "@TeleOp"
public class DsVisionTest extends LinearOpMode { // NameOfClass must be the name of your class. The file it is in must be the same as this ending with ".java". Also notice it "extends LinearOpMode"
    /*
     * Definitions
     */
    //public boolean variable = true;
    //private boolean another = false;
    private final int ZERO = 0;

    @Override // Because we extended LinearOpMode, we need to override its runOpMode() function and put our own code
    public void runOpMode() {
        /*
         * Init
         */
        // Usually hardware configuration here (see other OpModes for examples)

        waitForStart(); // Self explanatory
        /*
         * Loop
         */

        while (opModeIsActive()) { // Self explanatory. There does not have to be a loop here, but pretty much always seen in TeleOp, and less common in Autonomous.
            // Read sensors
            // Set hardware power
            // etc.

            telemetry.addData("Zero", ZERO); // Add telemetry. Think of it as a print() statement. The first argument is a label, the second is the value. Will be printed as "label: value"
            telemetry.update(); // Must update the telemetry otherwise it will not show up
        }
    }
}
