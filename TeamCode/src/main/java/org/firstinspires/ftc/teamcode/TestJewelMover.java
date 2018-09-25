
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Test JewelMover", group="Autonomous")

public class TestJewelMover extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private JewelMover jewelMover = new JewelMover();

    boolean sideSet = false;
    String sideColor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                sideColor = "BLUE";
                sideSet = true;
            }
            if (gamepad1.right_bumper) {
                sideColor = "RED";
                sideSet = true;
            }

            if(sideSet){
                runtime.reset();
                telemetry.addData("Side", "%s visible", sideColor);
                telemetry.update();
                jewelMover.init(hardwareMap, sideColor);
                jewelMover.run();
                sideSet = false;
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
            idle();
        }
    }
}
