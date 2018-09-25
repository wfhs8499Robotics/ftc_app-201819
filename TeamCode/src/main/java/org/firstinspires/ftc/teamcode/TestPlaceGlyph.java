

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="Test PlaceGlyph", group="Autonomous")

public class TestPlaceGlyph extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private PlaceGlyph placeGlyph = new PlaceGlyph();

    boolean vuMarkSet = false;

    RelicRecoveryVuMark vuMark;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                vuMark = RelicRecoveryVuMark.CENTER;
                vuMarkSet = true;
            }
            if (gamepad1.dpad_left) {
                vuMark = RelicRecoveryVuMark.LEFT;
                vuMarkSet = true;
            }
            if (gamepad1.dpad_right) {
                vuMark = RelicRecoveryVuMark.RIGHT;
                vuMarkSet = true;
            }
            if (gamepad1.dpad_down) {
                vuMark = RelicRecoveryVuMark.UNKNOWN;
                vuMarkSet = true;
            }

            if(vuMarkSet){
                runtime.reset();
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                placeGlyph.init(hardwareMap);
                placeGlyph.run(vuMark);
                vuMarkSet = false;
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
            idle();
        }
    }
}
