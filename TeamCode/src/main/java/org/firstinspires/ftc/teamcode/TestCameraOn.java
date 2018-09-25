
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This is a program to execute cameraOn tasks to for testing.
 */

@Autonomous (name="Test cameraOn", group="Autonomous")

public class TestCameraOn extends LinearOpMode {

    private CameraOn cameraOn = new CameraOn();

    RelicRecoveryVuMark vuMark;
    @Override
    public void runOpMode() {

        cameraOn.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        vuMark = cameraOn.run();
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
    }
}
