
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 *
 * This is a LinearOpMode that executes turnWheels routines for turning of the
 * robot in autonomous mode for testing purposes.
 *
 */
@Autonomous(name = "Test Autonomous Turns", group = "Autonomous OpMode")
public class TestAutonomousTurns extends LinearOpMode {
    private TurnWheels turnWheels = new TurnWheels();
    static final double     DRIVE_SPEED             = 0.5;
    static final double     HALF_SPEED              = 0.25;
    static final double     TURN_SPEED              = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {

        turnWheels.init(hardwareMap);
        telemetry.addData("Right Bumper", "Right 90");
        telemetry.addData("Left Bumper", "Left 90");
        telemetry.addData("Right Trigger", "Right 33");
        telemetry.addData("Left Trigger", "Left 33");
        telemetry.addData("X", "Right 180");
        telemetry.addData("A", "Left 180");
        telemetry.addData("Y", "Forward straight 12 inches");
        telemetry.addData("B", "Reverse straight 12 inches");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            if(gamepad1.right_bumper){
                turnWheels.right90();
            }
            if(gamepad1.left_bumper){
                turnWheels.left90();
            }
//            if(gamepad1.right_trigger > 0){
//                turnWheels.right33();
//            }
//            if(gamepad1.left_trigger > 0){
//                turnWheels.left33();
//            }
//            if(gamepad1.x){
//                turnWheels.right180();
//            }
//            if(gamepad1.a){
//                turnWheels.left180();
//            }
            if(gamepad1.y){
                turnWheels.gyroDrive(.3,12, turnWheels.getRobotHeading());
                        //.encoderDrive(.3,12,12,10);
            }
             if(gamepad1.b){
                turnWheels.gyroDrive(.3, -12, turnWheels.getRobotHeading());
                        //.encoderDrive(.3,-12,-12,10);
            }
            if(gamepad1.dpad_up){
                telemetry.addData("gyro heading", "%s", turnWheels.getRobotHeading());
                telemetry.update();
            }
            if(gamepad1.dpad_down) {
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                // Put a hold after each turn
                turnWheels.gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
                turnWheels.gyroTurn(TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
                turnWheels.gyroHold(TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
                turnWheels.gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
                turnWheels.gyroTurn(TURN_SPEED, 45.0);         // Turn  CW  to  45 Degrees
                turnWheels.gyroHold(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
                turnWheels.gyroTurn(TURN_SPEED, 0.0);         // Turn  CW  to   0 Degrees
                turnWheels.gyroHold(TURN_SPEED, 0.0, 1.0);    // Hold  0 Deg heading for a 1 second
                turnWheels.gyroDrive(DRIVE_SPEED, -48.0, 0.0);    // Drive REV 48 inches
            }
            // send the info back to driver station using telemetry function.
            telemetry.addData("Operation", "Complete");
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
