
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Test Ramp Motor Speed", group = "Autonomous")

public class TestRampMotor extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  0.3;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -0.3;     // Maximum REV power applied to motor

    // Define class members
    DcMotor leftmotor;
    DcMotor rightmotor;

    double  power   = 0;
    boolean rampUp  = true;


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        leftmotor = hardwareMap.get(DcMotor.class, "left motor");
        rightmotor = hardwareMap.get(DcMotor.class, "right motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            // Ramp the motors, according to the rampUp variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT ;
                if (power <= MAX_REV ) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            leftmotor.setPower(power);
            rightmotor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        leftmotor.setPower(0);
        rightmotor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
