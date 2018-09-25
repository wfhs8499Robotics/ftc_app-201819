package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static java.lang.Thread.sleep;

/**
 * TurnWheels.
 *
 * couple of methods to move the robot around..  one for inches and one for MM.
 *
 */

public class TurnWheels {


    private int     newLeftTarget;
    private int     newRightTarget;
    private int     moveCounts;
    private double  max;
    private double  error;
    private double  steer;
    private double  leftSpeed;
    private double  rightSpeed;

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);
    private static final double     TURN_SPEED              = 0.125;
    private static final double     HOLD_SPEED              = TURN_SPEED / 2;
    // Robot configuration and turning distances
//    private static final double     wheelWidth              = 13.8125;

    private static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    private static final int    CYCLE_MS    =   50;     // period of each cycle
    private static final double MAX_FWD     =  0.3;     // Maximum FWD power applied to motor
    private static final double MAX_REV     = -0.3;     // Maximum REV power applied to motor

    private static final double     HEADING_THRESHOLD       = 0 ;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable -  original is 0.15

    private DcMotor leftmotor = null; // Hardware Device Object
    private DcMotor rightmotor = null; // Hardware Device Object
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
//    private ModernRoboticsI2cRangeSensor rangeSensor;
    // Define class members
    private double  power   = 0;
    private boolean rampUp  = true;
    private int saveHeading;

    private double maxSpeed = 0;
    private double zeroSpeed = INCREMENT;

    /* Constructor */
    public TurnWheels(){
    }

    public void init(HardwareMap hwMap){
        leftmotor = hwMap.dcMotor.get("left motor");
        rightmotor = hwMap.dcMotor.get("right motor");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.
        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete

        while (modernRoboticsI2cGyro.isCalibrating())  {
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        //*rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {


        // Ensure that the opmode is still activ

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = leftmotor.getCurrentPosition() + moveCounts;
        newRightTarget = rightmotor.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftmotor.setTargetPosition(newLeftTarget);
        rightmotor.setTargetPosition(newRightTarget);

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        maxSpeed = speed;
        speed = INCREMENT;
        rampUp = true;

        leftmotor.setPower(speed);
        rightmotor.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftmotor.isBusy() && rightmotor.isBusy())) {
            if (rampUp){
               speed += INCREMENT ;
             if (speed >= maxSpeed ) {
                   speed = maxSpeed;
               }
             }

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftmotor.setPower(leftSpeed);
            rightmotor.setPower(rightSpeed);
        }

        // Stop all motion;
        leftmotor.setPower(0);
        rightmotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyroHold(TURN_SPEED,angle,0.166);

    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF) ) {
            // Update telemetry & Allow time for other processes to run.
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        // Stop all motion;
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftmotor.setPower(leftSpeed);
        rightmotor.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - modernRoboticsI2cGyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*
 *  Helper methods to perform a relative turn, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 */
    public void right90 () {
        saveHeading = 0; //modernRoboticsI2cGyro.getHeading();
            gyroTurn(TURN_SPEED, saveHeading - 90);
            gyroHold(HOLD_SPEED, saveHeading - 90, 0.5);
    }

    public void left90 () {
        saveHeading = 0; //modernRoboticsI2cGyro.getHeading();
            gyroTurn(TURN_SPEED,saveHeading + 90);
            gyroHold(HOLD_SPEED, saveHeading + 90, 0.5);
    }

    public void heading0 () {
        saveHeading = 0; //modernRoboticsI2cGyro.getHeading();
            gyroTurn(TURN_SPEED, saveHeading);
            gyroHold(HOLD_SPEED, saveHeading, 0.5);
    }

    public void heading180 () {
        saveHeading = 0; //modernRoboticsI2cGyro.getHeading();
            gyroTurn(TURN_SPEED,saveHeading + 180);
            gyroHold(HOLD_SPEED, saveHeading + 180, 0.5);
    }

    public int getRobotHeading(){
        int heading = modernRoboticsI2cGyro.getHeading();
        return heading;
    }
}
