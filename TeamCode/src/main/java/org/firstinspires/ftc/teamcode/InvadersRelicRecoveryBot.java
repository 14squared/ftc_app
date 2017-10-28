package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the IfSpace Invaders 2016/2017 Velocity Vortex season robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Ref: .\IfSpaceInvaders_ModernRobotics_Hardware_Defintions.txt for details
 */
public class InvadersRelicRecoveryBot {
    public static final double MID_SERVO = 0.5;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    Telemetry telemetry = null;
    OpMode activeOpMode = null;

    public ElapsedTime period = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    //FUNCTIONS
    private boolean opModeIsActive() {
        boolean isActive = true;
        // If we're running a linear op mode, then make sure we stop when our opmode is no longer active
        if (activeOpMode instanceof LinearOpMode) {
            isActive = ((LinearOpMode) activeOpMode).opModeIsActive();
        }
        return isActive;
    }


    public void setDriveTrainPower(double power) {
        setDriveTrainPower(power, power);
    }

    public void setDriveTrainPower(double leftPower, double rightPower) {
        if (leftDrive != null) leftDrive.setPower(leftPower);
        if (rightDrive != null) rightDrive.setPower(rightPower);
    }


    public void stop() {
        setDriveTrainPower(0);
    }


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /* Constructor */
    public InvadersRelicRecoveryBot() {

    }

    public void sleepMs(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
        }
    }


    /* Initialize standard Hardware interfaces */
    public void init(OpMode activeOpMode) {
        // Save reference to Hardware map
        hwMap = activeOpMode.hardwareMap;

        // Save reference to the OpMode's Telemetry
        telemetry = activeOpMode.telemetry;

        // Save reference to the active OpMode
        this.activeOpMode = activeOpMode;

        // Define and Initialize Motors
        leftDrive = hwMap.dcMotor.get("backLeft");
        rightDrive = hwMap.dcMotor.get("backRight");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        setDriveTrainPower(0);

        // Set all non-driving motors to run without encoders.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //Cold pizza normally.
    public enum CapBallState {
        UP,
        DOWN,
        OFF
    }


    /*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            setDriveTrainPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (period.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setDriveTrainPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleepMs(250);   // optional pause after each move
        }
    }


    public void timedDrive(double speed, double durationMs) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            setDriveTrainPower(speed);
            period.reset();
            while ((period.time() < durationMs) && opModeIsActive()) {
            }
            // Stop all motion;
            setDriveTrainPower(0);

            sleepMs(250);
        }
    }
}
