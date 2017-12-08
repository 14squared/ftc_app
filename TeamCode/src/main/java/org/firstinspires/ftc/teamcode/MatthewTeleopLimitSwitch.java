package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.InvadersRelicRecoveryBot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Matthew Encoder Testing", group="Iterative Opmode")
//@Disabled
public class MatthewTeleopLimitSwitch extends OpMode
{
    // Declare OpMode members.
    InvadersRelicRecoveryBot robot = new InvadersRelicRecoveryBot();
    private ElapsedTime runtime = new ElapsedTime();

    // fineMode is used to scale back the control speed of the motors.  This allows the driver
    // to make more precise changes to the robot mechanisms (better aiming, less jostling, etc).
    boolean fineMode = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(this);
        robot.liftMotor.setPower(-0.2);
        while (robot.liftMotorCutoff.getState() == true){
            robot.sleepMs(100);
        }
        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        robot.sleepMs(3000);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.clearAll();
        telemetry.addLine("TODO: Send Lift Home");
        /*@todo Add code to 'home' the lifter back to the down position (ideally,
         * we'd use the pushbutton sensor.  This will allow us to limit the total
         * height we try to lift and prevent us from snapping the lift string.
        */
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        boolean limitState = robot.liftMotorCutoff.getState();

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  -gamepad1.right_stick_x;

        // Keep track of the liftPosition (we want to make sure we don't go too high)
        double liftPosition = robot.liftMotor.getCurrentPosition();

        if(gamepad1.a == true){
            fineMode = false;
        }
        if (gamepad1.b == true){
            fineMode = true;
        }

        if(fineMode == true){
            leftPower = Range.clip(drive - turn, -0.3, 0.3);
            rightPower = Range.clip(drive + turn, -0.3, 0.3);
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);

        }
        else {
            leftPower = Range.clip(drive - turn, -1.0, 1.0);
            rightPower = Range.clip(drive + turn, -1.0, 1.0);
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
        }

        // Look at the left and right triggers to decide whether to raise/lower the lift
        if(gamepad1.left_trigger > 0.05)
        {
            robot.liftMotor.setPower(-gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0.05)
        {
            robot.liftMotor.setPower(gamepad1.right_trigger);
        }
        else
        {
            robot.liftMotor.setPower(0);
        }


        // Look at the left/right bumpers to decide whether to open/close the glyph-gripper
        double leftServoPos = robot.leftGrab.getPosition();
        double rightServoPos = robot.leftGrab.getPosition();
        if (gamepad1.right_bumper == true) {
            // Only increment the grip position if isn't already at its maximum
            //if(leftServoPos <= 0.99) robot.leftGrab.setPosition(leftServoPos+0.01);
            //if(rightServoPos <= 0.99) robot.rightGrab.setPosition(rightServoPos+0.01);
            robot.rightGrab.setPosition(0.3);
            robot.leftGrab.setPosition(0.3);
        }
        else if (gamepad1.left_bumper == true) {
            //if(leftServoPos > 0.01) robot.leftGrab.setPosition(leftServoPos-0.01);
            //if(rightServoPos > 0.01) robot.leftGrab.setPosition(rightServoPos-0.01);
            robot.rightGrab.setPosition(0.9);
            robot.leftGrab.setPosition(0.9);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Lifter", "position %.2f", liftPosition);
        telemetry.addData("Grippers", "left (%.2f), right (%.2f)", leftServoPos, rightServoPos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
