/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Hunter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.InvadersRelicRecoveryBot;

@TeleOp(name="HunterManuelOp", group="Iterative Opmode")
public class Hunter_BasicOpMode_Iterative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private InvadersRelicRecoveryBot robot = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo leftGrab = null;
    private Servo rightGrab = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robot = new InvadersRelicRecoveryBot();
        robot.init(this);

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftGrab.setDirection(Servo.Direction.FORWARD);
        rightGrab.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        leftPower = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;
        if (gamepad1.right_bumper == true) {
            double leftServoPos = leftGrab.getPosition();
            double rightServoPos = leftGrab.getPosition();
            leftGrab.setPosition(leftServoPos + 0.1);
            rightGrab.setPosition(rightServoPos + 0.1);
        }
        if (gamepad1.left_bumper == true) {
            double leftServoPos = leftGrab.getPosition();
            double rightServoPos = leftGrab.getPosition();
            leftGrab.setPosition(leftServoPos - 0.1);
            rightGrab.setPosition(rightServoPos - 0.1);
        }
        if (gamepad1.a == true)
        {
            robot.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
        }
        if (gamepad1.b == true)
        {
            robot.setJewelArmPosition(0, InvadersRelicRecoveryBot.JewelPush.Right);
        }


        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}