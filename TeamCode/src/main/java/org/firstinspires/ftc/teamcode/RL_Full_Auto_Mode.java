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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Red Left", group = "Linear Opmode")
//@Disabled
public class RL_Full_Auto_Mode extends LinearOpMode {
    InvadersRelicRecoveryBot robot = new InvadersRelicRecoveryBot();
    


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor robot.leftDrive = null;
    //private DcMotor robot.rightDrive = null;
    //InvadersRelicRecoveryBot robot = new InvadersRelicRecoveryBot();

    private void stopAll() {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the InvadersRelicRecoveryBot hardware variable.
        robot.init(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.update();

        // Read the vuMark before the robot moves
        RelicRecoveryVuMark visibleTargets = RelicRecoveryVuMark.CENTER; //robot.getVuforiaTargets(false);

        //grabber nonsense
        robot.rightGrab.setPosition(0.9);
        robot.leftGrab.setPosition(0.9);
        robot.sleepMs(500);

        // jewel nonsense
        robot.setJewelArmPosition(0.22, InvadersRelicRecoveryBot.JewelPush.Right);
        robot.sleepMs(3000);

        boolean iSawRed = robot.isRightJewelRed();
        boolean iSawBlue = robot.isRightJewelBlue();
        int jewelMove = 0;
        if (iSawRed) {
            jewelMove = 4;
            robot.encoderDrive(0.1, 4, 4, 2);
            robot.sleepMs(1000);
            robot.stopMotors();
            robot.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
            robot.sleepMs(2000);
            robot.stopMotors();
        } else if (iSawBlue) {

            jewelMove = -4;
            robot.encoderDrive(0.1, -4, -4, 2);
            robot.sleepMs(1000);
            robot.stopMotors();
            robot.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
            robot.sleepMs(2000);
            robot.stopMotors();
        } else {
            jewelMove = 0;
            robot.setJewelArmPosition(1, InvadersRelicRecoveryBot.JewelPush.Right);
            robot.sleepMs(2000);
            robot.stopMotors();
        }
        //Vu nonsense
        switch (visibleTargets) {
            case LEFT:
                robot.encoderDrive(0.1, 23 - jewelMove, 23 - jewelMove, 10);
                robot.encoderDrive(0.1, 6.5, -6.5,10);
                //robot.gyroTurn(0.1, 90);
                Log.i("VuMark Identification", "Left VuMark FOUND!");
                break;
            case CENTER:
                robot.encoderDrive(0.1, 19 - jewelMove, 19 - jewelMove, 10);
                robot.encoderDrive(0.1, 6.5, -6.5, 10);
                //robot.gyroTurn(0.1, 90);
                Log.i("VuMark Identification", "Right VuMark FOUND!");
                break;
            case RIGHT:
                robot.encoderDrive(0.1, 15 - jewelMove, 15 - jewelMove, 10);
                robot.encoderDrive(0.1, 6.5, -6.5, 10);
                //robot.gyroTurn(0.1, 90);
                Log.i("VuMark Identification", "Center VuMark FOUND!");
                break;
            case UNKNOWN:
                Log.w("VuMarkIdentification", "No VuMark Found");
            }
        }
    }