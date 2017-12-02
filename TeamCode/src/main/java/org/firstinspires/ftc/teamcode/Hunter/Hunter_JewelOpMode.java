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
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="HunterJewelOp", group="Iterative Opmode")
public class Hunter_JewelOpMode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor jewelSensorRight = null;
    private ColorSensor jewelSensorLeft = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initializing");

        jewelSensorRight = hardwareMap.get(ColorSensor.class, "jewelSensorRight");
        jewelSensorLeft = hardwareMap.get(ColorSensor.class, "jewelSensorLeft");
        telemetry.addData("Status", "Initialized");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

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
        int rightRed = jewelSensorRight.red();
        int rightGreen = jewelSensorRight.green();
        int rightBlue = jewelSensorRight.blue();
        int leftRed = jewelSensorLeft.red();
        int leftGreen = jewelSensorLeft.green();
        int leftBlue = jewelSensorLeft.blue();
        telemetry.addData("Color", "R-Red-%d, R-Green-%d, R-Blue-%d", rightRed, rightGreen, rightBlue);
        rightRed=rightRed-5;
        if((rightBlue > rightRed)||(rightGreen > rightRed)&&(rightRed <= 40))
        {
            telemetry.addLine("Right: Blue Ball");
        }
        else if(rightRed > rightBlue)
        {
            telemetry.addLine("Right: Red Ball");
        }
        telemetry.addData("Color", "L-Red-%d, L-Green-%d, L-Blue-%d", leftRed, leftGreen, leftBlue);
        leftRed=leftRed-5;
        if((leftBlue > leftRed)||(leftGreen > leftRed)&&(leftRed <= 40))
        {
            telemetry.addLine("Left: Blue Ball");
        }
        else if(leftRed > leftBlue)
        {
            telemetry.addLine("Left: Red Ball");
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}