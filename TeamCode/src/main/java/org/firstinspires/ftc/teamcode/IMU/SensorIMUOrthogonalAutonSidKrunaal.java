package org.firstinspires.ftc.teamcode.IMU;///* Copyright (c) 2022 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
///**
// * {@link SensorIMUOrthogonalAutonSiddharth} shows how to use the new universal {@link IMU} interface. This
// * interface may be used with the BNO055 IMU or the BHI260 IMU. It assumes that an IMU is configured
// * on the robot with the name "imu".
// * <p>
// * The sample will display the current Yaw, Pitch and Roll of the robot.<br>
// * With the correct orientation parameters selected, pitch/roll/yaw should act as follows:
// * <p>
// *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
// *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
// *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
// * <p>
// * The yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
// * <p>
// * This specific sample assumes that the Hub is mounted on one of the three orthogonal planes
// * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
// * <p>
// * Note: if your Hub is mounted on a surface angled at some non-90 Degree multiple (like 30) look at
// *       the alternative SensorImuNonOrthogonal sample in this folder.
// * <p>
// * This "Orthogonal" requirement means that:
// * <p>
// * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
// *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
// * <p>
// * 2) The USB ports can only be pointing in one of the same six directions:<br>
// *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
// * <p>
// * So, To fully define how your Hub is mounted to the robot, you must simply specify:<br>
// *    logoFacingDirection<br>
// *    usbFacingDirection
// * <p>
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
// * <p>
// * Finally, choose the two correct parameters to define how your Hub is mounted and edit this OpMode
// * to use those parameters.
// */
//@Autonomous(name = "IMU Siddharth straight", group = "Sensor")
//@Disabled   // Comment this out to add to the OpMode list
//public class SensorIMUOrthogonalAutonSiddharth extends LinearOpMode
//{
//    // The IMU sensor object
//    IMU imu;
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor lfDrive = null;
//    private DcMotor rfDrive = null;
//    private DcMotor lbDrive = null;
//    private DcMotor rbDrive = null;
//    //----------------------------------------------------------------------------------------------
//    // Main logic
//    //----------------------------------------------------------------------------------------------
//
//    @Override public void runOpMode() throws InterruptedException {
//
//        // Retrieve and initialize the IMU.
//        // This sample expects the IMU to be in a REV Hub and named "imu".
//        imu = hardwareMap.get(IMU.class, "imu");
//        lfDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
//        rfDrive = hardwareMap.get(DcMotor.class, "rf_drive");
//        lbDrive  = hardwareMap.get(DcMotor.class, "lb_drive");
//        rbDrive = hardwareMap.get(DcMotor.class, "rb_drive");
//        lfDrive.setDirection(DcMotor.Direction.REVERSE);
//        rfDrive.setDirection(DcMotor.Direction.FORWARD);
//        lbDrive.setDirection(DcMotor.Direction.REVERSE);
//        rbDrive.setDirection(DcMotor.Direction.FORWARD);
//        runtime.reset();
//        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
//         *
//         * Two input parameters are required to fully specify the Orientation.
//         * The first parameter specifies the direction the printed logo on the Hub is pointing.
//         * The second parameter specifies the direction the USB connector on the Hub is pointing.
//         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
//         */
//
//        /* The next two lines define Hub orientation.
//         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
//         *
//         * To Do:  EDIT these two lines to match YOUR mounting configuration.
//         */
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//
//        // Now initialize the IMU with this mounting orientation
//        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        waitForStart();
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        double heading = orientation.getYaw(AngleUnit.DEGREES);
//        // Loop and update the dashboard
//        while (opModeIsActive()) {
//
//            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
//
//
//            // Retrieve Rotational Angles and Velocities
//            YawPitchRollAngles orientation2 = imu.getRobotYawPitchRollAngles();
//            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//            double leftPower = 1;
//            double rightPower = 1;
//
//            if(orientation2.getYaw(AngleUnit.DEGREES) > heading + 0.5) {
//                rightPower = 0.9;
//            }
//            else if(orientation2.getYaw(AngleUnit.DEGREES) < heading - 0.5) {
//                leftPower = 0.9;
//            }
//            else {
//                rightPower = 1;
//                leftPower = 1;
//            }
//
//            lfDrive.setPower(leftPower);
//            rfDrive.setPower(rightPower);
//            lbDrive.setPower(leftPower);
//            rbDrive.setPower(rightPower);
//            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation2.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation2.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation2.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
//            telemetry.update();
//        }
//    }
//}
