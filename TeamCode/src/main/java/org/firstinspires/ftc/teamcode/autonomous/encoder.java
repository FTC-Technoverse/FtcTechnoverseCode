//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous(name = "Encoder Test ")
//public class encoder extends LinearOpMode {
//    DcMotor lf_drive;
//    DcMotor rf_drive;
//
//    double speed= 0.3;
//    double ticks = 537.7;
//    double newLfTarget;
//    double newRfTarget;
//
//
//
//    public void runOpMode() {
//        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
//        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
//
//
//        lf_drive.setDirection(DcMotor.Direction.REVERSE);
//
//
//        telemetry.addData("Hardware: ", "Initialized");
//        lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//        public void start() {
//            encoder(1,1);
//
//            boolean movementInProgress = true;
//
//            while (opmodeIsActive() && movementInProgress) {
//                if (lf_drive.isBusy() && rf_drive.isBusy()) {
//
//                } else {
//                    movementInProgress = false;
//                }
//            }
//        }
//
//
//
//        telemetry.addData("Motor Ticks: ", lf_drive.getCurrentPosition());
//
//
//    }
//
//
//    public void encoder(double turnage1,double turnage2){
//        newLfTarget = ticks*turnage1;
//        newRfTarget = ticks*turnage2;
//
//
//        lf_drive.setTargetPosition((int)newLfTarget);
//        rf_drive.setTargetPosition((int)newRfTarget);
//
//
//        lf_drive.setPower(speed);
//        rf_drive.setPower(speed);
//
//
//        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
//
//
//}