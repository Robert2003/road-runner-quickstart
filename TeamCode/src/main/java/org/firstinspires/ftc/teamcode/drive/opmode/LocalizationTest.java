package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double P=0.3;
        double flyWheelPower = 0;
        double intakePower = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor flyWheel = null;
        flyWheel  = hardwareMap.get(DcMotor.class, "flywheel");

        DcMotor intake = null;
        intake  = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        while (!isStopRequested())
        {
            if(gamepad1.right_bumper)
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y*P,
                                -gamepad1.left_stick_x*P,
                                -gamepad1.right_stick_x*P
                        )
                );
            else
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            drive.update();

            flyWheelPower = Range.clip(gamepad2.left_stick_y, -0.75, 0.75);
            flyWheel.setPower(flyWheelPower);

            intakePower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
            intake.setPower(intakePower);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
