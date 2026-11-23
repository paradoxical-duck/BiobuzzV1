//package org.firstinspires.ftc.teamcode.versionTesting.aprilTagSS;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystems.AprilTagSS;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
//
//@TeleOp(name = "AprilTag Test V1", group = "Version Testing")
//public class AprilTagTestV1 extends LinearOpMode {
//    RobotHardware robot = RobotHardware.getInstance();
//    AprilTagSS aprilTagSS;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        aprilTagSS = new AprilTagSS(robot);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            aprilTagSS.update();
//
//            telemetry.addData("Alliance", RobotHardware.getAlliance());
//            telemetry.addData("Ready to shoot", aprilTagSS.readyToShoot());
//            telemetry.addData("Detected clusters", aprilTagSS.detectedClusters);
//
//            for (AprilTagDetection detection : robot.arducam.getDetections()) {
//                if (detection instanceof AprilTagSingleDetection) {
//                    AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
//
//                    telemetry.addData("Tag ID", tag.id);
//                    telemetry.addData("Tag Name", tag.metadata != null ? tag.metadata.name : "Unknown");
//                } else if (detection instanceof AprilTagClusterDetection) {
//                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
//
//                    telemetry.addData("Cluster Name", cluster.metadata.name);
//                    telemetry.addData("Cluster Percentage", cluster.percentClusterFound + "%");
//                }
//            }
//
//            telemetry.update();
//        }
//    }
//}