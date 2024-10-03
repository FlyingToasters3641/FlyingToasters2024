package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
        public static final String kCameraName = "Maximocam";
        public static final String kSecondCameraName = "Aadithcam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-11.833), Units.inchesToMeters(6.653), Units.inchesToMeters(21.498)), new Rotation3d(0, Units.degreesToRadians(-10.0), Units.degreesToRadians(-30)));
        public static final Transform3d kRobotToSecondCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(-11.833), Units.inchesToMeters(6.653), Units.inchesToMeters(21.498)), new Rotation3d(0, Units.degreesToRadians(-10.0), Units.degreesToRadians(30)));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        /*
         *
         * http://10.36.41.74:5800/#/dashboard
         * Front-Left Camera = Arducam_OV2311_USB_Camera
         * x = - 0.307310, y = 0.264824
         * Front-Front Camera = Arducam_OV2311_USB_Cam_Front
         * x = -0.264824, y = 0.307310
         * 
         * http://10.36.41.28:5800/#/dashboard
         * Right-Front Camera = Maximocam
         * x = 0.264824, y = .307310
         * Right-Right Camera = Aadithcam
         * x = 0.307310, y = 0.264824
         * 
         */
    }