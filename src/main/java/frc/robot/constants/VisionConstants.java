package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class VisionConstants {
    public static final String kLimelight = "limelight-front";

    public static final double kMinTA = 0.10;
    public static final double kMaxTA = 0.50;

    public static final double kDontTrust = 9999999;

    public static final InterpolatingDoubleTreeMap kOneTagXY = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kOneTagRot = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kMultiTagXY = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kMultiTagRot = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kMegaTagTwo = new InterpolatingDoubleTreeMap();

    public static void SetUpTrustConstants() {
        kMultiTagXY.put(0.20, 100.0);
        kMultiTagXY.put(0.25, 4.0);
        kMultiTagXY.put(0.30, 2.0);
        kMultiTagXY.put(0.50, 0.25);

        kMultiTagRot.put(0.20, 400.0);
        kMultiTagRot.put(0.25, 10.0);
        kMultiTagRot.put(0.30, 5.0);
        kMultiTagRot.put(0.60, 0.5);

        kMegaTagTwo.put(0.05, 100.0);
        kMegaTagTwo.put(0.10, 10.0);
        kMegaTagTwo.put(0.20, 5.0);
        kMegaTagTwo.put(0.25, 2.0);
        kMegaTagTwo.put(0.30, 1.0);
        kMegaTagTwo.put(0.60, 0.25);
    }
}