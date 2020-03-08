package teamcode.Vision;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SkyDetector extends DogeCVDetector {
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0,30);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorer = new RatioScorer(1.25, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value


    // Results of the detector
    private Point screenPosition = new Point(); // Screen position of the mineral
    private Rect foundRect = new Rect(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy  = new Mat();

    Rect bestRect;

    public Rect foundRectangle() {
        return foundRect;
    }

    public SkyDetector() {
        detectorName = "Skystone Detector";
    }

    @Override
    public Mat process(Mat input) {
        //
        // input = input.submat(0, input.rows(), 0, input.cols() / 2);
        input = input.submat(0, input.rows(), input.cols() * (2/3), input.cols() / 3);

        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(255,30,30),2);

        // Current result
        bestRect = foundRect;
        double bestDifference = Double.MIN_VALUE; // MAX_VALUE since less difference = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score > bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }

        Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255,255,255), 1, Imgproc.LINE_4, 0);
        blackFilter.process(workingMat.clone(), blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Mat cannyOutput = new Mat();
        Imgproc.Canny(blackMask, cannyOutput,100,200);
        Imgproc.findContours(cannyOutput, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(255,255,255),2);


        for(MatOfPoint cont : contoursBlack) {
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }

        Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(0,255,0),4);
        Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

//        if(bestRect != null) {
//            // Show chosen result
//            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(0,255,0),4);
//            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));
//
//            screenPosition = new Point(bestRect.x, bestRect.y);
//            foundRect = bestRect;
//            found = true;
//        }
//        else {
//            found = false;
//        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    public double getScreenPosition() {
        return ((bestRect.tl().y + bestRect.br().y)) / 2;
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scorers depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
    }
}