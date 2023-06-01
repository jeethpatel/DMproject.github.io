#include <opencv2/opencv.hpp> // vs code is unalble to open this file 
// #include <openacc.h>
double calculateDistance(const cv::Point& p1, const cv::Point& p2) {
    // Calculate Euclidean distance between two points
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
bool matchFingerprints(const std::vector<cv::Point>& minutiaePoints1, const std::vector<cv::Point>& minutiaePoints2, double threshold) {
    // Perform fingerprint matching based on extracted minutiae points
    int matchedCount = 0;
    for (const auto& minutia1 : minutiaePoints1) {
        for (const auto& minutia2 : minutiaePoints2) {  //loop runs to compare the minutae points of the sample
            double distance = calculateDistance(minutia1, minutia2);

            // Check if the distance between two minutiae points is below the threshold
            if (distance <= threshold) {
                matchedCount++;
                break;
            }
        }
    }
// Determine if the fingerprints match based on the number of matched minutiae points
    double matchPercentage = static_cast<double>(matchedCount) / minutiaePoints1.size();
    if (matchPercentage >= 0.8) {
        return true;  // Fingerprints match
    } else {
        return false; // Fingerprints do not match
    }
}
int main() {
    // Load the fingerprint image
    cv::Mat fingerprintImage = cv::imread("fingerprint.jpg", cv::IMREAD_GRAYSCALE);

    if (fingerprintImage.empty()) {
        std::cout << "Failed to load the fingerprint image." << std::endl;
        return -1;
    }
    // Perform image preprocessing (e.g., noise removal, enhancement)
    cv::Mat processedImage;
    cv::GaussianBlur(fingerprintImage, processedImage, cv::Size(5, 5), 0);
    cv::equalizeHist(processedImage, processedImage);
    // Perform fingerprint feature extraction (e.g., minutiae detection)
    cv::Mat binaryImage;
    cv::threshold(processedImage, binaryImage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Apply morphological operations to enhance the fingerprint ridges
    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binaryImage, binaryImage, cv::MORPH_CLOSE, morphKernel);
    // Find contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

   // Extract minutiae points from the contours
    std::vector<cv::Point> minutiaePoints;
    for (const auto& contour : contours) {
        // Perform additional filtering or processing on the contour if needed
        // Find centroid of the contour as a minutiae point
        cv::Moments moments = cv::moments(contour);
        cv::Point centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

        minutiaePoints.push_back(centroid);
    }
   // Display the fingerprint image with extracted minutiae points
    cv::Mat outputImage;
    cv::cvtColor(fingerprintImage, outputImage, cv::COLOR_GRAY2BGR);
    for (const auto& minutia : minutiaePoints) {
        cv::circle(outputImage, minutia, 5, cv::Scalar(0, 0, 255), -1);
    }
    // Perform fingerprint matching (compare extracted features with a reference database)
    std::vector<cv::Point> minutiaePoints1;
    // Add your code here...
    //we have asumed that we have already stored an image as reference
    // Set a threshold for matching
    double matchingThreshold = 10.0;
    // Perform fingerprint matching
    bool fingerprintsMatch = matchFingerprints(minutiaePoints1, minutiaePoints2, matchingThreshold);
    if (fingerprintsMatch) {
        std::cout << "Fingerprints match." << std::endl;
    } else {
        std::cout << "Fingerprints do not match." << std::endl;
    }
    // Display the fingerprint image and extracted features
    cv::imshow("Fingerprint Image", fingerprintImage); //image file name-->fingerprint image
    cv::imshow("Processed Image", processedImage);  //final processed image filename->Processed Image
    cv::waitKey(0);
    return 0;
}