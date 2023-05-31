#include <eigen/Eigen/Dense>
#include "global_vars.h"
#include "task1.h"
#include "task2.h"
#include "task3.h"
#include "stop_vars.h"

#include <opencv2/opencv.hpp> //OpenCV header to use VideoCapture class//
#include <iostream>
#include <time.h>
#include <unistd.h>
using namespace std;
using namespace cv;

volatile double distFromCenter = 0;
volatile int pixelHeight = 0;

volatile int run_camera_flag = DISABLED;
volatile int targetAquired = DISABLED;
volatile int boxInRange = DISABLED;
volatile int abortBox = DISABLED;
volatile int oneDetected = DISABLED;

Rect roi;
Scalar lowerBound = Scalar(0, 0, 0);
Scalar upperBound = Scalar(255, 255, 255);
bool roiSelected = false;

void *Task2Thread(void *arg)
{
    run_camera_flag = DISABLED;
    Mat inputImage;
    Mat imgHSV, imgBW;
    Mat binaryImage;
    Mat imgFiltered;
    Mat erosion_dst;
    Mat dialition_dst;
    int widthOfLargestContour;
    int heightOfLargestContour;
    int zeroCounter;
    int onesCount;

    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 2 * 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 2 * 240);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open the video stream" << std::endl;
        return NULL;
    }

    while (global_stop_msg != ACTIVE)
    {
        if (run_camera_flag == ACTIVE)
        {
            cap >> inputImage;

            if (inputImage.empty())
            {
                break;
            }

            // Calculate the height of the image
            int imageHeight = inputImage.rows;

            // Define the ROI for the lower 70% of the image
            int roiHeight = 0.7 * imageHeight;
            Rect roi(0, imageHeight - roiHeight, inputImage.cols, roiHeight);

            // Crop the image to the ROI
            Mat croppedImage = inputImage(roi);

            // Convert the input image to HSV color space
            Mat hsvImage;
            cvtColor(croppedImage, hsvImage, COLOR_BGR2HSV);

            // Define the upper and lower HSV bounds for blue color
            Scalar lowerBound(95, 20, 10);    // 90, 30, 10
            Scalar upperBound(115, 255, 255); // 120, 255, 255

            // Create a binary mask for blue color
            Mat mask;
            inRange(hsvImage, lowerBound, upperBound, mask);

            GaussianBlur(mask, mask, Size(5, 5), 0);

            // Find contours in the binary mask
            vector<vector<Point>> contours;
            findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            // Find the largest blue contour
            int largestContourIndex = -1;
            double largestContourArea = 0.0;
            for (int i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                if (area > largestContourArea && area > 200)
                {
                    largestContourArea = area;
                    largestContourIndex = i;
                }
            }
            // Create a mask for the largest blue contour
            Mat largestContourMask;
            if (!contours.empty() && largestContourIndex > -1)
            {
                largestContourMask = Mat::zeros(croppedImage.size(), CV_8UC1);
                drawContours(largestContourMask, contours, largestContourIndex, Scalar(255), FILLED);
            }

            int nonMatchingPixelCount = 0;
            // Count the number of pixels inside the contour that are not within the HSV range
            if (!largestContourMask.empty())
            {
                targetAquired = ACTIVE;
                int startY = 0;
                int endY = croppedImage.rows;

                for (int y = startY; y < endY; y++)
                {
                    for (int x = 0; x < croppedImage.cols; x++)
                    {
                        if (largestContourMask.at<uchar>(y, x) == 255 && mask.at<uchar>(y, x) == 0)
                        {
                            nonMatchingPixelCount++;
                        }
                    }
                }
            }
            else
                targetAquired = DISABLED;
            double colorRatio;
            // Calculate the bounding rectangle for the largest contour
            if (!largestContourMask.empty())
            {
                // Calculate bounding rectangle only if contours are found
                Rect boundingRectOfLargestContour = boundingRect(contours[largestContourIndex]);

                // Retrieve the width and height of the bounding rectangle
                widthOfLargestContour = boundingRectOfLargestContour.width;
                heightOfLargestContour = boundingRectOfLargestContour.height;
                colorRatio = (double)nonMatchingPixelCount / (widthOfLargestContour * heightOfLargestContour);
            }
            double dy = 0;
            double dx = 0;
            if (!largestContourMask.empty())
            {
                Moments moments = cv::moments(contours[largestContourIndex]);

                if (moments.m00 != 0)
                {
                    double cx = moments.m10 / moments.m00;
                    double cy = moments.m01 / moments.m00;

                    int imageWidth = croppedImage.cols;
                    int imageHeight = croppedImage.rows;

                    dx = cx - imageWidth / 2;
                    dy = cy - imageHeight / 2;
                    distFromCenter = dx;
                }

                // Visualize the largest blue contour and the mask
                // Mat largestContourImage;
                // croppedImage.copyTo(largestContourImage, largestContourMask);
                // drawContours(croppedImage, contours, largestContourIndex, Scalar(0, 0, 255), 2);
                // imshow("Largest Blue Contour", croppedImage);
                // imshow("Largest Blue Contour Mask", largestContourImage);

                if (waitKey(1) == 27)
                    break;
            }
            // Classify the box based on the number of non-matching pixels
            double rak_upper_threshold = 0.08;
            double rak_lower_threshold = 0.04; // förut 0.02

            double sne_upper_threshold = 0.06; // Adjust this threshold as needed
            double sne_lower_threshold = 0.04; // förut 0.02

            // För 30cm från kameran.
            if ((heightOfLargestContour > 110 && heightOfLargestContour < 140) &&
                (widthOfLargestContour <= 205 && widthOfLargestContour > 180))
            {
                boxInRange = ACTIVE;
                if (colorRatio > rak_upper_threshold)
                {
                    onesCount = 0;
                    cout << "Box contains a ZERO." << endl;
                    oneDetected = DISABLED;
                }
                else if (colorRatio < rak_lower_threshold)
                {
                    onesCount = 0;
                    cout << "Box contains NO NUMBER." << endl;
                    oneDetected = DISABLED;
                }
                else
                {
                    onesCount++;
                    if (onesCount > 1)
                    {
                        cout << "Box contains a ONE." << endl;
                        oneDetected = ACTIVE;
                        abortBox = DISABLED;
                        onesCount = 0;
                    }
                }
                std::cout << "Rak" << std::endl;
            }
            else if ((heightOfLargestContour > 115 && heightOfLargestContour < 140) &&
                     (widthOfLargestContour < 230 && widthOfLargestContour > 205))
            {
                boxInRange = ACTIVE;
                if (colorRatio > sne_upper_threshold)
                {
                    onesCount = 0;
                    cout << "Box contains a ZERO." << endl;
                    oneDetected = DISABLED;
                }
                else if (colorRatio < sne_lower_threshold)
                {
                    onesCount = 0;
                    cout << "Box contains NO NUMBER." << endl;
                    oneDetected = DISABLED;
                }
                else
                {
                    onesCount++;
                    if (onesCount > 1)
                    {
                        cout << "Box contains a ONE." << endl;
                        oneDetected = ACTIVE;
                        abortBox = DISABLED;
                        onesCount = 0;
                    }
                }
                cout << "Sne" << endl;
            }

            run_camera_flag = DISABLED;
        }
    }
    cap.release();       // Releasing the buffer memory//
    destroyAllWindows(); // Remove all windows
    return NULL;
}
