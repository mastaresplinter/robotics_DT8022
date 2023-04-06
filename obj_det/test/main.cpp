#include<opencv2/opencv.hpp>//OpenCV header to use VideoCapture class//
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
#include <time.h>
using namespace std;
using namespace cv;

Rect roi;
Scalar lowerBound = Scalar(0,0,0);
Scalar upperBound = Scalar(255, 255, 255);
bool roiSelected = false;

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    static Point start;

    if (event == EVENT_LBUTTONDOWN)
    {
        start = Point(x, y);
    }
    else if (event == EVENT_LBUTTONUP)
    {   
        Point end(x, y);
        //If you haven´t drawn a box aka just pressed one point or drawn a line, nothing will happen
        if (start.x != end.x && start.y != end.y) {
            roi = Rect(start, end);
            roiSelected = true;
        }
    }
}

int main() {
    clock_t start_t, end_t;
    double total_t = 0;
    Mat myImage;//Declaring a matrix to load the frames//
    Mat imgHSV, imgBW; //Create and allocate the variables
    Mat binaryImage;
    Mat imgFiltered;
    Mat erosion_dst;
    Mat dialition_dst;
    
    
    VideoCapture cap(0);//Declaring an object to capture stream of frames from default camera//
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open the video stream" << std::endl;
        return -1;
    }

    //namedWindow("Video Stream");//Declaring the video to show the video//
    //setMouseCallback("Video Stream", mouseCallback, NULL);

    while (true) { //Taking an everlasting loop to show the video//

        start_t = clock();
        cap >> myImage;
        

        if (myImage.empty()) { //Breaking the loop if no video frame is detected//
            break;
        }
     
        //Convert image from BGR to HSV
        cvtColor(myImage, imgHSV, COLOR_BGR2HSV);

        

        // Select the region of interest
        /*if (roiSelected)
        {
            rectangle(myImage, roi, Scalar(0, 0, 255), 2);

            Mat roiImage = imgHSV(roi);


            // Calculate the mean and standard deviation of the colors in the ROI
            Scalar mean, stdDev;
            meanStdDev(roiImage, mean, stdDev);

            // Calculate the upper and lower bounds of the colors
            lowerBound = mean - stdDev * 3;
            upperBound = mean + stdDev * 3;

            //Don't recalculate each frame
            roiSelected = false;

        }*/
        //Show video stream
        //imshow("Video Stream", myImage);
        //Show video stream
        //imshow("HSV Stream", imgHSV);

	lowerBound = Scalar(80, 100, 100);
	upperBound = Scalar(120, 255, 255);

        //convert to thesholded Binary image
        inRange(imgHSV, lowerBound, upperBound, binaryImage);
        //imshow("thresholded Binary image", binaryImage);

        //add Gaussian Blur
        GaussianBlur(binaryImage, imgFiltered, Size(5, 5), 0);
        //imshow("GaussianBlur Demo", imgFiltered);

        //add Erosion
        erode(imgFiltered, erosion_dst, Mat(), Point(-1, -1), 2, 2);
        //imshow("Erosion Demo", erosion_dst);

        //add Dialation
        dilate(erosion_dst, dialition_dst, getStructuringElement(MORPH_RECT, Size(12, 12), Point(-1, -1)), Point(-1, -1), 2, 2);
        //imshow("Dialation Demo", dialition_dst);
      
       

        // Find contours
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(dialition_dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Draw inner and outer contours
        Mat drawing = Mat::zeros(dialition_dst.size(), CV_8UC3);
        for (int i = 0; i < contours.size(); i++)
        {
            if (hierarchy[i][3] != -1) // If contour has parent, it is an inner contour
                drawContours(drawing, contours, i, Scalar(255, 255, 255), -1); // Fill inner contour
            else
		{
             		drawContours(drawing, contours, i, Scalar(0, 0, 255), 2); // Draw outer contour
		}
        }

        imshow("Contours", drawing); // Display the resulting frame with contours
        if (waitKey(1) == 27) // Wait for 1ms and check if user pressed "Esc" key
            break;




        
            // Find the moments of the contour to determine the center point
        if (!contours.empty()) {
            Moments moments = cv::moments(contours[0]);

            if (moments.m00 != 0) {

                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;

                // Get the size of the image
                int imageWidth = myImage.cols;
                int imageHeight = myImage.rows;

                // Calculate the horizontal and vertical distance from the object center point to the image center point
                double dx = cx - imageWidth / 2;
                double dy = cy - imageHeight / 2;

                end_t = clock();

               
                total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;



                // Print out the object location relative to the image center point
                std::cout << "Object location relative to image center: (X:" << dx << " pixels, Y:" << dy << " pixels, cycle runs: )"<< 1/total_t <<"per second" << std::endl;
               

            }
        }

      


        char c = (char)waitKey(25);//Allowing 25 milliseconds frame processing time and initiating break condition//
        if (c == 27) { //If 'Esc' is entered break the loop//
            break;
        }
    }

    cap.release();//Releasing the buffer memory//
    destroyAllWindows(); //Remove all windows
    return 0;
}