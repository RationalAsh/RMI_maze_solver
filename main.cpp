///Standard template for getting a video feed from a webcam.
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <SerialStream.h>
#include "string.h"
//#include <system>
//#include <math.h>
//#include <complex>

using namespace std;
using namespace cv;

Point clickpoint(1,1);

void getCentroid(Mat &thresholded_image, Point &Centroid, int &Area)
{
    ///The object that holds all the centroids.
    ///Pass in the image. The boolean true tells the function that the image is binary
    Moments m = moments(thresholded_image, true);
    ///Moment along x axis
    double M10 = m.m10;
    ///Moment along y-axis;
    double M01 = m.m01;
    ///Area
    double M00 = m.m00;
    Centroid.x  = int(M10/M00);
    Centroid.y  = int(M01/M00);
    Area        = int(M00);
}


Point3d findCenterAndOrientation(const Mat& src)
{
    Moments m = cv::moments(src, true);
    double cen_x = m.m10/m.m00; //Centers are right
    double cen_y = m.m01/m.m00;

    double a = m.mu20-m.m00*cen_x*cen_x;
    double b = 2*m.mu11-m.m00*(cen_x*cen_x+cen_y*cen_y);
    double c = m.mu02-m.m00*cen_y*cen_y;

    double theta = a==c?0:atan2(b, a-c)/2.0;

    return Point3d(cen_x, cen_y, theta);
}

class Robot
{
    public:
    Point3d currentPose;
    Point3d initialPose;
    Point3d goalPose;
    Point frontMarker;
    Point rearMarker;
    int frontMarkerArea, rearMarkerArea;
    ///Constructor
    Robot(Point3d state);
    Robot();
};

Robot::Robot(Point3d state)
{
    currentPose = state;
    frontMarker = Point(0,0);
    rearMarker  = Point(0,0);
}

Robot::Robot()
{
    currentPose = Point3d(0,0,0);
    frontMarker = Point(0,0);
    rearMarker  = Point(0,0);
}

class imageCell
{
    public:
    ///constructor
    imageCell(Mat &image);
    ///Set image
    void setImage(Mat &image);
    ///convert from pixel to grid coordinates
    Point pixToGrid(int pix_row, int pix_col);
    ///Set the grid location
    void setGridLoc(int row, int col);
    void setGridLoc(Point gridLoc);

    ///Check is cell is path
    bool isCellPath();
    int getCellArea();

    ///Robot functions
    void getPosandOrientation();

    char getMotion(Point robotPos);

    ///Fill a cell with white
    void fillCell(int color);

    private:
    Mat img; Mat head; Mat tail;
    int cell_length;
    int rows, cols;
    Point gridPoint;
    Point cell_start;
    Point cell_end;
    Point botPos;
    Robot robot;

};

imageCell::imageCell(Mat &image)
{
    img = image;
    cvtColor(img, head, CV_RGB2GRAY);
    cvtColor(img, tail, CV_RGB2GRAY);
    cell_length = 20;
    rows = (img.rows)/cell_length; cols = (img.cols)/cell_length;
    gridPoint = Point(1,1);
    cout<<"\nThe size of the image is: "<<img.rows<<"x"<<img.cols<<"\n";
    botPos = Point(5,21);
}

void imageCell::setImage(Mat &image)
{
    img = image;
}

Point imageCell::pixToGrid(int pix_row, int pix_col)
{
    Point pt((int)(pix_col/cell_length)+1,(int)(pix_row/cell_length)+1);
    return pt;
}

void imageCell::setGridLoc(int row, int col)
{
    gridPoint.x  = col;
    gridPoint.y  = row;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

void imageCell::setGridLoc(Point gridLoc)
{
    gridPoint = gridLoc;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

int imageCell::getCellArea()
{
    int Area = 0; int i=0; int j=0;
    for(i=cell_start.y+1; i<cell_end.y; i++)
    {
        for(j=cell_start.x+1; j<cell_end.x; j++)
        {
            Area = Area + (img.at<uchar>(i,j))/255;
        }
    }
    return (Area);
}

bool imageCell::isCellPath()
{
    if(getCellArea() > (20*20*0.5))
    return true;
    else return false;
}

char imageCell::getMotion(Point robotPos)
{

}

void imageCell::fillCell(int color)
{
    int i=0; int j=0;
    for(i=cell_start.y+1; i<cell_end.y; i++)
    {
        for(j=cell_start.x+1; j<cell_end.x; j++)
        {
            img.at<uchar>(i,j) = color;
        }
    }
}


void mouseEvent(int event, int x, int y, int flags, void *param)
{
    imageCell *cellptr = (imageCell*) param;

    if(event == EVENT_LBUTTONDOWN )
    {
        clickpoint.x = x;
        clickpoint.y = y;
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(255);
        cout<<"\nX: "<<x<<" Y: "<<y;
    }

    if(event == EVENT_RBUTTONDOWN)
    {
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(0);
        printf("\nThe cell Area is: %d", cellptr->getCellArea());
    }

    if(event == EVENT_MBUTTONDBLCLK)
    {
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        printf("\nThe cell Area is: %d", cellptr->getCellArea());
    }

}

void HSV_threshold(Mat &image, Mat &output_image_gray, int H_upper, int H_lower, int S_upper, int S_lower, int V_upper, int V_lower)
{
    Mat HSV;///Temporary Mat to store HSV


    ///Converting input image to HSV
    cvtColor(image, HSV, CV_RGB2HSV);
    //cvtColor(output_image_gray, output_image_gray, CV_RGB2GRAY);

    ///Processing each pixel and thresholding the image.
    int i, j;
        for(i=0; i<image.rows; i++)
        {
            for(j=0; j<image.cols; j++)
            {
                if((HSV.at<Vec3b>(i,j)[0] > H_lower)&&(HSV.at<Vec3b>(i,j)[0] < H_upper)&&(HSV.at<Vec3b>(i,j)[1]>S_lower)&&(HSV.at<Vec3b>(i,j)[1]<S_upper)&&(HSV.at<Vec3b>(i,j)[2]<V_upper)&&(HSV.at<Vec3b>(i,j)[2]>V_lower)) output_image_gray.at<uchar>(i,j) = 255;
                else output_image_gray.at<uchar>(i,j) = 0;
            }
        }
}

void imageCell::getPosandOrientation()
{
    Point orientationVector; double orientation;
    HSV_threshold(img, head, 255,0, 255,0, 255,0);
    HSV_threshold(img, tail, 255,0, 255,0, 255,0);
    getCentroid(head, robot.frontMarker, robot.frontMarkerArea);
    getCentroid(tail, robot.rearMarker, robot.rearMarkerArea);
    orientationVector = robot.frontMarker - robot.rearMarker;
    orientation = atan2(orientationVector.y, orientationVector.x);
    robot.currentPose = Point3d(robot.frontMarker.x, robot.frontMarker.y, orientation);
}

int main(int argc, char** argv)
{
    ///Variables
    char keypress; int camera_number = 0;
    Mat tempp(900, 1600, CV_8UC3, Scalar(0));
    Mat camera_frame; Mat thresh_frame;
    ///Read image
    Mat maze = imread("Path_bin.jpg");
    //cvtColor(tempp, maze, CV_RGB2GRAY);
    if(maze.empty()) return -1;
    cvtColor(maze, maze, CV_RGB2GRAY);
    threshold(maze, maze, 250, 255, CV_THRESH_BINARY_INV);
    medianBlur(maze, maze, 5);

    VideoCapture camera;
    camera.open(camera_number);
    if(! camera.isOpened())
    {
        cerr<<"ERROR: COULD NOT ACCESS THE CAMERA!"<<endl;
        exit(1);
    }

    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


    ///Img processing class
    imageCell cells(maze);
    cells.setGridLoc(1,1);
    cells.fillCell(255);
    namedWindow("maze");
    setMouseCallback("maze", mouseEvent, &cells);
    int i,j;
    camera >> camera_frame;
    camera_frame.copyTo(thresh_frame);
    cvtColor(thresh_frame, thresh_frame, CV_RGB2GRAY);
    while(true)
    {
        camera >> camera_frame;
        if(camera_frame.empty())
        {
            cerr<<"ERROR: COULD NOT GRAB A FRAME!"<<endl;
            exit(1);
        }
        HSV_threshold(camera_frame, thresh_frame, 122, 79, 98, 9, 208, 163);
        Point3d point = findCenterAndOrientation(thresh_frame);
        cout<<"\nThe angle is :"<<point.z;
        //imshow("maze", maze);
        imshow("video", thresh_frame);
        keypress = waitKey(30);

        if(keypress==27) break;
    }


    return 0;
}
