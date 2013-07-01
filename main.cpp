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
using namespace LibSerial;

SerialStream serial_port ;
char str[1];
char serial_command[2];
Point clickpoint(1,1);
Mat head(480 , 640, CV_8UC1, 255); Mat tail(480 , 640, CV_8UC1, 255);
int j, k;

///Function that calculates the centroid of a binary image.
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

///This function isn't really being used.
///It's just there for reference.
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

///A robot class to hold all the data related to the
///robot like position and orientation
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

///The constructor for the robot class
///The variable state contains the position and the
///Orientation of the robot.
Robot::Robot(Point3d state)
{
    currentPose = state;
    frontMarker = Point(0,0);
    rearMarker  = Point(0,0);
}

///Overloaded constructor to declare a robot object without any
///arguments
Robot::Robot()
{
    currentPose = Point3d(0,0,0);
    frontMarker = Point(0,0);
    rearMarker  = Point(0,0);
}

///This is the image processing class. It contains all the
///functions and algorithms to divide an image into cells and
///perform operations on these cells rather than directly on pixels.
///The cell length is set as 20. That means that each cell will be
///20 pixels wide.
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
    void getRobotPosandOrientation();

    char getMotion(Point3d robotPos);

    ///Fill a cell with white
    void fillCell(int color);

    private:
    ///A dynamic array that stores the current state of the image
    int **imgState;
    ///An internal copy of the images to be divided into cells
    Mat img;
    ///The dimensions of the cell
    int cell_length;
    ///The number of rows and columns in the grid
    int rows, cols;
    ///The point in the grid that is being operated on
    Point gridPoint;
    ///variable that holds the begginning pixel value of the current cell
    Point cell_start;
    ///Variable that holds the end pixel value fo the current cell
    Point cell_end;
    ///Position of the bot?
    Point botPos;
    ///A robot object to hold information about the robot.
    Robot robot;

};

///Constructor for the class. It takes a pointer to a Mat as
///the argument. The constructor sets up internal variables.
imageCell::imageCell(Mat &image)
{
    ///Constructor. Setting up variables.
    ///A copy of the image
    img = image;
    ///Dimensions of the cell
    cell_length = 20;
    ///The number of rows and columns in the grid.
    rows = (img.rows)/cell_length; cols = (img.cols)/cell_length;
    ///assigning memory space to the array dynamically
    imgState = new int*[rows+1];
    for(k=0;k<rows+1;k++) imgState[k] = new int[cols+1];

    ///Filling the array with -1
    for(j=1;j<=rows;j++)
    {
        for(k=1;k<=cols;k++) imgState[j][k] = -1;
    }

    ///Initializing the gridpoint with value 1,1
    gridPoint = Point(1,1);
    ///For debugging
    cout<<"\nThe size of the image is: "<<img.rows<<"x"<<img.cols<<"\n";
    botPos = Point(5,21);
}

///Sets the image being operated on by the class.
void imageCell::setImage(Mat &image)
{
    img = image;
    ///Checking the array for the state
    ///This is important. Otherwise, the changes made to the
    ///cell won't stay permanent across the frames in a video.
    for(j=1;j<=rows;j++)
    {
        for(k=1;k<=cols;k++)
        {
            if(imgState[j][k] != -1)
            {
                setGridLoc(j,k);
                fillCell(imgState[j][k]);
            }

        }
    }
}

Point imageCell::pixToGrid(int pix_row, int pix_col)
{
    ///Convert from pixel coordinates to
    ///grid coordinates.
    Point pt((int)(pix_col/cell_length)+1,(int)(pix_row/cell_length)+1);
    return pt;
}

///Set the current point in the grid being operated on.
void imageCell::setGridLoc(int row, int col)
{
    ///Set the grid location
    gridPoint.x  = col;
    gridPoint.y  = row;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

///Overloaded function which accepts a Point type.
///This is a datatypy defined in opencv because it
///convenient for a lot of image processing tasks.
void imageCell::setGridLoc(Point gridLoc)
{
    ///Overloaded
    gridPoint = gridLoc;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

///Get the area of the cell. Since the image is intended to be
///Binary, this gives the number of white pixels.
int imageCell::getCellArea()
{
    ///Get the area of the cell.
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

///This function is under construction check back later.
bool imageCell::isCellPath()
{
    ///Determine if cell is part of the path.
    if(getCellArea() > (20*20*0.5))
    return true;
    else return false;
}

///This function is under construction. Check back later.
char imageCell::getMotion(Point3d robotPos)
{
    ///Check the surrounding cells and return
    ///correct motion - (W, A, S or D)
    ///The robot position is in pixel coordinates
    ///remember to convert it to grid coordinates.
    Point robotGridPos = pixToGrid(robotPos.y, robotPos.x);


}

///Fill the cell with a grayscale value.
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
    imgState[gridPoint.y][gridPoint.x] = color;
    //cout<<"\nThe state of 1,1 is: "<<imgState[gridPoint.y][gridPoint.x];
}

///Callback function for mouseevents. This function is called
///whenever a mouse event is detected within a window with a
///mouse callback set.
void mouseEvent(int event, int x, int y, int flags, void *param)
{
    imageCell *cellptr = (imageCell*) param;

    ///If the left mouse button is pressed make the cell white
    if(event == EVENT_LBUTTONDOWN )
    {
        clickpoint.x = x;
        clickpoint.y = y;
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(255);
        cout<<"\nX: "<<x<<" Y: "<<y;
    }

    ///If the right mouse button is pressed make the cell black.
    if(event == EVENT_RBUTTONDOWN)
    {
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(0);
        printf("\nThe cell Area is: %d", cellptr->getCellArea());
    }

    ///If the middle button is pressed, print the cell area.
    if(event == EVENT_MBUTTONDBLCLK)
    {
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        printf("\nThe cell Area is: %d", cellptr->getCellArea());
    }

}

///This function accepts an colour image and applies thresholding to it
///using given HSV values. It then writes a binary image (which is the
///result of the thresholding operation) to the Mat pointed to
///by output_image_gray
void HSV_threshold(Mat &image, Mat &output_image_gray, int H_upper, int H_lower, int S_upper, int S_lower, int V_upper, int V_lower)
{
    Mat HSV;///Temporary Mat to store HSV


    ///Converting input image to HSV
    cvtColor(image, HSV, CV_RGB2HSV);
    cvtColor(output_image_gray, output_image_gray, CV_RGB2GRAY);

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

///Function to get the robot position and orientation
///The HSV values need to be set to the colours of the
///two markers on the robot.
void imageCell::getRobotPosandOrientation()
{
    Point orientationVector; double orientation;
    HSV_threshold(img, head, 255,0, 255,0, 255,0);
    HSV_threshold(img, tail, 255,0, 255,0, 255,0);
    getCentroid(head, robot.frontMarker, robot.frontMarkerArea);
    getCentroid(tail, robot.rearMarker, robot.rearMarkerArea);
    orientationVector = robot.frontMarker - robot.rearMarker;
    orientation = atan2(orientationVector.y, orientationVector.x);
    robot.currentPose = Point3d((robot.frontMarker.x), (robot.frontMarker.y), orientation);
}

///This function sends a character over a serial connection to the
///Arduino which controls the RC module. The characer tells the robot
///how to move.
void SerialSend(char c)
{
    serial_command[0] = c;

    serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
    serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
    serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    serial_port.write(serial_command, 1);

}


int main(int argc, char** argv)
{
    ///Variables
    char keypress; int camera_number = 0;
    char response;
    Mat tempp(900, 1600, CV_8UC3, Scalar(0));
    Mat camera_frame; Mat thresh_frame;
    ///Read image. This is for debugging
    Mat maze = imread("Path_bin.jpg");
    //cvtColor(tempp, maze, CV_RGB2GRAY);
    if(maze.empty()) return -1;
    cvtColor(maze, maze, CV_RGB2GRAY);
    threshold(maze, maze, 250, 255, CV_THRESH_BINARY_INV);
    medianBlur(maze, maze, 5);

    ///Serial setup
    serial_port.Open("/dev/ttyACM0");

    if(!serial_port.IsOpen()) cout<<"ERROR";


    ///Video capture object
    VideoCapture camera;
    camera.open(camera_number);
    if(! camera.isOpened())
    {
        cerr<<"ERROR: COULD NOT ACCESS THE CAMERA!"<<endl;
        exit(1);
    }

    ///Setting the resolution of the webcam
    ///This needs to be higher during the actual event
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


    ///Img processing class declaration
    ///Intitialized with the maze image. For debugging.
    imageCell cells(maze);
    cells.setGridLoc(1,1);
    cells.fillCell(255);
    namedWindow("maze");
    namedWindow("video");
    camera >> camera_frame;
    imageCell Vid(camera_frame);

    ///Setting the mouse callback function
    setMouseCallback("video", mouseEvent, &Vid);

    ///Main loop of the program
    while(true)
    {
        ///Get the next frame from the camera
        camera >> camera_frame;
        //serial_port.read(str,1);
        //printf("\nChar: %s", str);
        if(camera_frame.empty())
        {
            cerr<<"ERROR: COULD NOT GRAB A FRAME!"<<endl;
            exit(1);
        }

        ///Convert the image from the webcam to grayscale.
        cvtColor(camera_frame, thresh_frame, CV_RGB2GRAY);
        ///Show the maze
        imshow("maze", maze);

        ///Sets the first cell in the video to white.
        ///I put this here to test if the image processing class was
        ///working with the video feed.
        Vid.setImage(thresh_frame);
        Vid.setGridLoc(1,1);
        Vid.fillCell(255);

        ///Show the video
        imshow("video", thresh_frame);
        keypress = waitKey(30);

        ///Keyhandlers for teleop mode. This is for controlling the robot
        ///Using the W A S D keys on the keyboard. This allows me to control
        ///the robot like a race car in NFS. :)
        if(keypress=='w' || keypress=='W') SerialSend('W');
        if(keypress=='a' || keypress=='A') SerialSend('A');
        if(keypress=='s' || keypress=='S') SerialSend('S');
        if(keypress=='d' || keypress=='D') SerialSend('D');
        //cout<<"\nThe received char is: "<<response<<"\n";
        ///Exit the main loop when esc is pressed.
        if(keypress==27) break;

    }

    ///Close the serial port.
    serial_port.Close();
    return 0;
}

