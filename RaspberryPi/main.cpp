#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ctime>
#include <cstdio>
#include <cstring>

using namespace std;
using namespace cv;

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

typedef struct
{
    int left;
    int right;
    int top;
    int bottom;
}SimpleRect;

Point DetectGoals(vector< vector< Point > > contours);
int ConnectToCRio(string port);
string ConvertIntsToString(int x, int y);

addrinfo* roboInfo;

int main()
{
    Mat img, hsv;
    VideoCapture cap("http://10.17.14.11/mjpg/video.mjpg");
    vector< vector< Point > > contours, filteredContours;
    vector<Vec4i> hierarchy;
    namedWindow("IMG", CV_WINDOW_AUTOSIZE);
    namedWindow("HSV", CV_WINDOW_AUTOSIZE);
    int sock = ConnectToCRio("17140");
    int xPos, yPos;
    float lastSent = clock()/CLOCKS_PER_SEC;

    while(true)
    {
        filteredContours.clear();
        //Get the newest image
        cap>>img;
        if(img.empty())cout<<"no\n";

        //Blur the image and Convery it to hsv
        GaussianBlur(img, img, Size(3, 3), 0, 0);
        cvtColor(img, hsv, CV_BGR2HSV);

        //Detect the goals in range
        //inRange(hsv, Scalar(0, 150, 120), Scalar(15, 190, 160), hsv);
        inRange(hsv, Scalar(60, 175, 45), Scalar(90, 255, 175), hsv);

        //find the contours
        findContours(hsv, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        //Determine which of the found contours are good
        for(int i = 0; i < (int)contours.size(); i++)
        {
            vector<Point> simpleContour;
            Rect rect;
            approxPolyDP(contours[i], simpleContour, 5, true);
            double area = contourArea(simpleContour);
            rect = boundingRect(simpleContour);
            if((rect.width > rect.height) && (simpleContour.size() <= 4) /*&& (area >= 20)*/)
            {
                cout<<area<<endl;
                filteredContours.push_back(simpleContour);
            }
        }

        drawContours(hsv, filteredContours, -1, Scalar(255, 255, 255), CV_FILLED);

        imshow("IMG", img);
        imshow("HSV", hsv);

        xPos = DetectGoals(filteredContours).x;
        yPos = DetectGoals(filteredContours).y;

        //Seeend the data to the CRio
        float curTime = clock()/CLOCKS_PER_SEC;
        if(abs(curTime - lastSent) > 0.1)//Control how often packets are sent
        {
            //Convert the ints to strings
            string msg;
            msg = ConvertIntsToString(xPos, yPos);

            if(sendto(sock, msg.c_str(), msg.length(), 0, roboInfo->ai_addr, roboInfo->ai_addrlen) == -1)
            {
                cout<<"Bad send to\n";
            }
        }

        //cout<<xPos<<" "<<yPos<<endl;
        if(waitKey(1)>=0)break;
    }
}

//Find the closest goal and say where it is
Point DetectGoals(vector< vector< Point > > contours)
{
    int closestGoal = -1;
    int closestGoalDist = 10000000;
    SimpleRect simpleGoalRect;
    Point goalDir;

    for(int i = 0; i < (int)contours.size(); i++)
    {
        SimpleRect simpleRect;
        Rect rect = boundingRect(contours[i]);
        Point midpoint;
        float distance;

        //Convert the rect into the simplerect and have 0,0 be the center of the screen
        simpleRect.left = (rect.x)-(IMG_WIDTH/2);
        simpleRect.right = (rect.x+rect.width)-(IMG_WIDTH/2);
        simpleRect.top = (rect.y)-(IMG_HEIGHT/2);
        simpleRect.bottom = (rect.y+rect.height)-(IMG_HEIGHT/2);

        //find the midpoint
        midpoint.x = (simpleRect.left + simpleRect.right)/2;
        midpoint.y = (simpleRect.top + simpleRect.bottom)/2;

        //calculate the distance
        distance = sqrt(pow(midpoint.x, 2)+ pow(midpoint.y, 2));
        if(distance < closestGoalDist)
        {
            closestGoalDist = distance;
            closestGoal = i;
            simpleGoalRect = simpleRect;
        }
    }
    //find out where the goal is elative to the center of the image
    //if closestGoal = -1 then no goal was found
    if(closestGoal == -1)
    {
        return Point(-2, -2);
    }

    //Check if it's left or right
    if(simpleGoalRect.left > 0)//If it's right
    {
        goalDir.x = 1;
    }
    else if(simpleGoalRect.right < 0)//If it;s left
    {
        goalDir.x = -1;
    }
    else //If it's centered horizontally
    {
        goalDir.x = 0;
    }

    //Check to see if the goal is up or down
    if(simpleGoalRect.top > 0)//If it's down
    {
        goalDir.y = 1;
    }
    else if(simpleGoalRect.bottom < 0)//If it's up
    {
        goalDir.y = -1;
    }
    else //If it's centered vertically
    {
        goalDir.y = 0;
    }

    return goalDir;
}

int ConnectToCRio(string port)
{
    int status;
    int sock;
    addrinfo hints;

    //Clear the hints struct
    memset(&hints, 0, sizeof(hints));

    //Set the values for the hints struct
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    //Get the information of the CRio
    status = getaddrinfo("10.17.14.2", port.c_str(), &hints, &roboInfo);
    if(status != 0)
    {
        cout<<gai_strerror(status)<<endl;
        cin.get();
        return -1;
    }

    //Create the socket
    sock = socket(roboInfo->ai_family, roboInfo->ai_socktype, roboInfo->ai_protocol);
    if(sock == -1)
    {
        cout<<"Error creating socket\n";
        cin.get();
        return -1;
    }

    return sock;
}

string ConvertIntsToString(int x, int y)
{
    string msg;

    if(x == -1)
    {
        msg += '-';
        msg += '1';
    }
    else if(x == 1)
    {
        msg += '+';
        msg += '1';
    }
    else if(x == 0)
    {
        msg +='0';
        msg += '0';
    }
    else
    {
        msg += '-';
        msg += '2';
    }

    if(y == -1)
    {
        msg += '-';
        msg += '1';
    }
    else if(y == 1)
    {
        msg += '+';
        msg += '1';
    }
    else if(y == 0)
    {
        msg += '0';
        msg += '0';
    }
    else
    {
        msg += '-';
        msg += '2';
    }

    return msg;
}
