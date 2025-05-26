#include "my_tracking.h"

int  start=0;
Mat Map1(640,480,CV_16SC2);
Mat Map2(640,480,CV_16UC1);
double frequency=10;

my_tracking::my_tracking()
    : it(nh)
{
    sub = it.subscribe("/image_raw", 1, &my_tracking::imageCallback,this);
    pub = nh.advertise<std_msgs::Float32> ("/my_radius",100);
    MatForce = nh.advertise<std_msgs::Float32> ("/Force_MatMod",100);
    delta_x = nh.advertise<std_msgs::Float32> ("/delta_x",100);
    pubi=it.advertise("camera/image",100);


}
my_tracking::~my_tracking()
{

    destroyWindow("Original Image");
    destroyWindow("Morphological");
    destroyWindow("Canny");
    destroyWindow("HSV color space");

}



void my_tracking::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    image_processing();
}


void my_tracking::morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_ELLIPSE,Size(5,5));

    //erode+dilate=closing
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    //dilatae + erode =opeing
    dilate(thresh,thresh,dilateElement);
    erode(thresh,thresh,erodeElement);
}


void my_tracking::image_processing(){

    std::string nn;
    nn = ros::this_node::getName();
    ros::NodeHandle nh2(nn.c_str());

    //Get all the parameters from the yaml file
    nh2.getParam("param/H_MAX",H_MAX);
    nh2.getParam("param/H_MIN",H_MIN);
    nh2.getParam("param/S_MAX",S_MAX);
    nh2.getParam("param/S_MIN",S_MIN);
    nh2.getParam("param/V_MAX",V_MAX);
    nh2.getParam("param/V_MIN",V_MIN);
    nh2.getParam("position/POS",POS);
    nh2.getParam("canny/LW_TH", lower_th);
    nh2.getParam("canny/UP_TH", upper_th);
    nh2.getParam("hough/HOUGH", disp_hough);
    nh2.getParam("sensor_param/R_0", R_0);
    nh2.getParam("sensor_param/X_0", X_0);
    nh2.getParam("sensor_param/MAX_R", MAX_R);
    nh2.getParam("sensor_param/K", K);
    nh2.getParam("calibration/camera_matrix",cam);
    nh2.getParam("calibration/distortion_coefficients",dist);

   vect_cam.clear();

   for(int i=0; i<9;i++)
   vect_cam.push_back(cam[i]);


 //Camera Calibration Matrix
    Mat CamMatrix(3,3,CV_8U);
    CamMatrix.data[0]=vect_cam.at(0);
    CamMatrix.data[1]=vect_cam.at(1);
    CamMatrix.data[2]=vect_cam.at(2);
    CamMatrix.data[3]=vect_cam.at(3);
    CamMatrix.data[4]=vect_cam.at(4);
    CamMatrix.data[5]=vect_cam.at(5);
    CamMatrix.data[6]=vect_cam.at(6);
    CamMatrix.data[7]=vect_cam.at(7);
    CamMatrix.data[8]=vect_cam.at(8);


   vect_dist.clear();
   for(int i=0;i<5;i++)
       vect_dist.push_back(static_cast<double>(dist[i]));

    // Distortion coefficients vector
    Mat dist_coeff(1,5,CV_64F);
    dist_coeff.data[0]=vect_dist.at(0);
    dist_coeff.data[1]=vect_dist.at(1);
    dist_coeff.data[2]=vect_dist.at(2);
    dist_coeff.data[3]=vect_dist.at(3);
    dist_coeff.data[4]=vect_dist.at(4);


    Mat undistorted;

    if (start==0) // Compute the maps to undistort the image (using only the first frame)
    {
        initUndistortRectifyMap(CamMatrix,dist_coeff,Mat(),getOptimalNewCameraMatrix(CamMatrix,dist_coeff,cv_ptr->image.size(),1,cv_ptr->image.size(),0),cv_ptr->image.size(),CV_32FC1,Map1,Map2);

        start=1;
    }

    else
    {

    remap(cv_ptr->image,undistorted,Map1,Map2,INTER_LINEAR);

    // From BGR to HSV channel
    cvtColor(undistorted,cameraFeed,CV_BGR2HSV );
    useMorphOps = true;

    //Colors Selection in the HSV channel
    inRange(cameraFeed,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

     //imshow("remap",threshold);
    //performs the morphological operations on the thresholded image to eliminate the noise
    if(useMorphOps)
        morphOps(threshold);

    //imshow("Morphological",threshold);
    threshold.copyTo(temp);
    Mat gray_im;


    if(disp_hough==true) //Display the hough circle
    {
        cvtColor( cameraFeed, gray_im, CV_BGR2GRAY );
        Canny(gray_im,canny,lower_th,upper_th);

        //Perform Gaussian Blur to eliminate the noise
        GaussianBlur(gray_im,gray_im,Size(3,3),0,0);

        cv::namedWindow( "Canny", CV_WINDOW_AUTOSIZE );
        imshow("Canny",temp);

        // Apply the Hough Transform to find the circles
        HoughCircles( gray_im, circles, CV_HOUGH_GRADIENT,1,temp.cols,upper_th,5,40,0);
        ROS_INFO("size circle %d ", circles.size());
        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            ROS_INFO("radius: %f\n",circles[i][2]);
            // circle center
            circle( cv_ptr->image, center, 3,cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( cv_ptr->image, center, radius,cv::Scalar(0,0,255), 3, 8, 0 );
        }
    }

    whites.resize(0);
    wp=0;

    float mx=0;
    float my=0;
    GaussianBlur(threshold,threshold,Size(23,23),0,0);
    imshow("After blur",threshold);
    //cv::moveWindow("After blur",720,550);
    //cv:: moveWindow("After blur",1500,550);

    for(int i=0; i<threshold.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<threshold.cols; j++)
        {
            //Go through all the channels (b, g, r)
            //Invert the image by subtracting image data from 255
            if((threshold.data[i*temp.cols+j]>250)){//&& ((((float) j-circles[0][0])*((float) j-circles[0][0]))+(((float) i-circles[0][1]*((float) i-circles[0][1]))))<circles[0][2]) {

                //if (i%10==0 && j%10 ==0) ROS_INFO(" %d %d %d. \n",i,j,threshold.data[i*threshold.cols+j]);

                //printf(" %d %d %d. \n",i,j,temp.data[i*temp.cols+j]);

                p.x= j;
                p.y= i;

                mx+=(float) j;
                my+=(float) i;

                whites.push_back(p);
                wp++;

            }

        }

    }

    mx/=wp;
    my/=wp;
    centroid.x=mx;
    centroid.y=my;

    try{

        Mwhite = Mat( whites);
        minEnclosingCircle(Mwhite, cc, rad);
        //printf("CCC: %f %f %f %d %d\n",p.x,p.y,rad,wp,whites.size());
        cv::Point center2(cvRound(cc.x), cvRound(cc.y));
        int radius2 = cvRound(rad);
        if(radius2>MAX_R)
            radius2=MAX_R;
        //if(abs(radius2-R_0)<3)
         //   radius2=R_0;
        circle( cv_ptr->image, center2, 3,cv::Scalar(125,255,0),-1,8,0);
        // circle outline
        circle( cv_ptr->image, center2, radius2,cv::Scalar(0,124,255), 3, 8, 0);

        deltaX=X_0*(1 -(R_0/rad));
        if(deltaX<0)
            deltaX=0;
        ms_delta_x.data=deltaX;
        //ROS_INFO("deltatrack%f: \n", ms_delta_x.data);
        delta_x.publish(ms_delta_x);
        // force=(K*X_0 *radius2-K*R_0)/radius2;
        force=(K* deltaX);
        ms_force.data=force;
        MatForce.publish(ms_force);
        msg.data=radius2;
        pub.publish(msg);
    }
    catch(cv::Exception &e){
        ROS_ERROR("cv exception %s", e.what());

    }



    cv::namedWindow( "Original Image", CV_WINDOW_AUTOSIZE );
    imshow("Original Image",cv_ptr->image);
    // cv::moveWindow("Original Image",0,0);
    //cv::namedWindow( "HSV color space", CV_WINDOW_AUTOSIZE );
    //imshow("HSV color space",cameraFeed);
    // cv::moveWindow("HSV color space",0,550);
    // cv::namedWindow( "Morphological", CV_WINDOW_AUTOSIZE );
    //cv::moveWindow("Morphological",720,550);
    //  cv::moveWindow("Morphological",720,0);

    cv::waitKey(30);
    pubi.publish(cv_ptr->toImageMsg());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracking");

        my_tracking object;
        ros::spin();

}
