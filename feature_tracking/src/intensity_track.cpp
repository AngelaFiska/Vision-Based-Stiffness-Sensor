#include "intensity_track.h"

// HSV Color Selection
int  start=0;
float frequency =10;
float tau=0.5;
float sample_time=0;
float alpha=0;



Mat Map1(680,460,CV_16SC2);
Mat Map2(680,460,CV_16UC1);

intensity_track::intensity_track()
    : it(nh)
{

    sub = it.subscribe("/camera/image", 1, &intensity_track::imageCallback,this);

    Force1 = nh.advertise<std_msgs::Float32> ("/Force_roi1",100);
    Force2= nh.advertise<std_msgs::Float32> ("/Force_roi2",100);
    Force3= nh.advertise<std_msgs::Float32> ("/Force_roi3",100);
    Force4= nh.advertise<std_msgs::Float32> ("/Force_roi4",100);
    pub_ks1=nh.advertise<std_msgs::Float32>("ks1",100);
    pub_ks2=nh.advertise<std_msgs::Float32>("ks2",100);
    pub_ks3=nh.advertise<std_msgs::Float32>("ks3",100);
    pub_ks4=nh.advertise<std_msgs::Float32>("ks4",100);
    delta_x1 = nh.advertise<std_msgs::Float32> ("/delta_x1r1",100);
    delta_x2 = nh.advertise<std_msgs::Float32> ("/delta_x2r2",100);
    delta_x3 = nh.advertise<std_msgs::Float32> ("/delta_x3r3",100);
    delta_x4= nh.advertise<std_msgs::Float32> ("/delta_x4r4",100);
    pubi=it.advertise("camera/image_processed",100);
    radius_roi1=nh.advertise<std_msgs::Float32> ("/radius_roi1",100);
    radius_roi2=nh.advertise<std_msgs::Float32> ("/radius_roi2",100);
    radius_roi3=nh.advertise<std_msgs::Float32> ("/radius_roi3",100);
    radius_roi4=nh.advertise<std_msgs::Float32> ("/radius_roi4",100);
    cen1_pub=nh.advertise<geometry_msgs::Point>("/centroid_roi1",100);
    cen2_pub=nh.advertise<geometry_msgs::Point>("/centroid_roi2",100);
    cen3_pub=nh.advertise<geometry_msgs::Point>("/centroid_roi3",100);
    cen4_pub=nh.advertise<geometry_msgs::Point>("/centroid_roi4",100);



    //    stringstream temp;
    //    for (int i=0;i<4;i++) {
    //        temp.str("");
    //        temp << "centroid" << i;

    //        c_pub.push_back(nh.advertise<geometry_msgs::Point>(temp.str(),100));
    //}
}
intensity_track::~intensity_track()
{

    destroyWindow("Original Image");
    destroyWindow("Morphological");
    destroyWindow("Canny");
    destroyWindow("HSV color space");

}



void intensity_track::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("ROSOpenCV::imageCallback::cv_bridge exception: %s", e.what());
        return;
    }

    image_processing();
}


void intensity_track::morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.

    Mat erodeElement = getStructuringElement(MORPH_ELLIPSE,Size(6,6));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_ELLIPSE,Size(6,6));
    //erode+dilate=closing
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    //dilatae + erode =opeing
    dilate(thresh,thresh,dilateElement);
    erode(thresh,thresh,erodeElement);
}


void intensity_track::image_processing(){

    std::string nn;
    nn = ros::this_node::getName();
    ros::NodeHandle nh2(nn.c_str());

    // HSV Color Selection
//    H_MIN = 61;
//    H_MAX = 99;
//    S_MIN = 48;
//    S_MAX = 192;
//    V_MIN = 90;
//    V_MAX = 256;
    H_MIN = 71;
    H_MAX = 84;
    S_MIN = 60;
    S_MAX = 198;
    V_MIN = 81;
    V_MAX = 256;
    //Maximum radius
    MAX_R1=180;
    MAX_R2=180;
    X_0= 18;
    //Springs parameters
    K1=0.190;
    K2=0.040;
    h=0.35;
    p0=sqrt(pow(X_0,2)+pow(h,2));

    //ROS_INFO("P0: %f\n",p0);


    //Camera Matrix
    Mat CamMatrix(3,3,CV_8U);
    CamMatrix.data[0]= 825.154121;
    CamMatrix.data[1]=0.0;
    CamMatrix.data[2]=933.004306;
    CamMatrix.data[3]=0.0;
    CamMatrix.data[4]=807.285607;
    CamMatrix.data[5]=523.664309;
    CamMatrix.data[6]=0.0;
    CamMatrix.data[7]=0.0;
    CamMatrix.data[8]=1.0;

    // Distortion coefficient
    Mat dist_coeff(1,5,CV_8U);
    dist_coeff.data[0]=-0.034300;
    dist_coeff.data[1]=0.015035;
    dist_coeff.data[2]=-0.001289;
    dist_coeff.data[3]=0.001152;
    dist_coeff.data[4]=0.000000;

    Mat undistorted;

    if (start==0)
    {

        initUndistortRectifyMap(CamMatrix,dist_coeff,Mat(),getOptimalNewCameraMatrix(CamMatrix,dist_coeff,cv_ptr->image.size(),1,cv_ptr->image.size(),0),cv_ptr->image.size(),CV_32FC1,Map1,Map2);
        start=1;

    }


    else
    {
        //ROS_INFO("start%d ",start);
        remap(cv_ptr->image,undistorted,Map1,Map2,INTER_LINEAR);

        // From BGR to HSV channel
        cvtColor(undistorted,cameraFeed,CV_BGR2HSV );
        inRange(cameraFeed,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

        // cv::namedWindow("HSV Channel",CV_WINDOW_NORMAL);
        //imshow("HSV Channel",cameraFeed);

        cv::namedWindow("Original Image",CV_WINDOW_NORMAL);
        imshow("Original Image",cv_ptr->image);

        //performs the morphological operations on the thresholded image to eliminate the noise

        GaussianBlur(threshold,threshold,Size(25,25),0,0);
        if(useMorphOps)
            morphOps(threshold);


        // cv::namedWindow("Tracker",CV_WINDOW_NORMAL);
        // imshow("Tracker",threshold);

        threshold.copyTo(temp);
        // cv::namedWindow("After blur",CV_WINDOW_NORMAL);
        //imshow("After blur",threshold);
        //C++: void fastNlMeansDenoising(InputArray src, OutputArray dst, float h=3, int templateWindowSize=7, int searchWindowSize=21 )¶


        cv::Rect roi1(300,0,660,540);
        cv::Rect roi2(960,0,660,540);
        cv::Rect roi3(300,540,660,540);
        cv::Rect roi4(960,540,660,540);

        cv::Mat roiImg1;
        roiImg1 = threshold(roi1);
        //cv::namedWindow("roi1",CV_WINDOW_NORMAL);
        // imshow("roi1",roiImg1);

        cv::Mat roiImg2;
        roiImg2 = threshold(roi2);
        //cv::namedWindow("roi2",CV_WINDOW_NORMAL);
        // imshow("roi2",roiImg2);

        cv::Mat roiImg3;
        roiImg3 = threshold(roi3);
        // cv::namedWindow("roi3",CV_WINDOW_NORMAL);
        // imshow("roi3",roiImg3);

        cv::Mat roiImg4;
        roiImg4 = threshold(roi4);
        //cv::namedWindow("roi4",CV_WINDOW_NORMAL);
        //imshow("roi4",roiImg4);


        /*ROI #1 */
        whites1.resize(0);
        wp1=0;
        float mx1=0;
        float my1=0;

        for(int i=0; i<roiImg1.rows; i++)
        {
            //Go through all the columns
            for(int j=0; j<roiImg1.cols; j++)
            {    //Go through all the channels (b, g, r)
                //Invert the image by subtracting image data from 255
                if((roiImg1.data[i*temp.cols+j]>250)){          //&& ((((float) j-circles[0][0])*((float) j-circles[0][0]))+(((float) i-circles[0][1]*((float) i-circles[0][1]))))<circles[0][2]) {
                    p1.x= j;
                    p1.y= i;
                    mx1+=(float) j;
                    my1+=(float) i;
                    whites1.push_back(p1);
                    wp1++;
                }
            }
        }

        mx1/=wp1;
        my1/=wp1;
        centroid1.x=mx1;
        centroid1.y=my1;

        /*ROI #2 */
        whites2.resize(0);
        wp2=0;
        float mx2=0;
        float my2=0;

        for(int i=0; i<roiImg2.rows; i++)
        {  //Go through all the columns
            for(int j=0; j<roiImg2.cols; j++)
            {   //Go through all the channels (b, g, r)
                //Invert the image by subtracting image data from 255 //&& ((((float) j-circles[0][0])*((float) j-circles[0][0]))+(((float) i-circles[0][1]*((float) i-circles[0][1]))))<circles[0][2]) {
                if((roiImg2.data[i*temp.cols+j]>250)){
                    p2.x= j;
                    p2.y= i;
                    mx2+=(float) j;
                    my2+=(float) i;
                    whites2.push_back(p2);
                    wp2++;
                }
            }
        }

        mx2/=wp2;
        my2/=wp2;
        centroid2.x=mx2;
        centroid2.y=my2;


        /*ROI #3 */
        whites3.resize(0);
        wp3=0;
        float mx3=0;
        float my3=0;
        for(int i=0; i<roiImg3.rows; i++)
        {
            //Go through all the columns
            for(int j=0; j<roiImg3.cols; j++)
            {
                //Go through all the channels (b, g, r)
                //Invert the image by subtracting image data from 255
                if((roiImg3.data[i*temp.cols+j]>250)){//&& ((((float) j-circles[0][0])*((float) j-circles[0][0]))+(((float) i-circles[0][1]*((float) i-circles[0][1]))))<circles[0][2]) {
                    p3.x= j;
                    p3.y= i;
                    mx3+=(float) j;
                    my3+=(float) i;
                    whites3.push_back(p3);
                    wp3++;
                }
            }
        }

        mx3/=wp3;
        my3/=wp3;
        centroid3.x=mx3;
        centroid3.y=my3;

        /*ROI #4*/
        whites4.resize(0);
        wp4=0;
        float mx4=0;
        float my4=0;
        for(int i=0; i<roiImg4.rows; i++)
        {
            //Go through all the columns
            for(int j=0; j<roiImg4.cols; j++)
            {
                //Go through all the channels (b, g, r)
                //Invert the image by subtracting image data from 255
                if((roiImg4.data[i*temp.cols+j]>250)){
                    p4.x= j;
                    p4.y= i;
                    mx4+=(float) j;
                    my4+=(float) i;
                    whites4.push_back(p4);
                    wp4++;
                }
            }
        }
        mx4/=wp4;
        my4/=wp4;
        centroid4.x=mx4;
        centroid4.y=my4;

        try{

            /*Circle in the ROI 1*/
            M1white = Mat( whites1);
            minEnclosingCircle(M1white, cc1, rad1);

            /*Circle in the ROI 2*/
            M2white = Mat( whites2);
            minEnclosingCircle(M2white, cc2, rad2);

            //    vector<Point2f> cm1=compute_moments("roi1", roiImg1);
            //    cout << cm1 << endl;
            //    if(cm1.size()>0){
            //        p_cm1 =cm1.at(0);
            //    }

            //    vector<Point2f> cm2=compute_moments("roi2", roiImg2);

            //    if(cm2.size()>0){
            //        p_cm2 =cm2.at(0);
            //    }

            //    vector<Point2f> cm3=compute_moments("roi3", roiImg3);

            //    if(cm3.size()>0){
            //        p_cm3 =cm3.at(0);
            //    }  waitKey(1);

            //    vector<Point2f> cm4=compute_moments("roi4", roiImg4);

            //    if(cm4.size()>0){
            //        p_cm4 =cm4.at(0);
            //    } /*Circle in the ROI 3*/
            M3white = Mat( whites3);
            minEnclosingCircle(M3white, cc3, rad3);


            /*Circle in the ROI 4*/
            M4white = Mat( whites4);
            minEnclosingCircle(M4white, cc4, rad4);


            //ROS_INFO("Radius first: r1 %f, r2 %f, r3 %f, r4 %f, cc1x %f,cc1y %f:  \n",rad1,rad2,rad3,rad4,cc1.x,cc1.y);


            if(start==1)
            {
                R1_0=rad1;
                R2_0=rad2;
                R3_0=rad3;
                R4_0=rad4;
                start=2;

            }

            // ROS_INFO("Radius filter, centroid: r1 %f, r2 %f, r3 %f, r4 %f ,cc1x %f,cc1y %f:  \n",rad1,rad2,rad3,rad4,cc1.x,cc1.y);






            //    ROS_INFO("Radius after abs 1:%f, 2:%f,3:%f,4:%f",rad1,rad2,rad3,rad4);
            //            ROS_INFO("diff1 %f,diff2 %f,diff3 %f,diff4 %f\n", abs(rad1-R1_0),abs(rad2-R2_0),abs(rad3-R3_0),abs(rad4-R4_0));


            cv::Point center1(cvRound(cc1.x+300), cvRound(cc1.y));
            int radius1 = cvRound(rad1);
            circle( cv_ptr->image, center1, 3,cv::Scalar(125,255,0),-1,8,0);
            circle( cv_ptr->image, center1, radius1,cv::Scalar(0,124,255), 3, 8, 0);

            cv::Point center2(cvRound(cc2.x+960), cvRound(cc2.y));
            int radius2 = cvRound(rad2);
            circle( cv_ptr->image, center2, 3,cv::Scalar(125,255,0),-1,8,0);
            circle( cv_ptr->image, center2, radius2,cv::Scalar(0,124,255), 3, 8, 0);

            cv::Point center3(cvRound(cc3.x+300), cvRound(cc3.y+540));
            int radius3 = cvRound(rad3);
            circle( cv_ptr->image, center3, 3,cv::Scalar(125,255,0),-1,8,0);
            // circle outline
            circle( cv_ptr->image, center3, radius3,cv::Scalar(0,124,255), 3, 8, 0);

            cv::Point center4(cvRound(cc4.x+960), cvRound(cc4.y+540));
            int radius4 = cvRound(rad4);
            circle( cv_ptr->image, center4, 3,cv::Scalar(125,255,0),-1,8,0);
            // circle outline
            circle( cv_ptr->image, center4, radius4,cv::Scalar(0,124,255), 3, 8, 0);

            p11=(R1_0/rad1)*p0;
            p12=(R2_0/rad2)*p0;
            p13=(R3_0/rad3)*p0;
            p14=(R4_0/rad4)*p0;



            deltaX1=(sqrt(pow(p0,2)-pow(h,2))-sqrt(pow(p11,2)-pow(h,2)));
            deltaX2=(sqrt(pow(p0,2)-pow(h,2))-sqrt(pow(p12,2)-pow(h,2)));
            deltaX3=(sqrt(pow(p0,2)-pow(h,2))-sqrt(pow(p13,2)-pow(h,2)));
            deltaX4=(sqrt(pow(p0,2)-pow(h,2))-sqrt(pow(p14,2)-pow(h,2)));

            cen1.x=center1.x;
            cen1.y=center1.y;
            cen2.x=center2.x;
            cen2.y=center2.y;
            cen3.x=center3.x;
            cen3.y=center3.y;
            cen4.x=center4.x;
            cen4.y=center4.y;
            cen1.z=0;
            cen2.z=0;
            cen3.z=0;
            cen4.z=0;
            cen1_pub.publish(cen1);
            cen2_pub.publish(cen2);
            cen3_pub.publish(cen3);
            cen4_pub.publish(cen4);

            mradius1.data=radius1;
            mradius2.data=radius2;
            mradius3.data=radius3;
            mradius4.data=radius4;
            radius_roi1.publish(mradius1);
            radius_roi2.publish(mradius2);
            radius_roi3.publish(mradius3);
            radius_roi4.publish(mradius4);

            ms_delta_x1.data=deltaX1;
            ms_delta_x2.data=deltaX2;
            ms_delta_x3.data=deltaX3;
            ms_delta_x4.data=deltaX4;
            delta_x1.publish(ms_delta_x1);
            delta_x2.publish(ms_delta_x2);
            delta_x3.publish(ms_delta_x3);
            delta_x4.publish(ms_delta_x4);


            // Lines between the center of the images and the sphere's center


            cv::Point imagecenter(960,540);

            cv::line(cv_ptr->image,imagecenter,center1,cv::Scalar(254,0,0), 4, CV_AA);

            cv::line(cv_ptr->image,imagecenter,center2,cv::Scalar(254,0,0), 4, CV_AA);

            cv::line(cv_ptr->image,imagecenter,center3,cv::Scalar(254,0,0), 4, CV_AA);

            cv::line(cv_ptr->image,imagecenter,center4,cv::Scalar(254,0,0), 4, CV_AA);


           // ROS_INFO("Radius plot: r1 %d, r2 %d, r3 %d, r4 %d \n",radius1,radius2,radius3,radius4);


            //forces=(K2*X2_0 * mradius2-K2*R2_0)/mradius2;
            //  forceh=(K1* deltaX1);

            force1=K2*deltaX1;
            force2=K1*deltaX2;
            force3=K1*deltaX3;
            force4=K2*deltaX4;
            //ROS_INFO("FORCE 1: %f\n delta_x1: %f\n R1_0: %f\n rad1: %f\n p0: %f\n p1: %f\n",force1,deltaX1,R1_0,rad1,p0,p11);
            ks1=(force1-force2)/(deltaX1-deltaX2);
            ks2=(force1-force3)/(deltaX1-deltaX3);
            ks3=(force4-force2)/(deltaX4-deltaX2);
            ks4=(force4-force3)/(deltaX4-deltaX3);


            //force1=alpha*force1+((1-alpha)*pre_f1);

            //ROS_INFO("FORCE 1 after: %f\n",force1);
            ms_force1.data=force1;
            ms_force2.data=force2;
            ms_force3.data=force3;
            ms_force4.data=force4;
            Force1.publish(ms_force1);
            Force2.publish(ms_force2);
            Force3.publish(ms_force3);
            Force4.publish(ms_force4);

            ms_ks1.data=ks1;
            ms_ks2.data=ks2;
            ms_ks3.data=ks3;
            ms_ks4.data=ks4;

            pub_ks1.publish(ms_ks1);
            pub_ks2.publish(ms_ks2);
            pub_ks3.publish(ms_ks3);
            pub_ks4.publish(ms_ks4);

        }
        catch(cv::Exception &e){
            ROS_ERROR("cv exception %s", e.what());

        }

        cv::namedWindow( "Original Image", CV_WINDOW_AUTOSIZE );
        imshow("Original Image",cv_ptr->image);
        cv::waitKey(1);
        pubi.publish(cv_ptr->toImageMsg());

        //geometry_msgs::Point c1;
        //c1.x=cc1.x;
        //c1.y=cc1.y;
        //c_pub.at(0).publish(c1);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracking");
    intensity_track object;
    ros::Rate loop_rate(40); //40 Hz

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
