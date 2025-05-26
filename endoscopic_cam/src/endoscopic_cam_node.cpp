#include <endoscopic_cam/endoscopic_cam_node.h>
int start=0;
Img_Proc::Img_Proc(int in_roi, Params ipar, Point3d ipos, Point2d icen )
    : n_roi( in_roi),pos(ipos), previous_centroid(icen) {
    par=ipar;
    window.resize(100);
    mov_av_i=0;
    kalman_centroid.resize(4);
    x.resize(4);
    y.resize(4);
    p.resize(4);
    par.rois_size.resize(2);
    par.rois_p.resize(8);
}

void Img_Proc::init_Kalman(double process_noise, double sensor_noise, double estimated_error,double initial_value_x,double initial_value_y, int n)
{
    this->q = process_noise;
    this->r = sensor_noise;
    this->p.at(n) = estimated_error;
    this->x.at(n) = initial_value_x;
    this->y.at(n) = initial_value_y;
}

double Img_Proc::getFilteredValue_x(double measurement_x, int n)
{
    //prediction update--time update
    this->p.at(n) = this->p.at(n) + this->q;

    //measurement update -- correction
    if(!isnan(this->p.at(n) + this->r) )
        this->k = this->p.at(n) / (this->p.at(n) + this->r);
    if(!isnan(this->k * (measurement_x - this->x.at(n))))
        this->x.at(n) = this->x.at(n) + this->k * (measurement_x - this->x.at(n));
    this->p.at(n) = (1 - this->k) * this->p.at(n);

    return this->x.at(n);
}

double Img_Proc::getFilteredValue_y(double measurement_y, int n) {

    //prediction update--time update
    this->p.at(n) = this->p.at(n) + this->q;

    //measurement update -- correction
    if(!isnan(this->p.at(n) + this->r) )
        this->k = this->p.at(n) / (this->p.at(n) + this->r);
    if(!isnan(this->k * (measurement_y - this->y.at(n))))
        this->y.at(n) = this->y.at(n) + this->k * (measurement_y - this->y.at(n));
    this->p.at(n) = (1 - this->k) * this->p.at(n);

    return this->y.at(n);
}


Point2d Img_Proc::computeCentroid(cv::Mat input_im, int n, filt_config filt){

    Mat cameraFeed,threshold,temp;
    cvtColor(input_im,cameraFeed,CV_BGR2HSV );
    //imshow("HSV Channel",cameraFeed);

    //Colors Selection in the HSV channel
    inRange(cameraFeed,Scalar(par.H_MIN,par.S_MIN,par.V_MIN),Scalar(par.H_MAX,par.S_MAX,par.V_MAX),threshold);
    //imshow("IN Range",threshold);

    //performs the morphological operations on the thresholded image to eliminate the noise
    if(par.useMorphOps)
        morphOps(threshold);

    imshow("Morphological",threshold);
    threshold.copyTo(temp);

    cv::Rect roi(this->par.rois_p.at(this->n_roi),this->par.rois_p.at(this->n_roi+1),this->par.rois_size.at(0),this->par.rois_size.at(1));
    cv::Mat roiImg1,roiImg;
    roiImg1=input_im(roi);
    roiImg= threshold(roi);

    std::ostringstream ss;

    ss << "roi"<< n+1;
    string out=ss.str();
    imshow(out,roiImg1);

    float m01=0;
    float m00=0;
    float m10=0;

    for(int i=0; i<roiImg.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<roiImg.cols; j++)
        {
            if((roiImg.data[i*temp.cols+j]>250)){
                m00+= roiImg.data[i*temp.cols+j];
                m10+= j * roiImg.data[i*temp.cols+j];
                m01+= i * roiImg.data[i*temp.cols+j];
            }
        }
    }

    Point2d centr;//raw centroid
    centr.x= m10/m00;
    centr.y= m01/m00;

    raw_centroid=centr;

    /************ low Pass Filter ************/
    if(isnan(this->previous_centroid.x)|| isnan(this->previous_centroid.y) ){
        centroid_filtered=centr;
    }

    else{
        centroid_filtered.x = filt.alpha *centr.x +(1-filt.alpha)*this->previous_centroid.x;
        centroid_filtered.y = filt.alpha *centr.y +(1-filt.alpha)*this->previous_centroid.y;
        after_low_pass=centroid_filtered;
    }

    this->previous_centroid.x=centroid_filtered.x;
    this->previous_centroid.y=centroid_filtered.y;


    /************ kalman filter ************/
    if(start<4 ){
        init_Kalman(filt.R,filt.P,filt.Q,centr.x,centr.y, n);
        start++;
    }

    kalman_x.x=  getFilteredValue_x(centr.x,n);
    kalman_x.y=  getFilteredValue_y(centr.y,n);
    if(!isnan(this->kalman_x.y)&& !isnan(this-> kalman_x.x) )
        this->x.at(n)=kalman_x.x;this->y.at(n)=kalman_x.y;
    // ROS_INFO("XXX %f  n:%d",this->x.at(n),n);
    after_kalman=kalman_x;

    /*********** moving window ************/
    window.at(mov_av_i%filt.N)=centr;
    double s_x=0,s_y=0;
    for(int j=0;j<filt.N;j++){
        s_x+=window.at(j).x;
        s_y+=window.at(j).y;
    }

    centroid_filtered.x=((3*centr.x)+s_x)/(filt.N+3);
    centroid_filtered.y=((3*centr.y)+s_y)/(filt.N+3);
    mov_av_i++;

    centroid_filtered=after_kalman;
    cv::Point centroid_f(cvRound(centroid_filtered.x+30), cvRound(centroid_filtered.y+this->ROI.y));

    //ROS_INFO("ROI: %d, C_x: %d, C_y: %d", n+1, cvRound(centr.x+30), cvRound(centr.y+this->ROI_y));
    circle(input_im, centroid_f, 1,cv::Scalar(51,0,102),2,8,0);

    waitKey(1);

    return centroid_filtered;
}


Point3d  Img_Proc ::calculate_3D_point(Point2d point){
    pos.x=point.x/par.resolution;
    return pos;
}


void Img_Proc::morphOps(Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement(MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(5,5));
    //erode+dilate=closing
    erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    //dilatae + erode =opeing
    dilate(thresh,thresh,dilateElement);
    erode(thresh,thresh,erodeElement);
}

Endoscopic_Cam_Node::Endoscopic_Cam_Node()
    :it(nh)
{

    sub = it.subscribe("/image_raw", 1, &Endoscopic_Cam_Node::imageCallback,this);
     pub = nh.advertise<endoscopic_cam::Endo_Feat_Locations>("features", 10);
    pub_centroids=nh.advertise<std_msgs::Float32MultiArray>("centroids",10);
    f = boost::bind(&Endoscopic_Cam_Node::DynamConfcallback,this, _1, _2);
    server.setCallback(f);

}
Endoscopic_Cam_Node::~Endoscopic_Cam_Node()
{

    destroyWindow("Input Image");
    destroyWindow("Morphological");
    destroyWindow("Canny");
    destroyWindow("HSV color space");

}


void Endoscopic_Cam_Node::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

   image_processing(cv_ptr->image);
}




Params Endoscopic_Cam_Node::load_parameter(std::string color){
    Params p;
    bool load;
    load=nh.getParam(color +"_param/H_MAX",p.H_MAX) &&
            nh.getParam(color +"_param/H_MIN",p.H_MIN) &&
            nh.getParam(color +"_param/S_MAX",p.S_MAX) &&
            nh.getParam(color +"_param/S_MIN",p.S_MIN) &&
            nh.getParam(color +"_param/V_MAX",p.V_MAX) &&
            nh.getParam(color +"_param/V_MIN",p.V_MIN) &&
            nh.getParam("options/useMorphOps", p.useMorphOps) &&
            nh.getParam("sensor_param/resolution", p.resolution)&&
            nh.getParam("ROIS/roi_points",p.rois_p)&&
            nh.getParam("ROIS/roi_size", p.rois_size);
    //nh.getParam("options/alpha", p.alpha);
    if(!load){
        ROS_ERROR("Parameters not loaded properly");
        exit(-1);
    }

    return p;
}

void Endoscopic_Cam_Node::image_processing(cv::Mat input_img){

    vector<Point2d> centroids;
    vector<Point3d> points;
    endoscopic_cam::Endo_Feat_Locations fl;
    geometry_msgs::Point p;

    std_msgs::Float32MultiArray raw_centroids;

    for(int i=0; i< features.size(); i++){

        centroids.push_back((features.at(i).computeCentroid(input_img,i,filt_cfg)));
        points.push_back(features.at(i).calculate_3D_point(centroids.at(i)));

        if (isnan(points.at(i).x) || isnan(points.at(i).y) || isnan(points.at(i).z)){
            p.x=0;
            p.y=0;
            p.z=0;

            fl.points.push_back(p);
        }
        else{
            p.x=points.at(i).x;
            p.y=points.at(i).y;
            p.z=points.at(i).z;
            fl.points.push_back(p);
        }
        raw_centroids.data.push_back(features.at(i).raw_centroid.x);
        raw_centroids.data.push_back(features.at(i).raw_centroid.y);
        raw_centroids.data.push_back(features.at(i).after_low_pass.x);
        raw_centroids.data.push_back(features.at(i).after_low_pass.y);
        raw_centroids.data.push_back(features.at(i).after_kalman.x);
        raw_centroids.data.push_back(features.at(i).after_kalman.y);
        raw_centroids.data.push_back(centroids.back().x);
        raw_centroids.data.push_back(centroids.back().y);
    }
    fl.header.stamp=ros::Time::now();
    fl.header.frame_id="indenter";
    pub.publish(fl);
    pub_centroids.publish(raw_centroids);

    //ROS_INFO_STREAM_THROTTLE(0.1,points << endl);
    imshow("Input image", input_img);


}

void Endoscopic_Cam_Node::DynamConfcallback(endoscopic_cam::endoscopic_dvarConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %f %f ",
             config.R,config.Q);

    filt_cfg.Q=config.Q;
    filt_cfg.R=config.R;
    filt_cfg.P=config.P;
    filt_cfg.alpha=config.alpha;
    filt_cfg.N=config.N;

}

void Endoscopic_Cam_Node::initialize()
{
    Size sz(400,200);
    Point3d p;
    Point2d r,c1;
    r.x=0;
    r.y=0;
    c1.x=0;
    c1.y=0;
    p.y = 0;
    p.z = 0;
    features.push_back(Img_Proc(0,load_parameter("green"),p,c1));

    c1.x=0;
    c1.y=189;
    p.y = 8.82;
    p.z = 5.12;
    r.x=0;
    r.y=200;
    features.push_back(Img_Proc(2,load_parameter("green"),p,c1));

    c1.x=247;
    c1.y=261;
    p.y = 8.82;
    p.z = 15.37;
    r.x=200;
    r.y=0;
    features.push_back(Img_Proc(4,load_parameter("green"),p,c1));

    c1.x=254;
    c1.y=381;
    p.y = 17.75;
    p.z = 0;
    r.x=200;
    r.y=200;
    features.push_back(Img_Proc(6,load_parameter("green"),p,c1));

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "endoscopic_camera");
    Endoscopic_Cam_Node object;
    object.initialize();
    ros::Rate loop_rate(40); // 40 Hz
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
