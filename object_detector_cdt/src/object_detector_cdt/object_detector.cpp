#include <object_detector_cdt/object_detector.h>
#include <opencv2/imgproc.hpp>

struct CompDouble {
  bool operator() (double i,double j) { return (i<j);}
} compareDoubles;


ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);
    lidar_sub_ = nh.subscribe(input_lidar_topic_,10, &ObjectDetector::lidarCallback, this);
    
    // Setup publisher
    objects_pub_ = nh.advertise<cdt_msgs::ObjectList>(output_objects_topic_, 10);

    // Extrinsic calibration. This must be updated accordingly
    camera_extrinsic_x_ = 0.2;
    camera_extrinsic_y_ = 0.2;
    camera_extrinsic_z_ = 0.0;

    // Intrinsic calibration
    camera_fx_ = 381.3;
    camera_fy_ = 381.3;
    camera_cx_ = 320.5;
    camera_cy_ = 240.5;

    // Real heights of objects
    barrel_real_height_     = 1.2;   // meters 
    barrow_real_height_     = 0.7;   // meters, note: includes the wheel and frame 
    computer_real_height_   = 0.5;   // meters 
    dog_real_height_        = 0.418; // meters, note: includes legs 
    angle_margin_ = 2*M_PI/180;
}

bool ObjectDetector::getObjectPosition(const float &pixelx, const float &pixely, const std_msgs::Header &imgheader, double &x_out, double &y_out, double &z_out)
{
    sensor_msgs::PointCloud2 pc2_msg_lidar, pc2_msg_cam;

    if (!findClosestLidarScan(imgheader.stamp,pc2_msg_lidar))
    {
        return false;
    }

    // Transform point cloud in camera frame
    try
    {
        pcl_ros::transformPointCloud(image_frame_, pc2_msg_lidar, pc2_msg_cam, tf_listener_);

    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    
    // convert point cloud to PCL
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(pc2_msg_cam, pcl_pc);

    // hack for now
    pcl::PointCloud<pcl::PointXYZ> pcl_xyz;
    pcl::fromPCLPointCloud2(pcl_pc, pcl_xyz);

    double hor_angle = atan((pixelx-camera_cx_)/camera_fx_);
    double vert_angle = atan((pixely-camera_cy_)/camera_fy_);

    double x,y,z, curr_hor_angle, curr_vert_angle;
    std::vector<double> all_x,all_y,all_z;

    for (int i; i<pcl_xyz.points.size(); i++)
    {
        x = pcl_xyz.points[i].x;
        y = pcl_xyz.points[i].y;
        z = pcl_xyz.points[i].z;

        if (z<0.0)
        {
            continue;
        }
        curr_hor_angle = atan2(x,z);
        curr_vert_angle = atan2(y,z);

        if ((abs(curr_hor_angle- hor_angle)<angle_margin_) && (abs(curr_vert_angle - vert_angle)< angle_margin_))
        {
            all_x.push_back(x);
            all_y.push_back(y);
            all_z.push_back(z);
            // ROS_INFO("Angles %f %f", curr_hor_angle, curr_vert_angle);
            // ROS_INFO("Pos %f %f %f", x, y, z);
        }
    }


    if (all_x.size() > 0)
    {
        std::sort (all_x.begin(), all_x.end(), compareDoubles);  
        std::sort (all_y.begin(), all_y.end(), compareDoubles);  
        std::sort (all_z.begin(), all_z.end(), compareDoubles);  

        tf::Point pos_cam( all_x.at(all_x.size()/2),all_y.at(all_y.size()/2),all_z.at(all_z.size()/2));
        tf::Stamped<tf::Point> pos_cam_stamped(pos_cam,imgheader.stamp,image_frame_);
        tf::Stamped<tf::Point> pos_fixed_stamped;

        // ROS_ERROR("Angles %f %f",hor_angle, vert_angle);

        // ROS_ERROR("Nb inliers %d",all_x.size());
        // ROS_ERROR("Pixel val tf %f %f", pixelx, pixely);
        // ROS_ERROR("Before tf %f %f %f", pos_cam_stamped.getX(), pos_cam_stamped.getY(), pos_cam_stamped.getZ());
        tf_listener_.waitForTransform(fixed_frame_, image_frame_,  ros::Time(0), ros::Duration(0.5));
            try
        {
            tf_listener_.transformPoint(fixed_frame_,pos_cam_stamped,pos_fixed_stamped);

        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }


        x_out= pos_fixed_stamped.getX();
        y_out= pos_fixed_stamped.getY();
        z_out= pos_fixed_stamped.getZ();
        // ROS_INFO("%f %f %f", x_out, y_out, z_out);
        return true;
    }
    else
    {
        return false;
    }
}

void ObjectDetector::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_image_topic", input_image_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }

    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }   

    if (!nh.getParam("input_lidar_topic", input_lidar_topic_))
    {
        ROS_ERROR("Could not read parameter `input_lidar_topic`.");
        exit(-1);
    }

    if (!nh.getParam("lidar_scans_kept", nb_scans_kept_))
    {
        ROS_ERROR("Could not read parameter `lidar_scans_kept`.");
        exit(-1);
    }   
    if (!nh.getParam("image_frame", image_frame_))
    {
        ROS_ERROR("Could not read parameter `image_frame`.");
        exit(-1);
    }   


    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

bool ObjectDetector::findClosestLidarScan(const ros::Time &time_query, sensor_msgs::PointCloud2 &point_cloud)
{
    double smallest_time_diff = 0.1;
    double curr_time_diff;
    unsigned best_elem_idx = -1;
    for (unsigned i=0; i<recent_lidar_scans_.size(); i++){
    // for (auto it = recent_lidar_scans_.begin(); it != recent_lidar_scans_.end(); ++it) {
        curr_time_diff = abs((time_query - recent_lidar_scans_[i].header.stamp).toSec());
        if (curr_time_diff < smallest_time_diff)
        {
            best_elem_idx = i;
            smallest_time_diff = curr_time_diff;
        }
        else
        {
            break;
        }
    }
    // No scan found
    if (best_elem_idx == -1)
    {
        return false;
    }
    // ROS_ERROR("Index %d", best_elem_idx);
    // ROS_ERROR("Size %d", recent_lidar_scans_.size());
    sensor_msgs::PointCloud2 out(recent_lidar_scans_[best_elem_idx]);
    // point_cloud = ();
    recent_lidar_scans_.erase(recent_lidar_scans_.begin()+best_elem_idx,recent_lidar_scans_.end());
    point_cloud = out;
    return true;
}

void ObjectDetector::lidarCallback(const sensor_msgs::PointCloud2 in_msg)
{
    recent_lidar_scans_.insert(recent_lidar_scans_.begin(),in_msg);
    if (recent_lidar_scans_.size() > nb_scans_kept_){
        recent_lidar_scans_.pop_back();
    }
    // ROS_INFO("Adding to LIDAR queue");
    // ROS_INFO("Queue size %d", recent_lidar_scans_.size());
    // ROS_INFO("First element timestamp %f", recent_lidar_scans_[0].header.stamp.toSec());
    // ROS_INFO("Last element timestamp %f", recent_lidar_scans_[recent_lidar_scans_.size()-1].header.stamp.toSec());
}


void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    double x_out, y_out, z_out;
    // ROS_INFO(in_msg->header.frame_id);
    std_msgs::Header h = in_msg->header;

    ROS_DEBUG("New image received!");

    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    if(!wasObjectDetected("dog"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeObject(image, Colour::RED, in_msg->header,  new_object.position.x,  new_object.position.y,  new_object.position.z);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            new_object.id = "dog";
            new_object.header.stamp = timestamp;
            new_object.header.frame_id = fixed_frame_;
            detected_objects_.objects.push_back(new_object);
            ROS_INFO("Found a dog!");
        }
    }
    if(!wasObjectDetected("barrow"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeObject(image, Colour::GREEN, in_msg->header,  new_object.position.x,  new_object.position.y,  new_object.position.z);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            new_object.id = "barrow";
            new_object.header.stamp = timestamp;
            new_object.header.frame_id = fixed_frame_;
            detected_objects_.objects.push_back(new_object);
            ROS_INFO("Found a barrow!");
        }
    }
    if(!wasObjectDetected("barrel"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeObject(image, Colour::YELLOW, in_msg->header,  new_object.position.x,  new_object.position.y,  new_object.position.z);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            new_object.id = "barrel";
            new_object.header.stamp = timestamp;
            new_object.header.frame_id = fixed_frame_;
            detected_objects_.objects.push_back(new_object);
            ROS_INFO("Found a barrel!");
        }
    }
    if(!wasObjectDetected("computer"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeObject(image, Colour::BLUE, in_msg->header,  new_object.position.x,  new_object.position.y,  new_object.position.z);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            new_object.id = "computer";
            new_object.header.stamp = timestamp;
            new_object.header.frame_id = fixed_frame_;
            detected_objects_.objects.push_back(new_object);
            ROS_INFO("Found a computer!");
        }
    }


    // Publish list of objects detected so far
    objects_pub_.publish(detected_objects_);
}

void ObjectDetector::convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat &out_image, ros::Time &out_timestamp)
{
    // Convert Image message to cv::Mat using cv_bridge
    out_image = cv_bridge::toCvShare(in_msg, "bgr8")->image;

    // Extract timestamp from header
    out_timestamp = in_msg->header.stamp;
}

cv::Mat ObjectDetector::applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour)
{
    assert(in_image_bgr.type() == CV_8UC3);
    // new image
    cv::Mat in_image_hsv;
    cv::cvtColor(in_image_bgr, in_image_hsv, cv::COLOR_BGR2HSV);
    double limit = .01;
    // Here you should apply some binary threhsolds on the image to detect the colors
    // The output should be a binary mask indicating where the object of a given color is located
    cv::Mat mask;
    // cv::imwrite("hsv.png", in_image_hsv);
    if (colour == Colour::RED) {
        cv::Mat mask2;
        inRange(in_image_hsv, cv::Scalar(  0./360.*255,  30./100.*255,  17./100.*255), cv::Scalar( 36./360.*255, 100./100.*255, 100./100.*255), mask);
        inRange(in_image_hsv, cv::Scalar(  320./360.*255,  30./100.*255,  17./100.*255), cv::Scalar( 359./360.*255, 100./100.*255, 100./100.*255), mask2);
        mask = mask | mask2;
    } else if (colour == Colour::YELLOW) {
        // inRange(in_image_hsv, cv::Scalar(  28.,  194,  30), cv::Scalar( 68., 255, 255), mask);
        // inRange(in_image_hsv, cv::Scalar(  47./360.*255,  15./100.*255,  15./100.*255), cv::Scalar( 78./360.*255, 100./100.*255, 100./100.*255), mask);
        inRange(in_image_hsv, cv::Scalar(  47./2,  15./100.*255,  15./100.*255), cv::Scalar( 78./2, 100./100.*255, 100./100.*255), mask);
        limit = .04;
    } else if (colour == Colour::GREEN) {
        // inRange(in_image_hsv, cv::Scalar(  83./360.*255,  30./100.*255,  17./100.*255), cv::Scalar( 154./360.*255, 100./100.*255, 100./100.*255), mask);
        inRange(in_image_hsv, cv::Scalar(  83./2,  30./100.*255,  17./100.*255), cv::Scalar( 154./2, 100./100.*255, 100./100.*255), mask);
    } else if (colour == Colour::BLUE) {
        // inRange(in_image_hsv, cv::Scalar(  200./360.*255,  80./100.*255,  15./100.*255), cv::Scalar( 280./360.*255, 100./100.*255, 100./100.*255), mask);
        inRange(in_image_hsv, cv::Scalar(  200./2,  80./100.*255,  15./100.*255), cv::Scalar( 280./2, 100./100.*255, 100./100.*255), mask);
    } else {
        // Report color not implemented
        ROS_ERROR_STREAM("[ObjectDetector::colourFilter] colour (" << colour << "  not implemented!");
    }
    // dilate -> erode to form cohesive connected components
    dilate(mask, mask, cv::Mat());
    dilate(mask, mask, cv::Mat());
    dilate(mask, mask, cv::Mat());
    erode(mask, mask, cv::Mat());
    erode(mask, mask, cv::Mat());
    erode(mask, mask, cv::Mat());
    cv::Mat labels;
    int n = cv::connectedComponents(mask, labels);
    int max_label = 0;
    double max_label_area = 0.;
    bool found = false;
    cv::Mat max_label_mask;
    for (int i=1; i<n; i++){
        cv::Mat lbl_mask;
        cv::Mat area_mask;
        inRange(labels, cv::Scalar(i), cv::Scalar(i), lbl_mask);
        lbl_mask.convertTo(area_mask, CV_32F);
        area_mask /= 255.;
        double area = cv::sum(area_mask)[0];
        if (area > max_label_area) {
            max_label = i;
            max_label_area = area;
            lbl_mask.copyTo(max_label_mask);
            found = true;
        }

    }
    if (found)
        return max_label_mask;
    return cv::Mat::zeros(mask.rows, mask.cols, mask.type()); 
    // double thresh = double(in_image_bgr.cols * in_image_bgr.rows) * limit;
    // // We return the mask, that will be used later
    // if (max_label_area > thresh){
    //     // cv::imwrite("pre-filt-mask.png", mask);
    //     return max_label_mask;
    // }
    // return cv::Mat::zeros(mask.rows, mask.cols, mask.type());
}

cv::Mat ObjectDetector::applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height) {
    // ROS_INFO("is CV_8U : %d", in_mask.type() == CV_8U);
    cv::Mat drawing; // it could be useful to fill this image if you want to debug
    in_mask.copyTo(drawing);

    // TODO: Compute the bounding box using the mask
    // You need to return the center of the object in image coordinates, as well as a bounding box indicating its height and width (in pixels)
    x = -1;
    y = -1;
    width = -1;
    height = -1;

    int x_min = 1000000;
    int x_max = 0;
    int y_min = 1000000;
    int y_max = 0;
    bool found = false;

    for(int x = 0; x < in_mask.cols; x++){
        for(int y = 0; y < in_mask.rows; y++){
            if (in_mask.at<uchar>(y, x) > 0) {
                x_min = std::min(x_min, x);
                x_max = std::max(x_max, x);
                y_min = std::min(y_min, y);
                y_max = std::max(y_max, y);
                found = true;
            } 
        }
    }
    if (found) {
        cv::rectangle(drawing, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(128));
        // ROS_INFO("%d %d %d %d %d", is_u8, x_min, y_min, x_max, y_max);
        width = double(x_max - x_min);
        height = double(y_max - y_min);
        x = double(x_min + x_max) / 2;
        y = double(y_min + y_max) / 2;
        // for(int col = x_min; col < x_max; col++){
        //     drawing.at<int>(y_min, col) = 255;
        //     drawing.at<int>(y_max, col) = 255;
        // }
        // for(int row = y_min; row < y_max; row++){
        //     drawing.at<int>(row, x_min) = 255;
        //     drawing.at<int>(row, x_max) = 255;
        // }
    }
    return drawing;
}


bool ObjectDetector::recognizeObject(const cv::Mat &in_image, const Colour &colour, const std_msgs::Header &in_header, 
                                  double& x_map, double& y_map, double& z_map)
{
    double obj_center_x;
    double obj_center_y;
    double obj_image_height;
    double obj_image_width;

    cv::Mat in_image_filt = applyColourFilter(in_image, colour);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_filt, obj_center_x, obj_center_y, obj_image_width, obj_image_height);

    if (obj_center_x < 0 || obj_image_width < 50 || obj_image_height < 50)
    {
        return false;
    }


    if(getObjectPosition(obj_center_x,obj_center_y,in_header,x_map, y_map, z_map))
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool ObjectDetector::wasObjectDetected(std::string object_name)
{
    bool detected = false;
    for(auto obj : detected_objects_.objects)
    {
        if(obj.id == object_name)
            detected = true;
    }

    return detected;
}