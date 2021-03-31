#include <object_detector_cdt/object_detector.h>
#include <opencv2/imgproc.hpp>

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);

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

    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    ROS_DEBUG("New image received!");

    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    double x, y, theta;
    getRobotPose(x, y, theta);

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    // Recognize object
    // Dog
    // TODO: This only publishes the first time we detect the dog
    cv::imwrite("input_image.png", image);
    if(!wasObjectDetected("dog"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeDog(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }
    if(!wasObjectDetected("barrow"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrow(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }
    if(!wasObjectDetected("barrel"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrel(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }
    if(!wasObjectDetected("computer"))
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBox(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
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

bool ObjectDetector::recognizeDog(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double center_x;
    double center_y;
    double height;
    double width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat filt_image = applyColourFilter(in_image, Colour::RED);
    

    cv::Mat in_image_bounding_box = applyBoundingBox(filt_image, center_x, center_y, width, height);
    if (center_x < 0 || width < 100 || height < 100){
        // ROS_INFO("Not dog");
        return false;
    }
    ROS_INFO("Maybe dog");
    cv::imwrite("dog.png", filt_image);
    cv::imwrite("dog_box.png", in_image_bounding_box);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = dog_real_height_ / height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double position_camera_x = depth / camera_fx_ * (center_x - camera_cx_);
    double position_camera_y = depth / camera_fy_ * (center_y - camera_cy_);
    double position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double position_base_x = (camera_extrinsic_x_ +  position_camera_z);
    double position_base_y = (camera_extrinsic_y_ + -position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "dog";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*position_base_x + sin(-robot_theta) * position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*position_base_x + cos(robot_theta) * position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -position_camera_y;

    return std::isfinite(depth);
}



bool ObjectDetector::recognizeBarrow(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double center_x;
    double center_y;
    double height;
    double width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat filt_image = applyColourFilter(in_image, Colour::GREEN);
    

    cv::Mat in_image_bounding_box = applyBoundingBox(filt_image, center_x, center_y, width, height);
    if (center_x < 0 || width < 100 || height < 100){
        // ROS_INFO("Not barrow");
        return false;
    }
    ROS_INFO("Maybe barrow");
    cv::imwrite("barrow.png", filt_image);
    cv::imwrite("barrow_box.png", in_image_bounding_box);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = barrow_real_height_ / height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double position_camera_x = depth / camera_fx_ * (center_x - camera_cx_);
    double position_camera_y = depth / camera_fy_ * (center_y - camera_cy_);
    double position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double position_base_x = (camera_extrinsic_x_ +  position_camera_z);
    double position_base_y = (camera_extrinsic_y_ + -position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "barrow";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*position_base_x + sin(-robot_theta) * position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*position_base_x + cos(robot_theta) * position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -position_camera_y;

    return std::isfinite(depth);
}



bool ObjectDetector::recognizeBarrel(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double center_x;
    double center_y;
    double height;
    double width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat filt_image = applyColourFilter(in_image, Colour::YELLOW);
    

    cv::Mat in_image_bounding_box = applyBoundingBox(filt_image, center_x, center_y, width, height);
    if (center_x < 0 || width < 100 || height < 100){
        // ROS_INFO("Not barrel");
        return false;
    }
    ROS_INFO("Maybe barrel");
    cv::imwrite("barrel.png", filt_image);
    cv::imwrite("barrel_box.png", in_image_bounding_box);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = barrel_real_height_ / height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double position_camera_x = depth / camera_fx_ * (center_x - camera_cx_);
    double position_camera_y = depth / camera_fy_ * (center_y - camera_cy_);
    double position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double position_base_x = (camera_extrinsic_x_ +  position_camera_z);
    double position_base_y = (camera_extrinsic_y_ + -position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "barrel";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*position_base_x + sin(-robot_theta) * position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*position_base_x + cos(robot_theta) * position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -position_camera_y;

    return std::isfinite(depth);
}


bool ObjectDetector::recognizeBox(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double center_x;
    double center_y;
    double height;
    double width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat filt_image = applyColourFilter(in_image, Colour::BLUE);
    

    cv::Mat in_image_bounding_box = applyBoundingBox(filt_image, center_x, center_y, width, height);
    if (center_x < 0 || width < 100 || height < 100){
        // ROS_INFO("Not box");
        return false;
    }
    ROS_INFO("Maybe box");
    cv::imwrite("box.png", filt_image);
    cv::imwrite("box_box.png", in_image_bounding_box);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = computer_real_height_ / height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double position_camera_x = depth / camera_fx_ * (center_x - camera_cx_);
    double position_camera_y = depth / camera_fy_ * (center_y - camera_cy_);
    double position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double position_base_x = (camera_extrinsic_x_ +  position_camera_z);
    double position_base_y = (camera_extrinsic_y_ + -position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "computer";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*position_base_x + sin(-robot_theta) * position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*position_base_x + cos(robot_theta) * position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -position_camera_y;

    return std::isfinite(depth);
}


// Utils
void ObjectDetector::getRobotPose(double &x, double &y, double &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(fixed_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
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