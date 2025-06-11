#include "ros2_orb_slam3/mono_cam.hpp"


MonoCam::MonoCam() :Node("mono_cam_cpp")
{
 
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_INFO(this->get_logger(), "File not set ......");

        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }
    else{
        RCLCPP_INFO(this->get_logger(), "File set ......");
        vocFilePath = homeDir + "/" + packagePath + vocFilePath; // Example ros2_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin
        settingsFilePath = homeDir + "/" + packagePath + settingsFilePath; // Example ros2_ws/src/ros2_orb_slam3/orb_slam3/config/Monocular/TUM2.yaml
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());

    subImgMsgName = "/airsim"; // topic to receive RGB image messages

    //* subscrbite to the image messages coming from the Python driver node
    // subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(
    //     subImgMsgName, 
    //     10, 
    //     std::bind(&MonoCam::Img_callback, this, _1));
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/airsim/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&MonoCam::Img_callback, this, std::placeholders::_1)
    );

    //* subscribe to receive the timestep
    // subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(
    //     subTimestepMsgName, 1, std::bind(&MonoCam::Timestep_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");

    initializeVSLAM("TUM1"); // Initialize the VSLAM framework with the node name as the config string

}

MonoCam::~MonoCam()
{       
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Method to bind an initialized VSLAM framework to this node
void MonoCam::initializeVSLAM(std::string configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonoCam node initialized" << std::endl; // TODO needs a better message
}


//* Callback to process image message and run SLAM node
void MonoCam::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        // cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // RCLCPP_ERROR(this->get_logger(),"Recieved reading image");
        // DEBUGGING, Show image
        // Update GUI Window
        cv::imshow("test_window", cv_ptr->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    double time_s = StampToSec(msg.header);
    // std::cout << "Time in seconds: " << time_s << std::endl; // Debug
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, time_s); //* Tcw is the pose with respect to the camera coordinate frame
    std::cout << "Tcw: " << Tcw.matrix() << std::endl; // Debug
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}

double MonoCam::StampToSec(const std_msgs::msg::Header& header)
{
    //* Convert ROS time stamp to seconds
    rclcpp::Time time = header.stamp;
    double seconds = time.seconds() + (time.nanoseconds() * pow(10,-9));
    // std::cout<<"StampToSec: "<<sec<<std::endl; // Debug
    return seconds;
}