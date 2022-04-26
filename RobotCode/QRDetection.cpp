#include "MiniLang/kinect.hpp"
#include "MiniLang/util.h"

#include <chrono>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index )
{
    // Initialize
    initialize();
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize()
{
    // Initialize Sensor
    initialize_sensor();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Default Device
    device = k4a::device::open( device_index );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );
}

// Finalize
void kinect::finalize()
{
    // Stop Cameras
    device.stop_cameras();

    // Close Device
    device.close();

    // Close Window
    cv::destroyAllWindows();
}

// Run
string kinect::run()
{
    // Main Loop
    while( true ){
        // Update
        update();

        // Draw
        draw();

        // Show
        string s = show();

        // Wait Key
        constexpr int32_t delay = 30;
        const int32_t key = cv::waitKey( delay );
        if( key == 'q' ){
            break;
        } else if (s != NULL) {
            return s;
        }
    }
    return NULL;
}

// Update
void kinect::update()
{
    // Update Frame
    update_frame();

    // Update Color
    update_color();

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        this->~kinect();
    }
}

// Update Color
inline void kinect::update_color()
{
    // Get Color Image
    color_image = capture.get_color_image();
}

// Draw
void kinect::draw()
{
    // Draw Color
    draw_mat();
}

// Draw Color
inline void kinect::draw_mat()
{
    if( !color_image.handle() ){
        return;
    }

    // Get cv::Mat from k4a::image
    mat = k4a::get_mat( color_image );

    // Release Color Image Handle
    color_image.reset();
}

// Show
String kinect::show()
{
    // Show Color
    return show_mat();
}

void display(Mat &im, Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  }
  imshow("Result", im);
}

// Show Color
inline string kinect::show_mat()
{
    if( mat.empty() ){
        return;
    }

    // Show Image
    const cv::String window_name = cv::format( "color (kinect %d)", device_index );
    cv::QRCodeDetector qrDecoder;

  Mat bbox, rectifiedImage;

  std::string data = qrDecoder.detectAndDecode(mat, bbox, rectifiedImage);
  if(data.length()>0)
  {
    string s << "Decoded Data : " << data << endl;

    display(mat, bbox);
    rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
    imshow("Rectified QRCode", rectifiedImage);

    waitKey(0);
    return s;
  else
    cout << "QR Code not detected" << endl;
    return NULL;
}
