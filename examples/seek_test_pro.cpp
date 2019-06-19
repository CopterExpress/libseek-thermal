/*
 *  Test program SEEK Thermal CompactPRO
 *  Author: Maarten Vandersteegen
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "seek.h"
#include <iostream>
#include <raspicam/raspicam.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <unistd.h>

#define VIDEO_DEVICE "/dev/video1" // Virtual video device for output
// Output frame size
#define FRAME_WIDTH 1920//1280 
#define FRAME_HEIGHT 1080//720
#define BYTES_PER_PIXEL 3

// Output video format
// #define FRAME_FORMAT0 V4L2_PIX_FMT_GREY
#define FRAME_FORMAT1 V4L2_PIX_FMT_BGR24 //MJPEG

bool apply_sobel = true;
bool apply_canny = false;
bool apply_colormap = true;
bool draw_temp = true;

int fdwr1 = 0;
char *video_device1 = VIDEO_DEVICE;
struct v4l2_format vid_format1;
struct v4l2_capability vid_caps1;

cv::Mat thermal_frame, visual_frame, full_visual_frame;
std::mutex lock_thermal, lock_visual;

// Raspicam cropping
// int x = 465;
// int y = 335;
// // Cropping area on visual image
// cv::Rect region_to_place_thermal(x, y, 320, 240);

// Gitup3 Duo cropping
int x = 565;
int y = 205;
int thermal_region_width = 800;
int thermal_region_height = 600;
// Cropping area on visual image
cv::Rect region_to_place_thermal(x, y, thermal_region_width, thermal_region_height);

void print_format(struct v4l2_format *vid_format)
{
  printf("     vid_format->type                =%d\n", vid_format->type);
  printf("     vid_format->fmt.pix.width       =%d\n", vid_format->fmt.pix.width);
  printf("     vid_format->fmt.pix.height      =%d\n", vid_format->fmt.pix.height);
  printf("     vid_format->fmt.pix.pixelformat =%d\n", vid_format->fmt.pix.pixelformat);
  printf("     vid_format->fmt.pix.sizeimage   =%u\n", vid_format->fmt.pix.sizeimage);
  printf("     vid_format->fmt.pix.field       =%d\n", vid_format->fmt.pix.field);
  printf("     vid_format->fmt.pix.bytesperline=%d\n", vid_format->fmt.pix.bytesperline);
  printf("     vid_format->fmt.pix.colorspace  =%d\n", vid_format->fmt.pix.colorspace);
}

// Initializing of output device
void start_v4l()
{
  int ret_code;

  fdwr1 = open(video_device1, O_RDWR);
  assert(fdwr1 >= 0);

  ret_code = ioctl(fdwr1, VIDIOC_QUERYCAP, &vid_caps1);
  assert(ret_code != -1);

  memset(&vid_format1, 0, sizeof(vid_format1));

  ret_code = ioctl(fdwr1, VIDIOC_G_FMT, &vid_format1);

  vid_format1.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  vid_format1.fmt.pix.width = FRAME_WIDTH;
  vid_format1.fmt.pix.height = FRAME_HEIGHT;
  vid_format1.fmt.pix.pixelformat = FRAME_FORMAT1;
  vid_format1.fmt.pix.sizeimage =  FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL;
  vid_format1.fmt.pix.field = V4L2_FIELD_NONE;
  vid_format1.fmt.pix.bytesperline = FRAME_WIDTH * BYTES_PER_PIXEL;
  vid_format1.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

  // set data format
  ret_code = ioctl(fdwr1, VIDIOC_S_FMT, &vid_format1);
  assert(ret_code != -1);

  print_format(&vid_format1);
}

// Translating device internal temperature into kelvin. We need it to correct temperature data from the sensor.
double device_sensor_to_k(double sensor) {
    // formula from http://aterlux.ru/article/ntcresistor-en
    double ref_temp = 297.0; // 23C from table
    double ref_sensor = 6616.0; // ref value from table
    double beta = 200; // best beta coef we've found
    double part3 = log(sensor) - log(ref_sensor);
    double parte = part3 / beta + 1.0 / ref_temp;
    return 1.0 / parte;
}

// Semi-experimental translating and scaling raw temp data from the Seek to normal celsius range
double temp_from_raw(int x){ //, double device_k) {
    double device_k = device_sensor_to_k(0.0);
    // Constants below are taken from linear trend line in Excel.
    // -273 is translation of Kelvin to Celsius
    // 330 is max temperature supported by Seek device
    // 16384 is full 14 bits value, max possible ()
    double base = x * 330 / 16384.0;
    double lin_k = -1.5276; // derived from Excel linear model
    double lin_offset = 0;//-470.8979; // same Excel model
    return base - device_k * lin_k + lin_offset - 283.0;
}

void apply_sobel_edge_detector(cv::Mat &sobel_thermal, cv::Mat &sobel_visual)
{
    cv::Mat abs_grad_x, abs_grad_y;
    int scale = 3;
    int delta = 0;
    int ddepth = CV_16S;

    cv::cvtColor(sobel_visual, sobel_visual, cv::COLOR_RGB2GRAY);

    cv::Sobel( sobel_visual, abs_grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );        
    cv::Sobel( sobel_visual, abs_grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );

    cv::convertScaleAbs( abs_grad_x, abs_grad_x );
    cv::convertScaleAbs( abs_grad_y, abs_grad_y );

    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_visual );

    // cv::cvtColor(sobel_visual, sobel_visual, cv::COLOR_GRAY2RGB);

    double alpha = 0.5, beta;
    beta = ( 1.0 - alpha );

    // Mix thermal with visual
    cv::addWeighted(sobel_visual, alpha, sobel_thermal, beta, 0.0, sobel_thermal); 
}

void apply_canny_edge_detector(cv::Mat &canny_thermal, cv::Mat &canny_visual)
{
    double alpha = 0.5, beta;
    beta = ( 1.0 - alpha );
    int lowThreshold = 15;
    cv::cvtColor(canny_visual, canny_visual, cv::COLOR_RGB2GRAY);
    cv::blur(canny_visual, canny_visual, cv::Size(3,3) );
    cv::Canny(canny_visual, canny_visual, lowThreshold, lowThreshold*3, 3);

    // Mix thermal with visual
    cv::addWeighted(canny_visual, alpha, canny_thermal, beta, 0.0, canny_thermal); 
}

void draw_temperature(cv::Mat &frame, char* temp_string)
{
    cv::line(frame, cv::Point(thermal_region_width/2 - 10, thermal_region_height/2), cv::Point(thermal_region_width/2 + 10, thermal_region_height/2), cv::Scalar(255,255,255), 1);
    cv::line(frame, cv::Point(thermal_region_width/2, thermal_region_height/2 -10), cv::Point(thermal_region_width/2 , thermal_region_height/2 + 10), cv::Scalar(255,255,255), 1);
    cv::putText(frame, temp_string, cv::Point(thermal_region_width/2 + 10, thermal_region_height/2 + 10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1);
}

void process_thermal_frame()
{
    LibSeek::SeekThermalPro seek("");//(argc == 2 ? argv[1] : "");
    if (!seek.open()) {
        std::cout << "Failed to open seek cam" << std::endl;
        return;
    }

    // Char buffer for temperature drawing
    char txt [64];

    // Temporary Mats for thermal_frame processing.
    cv::Mat thermal_tmp_frame, visual_tmp_frame;

    while (1)
    {
    // Read thermal frame from Seek Pro
        if (!seek.read(thermal_tmp_frame)) {
            std::cout << "Error: no more thermal img" << std::endl;
            seek.close();
            // seek.~SeekThermalPro();
            // LibSeek::SeekThermalPro seek(argc == 2 ? argv[1] : "");
            if (!seek.open()) {
                std::cout << "Failed to re-open seek cam" << std::endl;
                return;
            }
            std::cout << "Re-open seek cam" << std::endl;
        }

        // std::cout << "Central pixel: " << temp_from_raw(thermal_frame.at<uint16_t>(160, 120)) << std::endl;
        sprintf(txt, "%5.1fC", temp_from_raw(thermal_tmp_frame.at<uint16_t>(thermal_tmp_frame.cols/2.0, thermal_tmp_frame.rows/2.0)));

        // Process thermal frame
        seek.convertToGreyScale(thermal_tmp_frame, thermal_tmp_frame);
        // // cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);
        cv::GaussianBlur(thermal_tmp_frame, thermal_tmp_frame, cv::Size(7,7), 0);

        cv::flip(thermal_tmp_frame, thermal_tmp_frame, -1);
        cv::resize(thermal_tmp_frame, thermal_tmp_frame, cv::Size(800, 600));

        // If we want to apply any of edge_detectors we need to copy region of visual frame
        if (apply_sobel || apply_canny)
        {
            {
                std::lock_guard<std::mutex> lock_visual_guard(lock_visual);
                visual_tmp_frame = visual_frame(region_to_place_thermal);
            }
        }
        
        if (apply_sobel)
        {
            apply_sobel_edge_detector(thermal_tmp_frame, visual_tmp_frame);
        }
        else if (apply_canny)
        {
            apply_canny_edge_detector(thermal_tmp_frame, visual_tmp_frame);
        }
        
        if (apply_colormap)
        {
            cv::applyColorMap(thermal_tmp_frame, thermal_tmp_frame, cv::COLORMAP_HOT);
        }
        else
        {
            cv::cvtColor(thermal_tmp_frame, thermal_tmp_frame, cv::COLOR_GRAY2RGB);
        }

        if (draw_temp)
        {
            draw_temperature(thermal_tmp_frame, txt);
        }

        {
            std::lock_guard<std::mutex> lock_thermal_guard(lock_thermal);
            thermal_frame = thermal_tmp_frame.clone();
        }
    }
}


void process_visual_frame()
{
    raspicam::RaspiCam rpicam;
    // rpicam.setFormat(raspicam::RASPICAM_FORMAT_GRAY); // We can set frame format
    rpicam.setCaptureSize(FRAME_WIDTH, FRAME_HEIGHT); // Resize captured frame from raspicam

    // Trying to open raspicam
    if ( !rpicam.open() ) 
    {
        std::cout << "Cannot open the raspicam" << std::endl;
        return;
    }
    else
    {
        std::cout << "Raspicam succesfully opened" << std::endl;
    }

    // Opening /dev/video1 device
    start_v4l();

    // Allocating buffer for visual frame
    unsigned char *data=new unsigned char[rpicam.getImageBufferSize()];

    // FPS measuring variables
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::milliseconds milliseconds;

    while(1) {
        Clock::time_point frame_start_time = Clock::now();

        // process_thermal_frame();

        // Grab visual frame from raspicam
        rpicam.grab();
        rpicam.retrieve(data, raspicam::RASPICAM_FORMAT_IGNORE);

        // Creating Mat object from visual frame buffer
        full_visual_frame = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, data, FRAME_WIDTH*BYTES_PER_PIXEL);

        {
            std::lock_guard<std::mutex> lock_visual_guard(lock_visual);
            visual_frame = full_visual_frame.clone();
        }        

        // std::cout << "Visual frame width: " << visual_frame.cols << "; visual frame height: " << visual_frame.rows << std::endl;

        
        // cv::resize(visual_frame, visual_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
        // visual_frame = visual_frame(region_to_place_thermal);
        // // cv::flip(visual_frame, visual_frame, -1);
        // cv::resize(visual_frame, visual_frame, cv::Size(320, 240));

        // // Corner detection on a visual
        

        // cv::flip(visual_frame, visual_frame, -1);

        {
            std::lock_guard<std::mutex> lock_thermal_guard(lock_thermal);
            if (!thermal_frame.empty())
                thermal_frame.copyTo(full_visual_frame(region_to_place_thermal));//cv::Rect(x, y, visual_frame.cols, visual_frame.rows))); 
        }

        // cv::Rect cropping_area(300, 250, 680, 400);
        // full_visual_frame = full_visual_frame(cropping_area);

        cv::resize(full_visual_frame, full_visual_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

        Clock::time_point frame_end_time = Clock::now();
        
        milliseconds ms = std::chrono::duration_cast<milliseconds>(frame_end_time - frame_start_time);

        cv::putText(full_visual_frame, std::to_string(1000/ms.count()), cv::Point(30,30), cv::FONT_HERSHEY_DUPLEX, 0.25, cv::Scalar(255,255,255), 1);
        
        // Write everything to a video device
        write(fdwr1, full_visual_frame.data, FRAME_WIDTH*FRAME_HEIGHT*BYTES_PER_PIXEL); // writing result buffer
    }

    free(data);
    close(fdwr1);
}

int main(int argc, char** argv)
{
    std::thread t1(process_thermal_frame);
    std::thread t2(process_visual_frame);
    t1.join();
    t2.join();
    return 1;
}
