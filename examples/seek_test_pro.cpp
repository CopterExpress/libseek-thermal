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

#define VIDEO_DEVICE "/dev/video1" // gray scale thermal image
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define BYTES_PER_PIXEL 3

// #define FRAME_FORMAT0 V4L2_PIX_FMT_GREY
#define FRAME_FORMAT1 V4L2_PIX_FMT_BGR24 //MJPEG

int fdwr1 = 0;
char *video_device1 = VIDEO_DEVICE;
struct v4l2_format vid_format1;
struct v4l2_capability vid_caps1;


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

int main(int argc, char** argv)
{
    raspicam::RaspiCam rpicam;
    // rpicam.setFormat(raspicam::RASPICAM_FORMAT_GRAY); // We can set frame format
    rpicam.setCaptureSize(FRAME_WIDTH, FRAME_HEIGHT); // Resize captured frame from raspicam

    // Trying to open raspicam
    if ( !rpicam.open() ) 
    {
        std::cout << "Cannot open the raspicam" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Raspicam succesfully opened" << std::endl;
    }

    // Opening /dev/video1 device
    start_v4l();

    // Allocating buffer for visual frame
    unsigned char *data=new unsigned char[rpicam.getImageBufferSize()];

    LibSeek::SeekThermalPro seek(argc == 2 ? argv[1] : "");
    cv::Mat thermal_frame, visual_frame;

    if (!seek.open()) {
        std::cout << "Failed to open seek cam" << std::endl;
        return -1;
    }

    while(1) {
        // Read thermal frame from Seek Pro
        if (!seek.read(thermal_frame)) {
            std::cout << "Error: no more thermal img" << std::endl;
            seek.close();
            if (!seek.open()) {
                std::cout << "Failed to re-open seek cam" << std::endl;
                // return -1;
            }
            std::cout << "Re-open seek cam" << std::endl;
        }

        // Grab visual frame from raspicam
        rpicam.grab();
        rpicam.retrieve(data, raspicam::RASPICAM_FORMAT_IGNORE);

        // Creating Mat object from visual frame buffer
        visual_frame = cv::Mat(FRAME_WIDTH, FRAME_HEIGHT, CV_8UC3, data, FRAME_WIDTH*BYTES_PER_PIXEL);

        // Process thermal frame
        seek.convertToGreyScale(thermal_frame, thermal_frame);
        // // cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);
        cv::GaussianBlur(thermal_frame, thermal_frame, cv::Size(7,7), 0);
        cv::applyColorMap(thermal_frame, thermal_frame, cv::COLORMAP_HOT);
        cv::resize(thermal_frame, thermal_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

        // Cropping area on visual image
        cv::Rect myROI(235, 165, 160, 120);
        // cv::resize(visual_frame, visual_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
        visual_frame = visual_frame(myROI);
        cv::resize(visual_frame, visual_frame, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
        cv::flip(visual_frame, visual_frame, -1);

        // Corner detection on a visual
        double alpha = 0.5, beta;
        beta = ( 1.0 - alpha );
        int lowThreshold = 5;
        cv::cvtColor(visual_frame, visual_frame, cv::COLOR_RGB2GRAY);
        cv::blur(visual_frame, visual_frame, cv::Size(3,3) );
        cv::Canny(visual_frame, visual_frame, lowThreshold, lowThreshold*3, 3);
        cv::cvtColor(visual_frame, visual_frame, cv::COLOR_GRAY2RGB);

        // Mix thermal with visual
        cv::addWeighted(visual_frame, alpha, thermal_frame, beta, 0.0, visual_frame);   

        // Write everything to a video device
        write(fdwr1, visual_frame.data, FRAME_WIDTH*FRAME_HEIGHT*BYTES_PER_PIXEL); // jpg Visual Image
    }

    free(data);
    close(fdwr1);
}
