#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static void 
_Morphology_Operations( int, void* );


namespace tune_morphology
{
    struct morph_settings
    {
        cv::Mat element;
        int operation;
        int filter_sigma;
        int gauss_size;
        int hough_upper_tresh;
        int hough_center_tresh;
        int hough_min_radius;
        std::vector<cv::Vec3f> circles;
    };
}

struct morph_t
{    
    int elem = 0;
    int size = 0;
    int operator_t = 0;
    int frame = 0;
    int filter_sigma = 1;
    int gauss_size = 1;
    int hough_upper_tresh = 200;
    int hough_center_tresh = 100;
    int hough_min_radius = 0;
};

struct callback_args
{
    const char* name;    
    cv::Mat* dst;    
    tune_morphology::morph_settings settings;
};

static struct morph_t morph;
static std::vector<cv::Mat> _frames;

namespace tune_morphology
{
    static cv::VideoCapture 
    load_video(const std::string& path)
    {
        cv::VideoCapture cap(path);

        if(!cap.isOpened())
        {
            std::cout << "Error opening video stream or file" << std::endl;
        }
        return cap;
    }

    static morph_settings 
    choose_optimal_morph(const std::string& video_path)
    {
        cv::VideoCapture cap = load_video(video_path);
        cv::Mat frame;
        while(true)
        {
            cap >> frame;
            if(frame.empty()) break;
            _frames.push_back(frame.clone());
        }

        int const max_operator = 4;
        int const max_elem = 2;
        int const max_kernel_size = 21;
        int const max_gauss_kernel_size = 50;
        int const max_sigma = 100;
        const char* window_name = "Morphology Transformations tuning";
        cv::Mat dst;

        callback_args _args{window_name, &dst, cv::Mat(), 0};        
        
        cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE ); // Create window

        cv::createTrackbar( "Frame[number]", window_name,
                  &morph.frame, _frames.size() - 1,
                  _Morphology_Operations, (void*)&_args );

        cv::createTrackbar( "Hough_upper_Treshold", window_name,
                  &morph.hough_upper_tresh, 500,
                  _Morphology_Operations, (void*)&_args );
        
        cv::createTrackbar( "hough_center_tresh", window_name,
                  &morph.hough_center_tresh, 500,
                  _Morphology_Operations, (void*)&_args );
        
        cv::createTrackbar( "hough_min_radius", window_name,
                  &morph.hough_min_radius, 500,
                  _Morphology_Operations, (void*)&_args );
        
        cv::createTrackbar( "Gaus_kernel_size", window_name,
                  &morph.gauss_size , max_gauss_kernel_size,
                  _Morphology_Operations, (void*)&_args );

        cv::createTrackbar( "Gauss filter sigma ", window_name,
                  &morph.filter_sigma , max_sigma,
                  _Morphology_Operations, (void*)&_args );

        cv::createTrackbar("Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat", 
                window_name, &morph.operator_t, 
                max_operator, _Morphology_Operations, (void*)&_args );

        cv::createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_name,
                  &morph.elem, max_elem,
                  _Morphology_Operations, (void*)&_args );

        cv::createTrackbar( "Kernel size:\n 2n +1", window_name,
                  &morph.size, max_kernel_size,
                  _Morphology_Operations, (void*)&_args );

        _Morphology_Operations( 0, (void*)&_args  );
        cv::waitKey();
        debug::cout << _arg.settings.
    
        return _args.settings;
    }
}

static void 
_Morphology_Operations( int, void* user_data )
{
 callback_args* args = (callback_args*)user_data;
  
  // Since MORPH_X : 2,3,4,5 and 6
  int operation = morph.operator_t + 2;  
  cv::Mat element = getStructuringElement(morph.elem, cv::Size( 2 * morph.size + 1, 2 * morph.size + 1 ), cv::Point(morph.size, morph.size ));
  cv::Mat filtered_frame;

  int allowed_kernel_size = morph.gauss_size;
  if(allowed_kernel_size % 2 == 0) allowed_kernel_size--;

  //Apply blur and do the morphology with the chosen settings.
  cv::GaussianBlur(_frames[morph.frame], filtered_frame, cv::Size(allowed_kernel_size, allowed_kernel_size), morph.filter_sigma );
  cv::morphologyEx(filtered_frame, *args->dst, operation, element );
  
  //Apply HoughCircle transform and save all circles found to a vector
  cv::Mat grey_dst;
  cv::cvtColor(*args->dst, grey_dst, CV_BGR2GRAY );
  cv::HoughCircles(grey_dst, args->settings.circles, CV_HOUGH_GRADIENT, 1, 100, morph.hough_upper_tresh, morph.hough_center_tresh, morph.hough_min_radius, 0);
  
  //std::cout << "Circles found :" << args->settings.circles[0][0] << args->settings.circles[0][1] << args->settings.circles[0][2] << std::endl; 
  args->settings.element = element.clone();
  args->settings.operation = operation;

    
  // Draw the circles detected
  for( size_t i = 0; i < args->settings.circles.size(); i++ )
  {
      std::cout << "Circles found:" << std::endl;
      cv::Point center(cvRound(args->settings.circles[i][0]), cvRound(args->settings.circles[i][1]));
      int radius = cvRound(args->settings.circles[i][2]);
      // circle center
      cv::circle(*args->dst, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle(*args->dst, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
   }

  //Show the output of the combined output.
  cv::imshow((const char*)args->name, *args->dst);
}