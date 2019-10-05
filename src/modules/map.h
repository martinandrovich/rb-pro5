#pragma once
#include <mutex>
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

class map
{

    private:

    typedef struct points_polar
    {
        float angle;
        float len;
    } points_polar;

    public:
    
    map(size_t height, size_t width) : map_(height, width, CV_8UC1)
    {

        for (size_t i = 0; i < height; i++)
        {
            for (size_t j = 0; j < width; j++)
            {
                map_.at<uchar>(i,j) = 0;
            }
        }

        std::cout << "Map is constructed" << std::endl;

        integral_movement = 0;
        integral_direction = 0;
        x = width/2-1;
        y = height/2-1;
        
    };

    void timing_start_t1()
    {
        t1 = std::chrono::high_resolution_clock::now();
    }

    void timing_start_t2()
    {
        t2 = std::chrono::high_resolution_clock::now();
    }

    auto timing_get_time_ms()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
    }

    void update_estimate_position(double dir, double speed)
    {
        delta_movement = integral_movement;
        integral_movement = integral_movement + (speed*(double)this->timing_get_time_ms()/500);
        delta_movement -= integral_movement;

        integral_direction = integral_direction + (dir*(double)this->timing_get_time_ms()/500);

        //update x
        x += cos(integral_direction) * delta_movement;

        //update y
        y += sin(integral_direction) * delta_movement;

        map_.at<uchar>(x,y) = 255;

        std::cout << "Updated Estimate Pos" << std::endl;
        
    }
    
    void update_map_lidar(ConstLaserScanStampedPtr& msg)
    {
        /*
        float range_max = float(msg->scan().range_max());
        float angle_increment = float(msg->scan().angle_step());
        float angle_min = float(msg->scan().angle_min());
        int nranges = msg->scan().ranges_size();
        point_vec.resize(nranges);
        
        for (size_t i = 0; i < static_cast<size_t>(nranges); i++) 
        {
             float angle = angle_min + i * angle_increment;
             float range = std::min(float(msg->scan().ranges(i)), range_max);
             points_polar polar_; polar_.len = range; polar_.angle = angle;
             point_vec[i] = polar_;
        }
        */
        std::cout << "Updated Map Lidar" << std::endl;

    }

    void update_map()
    {
        /*
        for (size_t i = 0; i < point_vec.size(); i++) 
        {
            map_.at<uchar>(sin(point_vec[i].angle)*point_vec[i].len+y,cos(point_vec[i].angle)*point_vec[i].len+x) = 255;
        }
        */
    }

    void draw_map(std::mutex & mutex)
    {
        mutex.lock();
        cv::imshow("Map", map_);
        mutex.unlock();
    }

    private:

    cv::Mat map_;
    std::chrono::system_clock::time_point t1;
    std::chrono::system_clock::time_point t2;
    std::vector< points_polar > point_vec;
    double integral_direction;
    double integral_movement;
    double delta_movement;
    double x, y;
    
    
};