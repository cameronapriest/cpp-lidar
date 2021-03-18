// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <JetsonGPIO.h>         // for GPIO in C++

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{
    GPIO::setmode(GPIO::BOARD);

    int output_pin = 12;
    GPIO::setup(output_pin, GPIO::OUT, GPIO::LOW);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    std::cout << "I'm alive\n";

    int warmups = 0;
    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        //std::cout << "\nwidth in pixels: " << width;
        //std::cout << "\nheight in pixels: " << height;

        //break;

        int ptsTooClose = 0;

        int shaveoff = 70;

        warmups++;
        if (warmups > 30) {
            int i, j;
            int didwebreak = 0;
            for (i = shaveoff; i < width-shaveoff; i++) {
                for (j = 0; j < height; j++) {
                    float dist = depth.get_distance(i, j);
                    if (dist < 0.8 && dist > 0) {
                        //std::cout << i << " " << j << " " << dist << " meters \n";
                        ptsTooClose++;
                        //didwebreak = 1;
                    }
                }
            }

            if (ptsTooClose > 20) {
                didwebreak = 1;
            }

            //std::cout << didwebreak << "\n";
            GPIO::output(output_pin, didwebreak);
        }

        // Query the distance from the camera to the object in the center of the image
        //float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }

    GPIO::cleanup();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
                           
