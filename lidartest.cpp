#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <JetsonGPIO.h>         // for GPIO in C++


int main(int argc, char * argv[]) try {
    GPIO::setmode(GPIO::BOARD);

    int output_pin = 12;
    GPIO::setup(output_pin, GPIO::OUT, GPIO::LOW);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    std::cout << "Vision System is up and running\n";

    int warmups = 0; // "warm up" iterations to ensure data is valid

    while (true) {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        //std::cout << "\nwidth in pixels: " << width;
        //std::cout << "\nheight in pixels: " << height;

        int ptsTooClose = 0; // number of points that could be a hazard
        int shaveoff = 70; // number of pixels to shave off from either side
        warmups++;

        if (warmups > 30) {
            int i, j;
            int didwebreak = 0;
            for (i = shaveoff; i < width-shaveoff; i++) {
                for (j = 0; j < height; j++) {
                    float dist = depth.get_distance(i, j);
                    if (dist < 0.8 && dist > 0) {
                        ptsTooClose++; // measured a pixel too close to Herbie
                    }
                }
            }

            if (ptsTooClose > 20) { // 20+ pixels indicate valid hazard
                didwebreak = 1;
            }

            //std::cout << didwebreak << "\n";
            GPIO::output(output_pin, didwebreak); // output 1 if hazard, else 0
        }
    }

    GPIO::cleanup();

    return EXIT_SUCCESS;
}

catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}

catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
                           
