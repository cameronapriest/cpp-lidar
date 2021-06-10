#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <JetsonGPIO.h>         // for GPIO in C++

#define STOP_PIN 11
#define GO_PIN 12
#define LEFT_PIN 16
#define RIGHT_PIN 18
#define RECENTER_PIN 13
#define GANTRY_RECENTERED_PIN 19

#define SHAVEOFF_SIDES 200
#define GROUND_SAFETY 20

#define CLOSE_DIST 1
#define FAR_DIST 2.3
#define PERSON_PIXELS 1000

#define LEFT_THRESHOLD 100
#define RIGHT_THRESHOLD 100

int main(int argc, char * argv[]) try {
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(GO_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(STOP_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(LEFT_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(RIGHT_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(RECENTER_PIN, GPIO::OUT, GPIO::LOW);
    GPIO::setup(GANTRY_RECENTERED_PIN, GPIO::IN, GPIO::LOW);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    std::cout << "Vision System is up and running\n";

    int warmups = 0; // "warm up" iterations to ensure data is valid
    int prevDidwebreak = -1;
    int i, j, didwebreak;
    int goPoints, stopPoints, leftPoints, rightPoints;
    float dist;

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

        goPoints = 0; // number of points that could be a person
	stopPoints = 0;
	leftPoints = 0;
	rightPoints = 0;
	dist = 0;
	warmups++;

        if (warmups > 30) {
            didwebreak = 0;

	    for (i = 0; i < SHAVEOFF_SIDES; i += 2) {
		for (j = GROUND_SAFETY; j < height; j += 2) {
		    dist = depth.get_distance(i, j);
		    if (dist > CLOSE_DIST && dist < FAR_DIST) {
			leftPoints++;
		    }
		}
	    }

            for (i = SHAVEOFF_SIDES; i < width-SHAVEOFF_SIDES; i += 2) {
                for (j = GROUND_SAFETY; j < height; j += 2) {
                    dist = depth.get_distance(i, j);
		    if (dist > CLOSE_DIST && dist < FAR_DIST) {
			goPoints++;
		    } else if (dist < CLOSE_DIST && dist > 0) {
			stopPoints++;
		    }
                }
            }

	    for (i = width-SHAVEOFF_SIDES; i < width; i += 2) {
		for (j = GROUND_SAFETY; j < height; j += 2) {
		    dist = depth.get_distance(i, j);
		    if (dist > CLOSE_DIST && dist < FAR_DIST) {
			rightPoints++;
		    }
		}
	    }

	    //printf("left: %d go: %d stop: %d right: %d\n", leftPoints, goPoints, stopPoints, rightPoints);

	    if (leftPoints < LEFT_THRESHOLD && rightPoints < RIGHT_THRESHOLD) {
		if (std::max(goPoints, stopPoints) == goPoints) {
		    printf("go\n");
		} else {
		    printf("stop\n");
		}
	    } else if (leftPoints > LEFT_THRESHOLD && rightPoints < RIGHT_THRESHOLD) {
		printf("left\n");
		GPIO::output(LEFT_PIN, GPIO::HIGH);
		/* while (personInCenter() == NO) {
		    
		}
		GPIO::output(RECENTER_PIN, GPIO::HIGH);
		sleep(0.5);
		GPIO::output(LEFT_PIN, GPIO::LOW);
		GPIO::output(RECENTER_PIN, GPIO::LOW);*/
		//block until GANTRY_RECENTERED_PIN is set high 
	    } else if (leftPoints < LEFT_THRESHOLD && rightPoints > RIGHT_THRESHOLD) {
		printf("right\n");
	    } else {
		printf("BOTH LEFT AND RIGHT... ignoring\n");
	    }

            /*if (goPoints > PERSON_PIXELS) { // 20+ pixels indicate valid hazard
                didwebreak = 1;
            }

            //std::cout << didwebreak << "\n";
	    if ((prevDidwebreak != didwebreak) || (prevDidwebreak == -1)) {
		std::cout << didwebreak << "\n";
		GPIO::output(GO_PIN, didwebreak); // output 1 if hazard, else 0
	    }*/

	    prevDidwebreak = didwebreak;
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
                           
