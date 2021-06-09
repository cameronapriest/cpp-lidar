#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <JetsonGPIO.h>         // for GPIO in C++
#include <unistd.h>		// for UART
#include <fcntl.h>		// for UART
#include <termios.h>		// for UART
#include <stdio.h>


#define PIXEL_SHAVE_OFF_SIDES 70 // number of pixels to shave off from either side
#define PIXEL_SHAVE_OFF_GROUND 20 // number of pixels near the ground

#define UART_RXD_PIN 10
#define UART_TXD_PIN 8
#define GND_PIN 6

#define TXD_STOP 1
#define TXD_FORWARD 2
#define TXD_LEFT 3
#define TXD_RIGHT 4

#define RXD_START_LIDAR 1
#define RXD_STOP_LIDAR 2


void transmitData(int fd, uint8_t data, uint8_t dataLen) {
    if (fd >= 0) {
	int count = write(fd, &data, dataLen);
    
	if (count < 0) {
	    printf("UART transmit error\n");
	    perror("UART TXD");
	    exit(-1);
	} else {
	    printf("Sent %d bytes - [%d].\n", count, data);
	}
    } else {
	perror("File descriptor when sending");
	exit(-1);
    }
}

void receiveData(int fd, unsigned char * recvBuf) {
    if (fd >= 0) {
	int recvLen = read(fd, (void *) recvBuf, 10); // 10 is max number of bytes to read
	
	if (recvLen < 0) {
	    printf("Error occured when trying to receive data - maybe no bytes?\n");
	} else if (recvLen == 0) {
	    printf("No data ready to be received\n");
	} else {
	    printf("Received %d bytes.\n", recvLen);
	    for (int i = 0; i < recvLen; i++) {
		printf("Byte %d: [%d]\n", i, recvBuf[i]);
	    }
	}
    } else {
	perror("File descriptor when receiving");
	exit(-1);
    }
}

int main(int argc, char * argv[]) try {

    int uart0_filestream = -1;
    
    uart0_filestream = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY); // could use O_NDELAY for nonblocking 
    if (uart0_filestream < 0) {
	printf("Error - Unable to open UART.\n");
	perror("UART");
	exit(-1);
    }

    printf("UART filestream opened successfully!\n");

    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | /*PARENB |*/ CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcflush(uart0_filestream, TCOFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

    //uint8_t txBuf = 50;
    //transmitData(uart0_filestream, txBuf, 1);

    unsigned char rxBuf[10];
    receiveData(uart0_filestream, rxBuf); 

    // temp - for testing UART
    close(uart0_filestream);
    printf("UART filestream closed.\n");
    exit(0);

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
	warmups++;

        if (warmups > 30) {
            int i, j;
            int didwebreak = 0;
            for (i = PIXEL_SHAVE_OFF_SIDES; i < width - PIXEL_SHAVE_OFF_SIDES; i++) {
                for (j = PIXEL_SHAVE_OFF_GROUND; j < height; j++) {
                    float dist = depth.get_distance(i, j);
                    if (dist < 0.8 && dist > 0) {
                        ptsTooClose++; // measured a pixel too close to Herbie
                    }
                }
            }

            if (ptsTooClose > 20) { // 20+ pixels indicate valid hazard
                didwebreak = 1;
            }

            std::cout << didwebreak << "\n";
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
                           
