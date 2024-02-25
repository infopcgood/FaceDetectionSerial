// Standard C++ headers
#include <iostream>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <chrono>

// Serial communication headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// OpenCV2 headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

// DLib headers
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
// #include <dlib/image_processing/render_face_detections.h>
#include <dlib/opencv.h>
// #include <dlib/gui_widgets.h>
#include <dlib/image_io.h>

// Default namespaces
using namespace cv;
using namespace dlib;
using namespace std;
using namespace std::chrono;

// Get time in milliseconds
uint64_t timeMS() {
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Initialize serial on port
int initializeSerial(const char* port) {
    // Initialize serial device
    int serial_port = open(port, O_RDWR);

    // Check for errors
    if(serial_port < 0) {
        cerr << "Error " << errno << " from open: " << strerror(errno) << endl;
        return 0;
    }

    // Convert serial_port to termios struct
    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return 0;
    }

    // Set serial communication settings
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE; // Clear size settings
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // No RTS/CTS h/w flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON; // No canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // No special interpretation
    tty.c_oflag &= ~ONLCR; // No newline conversion
    tty.c_cc[VTIME] = 0; // No waiting for read()
    tty.c_cc[VMIN] = 0;  // No waiting for read()

    // Set baud rate to 9600
    cfsetspeed(&tty, B9600);

    // Apply settings to connection
    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << endl;
        return 0;
    }

    // Connection success
    cerr << "Successfully connected to serial device at " << port<< "!" << endl;
    return serial_port;
}

int main(int argc, const char** argv) {
    // Open serial port
    int serial_port = initializeSerial("/dev/ttyACM0");

    // Initialize frame and VideoCapture
    Mat camFrame;
    VideoCapture cap;
    if(!cap.open(0)) {
        cerr << "Error " << errno << " from VideoCapture.open(0): " << strerror(errno) << endl;
        return 0;
    }

    cerr << "Successfully connected to camera!" << endl;

    // Load detectors and predictors
    frontal_face_detector frontalFaceDetector = get_frontal_face_detector();
    shape_predictor shapePredictor;
    deserialize("shape_predictor_68_face_landmarks.dat") >> shapePredictor;

    // image window for debugging
    // image_window win;

    while(true) {
        // Set start time
        uint64_t startTime = timeMS();
        // Read frame until it can't
        if(!cap.read(camFrame)){
            break;
        }

        // Convert camera frame to dlib frame
        cv_image<bgr_pixel> dlibFrame(camFrame);

        // Detect faces from converted dlib frame
        std::vector<dlib::rectangle> detectedFaces = frontalFaceDetector(dlibFrame);

        // If you're not the only one detected, skip detection
        if(detectedFaces.size() != 1) {
            cerr << "Detection error!" << endl;
            continue;
        }
        // Else, get your face
        dlib::rectangle face = detectedFaces[0];

        // Detect shapes from your face
        full_object_detection shape = shapePredictor(dlibFrame, face);

        // Dummy vector for drawing
        std::vector<full_object_detection> shapes;
        shapes.push_back(shape);

        // Draw detected shapes on window
        // win.clear_overlay();
        // win.set_image(dlibFrame);
        // win.add_overlay(render_face_detections(shapes));
        
        // Print FPS
        cerr << 1000.0f / (double)(timeMS() - startTime) << " FPS" << endl;
    }

    // End of program, probably will not be called since power would just cut off
    cerr << "Bye!" << endl;
    return 0;
}
