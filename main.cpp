// defines for debugging
#define USE_GUI
#define PRINT_FPS

// Standard C++ headers
#include <iostream>
#include <iomanip>
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
#include <dlib/opencv.h>
#include <dlib/image_io.h>

// DLib GUI headers
#ifdef USE_GUI
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/gui_widgets.h>
#endif

// Default namespaces
using namespace cv;
using namespace dlib;
using namespace std;
using namespace std::chrono;

// Evaluate positive/negative movement
int checkMovement(double derivative, double threshold = 50) {
    if(derivative >= threshold) {
        return 1;
    }
    else if(derivative <= -threshold) {
        return -1;
    }
    else return 0;
}

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
    // cerr float precision
    cerr << setprecision(3);

    // Open serial port
    int serial_port = initializeSerial("/dev/ttyACM0");

    // Initialize frame and VideoCapture
    Mat camFrame, grayFrame, smallFrame;
    VideoCapture cap;
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    if(!cap.open(0, CAP_V4L2)) {
        cerr << "Error " << errno << " from VideoCapture.open(0): " << strerror(errno) << endl;
        return 0;
    }

    cerr << "Successfully connected to camera!" << endl;

    // Variables for detection results
    double leftEyeEAR, rightEyeEAR;
    double mouthWidth = 0, mouthHeight = 0;

    // Additional variables for derivative calculation
    double prevMouthWidth, prevMouthHeight;
    double mouthWidthDerivative, mouthHeightDerivative;

    // Load detectors and predictors
    frontal_face_detector frontalFaceDetector = get_frontal_face_detector();
    shape_predictor shapePredictor;
    deserialize("shape_predictor_68_face_landmarks.dat") >> shapePredictor;

    // Initialize image window
    #ifdef USE_GUI
    image_window win;
    #endif

    while(true) {
        // Set start time
        uint64_t startTime = timeMS();

        // Set prev values for derivative calculation
        prevMouthWidth = mouthWidth;
        prevMouthHeight = mouthHeight;

        // Read frame until it can't
        if(!cap.read(camFrame)){
            cerr << "Error " << errno << " from cap.read(camFrame): " << strerror(errno) << endl;
            break;
        }

        // Convert camera frame to dlib frame
        cv_image<bgr_pixel> dlibFrame(camFrame);

        // Make smallframe
        cvtColor(camFrame, smallFrame, COLOR_BGR2GRAY);
        resize(smallFrame, smallFrame, Size(), 0.333, 0.333);
        cv_image<unsigned char> dlibSmallFrame(smallFrame);

        // Detect faces from converted dlib frame
        std::vector<dlib::rectangle> detectedFaces = frontalFaceDetector(dlibSmallFrame, 0);

        // If you're not the only one detected, skip detection
        if(detectedFaces.size() != 1) {
            cerr << "Detection error!" << endl;
            continue;
        }

        // Else, get your face
        dlib::rectangle face = detectedFaces[0];
        face.set_left(face.left() * 3);
        face.set_right(face.right() * 3);
        face.set_top(face.top() * 3);
        face.set_bottom(face.bottom() * 3);

        // Detect shapes from your face
        full_object_detection shape = shapePredictor(dlibFrame, face);

        // Get EAR values for both eyes
        dlib::vector<double, 2> leftEyeHorizontal, leftEyeVertical, rightEyeHorizontal, rightEyeVertical;
        leftEyeHorizontal = shape.part(39) - shape.part(36);
        leftEyeVertical = (shape.part(37) - shape.part(41) + shape.part(38) - shape.part(40)) / 2;
        rightEyeHorizontal = shape.part(42) - shape.part(45);
        rightEyeVertical = (shape.part(43) - shape.part(47) + shape.part(44) - shape.part(46)) / 2;

        leftEyeEAR = leftEyeVertical.length() / leftEyeHorizontal.length();
        rightEyeEAR = rightEyeVertical.length() / rightEyeHorizontal.length();

        // Get mouth width and height
        dlib::vector<double, 2> mouthHorizontal, mouthVertical;
        mouthHorizontal = shape.part(60) - shape.part(64);
        mouthVertical = shape.part(62) - shape.part(66);

        mouthWidth = mouthHorizontal.length();
        mouthHeight = mouthVertical.length();

        uint64_t finishTime = timeMS();

        mouthWidthDerivative = double(1000) * (mouthWidth - prevMouthWidth) / (double)(finishTime - startTime);
        mouthHeightDerivative = double(1000) * (mouthHeight - prevMouthHeight) / (double)(finishTime - startTime);

        cerr << checkMovement(mouthWidthDerivative, 75) << " " << checkMovement(mouthHeightDerivative, 50) << endl;

        // Display face
        #ifdef USE_GUI
        std::vector<full_object_detection> shapes;
        shapes.push_back(shape);

        win.clear_overlay();
        win.set_image(dlibFrame);
        win.add_overlay(render_face_detections(shapes));
        #endif

        // Print FPS
        #ifdef PRINT_FPS
        cerr << 1000.0f / (double)(timeMS() - startTime) << " FPS" << endl;
        #endif
    }

    // End of program, probably will not be called since power would just cut off
    cerr << "Bye!" << endl;
    return 0;
}
