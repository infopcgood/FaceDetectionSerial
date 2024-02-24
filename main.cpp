#include <iostream>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/opencv.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>

using namespace cv;
using namespace dlib;
using namespace std;

int initializeSerial(const char* port) {
    // Initialize USB serial device
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

    cfsetspeed(&tty, B9600);

    // Apply settings to connection
    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << endl;
        return 0;
    }

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

    frontal_face_detector frontalFaceDetector = get_frontal_face_detector();
    shape_predictor shapePredictor;
    deserialize("shape_predictor_68_face_landmarks.dat") >> shapePredictor;

    image_window win;

    while(true) {
        if(!cap.read(camFrame)){
            break;
        }

        cv_image<bgr_pixel> dlibFrame(camFrame);

        std::vector<dlib::rectangle> detectedFaces = frontalFaceDetector(dlibFrame);

        if(detectedFaces.size() != 1) {
            cerr << "Detection error!" << endl;
            continue;
        }
        dlib::rectangle face = detectedFaces[0];

        full_object_detection shape = shapePredictor(dlibFrame, face);

        std::vector<full_object_detection> shapes;
        shapes.push_back(shape);

        cerr << detectedFaces.size() << endl;

        win.clear_overlay();
        win.set_image(dlibFrame);
        win.add_overlay(render_face_detections(shapes));
    }

    cerr << "Bye!" << endl;

    return 0;
}