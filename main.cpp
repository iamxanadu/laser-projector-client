#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <liblzr/liblzr.hpp>
#include <cstring>
#include <cobs/cobs.h>
#include <unistd.h>
#include <signal.h>


using namespace lzr;

FrameList frames;

int SERIAL;

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("received SIGINT\n");
        running = 0;
    }
}


int main(int argc, char **argv) {

    if (argc < 4) {
        printf("Usage: %s [filename] [serial port] [interframe delay (ms)]\n", argv[0]);
        return -1;
    }

    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        printf("\ncan't catch SIGINT\n");
    }

    printf("Opening animation %s\n", argv[1]);

    ILDA *f = ilda_open(argv[1], "r");

    printf("Found %zu projector(s)\n", ilda_projector_count(f));
    printf("Found %zu frames for projector 0\n", ilda_frame_count(f, 0));

    if (ilda_read(f, 0, frames) != LZR_SUCCESS) {
        printf("ERROR: failed to read frames from provided file!\n");
        return -1;
    }

    ilda_close(f);

    int interframe_delay = atoi(argv[3]);

    // Open serial port
    SERIAL = open(argv[2], O_RDWR | O_NOCTTY);

    if (SERIAL < 0) {
        printf("Could not open serial port\n");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Error Handling
    if (tcgetattr(SERIAL, &tty) != 0) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    // Set Baud Rate
    cfsetospeed(&tty, (speed_t) B230400);
    cfsetispeed(&tty, (speed_t) B230400);

    // Setting other Port Stuff
    tty.c_cflag &= ~PARENB;            // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    // disable input/output flow control, disable restart chars
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    //disable canonical input, disable echo,
    //disable visually erase chars,
    //disable terminal-generated signals
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // disable output processing
    tty.c_oflag &= ~OPOST;
    tty.c_cflag &= ~CRTSCTS;           // no flow control
    //tty.c_cc[VMIN] = 1;                  // read doesn't block
    //tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines



    if (tcsetattr(SERIAL, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }

    // Flush Port, then applies attributes
    tcflush(SERIAL, TCIFLUSH);

    printf("Serial port connected successfully! Press any key to continue...\n");

    getchar();

    printf("Starting broadcast to STM32. Issue Ctl-C to stop...\n");
    uint8_t inbuf[8];
    uint8_t outbuf[32];

    while (running) {

        for (auto it = frames.begin(); it != frames.end(); ++it) {
            for (auto itt = it->begin(); itt != it->end(); ++itt) {

                // fill buffer
                memcpy(inbuf, &itt->x, 4);
                memcpy(inbuf + 4, &itt->y, 4);

                cobs_encode_result cer = cobs_encode(outbuf, 32, inbuf, 8);

                if (cer.status != COBS_ENCODE_OK) {
                    printf("Failed to encode serial message\n");
                    return -1;
                }

                write(SERIAL, outbuf, cer.out_len);
                write(SERIAL, "\x00", 1); // Delimeter

            }
            usleep(interframe_delay * 1000);
        }
    }

    printf("Stopped! Shutting down...\n");
    close(SERIAL);
    return 0;
}