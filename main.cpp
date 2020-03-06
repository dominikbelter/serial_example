// Example from https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

int set_interface_attribs (int fd, int speed, int parity) {
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0) {
                std::cout << "error " << errno << "from tcgetattr\n";
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0) {
                std::cout << "error " <<errno << " from tcsetattr\n";
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block) {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0){
                std::cout << "error " << errno << " from tggetattr\n";
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                std::cout << "error " << errno << " setting term attributes\n";
}

int main(){
    std::string portname = "/dev/..."; //set proper port name

    int fd = open (portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
            std::cout << "error " << errno << " opening " << portname << ": " << strerror (errno) << "\n";
            return 0;
    }

    set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 1);                // set no blocking

    ssize_t bytesNo = write (fd, "...", 1);  // write proper character
    usleep (100);             // sleep enough to transmit the 7 plus
    if (bytesNo==0){
        std::cout << "Writing data failed\n";
        return 0;
    }

    while (1){
        char buf[1];
        size_t n = read (fd, buf, sizeof(buf));  // read up to 100 characters if ready to read
        if (n>0){
            for (const auto& c : buf){
                std::cout << c;
            }
            std::cout << "\n";
        }
    }
}
