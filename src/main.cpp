#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

class Serial {
 public:
  Serial(const std::string& device, speed_t baud);
  ~Serial(void);

  bool WaitReady(void);
  int GetSpeed(speed_t& speed);
  ssize_t Write(const void* buf, size_t count);
  ssize_t Read(void* buf, size_t count);

  int SendCommand(const std::string& command);
  int RecvReply(std::vector<uint8_t>& reply);
  int DoCommand(const std::string& command);
  
 private:
  int InitTty(speed_t baud, int parity);

  std::string device_;
  speed_t baud_;
  int fd_;
};

Serial::Serial(const std::string& device, speed_t baud)
  : device_(device), baud_(baud), fd_(-1) {

  //int oflags = (O_RDWR|O_NOCTTY|O_NONBLOCK);
  int oflags = (O_RDWR|O_NOCTTY);

  fd_ = open(device_.c_str(), oflags);
  if (fd_ < 0) {
    std::cerr << "open() failed: " << strerror(errno) << std::endl;
    return;
  }

  int rv;
  if ((rv = InitTty(baud, 0)) < 0) {
    std::cerr << "InitTty() failed: " << strerror(-rv) << std::endl;
    return;
  }
}
Serial::~Serial(void) {
  close(fd_);
}
bool Serial::WaitReady(void) {
  sleep(2);
  return true;
}
int Serial::InitTty(speed_t baud, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // get terminal attributes
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "tcgetattr() failed: " << strerror(errno) << std::endl;
    return -errno;
  }

  // set speed
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // disable break processing
  tty.c_iflag &= ~IGNBRK;
  // shut off xon/xoff ctrl
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  
  tty.c_oflag = 0;

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag |= ((parity > 0) ? PARODD : 0);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag = 0;

  // non-blocking read
  tty.c_cc[VMIN]  = 0;
  // 0.5 second read timeout
  tty.c_cc[VTIME] = 5;

  // flush
  if (tcflush(fd_, TCIOFLUSH) != 0) {
    std::cerr << "tcflush() failed: " << strerror(errno) << std::endl;
    return -errno;
  }

  // set terminal attributes
  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    std::cerr << "tcsetattr() failed: " << strerror(errno) << std::endl;
    return -errno;
  }
  
  return 0;
}


int Serial::GetSpeed(speed_t& speed) {
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "tcgetattr() failed: " << strerror(errno) << std::endl;
    return -1;
  }
  speed = cfgetospeed(&tty);
  return 0;
}
ssize_t Serial::Write(const void* buf, size_t count) {
  ssize_t rv = write(fd_, buf, count);
  if (rv < 0) {
    std::cerr << "write() failed: " << strerror(errno) << std::endl;
    return -errno;
  }
  return rv;
}
ssize_t Serial::Read(void* buf, size_t count) {
  ssize_t rv = read(fd_, buf, count);
  if (rv < 0) {
    std::cerr << "read() failed: " << strerror(errno) << std::endl;
    return -errno;
  }
  return rv;
}



int Serial::SendCommand(const std::string& command) {
  ssize_t rv;
  if ((rv = write(fd_, command.c_str(), command.length())) < 0) {
    std::cerr << "Write() failed: " << strerror(-rv) << std::endl;
    return -1;
  }
  if ((rv = write(fd_, "\r", 1)) < 0) {
    std::cerr << "Write() failed: " << strerror(-rv) << std::endl;
    return -1;
  }
  return 0;
}
int Serial::RecvReply(std::vector<uint8_t>& reply) {
  ssize_t rv;

  while (true) {
    uint8_t readbyte;
    rv = read(fd_, &readbyte, 1);

    //std::cerr << "read(): rv=" << rv << ", readbyte=" << static_cast<int>(readbyte) << std::endl;

    if (rv < 0) {
      std::cerr << "read() failed: " << strerror(errno) << std::endl;
      return -errno;
    } else if (rv == 1) {
      if (readbyte == '\n') {
        break;
      } else {
        reply.push_back(readbyte);
      }
    }
  }
  return 0;
}

int Serial::DoCommand(const std::string& command) {
  std::cerr << "DoCommand: command=\"" << command << "\"" << std::endl;

  int rv;
  if ((rv = SendCommand(command)) < 0) {
    std::cerr << "SendCommand() failed: " << strerror(-rv) << std::endl;
    return rv;
  }
  std::vector<uint8_t> reply;
  if ((rv = RecvReply(reply)) < 0) {
    std::cerr << "RecvReply() failed: " << strerror(-rv) << std::endl;
    return rv;
  }
  reply.push_back('\0');

  std::string replystr(reply.begin(), reply.end());
  std::cerr << "DoCommand: reply=\"" << replystr << "\"" << std::endl;

  if (reply[0] == 'O' && reply[1] == 'K') {
    return 0;
  } else if (reply[0] == 'E') {
    int errorcode = std::stoi(replystr.substr(1));
    return errorcode;
  } else {
    return -EINVAL;
  }
}




class Controller {
 public:
  Controller(std::shared_ptr<Serial>& serial)
    : serial_(serial) {}
  void Run(void);
 private:
  std::shared_ptr<Serial> serial_;
};

void Controller::Run(void) {
  if (!serial_->WaitReady()) {
    std::cerr << "WaitReady() failed" << std::endl;
    return;
  }
  
  std::chrono::time_point<std::chrono::system_clock> start =
    std::chrono::system_clock::now();

  serial_->DoCommand("R");

  while (true) {
    std::chrono::duration<float> elapsed =
      std::chrono::system_clock::now() - start;

    float elapsed_secs = elapsed.count();
    if (elapsed_secs < 2.0) {
      serial_->DoCommand("S0");
      serial_->DoCommand("T05");
    } else if (elapsed_secs < 4.0) {
      serial_->DoCommand("S30");
      serial_->DoCommand("T05");
    } else if (elapsed_secs < 6.0) {
      serial_->DoCommand("S0");
      serial_->DoCommand("T0");
    } else if (elapsed_secs < 8.0) {
      serial_->DoCommand("S-30");
      serial_->DoCommand("T-05");
    } else if (elapsed_secs < 10.0) {
      serial_->DoCommand("S0");
      serial_->DoCommand("T-05");
    } else {
      serial_->DoCommand("S0");
      serial_->DoCommand("T0");
      break;
    }
    usleep(1000);
  }
}


int main(int argc, char** argv) {
  std::shared_ptr<Serial> serial = std::make_shared<Serial>("/dev/ttyACM0", B115200);

  Controller ctrl(serial);
  ctrl.Run();
  return 0;
}
