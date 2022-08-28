#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>   
#include <nuttx/timers/pwm.h>


int fd0;
int fd1;
int fd2;
int fd3;

struct pwm_info_s info;


void setup_pwm(int pwm_num) {

  int fd;
  switch(pwm_num) {
  case 0:
    Serial.println("setup /dev/pwm0");
    fd0 = open("/dev/pwm0", O_RDONLY);
    fd = fd0;
    break;
  case 1:
    Serial.println("setup /dev/pwm1");
    fd1 = open("/dev/pwm1", O_RDONLY);
    fd = fd1;
    break;
  case 2:
    Serial.println("setup /dev/pwm2");
    fd2 = open("/dev/pwm2", O_RDONLY);
    fd = fd2;
    break;
  case 3:
    Serial.println("setup /dev/pwm3");
    fd3 = open("/dev/pwm3", O_RDONLY);
    fd = fd3;
    break;
  default:
    Serial.println("invalid pwm num");
    return;
  }
  
  info.frequency = 500; // Hz
  info.duty      = 0x0000;
  ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  ioctl(fd, PWMIOC_START, 0);
}

void pwm_control(int pwm_num, uint32_t duty) {
  int fd;
  switch (pwm_num) {
  case 0: fd = fd0; break;
  case 1: fd = fd1; break;
  case 2: fd = fd2; break;
  case 3: fd = fd3; break;
  default:
    Serial.println("invalid pwm num");
    return;
  }
  
  info.duty = map(duty, 0x00, 0xff, 0x0000, 0xffff);
  ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  ioctl(fd, PWMIOC_START, 0);
}
