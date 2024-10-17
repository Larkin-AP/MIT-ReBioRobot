#include <cstdio>


#include "../imu_receiver.h"
#include "Math/orientation_tools.h"


#include <stdio.h>
#include <unistd.h>


int main(int argc, char** argv) {
  std::string com_port;
  u32 baudrate;
  if(argc != 2) {
    printf("usage: imu-test com-port \n");
    return 1;
  }

  com_port = argv[1];
  baudrate = 921600;//std::atoi(argv[2]);

  IMUReceiver imu;
  if(imu.openPort(com_port, baudrate)) {
    imu.start();
    while(true) {
      usleep(10000);
      float imu1 = imu.imu_data[1];
      std::cout << imu1 << std::endl;
      //std::cout << "Q: " << imu.quat.transpose() << "\n";
      //Vec3<float> rpy = ori::quatToRPY(imu.quat); (void)rpy;
      //Vec3<float> acc = imu.acc; (void)acc;
      //Vec3<float> ang = imu.gyro; (void)ang;
      //printf("rpy: %.3f %.3f %.3f\n", rpy[0], rpy[1], rpy[2]);
      //printf("acc: %.3f %.3f %.3f\n", acc[0], acc[1], acc[2]);
      //printf("ang: %.3f %.3f %.3f\n", ang[0], ang[1], ang[2]);
    }
  }


  return 0;
}
