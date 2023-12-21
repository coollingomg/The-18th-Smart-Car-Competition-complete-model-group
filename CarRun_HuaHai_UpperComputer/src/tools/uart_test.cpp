#include "../include/uart.hpp"
#include <iostream>
#include "../include/serial.hpp"
#include "motion_control.cpp"
#include "../include/stop_watch.hpp"

using namespace std;

SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_460800);
int main()
{
    MotionController motionController;         // 运动控制
    StopWatch stopWatch;

    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
    {
        cout << "-------- 速度闭环控制 -------" << endl;
        serialInterface.set_PID(motionController.params.Kp_speed, motionController.params.Ki_speed, motionController.params.Kd_speed, 
                                motionController.params.Kp_current, motionController.params.Ki_current, motionController.params.Kd_current);
        cout << "Kp_speed = " << motionController.params.Kp_speed << endl;
        cout << "Ki_speed = " << motionController.params.Ki_speed << endl;
        cout << "Kd_speed = " << motionController.params.Kd_speed << endl;
        cout << "Kp_current = " << motionController.params.Kp_current << endl;
        cout << "Ki_current = " << motionController.params.Ki_current << endl;
        cout << "Kd_current = " << motionController.params.Kd_current << endl;
    }
	serialInterface.Start();


    while(1)
    {
        // cout << "Buzzer Count: ";
        // unsigned char n;
        // cin >> n;
        float speed = 0.0f;
        uint16_t servo = 1500;
        // cin >> speed;
        cout << "speed\tservo" << endl;
        cin >> speed >> servo;
        stopWatch.tic();
		serialInterface.set_control(-speed, servo);
        // cout << stopWatch.toc << endl;
    }
}
