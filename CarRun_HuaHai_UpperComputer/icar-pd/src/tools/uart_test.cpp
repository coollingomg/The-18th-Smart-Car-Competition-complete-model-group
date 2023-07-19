#include "../include/uart.hpp"
#include <iostream>
#include "../include/serial.hpp"
#include "motion_controller.cpp"

using namespace std;

SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_460800);
int main()
{
    MotionController motionController;         // 运动控制

    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
    {
        cout << "-------- 速度闭环控制 -------" << endl;
        serialInterface.set_PID(motionController.params.Kp, motionController.params.Ki, motionController.params.Kd, motionController.params.Kv);
        cout << "Kp = " << motionController.params.Kp << endl;
        cout << "Ki = " << motionController.params.Ki << endl;
        cout << "Kd = " << motionController.params.Kd << endl;
        cout << "Kv = " << motionController.params.Kv << endl;
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
        cin >> speed >> servo;
		serialInterface.set_control(speed, servo);
    }
}
