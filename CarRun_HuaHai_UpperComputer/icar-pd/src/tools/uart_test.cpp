#include "../include/uart.hpp"
#include <iostream>
#include "../include/serial.hpp"

using namespace std;

SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_115200);
int main()
{
    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
    {
        cout << "-------- 速度闭环控制 -------" << endl;
        serialInterface.set_PID(9000, 12.5, 4.5);
    }
	serialInterface.Start();


    while(1)
    {
        // cout << "Buzzer Count: ";
        // unsigned char n;
        // cin >> n;
        float speed;
        cin >> speed;
		serialInterface.set_control(speed, 1500);
    }
}