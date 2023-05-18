#include "../include/uart.hpp"
#include <iostream>

using namespace std;

Driver driver("/dev/ttyUSB0", BaudRate::BAUD_115200);
int main()
{
    int ret = driver.open();
    if(ret != 0)
    {
        cout << "uart open failed" << endl;
        return -1;
    }

    float Kp, Ki, Kd;
    cout << "PID: ";
    cin >> Kp >> Ki >> Kd;
    driver.PID_init(Kp, Ki, Kd);

    while(1)
    {
        cout << "Speed: ";
        float speed;
        cin >> speed;
        driver.carControl(speed, 1580);
    }
}