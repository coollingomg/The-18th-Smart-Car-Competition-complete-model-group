#include "../include/uart.hpp"
#include <iostream>
#include "../include/serial.hpp"

using namespace std;

SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_460800);
int main()
{
	int ret = serialInterface.open();
	if (ret != 0)
		return 0;
	serialInterface.Start();


    while(1)
    {
        cout << "Buzzer Count: ";
        unsigned char n;
        cin >> n;
		serialInterface.buzzerSound(n);
    }
}