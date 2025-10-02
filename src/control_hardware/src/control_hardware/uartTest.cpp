#include <lgpio.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
#include <termios.h>
#include <iomanip>

using namespace std;

const int READING_SIZE = 19;
const char SERIAL_DEVICE[20] = "/dev/serial0";
char buffer[4500];
uint8_t sequence = 0;
float yaw = 0, pitch = 0, roll = 0, xaccel = 0, yaccel = 0, zaccel = 0;

auto start = chrono::high_resolution_clock::now();
auto endTime = chrono::high_resolution_clock::now();

static void read(int h1)
{
  auto dataAvailable = lgSerialDataAvailable(h1);
  if (dataAvailable >= READING_SIZE)
  {
    cout << dec << dataAvailable << " bytes available." << endl;
    
    auto bytesRead = dataAvailable >= READING_SIZE * 2 ? lgSerialRead(h1, buffer, dataAvailable) :
                                                         lgSerialRead(h1, buffer, READING_SIZE);
    if (bytesRead < READING_SIZE)
    {
      cout << "WARN: Failed to read all data available (" << dec << bytesRead << ")" << endl
           << "----------------------" << endl;
      return;  // ReturnStatus::FAILURE
    }

    char* reading = NULL;
    for (int i = dataAvailable - READING_SIZE; i >= 0; i--)
    {
      if (buffer[i] == 0xAA && buffer[i + 1] == 0xAA)
      {
        reading = buffer + i + 2;
        break;
      }
    }
    if (reading == NULL)
    {
      cout << "ERROR: reading (" << dataAvailable
           << ") did not contain 0xAAAA header with enough space for the full message" << endl
           << "--------------------------" << endl;
      for (int i = 0; i < dataAvailable; i++)
      {
        cout << hex << uppercase << static_cast<int>(buffer[i]);
      }
      cout << endl;
      return;
    }
    endTime = chrono::high_resolution_clock::now();
    auto frequency = endTime <= start ? 0 : 1 / ((endTime - start).count() / pow(10.0, 9));
    start = endTime;
    cout << "Frequency: " << frequency << "Hz" << endl;

    sequence = reading[0];
    yaw = (int16_t)(reading[1] + (reading[2] << 8)) * 0.01;
    pitch = (int16_t)(reading[3] + (reading[4] << 8)) * 0.01;
    roll = (int16_t)(reading[5] + (reading[6] << 8)) * 0.01;
    xaccel = (int16_t)(reading[7] + (reading[8] << 8)) * 0.01;
    yaccel = (int16_t)(reading[9] + (reading[10] << 8)) * 0.01;
    zaccel = (int16_t)(reading[11] + (reading[12] << 8)) * 0.01;

    cout << dec << "Index: " << static_cast<int>(sequence) << endl
         << fixed << setprecision(2) 
         << "Yaw: " << yaw << ", Pitch: " << pitch << ", Roll: " << roll << " Degrees" << endl
         << "Acceleration X: " << xaccel << ", Y: " << yaccel << ", Z: " << zaccel << " m/s^2" << endl
         << endl;

    return;
  }
  else
  {
    cout << "data did not contain enough bytes for a full message.  Consider decreasing read frequency." << endl;
  }
}

static void test(int h1)
{
  int dataAvailable = lgSerialDataAvailable(h1);
  if (dataAvailable > 0)
  {
    cout << dec << dataAvailable << endl;
    char buffer[dataAvailable];
    auto dataRead = lgSerialRead(h1, buffer, READING_SIZE);
    if (dataRead != READING_SIZE)
    {
      cout << "Warn: didn't read all data." << endl;
    }
  }
  else
  {
    cout << "Too fast! Consider slowing down read frequency." << endl;
  }
}

int main()
{
  //    int fd;
  //    if ((fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  //    {
  //            printf("Could not open serial device %s: %d", SERIAL_DEVICE, fd);
  //    }

  int h1 = lgSerialOpen(SERIAL_DEVICE, 115200, 0);
  if (h1 < 0)
  {
    cout << "Could not get device handle for " << SERIAL_DEVICE << ": " << h1 << endl;
  }
  tcflush(h1, TCIOFLUSH);
  //    if(lgSerialWriteByte(h1, 0x33) != 0)
  //        printf("Failed to write byte %c\n", 0x33);
  while (true)
  {
    read(h1);
    // test(h1);

    this_thread::sleep_for(chrono::milliseconds(10));
  }
  lgSerialClose(h1);

  //	int gpioHandle = lgGpiochipOpen(0);
  //	int read = lgGpioRead(gpioHandle, 17);
  //	printf("GPIO 17 was %d\n", read);
  //	lgGpiochipClose(gpioHandle);
}
