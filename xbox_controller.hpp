#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include <string>
#include <libevdev/libevdev.h>
#include <libevdev/libevdev-uinput.h>

typedef enum XB_Event_type
{
    XB_EV_GAS = 0,
    XB_EV_X_AXSIS,
    XB_EV_BREAK,
    XB_EV_BURGER /* type 4 (EV_MSC), code 4 (MSC_SCAN), value 90008 */
} XB_Event_type;

typedef struct XB_Event
{
    XB_Event_type type;
    double val; /* 0 - 1 */
} XB_Event;

class XboxController
{
private:
    struct libevdev *dev;
    int fd;

public:
    XboxController(const std::string &devicePath);
    ~XboxController();

    bool isValid() const;
    bool readEvent(XB_Event &xboxEvent); // Updated readEvent function

    const int gas_max = 1023;
    const int x_max = 65535;
    const int x_mid = 32768;
    const int x_dead = 1000;
};

#endif // XBOXCONTROLLER_H
