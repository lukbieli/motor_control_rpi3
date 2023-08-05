#include "xbox_controller.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

XboxController::XboxController(const std::string &devicePath) : dev(nullptr), fd(-1)
{
    fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK); // Set non-blocking mode here
    if (fd < 0)
    {
        std::cerr << "Failed to open the event device." << std::endl;
        return;
    }

    int err = libevdev_new_from_fd(fd, &dev);
    if (err < 0)
    {
        std::cerr << "Failed to initialize the libevdev interface: " << strerror(-err) << std::endl;
        close(fd);
        return;
    }

    if (!libevdev_has_event_type(dev, EV_KEY) || !libevdev_has_event_type(dev, EV_ABS))
    {
        std::cerr << "The device is not a proper input device." << std::endl;
        libevdev_free(dev);
        close(fd);
        return;
    }
}


XboxController::~XboxController()
{
    if (dev)
    {
        libevdev_free(dev);
    }
    if (fd >= 0)
    {
        close(fd);
    }
}

bool XboxController::isValid() const
{
    return (dev != nullptr) && (fd >= 0);
}

bool XboxController::readEvent(XB_Event &xboxEvent)
{
    if (!isValid())
    {
        return false;
    }

    
    struct input_event ev;
    int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);


    if (rc == LIBEVDEV_READ_STATUS_SUCCESS)
    {
        if (ev.type == EV_ABS)
        {
            if (ev.code == 9) /* GAS */
            {
                xboxEvent.type = XB_EV_GAS;
                xboxEvent.val = (double)ev.value/(double)this->gas_max;
                return true;
            }
            else if (ev.code == 0) /* X_AXSIS */
            {
                xboxEvent.type = XB_EV_X_AXSIS;
                // if(ev.value > this->x_mid + this->x_dead)
                // {
                //     xboxEvent.val = (double)((this->x_max)-(ev.value))/(double)(this->x_mid - this->x_dead);
                //     return true;
                // }
                // else if(ev.value < this->x_mid - this->x_dead)
                // {
                //     xboxEvent.val = -(double)((ev.value))/(double)(this->x_mid - this->x_dead);
                //     return true;    
                // }
                // else
                // {
                //     xboxEvent.val = 0.0;
                //     return true;                    
                // }
                xboxEvent.val = (double)ev.value/(double)this->x_max;
                return true; 
            }
            else if(ev.code == 10) /* BREAK */
            {
                xboxEvent.type = XB_EV_BREAK;
                xboxEvent.val = (double)ev.value/(double)this->gas_max;
                return true;
            }
        }
        else if(ev.type == EV_MSC)
        {
            if(ev.code == 4 && ev.value == 589832)
            {
                xboxEvent.type = XB_EV_BURGER;
                return true;
            }
        }
    }
    else if (rc == LIBEVDEV_READ_STATUS_SYNC)
    {
        std::cerr << "Error while reading from the device. The device might have been disconnected." << std::endl;
        return false;
    }
    else if( rc == EAGAIN)
    {
        /* no events -  do nothing*/
        return false;
    }
    else
    {
        // std::cerr << "Error while reading from the device: "<< rc << std::endl; // ignore for now
        return false;
    }

    return false;
    // }
}
