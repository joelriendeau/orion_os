#pragma once

namespace lpc3230
{

struct timer_client
{
    virtual void timer_isr() = 0;
};

}