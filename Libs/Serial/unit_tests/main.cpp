#include "..\windows_portable_uart.hpp"
#include "..\uart_logger.hpp"

#include <windows.h>
#include <assert.h>

void test()
{
    uart::ctrl portable_uart;
    bool port_open;
    #ifdef WINCE
        port_open = com_port.init(1, 115200, uart::parity::none, 1024);
    #else
        port_open = portable_uart.init(4, 115200, uart::parity::none, 1024);
    #endif
    assert(port_open);

    
}

int main(int argc, char* argv[])
{
    test();
    getchar();
    return 0;
}