#include <libmem.h>
#include <libmem_loader.h>
#include <targets/LPC3230.h>

extern unsigned __IRAM_segment_used_end__, __IRAM_segment_end__;

int main()
{
    uint8_t *flash1_start = (uint8_t *)0xE0000000;
    size_t flash1_size;
    const int flash1_max_geometry_regions = 30;
    libmem_geometry_t flash1_geometry[flash1_max_geometry_regions];
    libmem_flash_info_t flash1_info;
    libmem_driver_handle_t flash1_handle;
    int res;

    // 16-bit flash, respect normal byte-lane select behavior
    EMCStaticConfig0 = 0x00000081;

    res = libmem_register_cfi_driver(&flash1_handle,
                                     flash1_start,
                                     flash1_geometry,
                                     flash1_max_geometry_regions,
                                     &flash1_info);

    if (res != LIBMEM_STATUS_SUCCESS)
    {
        libmem_rpc_loader_exit(res, "Could not register the driver");
        return 0;
    }

    res = libmem_unlock_all();
    if (res != LIBMEM_STATUS_SUCCESS)
    {
        libmem_rpc_loader_exit(res, "Could not unlock the memory");
        return 0;
    }

    libmem_rpc_loader_start(&__IRAM_segment_used_end__, &__IRAM_segment_end__ - 1);

    libmem_rpc_loader_exit(res, NULL);

    return 0;
}