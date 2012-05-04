#pragma once

#include "modules/init/globals.hpp"

// the simulator loads raw data from a file on the SD card, to be used as data for the GPS uart

#if ENABLE_GPS_BENCHMARKS

#include "modules/uart/uart_ctrl.hpp"
#include "GNSSCom/GNSSCom.hpp"
#include "Base/base.hpp"
#include "Rover/rover.hpp"

namespace benchmarks {

class base_from_logs
{
public:
    void run()
    {
        profile_begin("load_raw");
        fs::FILE raw_file;
        fs::fopen(&raw_file, "raw_uart.dat", 'r', true);
        u32 size = raw_file.fileinfo.filelen;
        if (size > buffer_size)
            size = buffer_size;
        fs::fread(data_buffer, size, 1, &raw_file);
        fs::fclose(&raw_file);
        profile_end();

        profile_begin("init_processing");
        get_gps_uart_io().init(data_buffer, size);

        gnss_com_ctrl.gnssport_open(&get_gps_uart_io(), 0, base_ctrl.get_datain());
        gnss_com_ctrl.gnssport_init();

        u32 bdata_index = 0;
        pbdata = &bdata[bdata_index++];
        base_ctrl.start(&pbdata);
        profile_end();

        profile_begin("compute_base");
        while (bdata_index < 20)
        {
            u32 state = gnss_com_ctrl.gnssrx_getnav();
            if (0 == state)
                break;
            state = base_ctrl.process();
            if ((state & base::BASEERR_NRDY) == 0)
                pbdata = &bdata[bdata_index++];
        }
        profile_end();

        profile_begin("save_cooked");
        fs::FILE cooked_file;
        fs::fopen(&cooked_file, "cooked.txt", 'w', true);
        for (u32 i = 0; i < bdata_index - 1; ++i)
        {
            fs::fprintf(&cooked_file, "PASS %d\r\n", i);
            fs::fprintf(&cooked_file, "datavalid : %s\r\n", bdata[i].datavalid ? "yes" : "no");
            fs::fprintf(&cooked_file, "rssi : %d\r\n", bdata[i].rssi);
            fs::fprintf(&cooked_file, "rxpckts : %d\r\n", bdata[i].rxpckts);
            fs::fprintf(&cooked_file, "rxbadpckts : %d\r\n", bdata[i].rxbadpckts);
            fs::fprintf(&cooked_file, "rxprgpckts : %d\r\n", bdata[i].rxprgpckts);
            fs::fprintf(&cooked_file, "tow : %f\r\n", bdata[i].tow);
            fs::fprintf(&cooked_file, "statusok : %d\r\n", bdata[i].statusok);
            fs::fprintf(&cooked_file, "battery : %d\r\n", bdata[i].battery);
            fs::fprintf(&cooked_file, "status0 : %d\r\n", bdata[i].status0);
            for (u32 j = 0; j < GNSS_CHAN; ++j)
            {
                fs::fprintf(&cooked_file, "prn[%d] : %d\r\n", j, bdata[i].prn[j]);
                fs::fprintf(&cooked_file, "meas[%d].qli : %d\r\n", j, bdata[i].meas[j].qli);
                fs::fprintf(&cooked_file, "meas[%d].cwarning : %d\r\n", j, bdata[i].meas[j].cwarning);
                fs::fprintf(&cooked_file, "meas[%d].pr : %f\r\n", j, bdata[i].meas[j].pr);
                fs::fprintf(&cooked_file, "meas[%d].pv : %f\r\n", j, bdata[i].meas[j].pv);
                fs::fprintf(&cooked_file, "meas[%d].cp : %f\r\n", j, bdata[i].meas[j].cp);
                fs::fprintf(&cooked_file, "meas[%d].cn0 : %d\r\n", j, bdata[i].meas[j].cn0);
                fs::fprintf(&cooked_file, "locktime[%d] : %d\r\n", j, bdata[i].locktime[j]);
                fs::fprintf(&cooked_file, "hcslip[%d] : %d\r\n", j, bdata[i].hcslip[j]);
            }
            fs::fprintf(&cooked_file, "pvalid : %d\r\n", bdata[i].pvalid);
            fs::fprintf(&cooked_file, "x : %f\r\n", bdata[i].x);
            fs::fprintf(&cooked_file, "y : %f\r\n", bdata[i].y);
            fs::fprintf(&cooked_file, "z : %f\r\n", bdata[i].z);
            fs::fprintf(&cooked_file, "pacc : %f\r\n", bdata[i].pacc);
        }
        fs::fclose(&cooked_file);
        profile_end();
    }

    static void static_thread(void* argument)
    {
        get_base_bench().run();
    }

private:
    static const u32 buffer_size = 1024 * 128;
    u8 data_buffer[buffer_size];

    gnss_com::ctrl gnss_com_ctrl;

    base::ctrl base_ctrl;
    basedata bdata[20];
    basedata* pbdata;
};

class rover_from_logs
{
public:
    void run()
    {
        u32 sample_number;

        profile_begin("load_raw");

        fs::FILE raw_file;
        fs::fopen(&raw_file, "base.dat", 'r', true);
        u32 size = raw_file.fileinfo.filelen;
        if (size > base_buffer_size)
            size = base_buffer_size;
        sample_number = 60; // 60
        fs::fread(base_data_buffer, size, 1, &raw_file);
        fs::fclose(&raw_file);
        base_data_walker = base_data_buffer;

        fs::fopen(&raw_file, "rover.dat", 'r', true);
        size = raw_file.fileinfo.filelen;
        if (size > gnss_buffer_size)
            size = gnss_buffer_size;
        fs::fread(gnss_data_buffer, size, 1, &raw_file);
        fs::fclose(&raw_file);
        gnss_data_walker = gnss_data_buffer;

        fs::fopen(&raw_file, "eph.dat", 'r', true);
        size = raw_file.fileinfo.filelen;
        if (size > rover_eph_size)
            size = rover_eph_size;
        fs::fread(rover_eph_buffer, size, 1, &raw_file);
        fs::fclose(&raw_file);
        rover_eph_walker = rover_eph_buffer;

        profile_end();

        profile_begin("init_processing");

        rover_ctrl.start();

        load_ephemeris(rover_ctrl.get_gps_ephemeris(), GPSL1_SVN);

        for (u32 sample = 0; sample < sample_number; ++sample)
        {
            load_basedata(**rover_ctrl.get_basedatain());

            for (u32 rover_loop = 0; rover_loop < 10; ++rover_loop)
            {
                if (rover_loop == 0)
                    load_gnss_nav(**rover_ctrl.get_datain());
                load_gnss_meas(**rover_ctrl.get_datain());
                rover_ctrl.process();
            }
        }

        profile_end();

        get_central().send_message(msg::src::main, msg::id::shutdown_request); // we're done here
    }

    static void static_thread(void* argument)
    {
        get_rover_bench().run();
    }

private:
    void load_ephemeris(ephemeris* eph, u32 len)
    {
        for (u32 i = 0; i < len; ++i)
        {
            memcpy(&eph[i].valid, rover_eph_walker, sizeof(eph[i].valid));         rover_eph_walker += sizeof(eph[i].valid);
            memcpy(&eph[i].WNe, rover_eph_walker, sizeof(eph[i].WNe));             rover_eph_walker += sizeof(eph[i].WNe);
            memcpy(&eph[i].fitintvl, rover_eph_walker, sizeof(eph[i].fitintvl));   rover_eph_walker += sizeof(eph[i].fitintvl);
            memcpy(&eph[i].health, rover_eph_walker, sizeof(eph[i].health));       rover_eph_walker += sizeof(eph[i].health);
            memcpy(&eph[i].toc, rover_eph_walker, sizeof(eph[i].toc));             rover_eph_walker += sizeof(eph[i].toc);
            memcpy(&eph[i].toe, rover_eph_walker, sizeof(eph[i].toe));             rover_eph_walker += sizeof(eph[i].toe);
            memcpy(&eph[i].Tgd, rover_eph_walker, sizeof(eph[i].Tgd));             rover_eph_walker += sizeof(eph[i].Tgd);
            memcpy(&eph[i].af2, rover_eph_walker, sizeof(eph[i].af2));             rover_eph_walker += sizeof(eph[i].af2);
            memcpy(&eph[i].af1, rover_eph_walker, sizeof(eph[i].af1));             rover_eph_walker += sizeof(eph[i].af1);
            memcpy(&eph[i].af0, rover_eph_walker, sizeof(eph[i].af0));             rover_eph_walker += sizeof(eph[i].af0);
            memcpy(&eph[i].M0, rover_eph_walker, sizeof(eph[i].M0));               rover_eph_walker += sizeof(eph[i].M0);
            memcpy(&eph[i].deltan, rover_eph_walker, sizeof(eph[i].deltan));       rover_eph_walker += sizeof(eph[i].deltan);
            memcpy(&eph[i].sqra, rover_eph_walker, sizeof(eph[i].sqra));           rover_eph_walker += sizeof(eph[i].sqra);
            memcpy(&eph[i].e, rover_eph_walker, sizeof(eph[i].e));                 rover_eph_walker += sizeof(eph[i].e);
            memcpy(&eph[i].OMEGA0, rover_eph_walker, sizeof(eph[i].OMEGA0));       rover_eph_walker += sizeof(eph[i].OMEGA0);
            memcpy(&eph[i].i0, rover_eph_walker, sizeof(eph[i].i0));               rover_eph_walker += sizeof(eph[i].i0);
            memcpy(&eph[i].omega, rover_eph_walker, sizeof(eph[i].omega));         rover_eph_walker += sizeof(eph[i].omega);
            memcpy(&eph[i].OMEGADOT, rover_eph_walker, sizeof(eph[i].OMEGADOT));   rover_eph_walker += sizeof(eph[i].OMEGADOT);
            memcpy(&eph[i].IDOT, rover_eph_walker, sizeof(eph[i].IDOT));           rover_eph_walker += sizeof(eph[i].IDOT);
            memcpy(&eph[i].Cuc, rover_eph_walker, sizeof(eph[i].Cuc));             rover_eph_walker += sizeof(eph[i].Cuc);
            memcpy(&eph[i].Cus, rover_eph_walker, sizeof(eph[i].Cus));             rover_eph_walker += sizeof(eph[i].Cus);
            memcpy(&eph[i].Crc, rover_eph_walker, sizeof(eph[i].Crc));             rover_eph_walker += sizeof(eph[i].Crc);
            memcpy(&eph[i].Crs, rover_eph_walker, sizeof(eph[i].Crs));             rover_eph_walker += sizeof(eph[i].Crs);
            memcpy(&eph[i].Cic, rover_eph_walker, sizeof(eph[i].Cic));             rover_eph_walker += sizeof(eph[i].Cic);
            memcpy(&eph[i].Cis, rover_eph_walker, sizeof(eph[i].Cis));             rover_eph_walker += sizeof(eph[i].Cis);
            memcpy(&eph[i].ura, rover_eph_walker, sizeof(eph[i].ura));             rover_eph_walker += sizeof(eph[i].ura);
        }
    }

    void load_basedata(basedata& dest)
    {
        memcpy(&dest.datavalid, base_data_walker, sizeof(dest.datavalid));    base_data_walker += sizeof(dest.datavalid);
        memcpy(&dest.rssi, base_data_walker, sizeof(dest.rssi));              base_data_walker += sizeof(dest.rssi);
        memcpy(&dest.rxpckts, base_data_walker, sizeof(dest.rxpckts));        base_data_walker += sizeof(dest.rxpckts);
        memcpy(&dest.rxbadpckts, base_data_walker, sizeof(dest.rxbadpckts));  base_data_walker += sizeof(dest.rxbadpckts);
        memcpy(&dest.rxprgpckts, base_data_walker, sizeof(dest.rxprgpckts));  base_data_walker += sizeof(dest.rxprgpckts);
        memcpy(&dest.tow, base_data_walker, sizeof(dest.tow));                base_data_walker += sizeof(dest.tow);
        memcpy(&dest.statusok, base_data_walker, sizeof(dest.statusok));      base_data_walker += sizeof(dest.statusok);
        memcpy(&dest.battery, base_data_walker, sizeof(dest.battery));        base_data_walker += sizeof(dest.battery);
        memcpy(&dest.status0, base_data_walker, sizeof(dest.status0));        base_data_walker += sizeof(dest.status0);
        for (u32 i = 0; i < GNSS_CHAN; ++i)
        {
            memcpy(&dest.prn[i], base_data_walker, sizeof(dest.prn[i]));                      base_data_walker += sizeof(dest.prn[i]);
            memcpy(&dest.meas[i].qli, base_data_walker, sizeof(dest.meas[i].qli));            base_data_walker += sizeof(dest.meas[i].qli);
            memcpy(&dest.meas[i].cwarning, base_data_walker, sizeof(dest.meas[i].cwarning));  base_data_walker += sizeof(dest.meas[i].cwarning);
            memcpy(&dest.meas[i].cn0, base_data_walker, sizeof(dest.meas[i].cn0));            base_data_walker += sizeof(dest.meas[i].cn0);
            memcpy(&dest.meas[i].pr, base_data_walker, sizeof(dest.meas[i].pr));              base_data_walker += sizeof(dest.meas[i].pr);
            memcpy(&dest.meas[i].pv, base_data_walker, sizeof(dest.meas[i].pv));              base_data_walker += sizeof(dest.meas[i].pv);
            memcpy(&dest.meas[i].cp, base_data_walker, sizeof(dest.meas[i].cp));              base_data_walker += sizeof(dest.meas[i].cp);
            memcpy(&dest.locktime[i], base_data_walker, sizeof(dest.locktime[i]));            base_data_walker += sizeof(dest.locktime[i]);
            memcpy(&dest.hcslip[i], base_data_walker, sizeof(dest.hcslip[i]));                base_data_walker += sizeof(dest.hcslip[i]);
        }
        memcpy(&dest.pvalid, base_data_walker, sizeof(dest.pvalid));          base_data_walker += sizeof(dest.pvalid);
        memcpy(&dest.x, base_data_walker, sizeof(dest.x));                    base_data_walker += sizeof(dest.x);
        memcpy(&dest.y, base_data_walker, sizeof(dest.y));                    base_data_walker += sizeof(dest.y);
        memcpy(&dest.z, base_data_walker, sizeof(dest.z));                    base_data_walker += sizeof(dest.z);
        memcpy(&dest.pacc, base_data_walker, sizeof(dest.pacc));              base_data_walker += sizeof(dest.pacc);
    }

    void load_gnss_nav(gnssdata& dest)
    {   
        memcpy(&dest.tvalid, gnss_data_walker, sizeof(dest.tvalid));        gnss_data_walker += sizeof(dest.tvalid);
        memcpy(&dest.pos.qli, gnss_data_walker, sizeof(dest.pos.qli));      gnss_data_walker += sizeof(dest.pos.qli);
        memcpy(&dest.tow, gnss_data_walker, sizeof(dest.tow));              gnss_data_walker += sizeof(dest.tow);
        memcpy(&dest.utc, gnss_data_walker, sizeof(dest.utc));              gnss_data_walker += sizeof(dest.utc);
        memcpy(&dest.wn, gnss_data_walker, sizeof(dest.wn));                gnss_data_walker += sizeof(dest.wn);
        memcpy(&dest.pos.x, gnss_data_walker, sizeof(dest.pos.x));          gnss_data_walker += sizeof(dest.pos.x);
        memcpy(&dest.pos.y, gnss_data_walker, sizeof(dest.pos.y));          gnss_data_walker += sizeof(dest.pos.y);
        memcpy(&dest.pos.z, gnss_data_walker, sizeof(dest.pos.z));          gnss_data_walker += sizeof(dest.pos.z);
        memcpy(&dest.pos.vx, gnss_data_walker, sizeof(dest.pos.vx));        gnss_data_walker += sizeof(dest.pos.vx);
        memcpy(&dest.pos.vy, gnss_data_walker, sizeof(dest.pos.vy));        gnss_data_walker += sizeof(dest.pos.vy);
        memcpy(&dest.pos.vz, gnss_data_walker, sizeof(dest.pos.vz));        gnss_data_walker += sizeof(dest.pos.vz);
        memcpy(&dest.pos.lat, gnss_data_walker, sizeof(dest.pos.lat));      gnss_data_walker += sizeof(dest.pos.lat);
        memcpy(&dest.pos.lon, gnss_data_walker, sizeof(dest.pos.lon));      gnss_data_walker += sizeof(dest.pos.lon);
        memcpy(&dest.pos.hellip, gnss_data_walker, sizeof(dest.pos.hellip));gnss_data_walker += sizeof(dest.pos.hellip);
        memcpy(&dest.pos.und, gnss_data_walker, sizeof(dest.pos.und));      gnss_data_walker += sizeof(dest.pos.und);
        memcpy(&dest.pos.b, gnss_data_walker, sizeof(dest.pos.b));          gnss_data_walker += sizeof(dest.pos.b);
        memcpy(&dest.pos.vb, gnss_data_walker, sizeof(dest.pos.vb));        gnss_data_walker += sizeof(dest.pos.vb);
        memcpy(&dest.pos.gdop, gnss_data_walker, sizeof(dest.pos.gdop));    gnss_data_walker += sizeof(dest.pos.gdop);
        memcpy(&dest.pos.hdop, gnss_data_walker, sizeof(dest.pos.hdop));    gnss_data_walker += sizeof(dest.pos.hdop);
        memcpy(&dest.pos.vdop, gnss_data_walker, sizeof(dest.pos.vdop));    gnss_data_walker += sizeof(dest.pos.vdop);
        memcpy(&dest.pos.pacc, gnss_data_walker, sizeof(dest.pos.pacc));    gnss_data_walker += sizeof(dest.pos.pacc);
        memcpy(&dest.pos.vacc, gnss_data_walker, sizeof(dest.pos.vacc));    gnss_data_walker += sizeof(dest.pos.vacc);
        memcpy(&dest.datavalid, gnss_data_walker, sizeof(dest.datavalid));  gnss_data_walker += sizeof(dest.datavalid);

        for (u32 chan = 0; chan < 16; chan++)
        {
            memcpy(&dest.satp[chan].qli, gnss_data_walker, sizeof(dest.satp[chan].qli));    gnss_data_walker += sizeof(dest.satp[chan].qli);
            memcpy(&dest.satp[chan].elev, gnss_data_walker, sizeof(dest.satp[chan].elev));  gnss_data_walker += sizeof(dest.satp[chan].elev);
            memcpy(&dest.satp[chan].azim, gnss_data_walker, sizeof(dest.satp[chan].azim));  gnss_data_walker += sizeof(dest.satp[chan].azim);
        }
    }
    
    void load_gnss_meas(gnssdata& dest)
    {
        for (s32 n = 0; n < dest.buflen; n++)
        {
            memcpy(&dest.mtow[n], gnss_data_walker, sizeof(dest.mtow[n]));  gnss_data_walker += sizeof(dest.mtow[n]);

            for (u32 chan = 0; chan < 16; chan++)
            {
                memcpy(dest.prn[chan], gnss_data_walker, sizeof(*dest.prn[chan]));                              gnss_data_walker += sizeof(*dest.prn[chan]);
    
                memcpy(&dest.meas[chan].qli[n], gnss_data_walker, sizeof(dest.meas[chan].qli[n]));              gnss_data_walker += sizeof(dest.meas[chan].qli[n]);
                memcpy(&dest.meas[chan].cwarning[n], gnss_data_walker, sizeof(dest.meas[chan].cwarning[n]));    gnss_data_walker += sizeof(dest.meas[chan].cwarning[n]);
                memcpy(&dest.meas[chan].pr[n], gnss_data_walker, sizeof(dest.meas[chan].pr[n]));                gnss_data_walker += sizeof(dest.meas[chan].pr[n]);
                memcpy(&dest.meas[chan].pv[n], gnss_data_walker, sizeof(dest.meas[chan].pv[n]));                gnss_data_walker += sizeof(dest.meas[chan].pv[n]);
                memcpy(&dest.meas[chan].cp[n], gnss_data_walker, sizeof(dest.meas[chan].cp[n]));                gnss_data_walker += sizeof(dest.meas[chan].cp[n]);
                memcpy(&dest.meas[chan].cn0[n], gnss_data_walker, sizeof(dest.meas[chan].cn0[n]));              gnss_data_walker += sizeof(dest.meas[chan].cn0[n]);
            }
        }
    }

    static const u32 base_buffer_size = 1024 * 128;
    u8 base_data_buffer[base_buffer_size];
    u8* base_data_walker;
    
    static const u32 gnss_buffer_size = 1024 * 512;
    u8 gnss_data_buffer[gnss_buffer_size];
    u8* gnss_data_walker;

    static const u32 rover_eph_size = 1024 * 10;
    u8 rover_eph_buffer[rover_eph_size];
    u8* rover_eph_walker;

    rover::ctrl rover_ctrl;
    basedata bdata[20];
    basedata* pbdata;
};

}

#endif