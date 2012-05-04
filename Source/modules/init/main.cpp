#include "project.hpp"
#include "globals.hpp"
#include <ctl_api.h>

#include "dev/timer_lpc3230.hpp"
#include "dev/spi_lpc3230.hpp"
#include "dev/sd_lpc3230.hpp"

#include "modules/uart/uart_ctrl.hpp"
#include "modules/bluetooth/stack.hpp"
#include "modules/console/console.hpp"
#include "modules/time_queue/time_queue.hpp"
#include "modules/aux_ctrl/aux_ctrl.hpp"
#include "modules/async/messages.hpp"
#include "modules/file_system/file_system.hpp"
#include "modules/file_system/file_system_queue.hpp"
#include "modules/file_system/fat/dosfs.hpp"
#include "modules/profiling/profiler.hpp"
#include "modules/gps/gps_processor.hpp"
#include "modules/init/abort_handler_buffer.hpp"

#include "simulator/rover_simulator.hpp"
#include "simulator/multitask_simulator.hpp"
#include "simulator/math_benchmark.hpp"
#include "simulator/gps_benchmark.hpp"
#include "simulator/sd_benchmark.hpp"

using namespace lpc3230;

static CTL_TASK_t main_task;

static CTL_TASK_t idle_task;
static u32 idle_stack[512];

static CTL_TASK_t time_queue_task;
static u32 time_queue_stack[512];

#if ENABLE_AUX_CONTROL
    static CTL_TASK_t aux_task;
    static u32 aux_stack[512];
#endif

#if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR || ENABLE_GPS_BENCHMARKS
    static CTL_TASK_t gps_processor_task;
    static u32 gps_processor_stack[2048];
#endif

#if ENABLE_FS_QUEUE
    static CTL_TASK_t fs_queue_task;
    static u32 fs_queue_stack[512];
#endif

#if ENABLE_BLUETOOTH
    static CTL_TASK_t bluetooth_task;
    static u32 bluetooth_stack[512];
#endif

#if ENABLE_CONSOLE
    static CTL_TASK_t console_task;
    static u32 console_stack[512];
#endif

// with the CTL, an idle thread is needed so that when all real threads are in a wait state, the system can redirect somewhere. this thread must _never_ use wait, or the CTL won't have any active tasks to switch to.
void idle_thread(void* arg)
{
    while (true)
    {
    }
}

// wait for a task to die
void task_join(CTL_TASK_t& task)
{
    while (CTL_STATE_SUSPENDED != task.state)
        ctl_task_reschedule();
    ctl_task_remove(&task);
}

// await the end of program
void wait_shutdown()
{
    CTL_EVENT_SET_t message_event;
    ctl_events_init(&message_event, 0);
    get_central().set_event(msg::src::main, &message_event, 1);

    get_central().subscribe_to_global_message(msg::src::main, msg::id::shutdown_request);

    // go to lower priority : very important or else the other tasks won't run
    ctl_task_set_priority(&main_task, thread_priorities::main);

    // debugging stuff, should not be enabled for GPS builds
    #if ENABLE_ROVER_SIMULATOR
        get_rover_sim().run();
    #endif
    #if ENABLE_MATH_BENCHMARKS
        get_math_sim().run(100000);
    #endif
    #if ENABLE_SD_BENCHMARKS
        benchmarks::sd::static_thread(0); // this benchmark could also be run as a thread for further debugging
        profile::controller::report();
    #endif

    bool shutdown = false;
    while (!shutdown) // message loop : waits for only one message, the shutdown_request
    {
        u32 event_received = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR, &message_event, 1, CTL_TIMEOUT_NONE, 0);
        if (0 == event_received) // timeout
            continue;

        msg::id::en msg_id;
        u32 len;
        while (true)
        {
            bool got_msg = get_central().get_message(msg::src::main, msg_id, len);
            if (!got_msg) break;
            
            if (msg::id::shutdown_request == msg_id)
            {
                shutdown = true;
                break;
            }
        }
    }
}

int main()
{
    init_clocks();

    // turn main into a task, at the highest priority (until we're done creating the other tasks, to prevent them from running)
    ctl_task_init(&main_task, 255, "main");

    memset(idle_stack, 0xbe, sizeof(idle_stack)); // init the stack to a recognizable value
    ctl_task_run(&idle_task, thread_priorities::idle, idle_thread, 0, "idle", sizeof(idle_stack) / sizeof(u32), (unsigned int*)idle_stack, 0); // create the idle task

    init_modules();

    memset(time_queue_stack, 0xbe, sizeof(time_queue_stack)); // init the stack to a recognizable value
    ctl_task_run(&time_queue_task, thread_priorities::time_queue, time_queue::queue::static_thread, 0, "time_queue", sizeof(time_queue_stack) / sizeof(u32), (unsigned int*)time_queue_stack, 0); // create the time_queue task

    #if ENABLE_AUX_CONTROL
        memset(aux_stack, 0xbe, sizeof(aux_stack)); // init the stack to a recognizable value
        ctl_task_run(&aux_task, thread_priorities::aux, aux_ctrl::link::static_aux_ctrl_thread, 0, "aux", sizeof(aux_stack) / sizeof(u32), (unsigned int*)aux_stack, 0); // create the aux task
    #endif

    #if ENABLE_FS_QUEUE
        memset(fs_queue_stack, 0xbe, sizeof(fs_queue_stack)); // init the stack to a recognizable value
        ctl_task_run(&fs_queue_task, thread_priorities::fs_queue, fs::queue::static_thread, 0, "fs_queue", sizeof(fs_queue_stack) / sizeof(u32), (unsigned int*)fs_queue_stack, 0); // create the file system write queue task
    #endif

    #if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR || ENABLE_GPS_BENCHMARKS
        memset(gps_processor_stack, 0xbe, sizeof(gps_processor_stack)); // init the stack to a recognizable value
        ctl_task_run(&gps_processor_task, thread_priorities::gps_processor, gps::processor::static_thread, 0, "gps_processor", sizeof(gps_processor_stack) / sizeof(u32), (unsigned int*)gps_processor_stack, 0); // create the base station task
    #endif

    #if ENABLE_BLUETOOTH
        memset(bluetooth_stack, 0xbe, sizeof(bluetooth_stack)); // init the stack to a recognizable value
        ctl_task_run(&bluetooth_task, thread_priorities::bluetooth, bluetooth::stack<lpc3230::high_speed_uart::uart<uart_ids::bluetooth>, irq_priorities::bluetooth>::static_bluetooth_thread, 0, "bluetooth", sizeof(bluetooth_stack) / sizeof(u32), (unsigned int*)bluetooth_stack, 0); // create the bluetooth_task
    #endif

    #if ENABLE_CONSOLE
        memset(console_stack, 0xbe, sizeof(console_stack)); // init the stack to a recognizable value
        ctl_task_run(&console_task, thread_priorities::console, console::simple::static_thread, 0, "console", sizeof(console_stack) / sizeof(u32), (unsigned int*)console_stack, 0); // create the console thread
    #endif

    #if ENABLE_MULTITASK_SIMULATOR
        simulator::multitask_cooperative::run(&main_task);
    #endif

    // wait end of program
    wait_shutdown();

    get_central().send_message(msg::src::time_queue, msg::id::request_to_end_task);
    task_join(time_queue_task); // wait the end of the task

    #if ENABLE_BASE_PROCESSOR || ENABLE_ROVER_PROCESSOR || ENABLE_GPS_BENCHMARKS
        get_central().send_message(msg::src::gps_processor, msg::id::request_to_end_task);
        task_join(gps_processor_task); // wait the end of the task
    #endif
    #if ENABLE_CONSOLE
        get_central().send_message(msg::src::console, msg::id::request_to_end_task);
        task_join(console_task);
    #endif

    debug::stop(); // flushes the profile.txt file if it was used

    #if ENABLE_FS_QUEUE
        get_central().send_message(msg::src::fs_queue, msg::id::request_to_end_task);
        task_join(fs_queue_task);
    #endif

    fs::end();     // flushes the file system cache and blocks access to the SD

    #if ENABLE_AUX_CONTROL
        get_central().send_message(msg::src::aux, msg::id::shutdown); // confirm we are ready to be shutdown
        // we will die shortly...
        // do not tell the aux task to shutdown, it needs to process a little bit
    #endif

    ctl_task_remove(&idle_task);

    return 0;
}