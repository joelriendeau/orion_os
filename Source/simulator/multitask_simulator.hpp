#pragma once

#include "modules/init/globals.hpp"

namespace simulator {

// a simulator creating multiple tasks, allowing testing of benchmark tools in these situations
// also useful to assess the inner workings of the CrossWorks Task Library (CTL)

// several cooperative tasks, no interrupts involved in task switching
namespace multitask_cooperative {

    void thread_1(void* argument);
    void thread_2(void* argument);
    void thread_3(void* argument);
    void run(CTL_TASK_t* main_task);

}

}