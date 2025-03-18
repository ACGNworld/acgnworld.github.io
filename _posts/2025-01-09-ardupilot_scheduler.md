---
layout: post
title: "ardupilot scheduler调度器"
date:   2025-01-09
tags: [ardupilot]
comments: true
toc: true
author: 嘉余
---

注意：飞行控制的调度器是ardupilot自己实现的，**将整个控制过程包装成一个应用程序**，与底层操作系统做了隔离。不使用ChibiOS的任务调度。本文在线markdown版见[https://kdocs.cn/l/cmagVinZy8Kf](https://kdocs.cn/l/cmagVinZy8Kf)

# 初始化

先来看scheduler的构造函数。libraries/AP\_Scheduler/AP\_Scheduler.cpp，line78

```
AP_Scheduler::AP_Scheduler()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many schedulers");
#endif
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}
```

setup\_object\_defaults() 在构造时就配置了参数，包括hz、可选选项等···

libraries/AP\_Vehicle/AP\_Vehicle.cpp，line297

```
    const AP_Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);        //引入copter任务
    AP::scheduler().init(tasks, task_count, log_bit); 
```

get\_scheduler\_tasks在Copter.cpp被重写，将配置好的任务和个数赋值给这里的tasks和task\_count，并调用init。tasks应该都是有序列表

```
void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}
```

libraries/AP\_Scheduler/AP\_Scheduler.cpp，line101，init()，将Copter.cpp的tasks赋值给\_vehicle\_tasks，初始化设备相关任务`_vehicle_tasks`和`_num_vehicle_tasks`；

在init()中，有`vehicle->get_common_scheduler_tasks(_common_tasks, _num_common_tasks);`，点进去

```
void AP_Vehicle::get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks)
{
    tasks = scheduler_tasks;
    num_tasks = ARRAY_SIZE(scheduler_tasks);
}
```

这里往上翻，有一个数组 libraries/AP\_Vehicle/AP\_Vehicle.cpp line511，里面就是common\_task，用这个列表初始化AP\_Vehicle类`_common_tasks`和`_num_common_tasks`；

```
const AP_Scheduler::Task AP_Vehicle::scheduler_tasks[] = {
#if HAL_GYROFFT_ENABLED
    FAST_TASK_CLASS(AP_GyroFFT,    &vehicle.gyro_fft,       sample_gyros),
#endif
#if AP_AIRSPEED_ENABLED
    SCHED_TASK_CLASS(AP_Airspeed,  &vehicle.airspeed,       update,                   10, 100, 41),   
···
}
```

# 运行

libraries/AP\_Vehicle/AP\_Vehicle.cpp line455，每次AP\_Vehicle::loop()会调用scheduler.loop()

接下来看loop，位于libraries/AP\_Scheduler/AP\_Scheduler.cpp line339。

line344 `AP::ins().wait_for_sample()`， 等待Gyro&Acc有效采样数据

> **等待样本可用**：
> 
> 这个函数的作用是等待传感器数据（样本）变得可用。它决定了 ArduPilot 主循环的执行频率。
> 
> **理想返回频率**：
> 
> 理想情况下，该函数应该按照 `AP_InertialSensor::init()` 中设置的 `sample_rate` 参数精确地返回。也就是说，它应该以固定的频率调用，确保主循环按时执行。
> 
> **关键输出 `_delta_time`**：
> 
> `_delta_time` 是一个重要的输出变量，表示从上一次样本到当前样本之间的时间间隔。这个时间间隔用于陀螺仪和加速度计的数据积分计算。
> 
> 尽量保持 `_delta_time` 恒定是非常重要的，但如果有延迟发生，系统也需要能够应对这些情况。
> 
> 长期来看，所有 `_delta_time` 的总和应该与实际经过的时间完全一致，以确保系统的时钟同步和数据准确性。

line369 `tick()`，使tick+1，之后run会用到。

line389 `run(time_available)`，time\_available是每次循环总时间减去传感器采样时间后的剩余，就是指给多少时间去跑任务列表里的任务

# 调度

进入line182，`void AP_Scheduler::run(uint32_t time_available)`

1. 根据优先级判断vehicle\_task和common\_task哪个任务先行执行；
2. 快速任务和非快速任务在有限剩余时长下，决定是否执行；
3. 执行任务；
4. 统计任务执行时长，并进行辅助参数更新；

```
for (uint8_t i=0; i<_num_tasks; i++) {
        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the
            // priorities.  In case of a tie the vehicle-specific
            // entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
        const AP_Scheduler::Task &task = run_vehicle_task ? _vehicle_tasks[vehicle_tasks_offset] : _common_tasks[common_tasks_offset];
        if (run_vehicle_task) {
            vehicle_tasks_offset++;
        } else {
            common_tasks_offset++;
        }
```

run开始一个循环，每次按优先级判断下一个任务应该执行common task还是vehicle tasks，如果是vehicle tasks就把`run_vehicle_task`设为true，**并使用`task`作为要执行的任务的别名**。

common tasks和vehicle tasks各自维护一个指向下一个待执行任务的指针，进入run时置0，每次执行了一个相应任务就+1。

接下来开始处理是否是快速任务

## 非快速任务

优先级大于3的就不是快速任务，**越小越优先**

```
const uint16_t dt = _tick_counter - _last_run[i];
//we allow 0 to mean loop rate
uint32_t interval_ticks = (is_zero(task.rate_hz) ? 1 : _loop_rate_hz / task.rate_hz);
if (interval_ticks < 1) {
    interval_ticks = 1;
}
if (dt < interval_ticks) {
    // this task is not yet scheduled to run again
    continue;
}
// this task is due to run. Do we have enough time to run it?
_task_time_allowed = task.max_time_micros;
```

interval\_ticks是指多少ticks执行一次该任务，dt=(当前tick数-上次执行时的tick数)

```
if (dt >= interval_ticks*2) {
    perf_info.task_slipped(i);
}
if (dt >= interval_ticks*) {
    // we are going beyond the maximum slowdown factor for a
    // task. This will trigger increasing the time budget
    task_not_achieved++;
}
if (_task_time_allowed > time_available) {
    // not enough time to run this task.  Continue loop -
    // maybe another task will fit into time remaining
    continue;
}
```

先看第九行，时间不够的话就会跳过该任务，超过4次run都没执行就使task\_not\_achieved++，留在之后空闲时处理，防止某任务始终得不到运行。4是`max_task_slowdown`的默认值

两次run都没运行这个任务，就会调用`perf_info.task_slipped(i)`给这个任务的overrun变量+1，perf\_info是辅助类，只记录数据，不执行任何功能。

**所以maxtime一定不要填的太大。**

## 快速任务

```
else {
    _task_time_allowed = get_loop_period_us();
}
```

不管时间有没有剩余都要执行

## 执行任务

之后的处理过程相同

libraries/AP\_Scheduler/AP\_Scheduler.cpp line261

```
    task.function();//完整执行一次任务
    hal.util->persistent_data.scheduler_task = -1;    
    // record the tick counter when we ran. This drives
    // when we next run the event
    _last_run[i] = _tick_counter;
    // work out how long the event actually took
    now = AP_HAL::micros();
    uint32_t time_taken = now - _task_time_started;
    bool overrun = false;
    if (time_taken > _task_time_allowed) {
        overrun = true;
        // the event overran!
        debug(3, "Scheduler overrun task[%u-%s] (%u/%u)\n",
              (unsigned)i,
              task.name,
              (unsigned)time_taken,
              (unsigned)_task_time_allowed);
    }
    perf_info.update_task_info(i, time_taken, overrun);
```

英语注释很清楚。会记录这个任务运行了多久，太长会被标记

由此可以看出，这个系统没有多线程切换，需自行注意每个功能的运行时间并写到maxtime里。

至此，run中的一次循环结束，准备提取下一个任务。

# extra time 补偿

loop函数在运行完run后还有一些事情要处理，就是根据这次run中的任务完成情况来决定下次的extra time，回顾run之前的`time_available += extra_loop_us`发现有一个extra\_loop\_us

line396:

```
if (task_not_achieved > 0) {
        // add some extra time to the budget
        extra_loop_us = MIN(extra_loop_us+100U, 5000U);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            // we have gone through 50 loops without a task taking too
            // long. CPU pressure has eased, so drop the extra time we're
            // giving each loop
            task_all_achieved = 0;
            // we are achieving all tasks, slowly lower the extra loop time
            extra_loop_us = MAX(0U, extra_loop_us-50U);
        }
    }
```

task\_not\_achieved在上一节提到过（[跳转](https://kdocs.cn/l/cmagVinZy8Kf?linkname=h7F53Xr9AD)），在剩余时间小于任务最大运行时间时会+1，由代码可知若>0，则会多比上次分配100us的extra time，最大5ms（400Hz下两个循环的时间）。

如果连着50次run所有任务都完成了，就慢慢缩减extra time。

# 设置任务

先来看几个重要的宏，用来设置任务：

libraries/AP\_Scheduler/AP\_Scheduler.h line43

```
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}
#define FAST_TASK_CLASS(classname, classptr, func) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_FAST_NAME_INITIALIZER(classname, func)\
    .rate_hz = 0,\
    .max_time_micros = 0,\
    .priority = AP_Scheduler::FAST_TASK_PRI0 \
}
```

ArduCopter/Copter.cpp line85

```
#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)
```

另附Task结构

```
struct Task {
        task_fn_t function;
        const char *name;
        float rate_hz;
        uint16_t max_time_micros;
        uint8_t priority; // task priority
    };
```

先讨论SCHED\_TASK和FAST\_TASK，这两个函数默认了你调用的函数属于Copter类

| 不同点 | SCHED_TASK | FAST_TASK |
| :--- | :--- | :--- |
| max_time_micros | 自定义 | 0 |
| priority | 自定义 | 0 |
| rate_hz | 自定义 | 等于调度器hz，默认为400 |

若需要调用不属于Copter类的函数，则需要使用SCHED\_TASK\_CLASS和FAST\_TASK\_CLASS，区别同上表。

**总结：**

1. 属于Copter类的优先度较高的任务，期望每次循环都执行的，配置如下：`FAST_TASK(update_flight_mode)`；优先度不高的，用`SCHED_TASK(auto_disarm_check, 10, 50, 27)`，不要配置太多FAST\_TASK。时间单位是微秒us
2. 不属于Copter的，用`SCHED_TASK_CLASS(AP_GPS, &copter.gps, update, 50, 200, 9)`，自己写上函数属于什么类。
3. 源码已经有几种预设任务，存放在Userhook.cpp文件中，里面有`Copter::userhook_50Hz()`等多种方法并已经启用。如果新功能调用的对象属于Copter类则可以考虑直接放进去。

# AP\_Scheduler其他方法，调试用

## load\_average占用率

任务执行占用CPU的时间

```
/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average()
{
    // return 1 if filtered main loop rate is 5% below the configured rate
    if (get_filtered_loop_rate_hz() < get_loop_rate_hz() * 0.95) {  //去噪（滤波）后的循环时间，如果超过95%循环间隔，那就是100%CPU
        return 1.0;
    }
    if (_spare_ticks == 0) {  //loop循环在跑就不太可能是0, 这个场景基本不存在
        return 0.0f;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (_spare_micros/_spare_ticks);
    return constrain_float(used_time / (float)loop_us, 0, 1);  //剩余时间比上单位时间 = 任务占用的CPU百分率
}
```

## task\_info任务信息

1. task: 任务名
2. MIN: 最小执行时间
3. MAX: 最大执行时间
4. AVG: 平均执行时间
5. OVR: 超时次数
6. SLP: 错过次数
7. TOT: 占总任务的百分比

```
// display task statistics as text buffer for @SYS/tasks.txt
void AP_Scheduler::task_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("TasksV2\n");
    // dynamically enable statistics collection
    if (!(_options & uint8_t(Options::RECORD_TASK_INFO))) {
        _options.set(_options | uint8_t(Options::RECORD_TASK_INFO));
        return;
    }
    if (perf_info.get_task_info(0) == nullptr) {
        return;
    }
    // baseline the total time taken by all tasks
    float total_time = 1.0f;
    for (uint8_t i = 0; i < _num_tasks + 1; i++) {
        const AP::PerfInfo::TaskInfo* ti = perf_info.get_task_info(i);
        if (ti != nullptr && ti->tick_count > 0) {
            total_time += ti->elapsed_time_us;
        }
    }
    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;
    for (uint8_t i = 0; i < _num_tasks; i++) {
        const AP::PerfInfo::TaskInfo* ti = perf_info.get_task_info(i);
        const char *task_name;
        // determine which of the common task / vehicle task to run
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < _num_vehicle_tasks &&
            common_tasks_offset < _num_common_tasks) {
            // still have entries on both lists; compare the
            // priorities.  In case of a tie the vehicle-specific
            // entry wins.
            const Task &vehicle_task = _vehicle_tasks[vehicle_tasks_offset];
            const Task &common_task = _common_tasks[common_tasks_offset];
            if (vehicle_task.priority <= common_task.priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < _num_vehicle_tasks) {
            // out of common tasks to run
            run_vehicle_task = true;
        } else if (common_tasks_offset < _num_common_tasks) {
            // out of vehicle tasks to run
            run_vehicle_task = false;
        } else {
            // this is an error; the outside loop should have terminated
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }
        if (run_vehicle_task) {
            task_name = _vehicle_tasks[vehicle_tasks_offset++].name;
        } else {
            task_name = _common_tasks[common_tasks_offset++].name;
        }
        ti->print(task_name, total_time, str);
    }
}
```

# ChibiOS 调度器

在执行main\_loop（见[启动文档](https://kdocs.cn/l/cfAs3zt5COC2?linkname=d0hZuLUmek)）时执行了schedulerInstance的初始化，在创建schedulerInstance即操作系统调度器实例化对象时，定义了线程、操作系统任务创建等的管理。此处的任务为**操作系统底层任务**，继承自AP\_HAL::Scheduler，用于对定时器、串口等和顶层应用程序分线程管理。

`hal.scheduler->init();`

libraries/AP\_HAL\_ChibiOS/Scheduler.cpp line93

```
void Scheduler::init()
{
    chBSemObjectInit(&_timer_semaphore, false);
    chBSemObjectInit(&_io_semaphore, false);
#ifndef HAL_NO_MONITOR_THREAD
    // setup the monitor thread - this is used to detect software lockups
    _monitor_thread_ctx = chThdCreateStatic(_monitor_thread_wa,
                     sizeof(_monitor_thread_wa),
                     APM_MONITOR_PRIORITY,        /* Initial priority.    */
                     _monitor_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
#ifndef HAL_NO_TIMER_THREAD
    // setup the timer thread - this will call tasks at 1kHz
    _timer_thread_ctx = chThdCreateStatic(_timer_thread_wa,
                     sizeof(_timer_thread_wa),
                     APM_TIMER_PRIORITY,        /* Initial priority.    */
                     _timer_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
#ifndef HAL_NO_RCOUT_THREAD
    // setup the RCOUT thread - this will call tasks at 1kHz
    _rcout_thread_ctx = chThdCreateStatic(_rcout_thread_wa,
                     sizeof(_rcout_thread_wa),
                     APM_RCOUT_PRIORITY,        /* Initial priority.    */
                     _rcout_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
#ifndef HAL_NO_RCIN_THREAD
    // setup the RCIN thread - this will call tasks at 1kHz
    _rcin_thread_ctx = chThdCreateStatic(_rcin_thread_wa,
                     sizeof(_rcin_thread_wa),
                     APM_RCIN_PRIORITY,        /* Initial priority.    */
                     _rcin_thread,             /* Thread function.     */
                     this);                     /* Thread parameter.    */
#endif
#ifndef HAL_USE_EMPTY_IO
    // the IO thread runs at lower priority
    _io_thread_ctx = chThdCreateStatic(_io_thread_wa,
                     sizeof(_io_thread_wa),
                     APM_IO_PRIORITY,        /* Initial priority.      */
                     _io_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif
#ifndef HAL_USE_EMPTY_STORAGE
    // the storage thread runs at just above IO priority
    _storage_thread_ctx = chThdCreateStatic(_storage_thread_wa,
                     sizeof(_storage_thread_wa),
                     APM_STORAGE_PRIORITY,        /* Initial priority.      */
                     _storage_thread,             /* Thread function.       */
                     this);                  /* Thread parameter.      */
#endif
}
```

飞行控制线程是mainloop本体

每个chThdCreateStatic函数创建一个线程，以时间片轮转的方式运行一个函数。

**以\_io\_thread为例，**初始化调度器实例对象时，先创建io线程，对应函数中执行了线程初始化，并无限循环执行run\_io，run\_io首先判断是否处于runio状态，避免重复执行。而后“无限等待”二进制信号量，获取到信号量后执行“将该信号量置位”。

# 参考文献

[https://blog.csdn.net/lida2003/article/details/131074363](https://blog.csdn.net/lida2003/article/details/131074363)

[https://blog.csdn.net/weixin\_47710106/article/details/139069453](https://blog.csdn.net/weixin_47710106/article/details/139069453)