---
layout: post
title: "ardupilot启动流程代码解析"
date:   2025-03-18
tags: [ardupilot]
comments: true
author: 嘉余
---
# **一、系统启动**

ArduCopter/Copter.cpp

`AP_HAL_MAIN_CALLBACKS(&copter);`

展开后

```
extern "C" {
    int main(int argc, char* const argv[]);
    int main(int argc, char* const argv[])
    {
        hal.run(argc, argv, &copter); //--> libraries/AP_HAL/AP_HAL_Main.h
        return 0;
    }
}
```

## **两个目标，hal和run**

### **先看hal（不重要）**

Copter.cpp line83

`const AP_HAL::HAL& hal = AP_HAL::get_HAL();` 得到继承自AP\_HAL的AP\_HAO\_Chibios类型的hal，硬件被抽象。

libraries/AP\_HAL\_ChibiOS/HAL\_ChibiOS\_Class.cpp里开头有

`#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS` 和 `#include <hwdef.h>`

hwdef是每次配置版型的时候waf生成的,`CONFIG_HAL_BOARD`是waf配置版型时给的编译参数，只有`CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS`时才会用ChibiOS启动

hal在line181，初始化的时候生成了一个HAL对象，并传入接口参数，在line353，定义了一个HAL\_ChibiOS，硬件抽象结束

### **接下来看run**

libraries/AP\_HAL\_ChibiOS/HAL\_ChibiOS\_Class.cpp

```
void HAL_ChibiOS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    usb_initialise();
    g_callbacks = callbacks;
    main_loop();
}
```

Copter.cpp的run会跳到这里，启动了操作系统。这里的传入的callbacks参数是&copter，而Copter对象继承自AP\_Vehicle。所以可以知道g\_callbacks里面的setup/loop是`AP_Vehicle::setup / AP_Vehicle::loop`。配置完以后进入mainloop()

## mainloop()

```
static void main_loop()
{
    daemon_task = chThdGetSelfX();
    chThdSetPriority(APM_MAIN_PRIORITY);  //switch to high priority for main loop
    ChibiOS::Shared_DMA::init();
    peripheral_power_enable();
    hal.serial(0)->begin(SERIAL0_BAUD);
    hal.analogin->init();
    hal.scheduler->init();//注意，此处的scheduler是ChibiOS的，用来执行系统任务，不执行飞行任务
     /*设置本任务(daemon_task)的优先级
      run setup() at low priority to ensure CLI doesn't hang the
      system, and to allow initial sensor read loops to run
     */
    hal_chibios_set_priority(APM_STARTUP_PRIORITY);
    ····
    ····看代码
    g_callbacks->setup();
    g_callbacks->loop();
    ····
}
```

mainloop中做了基本的初始化，例如打开console，初始化ChibiOS的调度器，创建一些必要的线程（io控制器等）。接着执行一次setup(),最后进入loop开始无限循环。

# 二、飞控部分

## setup

libraries/AP\_Vehicle/AP\_Vehicle.cpp line 267 `void AP_Vehicle::setup()`

这里是各种构型通用的初始化，例如串口、地面站通讯、飞行任务调度器。如果有自己写的功能需要初始化，不要放到这里。

libraries/AP\_Vehicle/AP\_Vehicle.cpp line 297

```
 // initialise the main loop scheduler
    const AP_Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);        //引入copter任务
    AP::scheduler().init(tasks, task_count, log_bit); 
```

在这里将Copter.cpp中写的任务引入，并初始化调度器，调度器原理和任务配置方法具体见[https://kdocs.cn/l/cmagVinZy8Kf](https://kdocs.cn/l/cmagVinZy8Kf)

libraries/AP\_Vehicle/AP\_Vehicle.cpp line 350 `init_ardupilot();` 属于Copter子类的初始化。这个函数位于ArduCopter/system.cpp，line16。里面有各种初始化功能，如果有自己写的功能需要初始化，应该放到这里。

## loop

ArduCopter任务的调用栈逻辑依次是：

```
AP_Vehicle::loop
 └──> scheduler.loop
     └──> run
         └──> task.function
```

task.function是ArduCopter/Copter.cpp中给出的任务列表对应的函数。

要执行自己的任务，只需要在任务列表里仿照已有的添加即可，注意自己写的库的头文件要放在Copter.h中才能被找到。