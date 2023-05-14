# CM3_on_7010
基于zynq-7010(博宸电子MINI开发板)的Cortex-M3软核移植
一、实验准备
（一）软件准备
1.Vivado工程环境
2019.1版本
2.Keil MDK（Microcontroller Development Kit）开发环境
Community 5.38a版本，下载地址https://www.keil.arm.com/mdk-community/
在下载界面，有MDK的安装教程：
 
点击download MDK后，进入信息提交界面。信息提交完成，
 
安装完成后在电脑上出现软件图标：
 
3.Cortex-M3 软核IP包
下载地址：Cortex-M3 DesignStart FPGA XilinxEdition (AT426) (arm.com)
 
解压压缩包后文件目录：
 
各个文件夹存放的内容：
·docs
存放ARM Cortex-m3处理器参考手册，DesignStart FPGA版本使用说明，基于Arty-A7开发板的顶层BlockDesign文件。
·hardware
存放基于Digilent Arty-A7开发板的Vivado工程，顶层BlockDesign文件，管脚约束文件
·software
存放Keil-MDK工程，SPI Flash的编程算法文件等。
·Vivado
包含DesignStart Cortex-M3 Xilinx FPGA版本的IP核文件，其中Arm_ipi_repository就是内核源文件，因为已经加密，没有可读性。
Cortex_M3处理器技术参考手册位置： AT426-r0p1-00rel0-1\vivado\Arm_ipi_repository\CM3DbgAXI\docs
4.Keil 器件支持包
从Keil官网上下载DesignStart Cortex-M3专用的器件支持包
 

（二）硬件准备
1.博宸开发板
硬件核心：Zynq-7010
2.一个Jlink调试器
可以选择的版本有Jlink v9, Jlink v10和Jlink-OB,本次实验使用的是一个Jlink-OB.
 
3.安装Jlink驱动
本报告的附件提供了Jlink-OB的驱动程序：Setup_Jlink_V492.EXE
 
安装过程中可能会提示更新编程环境软件，在对应选项前打√，点击ok
 
软件安装成功后，在桌面出现图标，分别是Jlink控制程序和Jlink下载程序。
   
注意，本实验所使用的开发板是5V,1-2A直流供电，Jlink调试器是3.3V，200mA直流输出，不使用调试器间接给开发板供电，调试器上的黄色杜邦线不接，防止烧坏调试器。
驱动安装好后，在PC端设备管理器中出现J-Link1 driver。
 



























二、实验过程
（一）工程模块设计
1.新建工程文件夹
在工作目录下建立文件夹cortex_m3_on_7010，说明该工程是移植在7010平台上的软核。
 
在工程目录文件夹下新建几个文件夹，用于存放Vivado工程文件：
 
·bd：存放BlockDesign设计
·cm3_core文件夹: ARM Cortex-M3内核IP核文件，
·doc文件夹：存放设计文档
·flash文件夹：用来存放生成的bit和mcs文件
·rtl文件夹：存放用户设计的verilog源文件
·xdc文件夹：用来存放管脚、时序约束文件
将官方的Cortex-M3核文件复制到com_core文件夹下：
 
2.建立Vivado工程
工程位置是刚刚创建的文件夹位置。
 
核心器件选择7010版本：
 
在project Settings-IP中，将软核IP所在的位置添加到IP搜索路径中：
 
3. 新建BlockDesign
命名为cm3_core，保存的位置是新建的bd文件夹。
 
将M3核加入设计：
  
图上添加的是使用Jlink进行调试的M3软核，如果有DAP调试条件，可以选择DAP核。
4.配置Cortex-M3核
中断-保持默认：
 
调试：禁用Trace功能，禁用JTAG端口：
 
设置指令空间和数据空间的大小，均为64KB，都不进行初始化
  
设置完后的IP核：
 
5.添加一些必要的IP核
（1）时钟PLL
用于提供给内核、总线和外设的时钟。
配置为50MHz单端输入，PLL输出配置为50MHz,如果时钟频率更高，综合后会提示WNS，TNS时序不满足，可能无法正常运行。
   
连线时clk_out1端口提供给内核50MHz的时钟
（2）复位网络构建
用于提供内核、外设、互联组件所需要的复位信号，不需要进行定制，保持默认设置。
 
连线方式：将系统时钟信号接入Sync_clk，外部复位信号接入ext_reset_in，时钟的locked信号接入dcm_locked中。
输出端口：mb_reset：输出的复位，将这个复位信号作为Cortex-M3 IP核的复位信号以及debug的系统复位信号，需要注意的是，mb_reset是高电平有效的信号，需要加一个NOT门后接入ARM的IP核中。
Interconnect——resn用来当做总线的复位源，periphera当做外设的复位源，这样基本的复位网络就搭建好了。
添加一个基本逻辑非门，搜索vector logic IP,选择Utility Vector Logic,门类型选择非门。
 
 
（3）总线网络构建
IP核中搜索总线与外设的连接核IP：AXI Interconnect
 
从机数量设置为1，主机数量设置为2. 
  
（4）添加常量IP
本次软核搭建不涉及中断部分，所以IRQ和NMI都给定常量0即可，如果需要，将中断接入处理器，可以通过Concat核将多个中断源合并成为一个连接到IRQ。
在IP搜索中输入constant，一共添加三个常量IP即可。
  
其中xlconstant_1设置为：
 
xlconstant_2设置为：
 
Run Connection Automation，选择时钟的两个input:
（5）添加GPIO与UART外设
Xilinx官方提供的AXI GPIO外设具有以下特性：
·内部有两个通道：通道1和通道2，每个通道最多支持32个管脚
·每个管脚可以配置为输入或输出模式
·每个管脚可以设置复位初值
·支持中断输出
提供的AXI UART外设具有以下特性：
·全双工
·支持5-8位数据位
·支持奇偶校验
·可配置波特率110-230400
本实验挂在AXI总线上的GPIO配置为双通道，通道1为输出模式，通道2为输入模式。在三、Cortex-M3内核的应用部分，利用板载的两个按键（PL-KEY1,PL-KEY2）和LED灯D1-D2，完成点灯实验，所以通道1宽度设置为2，通道2宽度设置为2。
在search IP中搜索GPIO，选择AXI GPIO IP：
 
GPIO设置为双通道（Enabled Dual Channel打对钩），GPIO-All Outputs，GPIO2-All Inputs两个通道的宽度设置为2:
 
在add IP中输入uart，搜索AXI Uartlite IP：
 
时钟输入选择manual，设置为50MHz,UART配置为115200波特率，8位数据位，无奇偶校验：
 
（6）引出外设端口
在整体连线之前，选择run connection automation：
 
在弹出的窗口里对时钟IP的两个端口打钩，然后点击ok：
 
外部复位和时钟接口出现：
 
同样的方法引出GPIO的端口，端口名称可以在右键-External Interface Properties中设置：
 
 
 
自己设计的SWD端口在swd_io处需要引出一个Inout port，在BlockDesign空白处右键-create port：
 
将生成的swdio连接到SWD IP的swd_io端口上：
 
用create port的方法引出M3核的SW时钟：
 
 
用create port的方法引出UART的发送端和接收端：
  
 
至此，所有连接到外部的端口已经引出。
6.全局连线
（1）-（5）手动连线。
（1）系统复位
 
（2）时钟添加
 
（3）复位输出
 
（4）常量IP
 
（5）AXI总线，GPIO与UART外设
 
 
在打包之前，将所有的port统一命名：
Reset_rt_0 -> cm3_resetn
Clk_100MHz -> cm3_clk
gpio_rtl_in ->cm3_gpio_in
gpio_rtl_out ->cm3_gpio_out
swdio -> cm3_swdio
最后的连线如图示（包含了（二）中的SWD调试接口）：
 
命名后：
 
从官方手册中可知，ARM提供的软核IP已经包括了ITCM和DTCM存储器，所以无需添加外部的BRAM来作为程序和数据的存储区。



















（二）SWD调试接口
本实验使用了Jlink调试器，调试器工作于SWD模式，开发板上没有针对这个调试模式的接口，需要自行用硬件描述语言书写。
1.SWD接口代码
在Cortex-M3上的接口是SWTCLK,SWDI/TMS,SWDOEN这几个引脚。
·SWDCLK：调试器给的外部时钟，直接引出引脚即可，SWDI和SWDO是输入输出，
首先Add sources，选择创建源文件：
 
文件命名为swdiobuf.v,路径选择为一中建立的工程文件夹目录下的rtl文件夹。
 
 
先不对文件进行例化：
 
此时我们在design sources中看到空白的swdio源码：
 
只需要在这个module里写入SWD模块的源码即可。代码如下：
 
在design sources中将swdiobuf加入BlockDesign:
  
2.与原设计连线
swd_o -> SWDO，swd_oe -> SWDOEN，swd_i -> SWDITMS，swd_io -> cm3_swdio
 



































（三）分配地址
添加完外设IP后，需要对外设进行基地址和空间分配，在Address Editor中右键-Auto Assign Address即可。
 
分配完成：
 
使用设计验证（Validate Design）功能，可以检查当前BlockDesign设计连接的合法性：
 
 
（四）生成Wrapper并例化到顶层
为了方便后续添加自定义的FPGA逻辑模块，我们将Cortex-M3软核处理器作为一个处理器例化到顶层设计中。
在BlockDesign源文件上右键-Generate Output Products:
 
点击Generate:
 
等待生成完成后
 
再选择create HDL Wrapper:
 
生成一个_wrapper.v的文件：
 
在DesignSource中新建顶层文件top.hdl.v并保存到rtl文件夹，将Wrapper例化到顶层：
  
 
保存top_hdl.v文件，工程目录变为：
 
对整个工程进行综合：
 
综合过程中如果是BlockDesign中出错，回到BlockDesign中修改，保存后重复Validate Design之后的步骤，直至综合通过；如果是top_hdl.v中的error，回到top_hdl.v中修改保存，不需要重复进行Validate Design，直至综合通过。
 
 
综合通过后，不进行run implentation，下一步将以上工程映射到管脚上。









（五）管脚分配
菜单栏选择RTL ANALYSIS-open Elaborated Design：
 
打开的电路图中，右上角选择10I/O Ports:
 
在下方弹出：
 
管脚分配：
 
（六）BIT流文件生成和烧写
板子使用QSPI Flash,为了提高下载和启动速度，在生成比特流时，配置生成选项：数据压缩，50M读取速度，4位数据线。
当然，也可以不做，耐心等待执行完成。
 
选择open Hardware Manager-Auto connect-Program device：
 
烧写完成：
 
烧写完成后开发板Done指示灯亮起，由于我们未在Vivado工程下对外设（LED UART）进行编程，所以程序烧写完后没有指示，需要完成三后才能确认移植完成。
烧写完成，Done指示灯亮起，只有内核，没有应用：
 
三、Keil端软件编写
事先需要安装M3的核心器件支持包：
在工具栏上方点击图标 （Package Installer），进行M3需要的核心器件包安装：
 
 
安装后在Action栏会显示Up to Date:
 
1.新建项目
（1）组织工程
打开Keil软件，选择Project-New μVision Project
 
新建项目的名称写为M3on7010
 
设置处理器，选择ARM Cortex M3 DesignStart:
 
在接下来的组件安装界面中，添加CMSIS内核文件和Startup启动文件：
 
点击ok，按照如下结构组织文件：
 
将原来的NewGroup删掉，添加新的Group：
 
右键Application，选择Add new items,建立main.c文件;
 
空白的main.c文件：
 
在main.c文件中添加以下内容并保存：
 
（2）设置RAM和ROM地址
在工程选项中设置 Options for Target-> Target
片上的ITCM的起始地址0x0,大小64K；片上DTCM起始地址0x20000000，大小64K：
 
（3）GPIO输入输出控制
查看AXI GPIO的使用手册，通道1的数据寄存器偏移地址为，通道2的数据寄存器偏移地址为0x08,根据Vivado中的连接，LED连接到通道1，按键连接到通道2上，所以只需要对这两个寄存器地址进行读写，就能实现LED的控制和拨码开关状态的读取。
在二（三）中我们已经为外设分配了基址：
 
可以看到GPIO的起始基址是0x40000000，UART的起始基址是0x40600000，
LED控制和拨码开关读取，需要在main.c中写入：
*(volatile uint32_t *) (0x40000000+0x0) = 0x0f; //GPIO通道1低4位写1
*(volatile uint32_t *) (0x40000000+0x0) = 0x00; //GPIO通道1低4位写0
uint32_t sw = 0;
sw = *(uint32_t *) (0x40000000+0x08);   //获取GPIO通道2的32位输入状态
（4）串口数据发送和接收
向串口FIFO写入一字节数据：
while((*(volatile uint32_t *)(0x40600000 + 0x08)) & 0x08 != 0x08);  //等待发送FIFO不满
*(volatile uint32_t *) (0x40600000+0x04) = 0x41;    //向串口发送FIFO写入字符'A'=0x41
从串口接收一字节数据：
uint8_t dat = 0;
if((*(volatile uint32_t *)(0x40600000 + 0x08)) & 0x01 == 1) //串口接收FIFO中有数据
    dat = (*(volatile uint32_t *)(0x40600000 + 0x00));      //从接收FIFO中读取1字节数据。
（5）延时函数的实现
LED的闪烁需要被人眼捕捉到，需要对亮灭进行延时，这里使用系统滴答计时器实现一个延时函数：
volatile uint32_t cnt = 0;  //volatile类型
void SysTick_Handler(void)
{
    cnt++;
}
void delay_ms(uint32_t t)
{
    cnt = 0;
    while(cnt-t>0);
}
（6）main.c实现的功能
2颗LED每100ms闪烁一次，同时串口输出此时拨码开关的状态。
Main.c完成后，对main.c进行编译：
 
 
如果编译没有错误，就可以进行程序下载。
2.FLASH编程算法生成
使用Jlink下载程序需要指定Flash编程算法，但是Keil没有自带我们需要的算法。
在Options for Target 中选择debug，Use一栏选择“JLINK/J-TRACE Cortex”
 
然后点击旁边的Settings加入算法：
如果点击Settings出现：
 
直接点击ok即可。
在弹出的窗口中选择Cortex-M3器件：
 
窗口中设置Port为SW，时钟频率最大设置为50MHz：
 
确定后返回Settings界面：
Settings->Flash Download->勾选“Program, Verify, Reset and run”，然后点击Add:
 
弹出的算法是Keil自带的算法，没有我们需要的。
定制Flash编程算法的方法是，打开Keil安装目录下的\ARM\Flash文件夹，将_Template文件夹复制一份，并命名为DS_CM3：
 
 
打开DS_CM3中的Keil工程：
 
这个Keil工程中可以设置自己要编程的Flash起始地址、大小，擦除大小等,在FlashDev.c文件填入以下内容，和我们之前ITCM的配置保持一致，起始地址0x0，大小64K。
#include "..\FlashOS.H"        // FlashOS Structures
struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MyCM3on7010",              // Device Name 
   ONCHIP,                     // Device Type
   0x00000000,                 // Device Start Address
   0x00010000,                 // 修改为64KB
   1024,                       // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   100,                        // Program Page Timeout 100 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x010000, 0x000000,         // 只有一个扇区，起始地址为0
   SECTOR_END
};
FlashPrg.c文件，实现一些存储区擦除的函数：
#include "..\FlashOS.H"        // FlashOS Structures
#include "string.h"
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
  return (0);                                  // Finished without Errors
}
int UnInit (unsigned long fnc) {
  return (0);                                  // Finished without Errors
}
int EraseChip (void) {
  memset((unsigned char *)0, 0, 0x10000);
  return (0);                                  // Finished without Errors
}
int EraseSector (unsigned long adr) {
  memset((unsigned char *)adr, 0, 1024);
  return (0);                                  // Finished without Errors
}
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  memcpy((unsigned char *)adr, buf, sz);
  return (0);                                  // Finished without Errors
}
以上操作完成后，保存并编译打开的这个工程：
 
编译完成后生成了文件：NewDevice.FLM
 
将这个文件复制到Flash目录下：
 
3.实现效果
以上步骤完成后，将我们自己建立的Flash算法加到工程中：
 
通过Download将程序下载到开发板上：
 
最终实现效果：
（1）LED1,LED2每500ms闪烁一次
 
（2）串口监听工具1s每次监听KEI1,KEY2按键状态
需要硬件：CH340E USB转串口工具
需要软件：CH3401SER驱动
 
 
按ch340E txd-开发板上的rxd；ch340E rxd-开发板上的txd回环方式连接：
 
打开设备管理器，在通用串行控制总线下的USB Serial Converter设备已经转换为端口（COM和LPT）中的设备。
 
 
使用XCOM工具监听对应端口COM7，可以监听到KEI1,KEY2是否被按下：KEY1按键按下，键状态由1改为0，KEY2按键按下，系统复位：
 
需要注意的是，需要确认CH340模块上的rxd指示灯与开发板上的LED灯同频闪烁，证明串口消息被收到，否则需要检查连线方式。





四、实验过程中遇到的问题
1.顶层文件与Wrapper中定义的函数名称不同，综合时报错
 
解决方法：
在BD中的外部端口类型要和top中定义的名称一样。（BD产生Wrapper，成为一种函数，top中用的就是这些函数）
2.外部单端时钟swclk输入，分配的管脚是N管脚执行时遇到的错误
 
解决方法：
将swclk放到P管脚，U19改为U18,执行成功,
 
3. 没有指定GPIO电平标准，生成bit流文件时报错
 
解决方法：实验用到的FPGA的几个BANK都是3.3V供电，给定标准电压
  
4.执行后时序不满足要求
 
时序错误的两个可能原因：
（1）clk_wizard单端输入为50MHz
 
 
（2）没有写入算法前，出现时序不满足是因为连线错误，仔细检查连线。
 
本实验中时序WNS和TNS报告负绝对值变小的几个方法：
（1）	时钟输出IP由PLL改为MMCM
（2）	拉低GPIO的电压标准：LVCMOS33->LVCMOS18
（3）	打开BlockDesign,选择AXI总线互连IP，选择主机接口，打开寄存器切片（Register Slice）
（4）	降低时钟频率（M3最高工作到50MHz,如果设置为100MHz，时序报告会出现负几万数量级的TNS）
本实验只建议使用（1）和（3）改善时序。最终的时序报告中TNS仍然有负值，这是因为引入了与系统异步时钟的SWD接口模块，负值接近于0，暂且忽略。
5.Keil无法编译
 
解决方法：
这个报错原因是Compiler Version 5编译器在Keil 5.37以后就不再默认安装了
从5.37版本开始，Keil 默认安装的是 Compiler Version 6.18，在option for target-Target-ARM Compiler中选择Use default version6，点击ok，再次编译，不会报错。
 
6.USB下载接入只有USB Serial Converter，没有COM显示
解决办法：正确安装Jlink驱动后,在Windows下的设备管理器-通用总线控制设备会找到USB Serial Converter，运行Vivado/2019.1/data/xicom/cable_drivers/nt64/digilent下的digilent程序，运行时需要关闭所有正在运行的程序，之后将CH340等USB转串口的设备正确连接，重新打开设备管理器，找到对应串口即可。
 
