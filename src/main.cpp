//重构循环结构，状态机驱动
//开机界面、恒功率待机界面、恒电压待机界面、功率设置界面、电压设置界面、能耗统计界面




#include "OneButton.h"
#include <ESP32Encoder.h> 
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <ModbusMaster.h>
#include <ModbusRTU.h>
#include "BLEDevice.h"
#include <Preferences.h>

#define DEBUG
#define TIME_LIMIT 5000

//定义按键和按键指示灯，这个按键用于切换工作状态
#define PIN_INPUT 34
OneButton button;
unsigned long inerface_status_time_stamp = 0;

//旋转编码器定义部分
#define CLK 39 // CLK ENCODER 
#define DT 36 // DT ENCODER 
ESP32Encoder encoder;

//modbus主机部分
ModbusMaster node;

//状态助记符和状态机参数
#define START_INTERFACE 0
#define CW_INTERFACE 1
#define CV_INTERFACE 2
#define WSET_INTERFACE 3
#define VSET_INTERFACE 4
#define ITEMS_INTERFACE 5
uint8_t interface_status = 0;
//上次复位前运行的状态
uint8_t last_interface_status = 1;

//永久存储部分
Preferences preferences;
//结构体才是正道
struct logtype {
    uint32_t ODO_Ws;
    uint32_t ODO_HS;
    uint32_t TRIP_Ws;
    uint32_t TRIP_HS;
    uint32_t LAST_MODE;
    uint32_t LAST_SETTING;
    uint32_t DAY;
};
logtype nvs_logger = {0,0,0,0,1,200,1};

//屏幕相关定义
//U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 5, 12, 13);

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

//一些重要的全局变量
uint16_t power_set = 200;
uint16_t voltage_set = 2480;

// //所有的运行参数一律放入一个结构体，方便管理
struct datatype {
    uint16_t output_power;
    uint16_t cadence;
    uint16_t heart_rate;
    uint16_t trip_time;

    uint16_t max_output_power;
    uint16_t max_cadence;
    uint16_t max_heart_rate;

    uint16_t odo_time;
    uint16_t odo_wh;
    uint16_t running_mode;
    uint16_t running_setting;

    uint16_t voltage_in;
    uint16_t voltage_out;
    uint16_t current_out;
};
datatype data_details = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

unsigned long currentadjust_time_stamp = 0;
unsigned long outputpower_updatetime_stamp = 0;
//每次按键按下都会出发该函数，进而切换状态
//在每次切换状态时，还有一些工作要做
//虽然这种状态机的方式还是很复杂，但是起码把运行和状态切换进行了解耦
void ButtonClick(){
    // Serial.print("buttonClicked");
    switch (interface_status){
        case START_INTERFACE:
            break;
        case CW_INTERFACE:
            //“恒功率待机界面”的操作会使得系统进入“恒功率设置界面”
            interface_status = WSET_INTERFACE;
            //首次进入WSET界面，要执行状态初始化
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CW:");
                u8g2.setCursor(3, 60);
                u8g2.print(power_set);u8g2.print("W");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis();            
            break;
        case CV_INTERFACE:
            //“恒压待机界面”的操作会使得系统进入“恒压设置界面”
            interface_status = VSET_INTERFACE;
            //首次进入VSET界面，要执行状态初始化
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CV:");
                u8g2.setCursor(3, 60);
                u8g2.print(voltage_set/100.0F,1);u8g2.print("V");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis();   
            break;
        case WSET_INTERFACE:
            //“恒功率设置界面”的操作会使得系统进入“恒压设置界面”，如此循环
            interface_status = VSET_INTERFACE;
            //首次进入VSET界面，要执行状态初始化
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CV:");
                u8g2.setCursor(3, 60);
                u8g2.print(voltage_set/100.0F,1);u8g2.print("V");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis();
            break;
        case VSET_INTERFACE:
            interface_status = ITEMS_INTERFACE;
            //首次进入ITEMS_INTERFACE界面，要执行状态初始化
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("ITEMS:");
            } while ( u8g2.nextPage() );
            break;
        case ITEMS_INTERFACE:
            interface_status = WSET_INTERFACE;
            //首次进入WSET界面，要执行状态初始化
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CW:");
                u8g2.setCursor(3, 60);
                u8g2.print(power_set);u8g2.print("W");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis();
            break;        
        default:
            break;
    }
    #ifdef DEBUG
    Serial.print("Button Click Change Status -- ");
    Serial.println(interface_status);
	#endif


}
//状态机的另一个驱动来自时间的倒计时，该函数是不断重复执行的
void TimeCountdownTick(){
    //开机就重置倒计时，为开机界面倒数
    //借助按键函数，每次更换为倒计时状态就重置定时器
    switch (interface_status){
        case START_INTERFACE:
            //需要倒计时5S进入主界面
            if((millis() - inerface_status_time_stamp) > TIME_LIMIT){
                interface_status = last_interface_status;
                #ifdef DEBUG
                Serial.print("CountDown Change Status -- ");
                Serial.println(interface_status);
                #endif
            }
            break;
        case WSET_INTERFACE:
            //需要倒计时5S进入主界面
            if((millis() - inerface_status_time_stamp) > TIME_LIMIT){
                interface_status = CW_INTERFACE;
                #ifdef DEBUG
                Serial.print("CountDown Change Status -- ");
                Serial.println(interface_status);
                #endif
            }
            break;
        case VSET_INTERFACE:
            //需要倒计时5S进入主界面
            if((millis() - inerface_status_time_stamp) > TIME_LIMIT){
                interface_status = CV_INTERFACE;
                #ifdef DEBUG
                Serial.print("CountDown Change Status -- ");
                Serial.println(interface_status);
                #endif
            }
            break;
        default:
            break;
    }
}

unsigned long cadence_time_stamp = 0;
unsigned long cadence_lastpulse_timestamp = 0;
uint16_t cadence_buffer[12];
//频率计部分
void frequency_meter()
{
	unsigned long cadence_sum = 0;
	//刷新该时间，可以使得转速在长时间不中断的情况下清零
	cadence_time_stamp = millis();
	//波形必须得滤波，要不然变化太快
	for (int i=0;i<11;i++){
		cadence_buffer[i] = cadence_buffer[i+1];
	}
    //最好进行一个异常数检测，软件的那种
	//直接计算得到
	cadence_buffer[11] = int(16000000.0/(micros()-cadence_lastpulse_timestamp));
	cadence_lastpulse_timestamp = micros();
	//然后对踏频求平均
	for (int i=0;i<12;i++){
		cadence_sum = cadence_buffer[i] + cadence_sum;
	}
	data_details.cadence = int(cadence_sum/12);
}
void setup(){
    Serial.begin(115200);
	//MODBUS主机部分
	Serial2.begin(115200);
	node.begin(1, Serial2);
    //初始化开机倒计时
    inerface_status_time_stamp = millis();
    //初始化按键设置
    button.setup(PIN_INPUT, INPUT_PULLUP, true);
    button.attachClick(ButtonClick);
	//旋转编码器初始化部分
	encoder.attachHalfQuad(DT,CLK);
	encoder.setCount(0);
    //使用硬件频率计
	attachInterrupt(35,frequency_meter,FALLING);
    //取出永久存储的设置
    preferences.begin("nvs-log", false);
    preferences.getBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
    data_details.trip_time = nvs_logger.TRIP_HS;
    last_interface_status = nvs_logger.LAST_MODE;
    if(1 == last_interface_status){
        power_set = nvs_logger.LAST_SETTING;
    }
    if(2 == last_interface_status){
        voltage_set = nvs_logger.LAST_SETTING;
    }
    //显示部分改为显示累积数值
    u8g2.begin();
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(3, 13);
        u8g2.print("ODO:");u8g2.print((nvs_logger.ODO_Ws)/3600);u8g2.print("WH");
        u8g2.setCursor(3, 23);
        u8g2.print("ODO:");u8g2.print(nvs_logger.ODO_HS/3600);u8g2.print("H");u8g2.print(nvs_logger.ODO_HS%3600);u8g2.print("S");
        u8g2.setCursor(3, 33);
        u8g2.print("TRIP:");u8g2.print((nvs_logger.TRIP_Ws)/3600);u8g2.print("WH");
        u8g2.setCursor(3, 43);
        u8g2.print("TRIP:");u8g2.print(nvs_logger.TRIP_HS);u8g2.print("S");
        u8g2.setCursor(3, 53);
        u8g2.print("LAST_MODE:");u8g2.print(nvs_logger.LAST_MODE);
        u8g2.setCursor(3, 63);
        u8g2.print("LAST_SETTING:");u8g2.print(nvs_logger.LAST_SETTING);
    } while ( u8g2.nextPage() );


    //每次开机检查,检查本次开机的时间，是不是和上次开机的时间，是同一天，如果不是。
    //将trip信息写入日志，并将永久记录中的trip信息都清零

}
//主循环
void loop(){
    //主循环中不断查询按键状态
    button.tick();
    //主循环中检查时间戳
    TimeCountdownTick();

    if(START_INTERFACE == interface_status){
        //显示重要数据，计划是5个，ODO_wh ODO_HS TRIP_WH TRIP_HS MODE
        // //显示部分改为显示累积数值
        // u8g2.begin();
        // u8g2.firstPage();
        // do {
        //     u8g2.setFont(u8g2_font_6x10_tf);
        //     u8g2.setCursor(3, 13);
        //     u8g2.print("ODO:");u8g2.print((nvs_logger.ODO_Ws)/3600);u8g2.print("WH");
        //     u8g2.setCursor(3, 23);
        //     u8g2.print("ODO:");u8g2.print(nvs_logger.ODO_HS/3600);u8g2.print("H");u8g2.print(nvs_logger.ODO_HS%3600);u8g2.print("S");
        //     u8g2.setCursor(3, 33);
        //     u8g2.print("TRIP:");u8g2.print((nvs_logger.TRIP_Ws)/3600);u8g2.print("WH");
        //     u8g2.setCursor(3, 43);
        //     u8g2.print("TRIP:");u8g2.print(nvs_logger.TRIP_HS);u8g2.print("S");
        //     u8g2.setCursor(3, 53);
        //     u8g2.print("LAST_MODE:");u8g2.print(nvs_logger.LAST_MODE);
        //     u8g2.setCursor(3, 63);
        //     u8g2.print("LAST_SETTING:");u8g2.print(nvs_logger.LAST_SETTING);
        // } while ( u8g2.nextPage() );

    }
    if(CW_INTERFACE == interface_status){
        // 显示和更新屏幕，为了直观显示两种不同的工况，是否可以用阴阳两种颜色
		u8g2.firstPage();
		do {
			u8g2.drawHLine(2,1,128);u8g2.drawHLine(2,32,128);u8g2.drawHLine(2,63,128);
			u8g2.drawVLine(2,1,64);u8g2.drawVLine(65,1,64);u8g2.drawVLine(127,1,64);

			u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(6, 24);
            u8g2.print(data_details.output_power);u8g2.print("W");

			u8g2.setFont(u8g2_font_VCR_OSD_tu);
            if(0 == data_details.cadence){
                u8g2.setCursor(69, 24);
                u8g2.print("---");
            }
            else{
                u8g2.setCursor(69, 24);
                u8g2.print(data_details.cadence);u8g2.print("R");
            }

            u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(6, 55);
            u8g2.print(data_details.heart_rate);u8g2.print("B");

            u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(67, 55);
            u8g2.print(data_details.trip_time);u8g2.print("S");

		} while ( u8g2.nextPage() );        
        //驱动电源板，不断改变其设置，改成5S更新一次，且每次更新完，读取数据之前，需要延时，否则会卡住
		unsigned long currentadjust_time_gap = millis() - currentadjust_time_stamp;
		if(currentadjust_time_gap > 5000){
			// MODBUS更新部分，要不断的更新，因为输出电压在不断变化
			uint16_t current_set = 0;
			//读取输出电压
			uint8_t result = node.readHoldingRegisters(2, 1);
			if (result == node.ku8MBSuccess)
			{
				//直接计算设定功率除以输出电压
				current_set = power_set/(node.getResponseBuffer(0)/100.0)*100;
				//大于20A则等于20A
				current_set = MIN(current_set,2000);
			}
			//回写设定电流和限制电压
			node.setTransmitBuffer(0, 1590);
			node.setTransmitBuffer(1, current_set);
			result = node.writeMultipleRegisters(0, 2);
			// //稍作延时，否则读写DC-DC的频率太高了,
			delay(50);
			currentadjust_time_stamp = millis();
		}
        //更新1S频率的任务，检查最大值，更新永久累计和设定
    	unsigned long outputpower_updatetime_gap = millis() - outputpower_updatetime_stamp;
    	if(outputpower_updatetime_gap > 1000){
    		uint8_t result = node.readHoldingRegisters(4, 1);
    		if (result == node.ku8MBSuccess)
    		{
    			data_details.output_power = int(node.getResponseBuffer(0)/10);
                //功率大于100W，一切开始累计
                if(data_details.output_power > 100)
                {
                    data_details.trip_time ++;
                    //更新永久数据并及时写入
                    nvs_logger.ODO_Ws = nvs_logger.ODO_Ws + data_details.output_power;
                    nvs_logger.ODO_HS ++;
                    nvs_logger.TRIP_Ws = nvs_logger.TRIP_Ws + data_details.output_power;
                    nvs_logger.TRIP_HS ++;
                    nvs_logger.LAST_MODE = 1;
                    nvs_logger.LAST_SETTING = power_set;
                    preferences.putBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
                }
    		}
            //更新最大值
            if(data_details.output_power > data_details.max_output_power)
            {
                data_details.max_output_power = data_details.output_power;
            }
            if(data_details.cadence > data_details.max_cadence)
            {
                data_details.max_cadence = data_details.cadence;
            }
            if(data_details.heart_rate > data_details.max_heart_rate)
            {
                data_details.max_heart_rate = data_details.heart_rate;
            }
            //复位倒计时
            outputpower_updatetime_stamp = millis();
    	}
        //长时间没有脉冲的时候，显示为---
        unsigned long cadence_gap = millis() - cadence_time_stamp;
        if(cadence_gap > 5000){
            data_details.cadence = 0;
        }
    }
    if(CV_INTERFACE == interface_status){
        // 显示和更新屏幕，为了直观显示两种不同的工况，是否可以用阴阳两种颜色
		u8g2.firstPage();
		do {
            u8g2.setFontMode(1);
            u8g2.setDrawColor(1);
            u8g2.drawBox(1,1,128,64);
            u8g2.setDrawColor(2);

			u8g2.drawHLine(2,1,128);u8g2.drawHLine(2,32,128);u8g2.drawHLine(2,63,128);
			u8g2.drawVLine(2,1,64);u8g2.drawVLine(65,1,64);u8g2.drawVLine(127,1,64);

			u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(6, 24);
            u8g2.print(data_details.output_power);u8g2.print("W");

			u8g2.setFont(u8g2_font_VCR_OSD_tu);
            if(0 == data_details.cadence){
                u8g2.setCursor(69, 24);
                u8g2.print("---");
            }
            else{
                u8g2.setCursor(69, 24);
                u8g2.print(data_details.cadence);u8g2.print("R");
            }

            u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(6, 55);
            u8g2.print(data_details.heart_rate);u8g2.print("B");

            u8g2.setFont(u8g2_font_VCR_OSD_tu);
            u8g2.setCursor(67, 55);
            u8g2.print(data_details.trip_time);u8g2.print("S");

		} while ( u8g2.nextPage() );        
        //驱动电源板，不断改变其设置，改成5S更新一次，且每次更新完，读取数据之前，需要延时，否则会卡住
		unsigned long currentadjust_time_gap = millis() - currentadjust_time_stamp;
		if(currentadjust_time_gap > 5000){
			node.setTransmitBuffer(0, voltage_set);
			node.setTransmitBuffer(1, 2000);
			uint8_t result = node.writeMultipleRegisters(0, 2);
			delay(50);
			currentadjust_time_stamp = millis();
		}
        //更新1S频率的任务，检查最大值，更新永久累计和设定
    	unsigned long outputpower_updatetime_gap = millis() - outputpower_updatetime_stamp;
    	if(outputpower_updatetime_gap > 1000){
    		uint8_t result = node.readHoldingRegisters(4, 1);
    		if (result == node.ku8MBSuccess)
    		{
    			data_details.output_power = int(node.getResponseBuffer(0)/10);
                //功率大于100W，一切开始累计
                if(data_details.output_power > 100)
                {
                    data_details.trip_time ++;
                    //更新永久数据并及时写入
                    nvs_logger.ODO_Ws = nvs_logger.ODO_Ws + data_details.output_power;
                    nvs_logger.ODO_HS ++;
                    nvs_logger.TRIP_Ws = nvs_logger.TRIP_Ws + data_details.output_power;
                    nvs_logger.TRIP_HS ++;
                    nvs_logger.LAST_MODE = 2;
                    nvs_logger.LAST_SETTING = voltage_set;
                    preferences.putBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
                }
    		}
            //更新最大值
            if(data_details.output_power > data_details.max_output_power)
            {
                data_details.max_output_power = data_details.output_power;
            }
            if(data_details.cadence > data_details.max_cadence)
            {
                data_details.max_cadence = data_details.cadence;
            }
            if(data_details.heart_rate > data_details.max_heart_rate)
            {
                data_details.max_heart_rate = data_details.heart_rate;
            }
            //复位倒计时
            outputpower_updatetime_stamp = millis();
    	}
        //长时间没有脉冲的时候，显示为---
        unsigned long cadence_gap = millis() - cadence_time_stamp;
        if(cadence_gap > 5000){
            data_details.cadence = 0;
        }
    }
    if(WSET_INTERFACE == interface_status){
        int64_t encoderpos = encoder.getCount();
        if (0 != encoderpos){
            power_set = power_set + encoderpos;
            power_set = MAX(power_set,50);
            power_set = MIN(power_set,300);
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CW:");
                u8g2.setCursor(3, 60);
                u8g2.print(power_set);u8g2.print("W");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis(); 
            //每次都清零编码器，省了不少事
            encoder.setCount(0);
		}        
    }
    if(VSET_INTERFACE == interface_status){
        int64_t encoderpos = encoder.getCount();
        if (0 != encoderpos){
            voltage_set = voltage_set + encoderpos*5;
            voltage_set = MAX(voltage_set,500);
            voltage_set = MIN(voltage_set,6000);
            u8g2.firstPage();
            do {
                u8g2.setFont(u8g2_font_inb24_mf);
                u8g2.setCursor(3, 28);
                u8g2.print("CV:");
                u8g2.setCursor(3, 60);
                u8g2.print(voltage_set/100.0F,1);u8g2.print("V");
            } while ( u8g2.nextPage() );
            //重置界面倒计时，编码器旋转也会重置倒计时
            inerface_status_time_stamp = millis(); 
            //每次都清零编码器，省了不少事
            encoder.setCount(0);
		}       
    }
    if(ITEMS_INTERFACE == interface_status){
        
    }
}








// /*待处理事项
// 1、首次进入工作循环后，电压模式下，不触发设置。因旋转编码器才能触发写modbus。
// 2、蓝牙工作距离太近，蓝牙无法重新连接，怀疑是敷铜PCB遮挡了天线。
// 3、声明最大功率、最大踏频、最大心率的时候都赋值了，需清零。
// 4、恒流限压模式下，截止电压太低，设置15.7V，实际15.3V就截止了。
// 5、开机界面下，无法调整工作模式，只能等待机后，才可以调整，应该又bug。//可能是蓝牙再阻塞扫描。
// 6、有一款软件很神奇，装上会让我那个ESPDUINO32板子上的CH340显示端口号，但是无论怎么打开串口，都提示串口不可用。其他的串口转换芯片都没事，比如CP2102。
// 7、为了能够现场调试，我不得不使用笔记本，但是串口不可用，我只能重新做了一版PCB，新PCB端口定义不同，代码需重写。
// 8、添加了部分ODO功能，同意输出总电量和有效总时长
// 9、下一步计划使用setcursor和print功能代替一个字符一个字符的编辑
// 10、新版本的编码器反转，调整一下
// 11、添加一个使能电源板开机功能。
// */

// #include <ESP32Encoder.h> 
// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <SPI.h>
// #include <ModbusMaster.h>
// #include <ModbusRTU.h>
// #include "BLEDevice.h"
// #include <Preferences.h>

// #define MIN(a,b) (((a)<(b))?(a):(b))
// #define MAX(a,b) (((a)>(b))?(a):(b))

// // #define DEBUG

// // //旋转编码器定义部分
// #define CLK 39 // CLK ENCODER 
// #define DT 36 // DT ENCODER 
// ESP32Encoder encoder;

// //屏幕相关定义
// //U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
// U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 5, 12, 13);

// //modbus主机部分
// ModbusMaster node;

// //永久存储部分
// Preferences preferences;
// //结构体才是正道
// struct logtype {
//   uint32_t cumulative_Ws;
//   uint32_t cumulative_Seconds;
// };
// logtype nvs_logger = {0,0};
// //结构体按理说比较容易阅读，但是我的能力很有限，所以用一个64bit的量代替之
// // uint32_t cumulative_Ws = 0;
// // uint32_t cumulative_Seconds = 0;

// //输出参数及模式设置相关变量
// //功率设定值
// #define power_pre_set 200
// uint16_t power_set = 200;
// //模式1恒流限压模式的电压限定上限
// uint16_t voltage_limit = 1590;
// //模式2恒压模式下的电压设定
// #define voltage_pre_set 2480
// uint16_t voltage_set = 2480;
// //模式声明
// uint8_t working_mode = 200;
// //通过各种条件触发或解除待机显示
// uint8_t starup_mode = 1;
// uint8_t screen_mode = 0;
// unsigned long screen_mode_timestamp = 0;
// //声明2个变量来记录编码器的偏移
// // int16_t pos_modeA = 0;
// // int16_t pos_modeB = 0;
// int pos = 0;
// int newpos = 0;
// //为了防止模式切换造成的编码器清零，此处记录之
// int power_set_encoderpos = 0;
// int power_set_encodernewpos = 0;
// int voltage_set_encoderpos = 0;
// int voltage_set_encodernewpos = 0;

// //辅助字符数组
// char power_set_string[6] = "200W";
// // String power_set_string;
// char voltage_set_string[6] = "24.8V";
// // 定义字符缓冲区

// //定义变量方便计时
// unsigned long cumulative_time_stamp = 0;
// unsigned long currentadjust_time_stamp = 0;
// unsigned long outputpower_updatetime_stamp = 0;
// unsigned long voltageadjust_time_stamp = 0;
// //MODEBUS从机部分
// #define SLAVE_ID 1
// //定义1个从机类
// ModbusRTU mb;
// //定义全局变量
// // uint16_t output_power = 0;
// // uint16_t cadence = 0;
// // uint16_t heart_rate = 0;
// // uint16_t max_output_power = 0;
// // uint16_t max_cadence = 0;
// // uint16_t max_heart_rate = 0;
// // uint16_t cumulative_time = 0;
// // uint16_t accumulated_wh  = 0;
// uint16_t output_power = 0;
// uint16_t cadence = 0;
// uint16_t heart_rate = 0;
// uint16_t max_output_power = 0;
// uint16_t max_cadence = 0;
// uint16_t max_heart_rate = 0;
// uint16_t cumulative_time = 0;
// uint16_t accumulated_wh  = 0;

// void modbusrtu_dataprepare()
// {
// 	mb.addHreg(0);mb.Hreg(0,output_power);
// 	mb.addHreg(1);mb.Hreg(1,cadence);
// 	mb.addHreg(2);mb.Hreg(2,heart_rate);
// 	mb.addHreg(3);mb.Hreg(3,max_output_power);
// 	mb.addHreg(4);mb.Hreg(4,max_cadence);
// 	mb.addHreg(5);mb.Hreg(5,max_heart_rate);
// 	mb.addHreg(6);mb.Hreg(6,cumulative_time);
// 	mb.addHreg(7);mb.Hreg(7,accumulated_wh);
// }

// unsigned long cadence_time_stamp = 0;
// unsigned long cadence_lastpulse_timestamp = 0;
// uint16_t cadence_buffer[12];
// //频率计部分
// void frequency_meter()
// {
// 	unsigned long cadence_sum = 0;
// 	//刷新该时间，可以使得转速在长时间不中断的情况下清零
// 	cadence_time_stamp = millis();
// 	//波形必须得滤波，要不然变化太快
// 	for (int i=0;i<11;i++){
// 		cadence_buffer[i] = cadence_buffer[i+1];
// 	}
// 	//直接计算得到
// 	cadence_buffer[11] = int(16000000.0/(micros()-cadence_lastpulse_timestamp));
// 	cadence_lastpulse_timestamp = micros();
// 	//然后对踏频求平均
// 	for (int i=0;i<12;i++){
// 		cadence_sum = cadence_buffer[i] + cadence_sum;
// 	}
// 	cadence = int(cadence_sum/12);
// }

// //蓝牙定义部分
// //蓝牙部分
// static BLEUUID service_HR_UUID(BLEUUID((uint16_t)0x180D));
// static BLEUUID char_HR_UUID(BLEUUID((uint16_t)0x2A37));

// // static BLEUUID service_cadence_UUID(BLEUUID((uint16_t)0x1816));
// // static BLEUUID char_cadence_UUID(BLEUUID((uint16_t)0x2A5B));

// static boolean doConnect = false;
// static boolean connected = false;
// static boolean doScan = false;
// static BLERemoteCharacteristic* pRemoteCharacteristic;
// static BLEAdvertisedDevice* myDevice;

// static void notifyCallback(
// 	BLERemoteCharacteristic* pBLERemoteCharacteristic,
// 	uint8_t* pData,
// 	size_t length,
// 	bool isNotify) {
// 	heart_rate = pData[1];
// 	#ifdef DEBUG
//     Serial.print("heart rate: ");
//     Serial.print(heart_rate);
// 	Serial.println("bpm");
// 	#endif
// }

// class MyClientCallback : public BLEClientCallbacks {
// 	void onConnect(BLEClient* pclient) {
// 	}

// 	void onDisconnect(BLEClient* pclient) {
// 		connected = false;
// 		#ifdef DEBUG
// 		Serial.println("onDisconnect");
// 		#endif
// 	}
// };

// bool connectToServer() {
// 	#ifdef DEBUG
// 	Serial.print("Forming a connection to ");
//     Serial.println(myDevice->getAddress().toString().c_str());
// 	#endif
    
//     BLEClient*  pClient  = BLEDevice::createClient();
// 	#ifdef DEBUG
//     Serial.println(" - Created client");
// 	#endif
//     pClient->setClientCallbacks(new MyClientCallback());
//     // Connect to the remove BLE Server.
//     pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
// 	#ifdef DEBUG
//     Serial.println(" - Connected to server");
// 	#endif
//     pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
//     // Obtain a reference to the service we are after in the remote BLE server.
//     BLERemoteService* pRemoteService = pClient->getService(service_HR_UUID);
//     if (pRemoteService == nullptr) {
// 		#ifdef DEBUG
// 		Serial.print("Failed to find our service UUID: ");
// 		Serial.println(service_HR_UUID.toString().c_str());
// 		#endif
// 		pClient->disconnect();
// 		return false;
//     }
// 	#ifdef DEBUG
//     Serial.println(" - Found our service");
// 	#endif

//     // Obtain a reference to the characteristic in the service of the remote BLE server.
//     pRemoteCharacteristic = pRemoteService->getCharacteristic(char_HR_UUID);
//     if (pRemoteCharacteristic == nullptr) {
// 		#ifdef DEBUG
// 		Serial.print("Failed to find our characteristic UUID: ");
// 		Serial.println(char_HR_UUID.toString().c_str());
// 		#endif
// 		pClient->disconnect();
// 		return false;
//     }
// 	#ifdef DEBUG
//     Serial.println(" - Found our characteristic");
// 	#endif
//     // Read the value of the characteristic.
//     if(pRemoteCharacteristic->canRead()) {
// 		std::string value = pRemoteCharacteristic->readValue();
// 		#ifdef DEBUG
// 		Serial.print("The characteristic value was: ");
// 		Serial.println(value.c_str());
// 		#endif
//     }

//     if(pRemoteCharacteristic->canNotify())
//       pRemoteCharacteristic->registerForNotify(notifyCallback);

//     connected = true;
//     return true;
// }
// /**
//  * Scan for BLE servers and find the first one that advertises the service we are looking for.
//  */
// class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
//  /**
//    * Called for each advertising BLE server.
//    */
//   void onResult(BLEAdvertisedDevice advertisedDevice) {
// 	#ifdef DEBUG
//     Serial.print("BLE Advertised Device found: ");
//     Serial.println(advertisedDevice.toString().c_str());
// 	#endif
//     // We have found a device, let us now see if it contains the service we are looking for.
//     if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(service_HR_UUID)) {
// 		BLEDevice::getScan()->stop();
// 		myDevice = new BLEAdvertisedDevice(advertisedDevice);
// 		doConnect = true;
// 		doScan = true;
//     } // Found our server
//   } // onResult
// }; // MyAdvertisedDeviceCallbacks

// void setup(void) {
// 	//旋转编码器初始化部分
// 	encoder.attachHalfQuad(DT,CLK);
// 	encoder.setCount(0);
// 	//ODO累计值初始化
// 	preferences.begin("nvs-log", false);
// 	preferences.getBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
// 	//显示部分初始化
// 	// u8g2.begin();
// 	// u8g2.firstPage();
// 	// do {
// 	// 	u8g2.setFont(u8g2_font_lubB14_te);
// 	// 	u8g2.drawStr(1, 40, "standby");
// 	// } while ( u8g2.nextPage() );
// 	//显示部分改为显示累积数值
// 	u8g2.begin();
// 	u8g2.firstPage();
// 	do {
// 		u8g2.setFont(u8g2_font_VCR_OSD_tu);

// 		u8g2.setCursor(1, 20);
// 		u8g2.print("ODO:");
// 		u8g2.print((nvs_logger.cumulative_Ws)/3600);
// 		u8g2.print("WH");

// 		u8g2.setCursor(1, 52);
// 		u8g2.print("ODO:");
// 		u8g2.print(nvs_logger.cumulative_Seconds);	
// 		u8g2.print("S");
// 	} while ( u8g2.nextPage() );

// 	//MODBUS主机部分
// 	Serial2.begin(115200);
// 	node.begin(1, Serial2);
// 	//设置按键切换状态
// 	pinMode(34,INPUT_PULLUP);
// 	//MODEBUS从机设置
// 	Serial.begin(115200, SERIAL_8N1);
// 	mb.begin(&Serial);
// 	mb.slave(SLAVE_ID);
// 	modbusrtu_dataprepare();
// 	// Serial.println("Starting Arduino BLE Client application...");
// 	BLEDevice::init("");
// 	BLEScan* pBLEScan = BLEDevice::getScan();
// 	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
// 	pBLEScan->setInterval(1349);
// 	pBLEScan->setWindow(449);
// 	pBLEScan->setActiveScan(true);
// 	pBLEScan->start(5, false);
// 	//使用硬件频率计
// 	attachInterrupt(35,frequency_meter,FALLING);
// 	//修复一个bug，自动开机设置
// 	node.writeSingleRegister(0x0012,1);
// 	//稍作延时，等待参数写入完毕
// 	delay(50);
// }

// void loop(void) {
//   //0、开机等待模式
//   //1、恒功率模式给超级电容充电
//   //2、恒压模式牵引固定电阻实现恒功率
//   //3、轮显所有数据
// 	uint8_t result;
// 	//定义一个时间戳，来研究运行速度
// 	// unsigned long running_time_stamp = micros();
// 	if (0 == working_mode)
// 	{
// 			//一旦发生变化，立刻更新
// 		power_set_encodernewpos = encoder.getCount();
// 		if (power_set_encoderpos != power_set_encodernewpos) {
// 			//首先立刻退出待机界面
// 			screen_mode = 0;
// 			screen_mode_timestamp = millis();
// 			//检查极限值，不能太小也不能太大
// 			power_set = power_pre_set + power_set_encodernewpos;
// 			power_set = MAX(power_set,50);
// 			power_set = MIN(power_set,300);
// 			//笨办法拼凑字符串
// 			power_set_string[2] = 0x30 + power_set%10;
// 			power_set_string[1] = 0x30 + (power_set/10)%10;
// 			power_set_string[0] = 0x30 + (power_set/100)%10;
// 			power_set_string[3] = 'W';
// 			power_set_string[4] = ' ';
// 			//更新位置信息
// 			power_set_encoderpos = power_set_encodernewpos;
// 			//更新显示
// 			u8g2.firstPage();
// 			do {
// 				u8g2.setFont(u8g2_font_inb24_mf);
// 				u8g2.drawStr(1, 48, power_set_string);
// 			} while ( u8g2.nextPage() );
// 		} 

// 		//改成5S更新一次，且每次更新完，读取数据之前，需要延时，否则会卡住
// 		unsigned long currentadjust_time_gap = millis() - currentadjust_time_stamp;
// 		if(currentadjust_time_gap > 5000){
// 			// MODBUS更新部分，要不断的更新，因为输出电压在不断变化
// 			uint16_t current_set = 0;
// 			//读取输出电压
// 			result = node.readHoldingRegisters(2, 1);
// 			if (result == node.ku8MBSuccess)
// 			{
// 				//直接计算设定功率除以输出电压
// 				current_set = power_set/(node.getResponseBuffer(0)/100.0)*100;
// 				//大于20A则等于20A
// 				current_set = MIN(current_set,2000);
// 			}
// 			//回写设定电流和限制电压
// 			node.setTransmitBuffer(0, voltage_limit);
// 			node.setTransmitBuffer(1, current_set);
// 			result = node.writeMultipleRegisters(0, 2);
// 			// //稍作延时，否则读写DC-DC的频率太高了,
// 			delay(50);
// 			currentadjust_time_stamp = millis();
// 		}
// 	}
// 	if (1 == working_mode)
// 	{
// 		voltage_set_encodernewpos = encoder.getCount();
// 		//一旦发生变化，立刻更新
// 		if (voltage_set_encoderpos != voltage_set_encodernewpos) {
// 			//首先立刻退出待机界面
// 			screen_mode = 0;
// 			screen_mode_timestamp = millis();
// 			//有新的输入要先检查当前模式
// 			voltage_set = voltage_pre_set + voltage_set_encodernewpos*5;
// 			voltage_set = MAX(voltage_set,500);
// 			voltage_set = MIN(voltage_set,3000);
// 			voltage_set_string[2] = '.';
// 			voltage_set_string[3] = 0x30 + (voltage_set/10)%10;
// 			voltage_set_string[1] = 0x30 + (voltage_set/100)%10;
// 			voltage_set_string[0] = 0x30 + (voltage_set/1000)%10;
// 			voltage_set_string[4] = 'V';
// 			//更新变化对比存储
// 			voltage_set_encoderpos = voltage_set_encodernewpos;
// 			//更新显示
// 			u8g2.firstPage();
// 			do {
// 				u8g2.setFont(u8g2_font_inb24_mf);
// 				u8g2.drawStr(1, 48, voltage_set_string);
// 			} while ( u8g2.nextPage() );
// 			// //不同于恒功率控制，电压设置只需要一次,图方便就一直设置好了。
// 			// //回写设定电压和限制电流
// 			// node.setTransmitBuffer(0, voltage_set);
// 			// node.setTransmitBuffer(1, 2000);
// 			// result = node.writeMultipleRegisters(0, 2);
// 			// delay(100);	
// 		}
// 		//改成5S更新一次，且每次更新完，读取数据之前，需要延时，否则会卡住
// 		unsigned long voltageadjust_time_gap = millis() - voltageadjust_time_stamp;
// 		if(voltageadjust_time_gap > 2000){
// 			node.setTransmitBuffer(0, voltage_set);
// 			node.setTransmitBuffer(1, 2000);
// 			result = node.writeMultipleRegisters(0, 2);
// 			delay(50);
// 			voltageadjust_time_stamp = millis();
// 		}
// 	}
// 	if (0 == digitalRead(34))
// 	{

// 		//消抖后改变模式
// 		delay(20);
// 		while(!digitalRead(34));
// 		delay(20);
// 		working_mode ++;

// 		//开机模式，开机后必须按键才能进入
// 		if (1 == starup_mode)
// 		{
// 			starup_mode = 0;
// 			working_mode = 0;
			
// 			// goto breakflag;
// 		}
// 		//待机模式下，按键立刻退出，但不影响模式变化
// 		if (1 == screen_mode)
// 		{
// 			//首先立刻退出待机界面，但不影响模式
// 			screen_mode = 0;
// 			screen_mode_timestamp = millis();
// 			working_mode--;
// 			// goto breakflag;
// 		}

// 		//即便不在待机模式下，按键也应清零倒计时
// 		screen_mode_timestamp = millis();

// 		//循环挡位
// 		working_mode = working_mode%2;
// 		if (0 == working_mode)
// 		{
// 			//恢复其编码器偏移数值
// 			encoder.setCount(power_set_encoderpos);
// 			//更新显示
// 			u8g2.firstPage();
// 			do {
// 				u8g2.setFont(u8g2_font_inb24_mf);
// 				u8g2.drawStr(1, 48, power_set_string);
// 			} while ( u8g2.nextPage() );
// 		}
// 		if (1 == working_mode)
// 		{
// 			//恢复其编码器偏移数值
// 			encoder.setCount(voltage_set_encoderpos);
// 			//更新显示
// 			u8g2.firstPage();
// 			do {
// 				u8g2.setFont(u8g2_font_inb24_mf);
// 				u8g2.drawStr(1, 48, voltage_set_string);
// 			} while ( u8g2.nextPage() );
// 		}

// 	}
// 	if (1 == screen_mode)
// 	{
// 		//按键按下，会退出
// 		//编码器转动，会退出
// 		//退出时标记时间，时间到了，又会进入
// 		u8g2.firstPage();
// 		do {
// 			u8g2.drawHLine(0,0,128);
// 			u8g2.drawHLine(0,31,128);
// 			u8g2.drawHLine(0,63,128);
// 			u8g2.drawVLine(0,0,64);
// 			u8g2.drawVLine(63,0,64);
// 			u8g2.drawVLine(127,0,64);
// 			char output_power_string[5] = "100W";
// 			output_power_string[2] = 0x30 + output_power%10;
// 			output_power_string[1] = 0x30 + (output_power/10)%10;
// 			output_power_string[0] = 0x30 + (output_power/100)%10;
// 			if(output_power < 10){
// 				output_power_string[2] = 0x2D;
// 				output_power_string[1] = 0x2D;
// 				output_power_string[0] = 0x2D;
// 			}
// 			u8g2.setFont(u8g2_font_VCR_OSD_tu);
// 			u8g2.drawStr(6, 24, output_power_string);

// 			char cadence_string[5] = "100R";
// 			cadence_string[2] = 0x30 + cadence%10;
// 			cadence_string[1] = 0x30 + (cadence/10)%10;
// 			cadence_string[0] = 0x30 + (cadence/100)%10;
// 			if(cadence < 10){
// 				cadence_string[2] = 0x2D;
// 				cadence_string[1] = 0x2D;
// 				cadence_string[0] = 0x2D;
// 			}
// 			u8g2.setFont(u8g2_font_VCR_OSD_tu);
// 			u8g2.drawStr(69, 24, cadence_string);

// 			char heart_rate_string[5] = "100B";
// 			heart_rate_string[2] = 0x30 + heart_rate%10;
// 			heart_rate_string[1] = 0x30 + (heart_rate/10)%10;
// 			heart_rate_string[0] = 0x30 + (heart_rate/100)%10;
// 			if(!connected){
// 				heart_rate_string[2] = 0x2D;
// 				heart_rate_string[1] = 0x2D;
// 				heart_rate_string[0] = 0x2D;
// 			}
// 			u8g2.setFont(u8g2_font_VCR_OSD_tu);
// 			u8g2.drawStr(6, 55, heart_rate_string);

// 			char cumulative_time_string[6] = "1000S";
// 			cumulative_time_string[3] = 0x30 + cumulative_time%10;
// 			cumulative_time_string[2] = 0x30 + (cumulative_time/10)%10;
// 			cumulative_time_string[1] = 0x30 + (cumulative_time/100)%10;
// 			cumulative_time_string[0] = 0x30 + (cumulative_time/1000)%10;
// 			u8g2.setFont(u8g2_font_VCR_OSD_tu);
// 			u8g2.drawStr(65, 55, cumulative_time_string);

// 		} while ( u8g2.nextPage() );
// 	}
// 	if (0 == screen_mode)
// 	{
// 		//不断刷新倒计时
// 		unsigned long screen_timer_gap;
// 		screen_timer_gap = millis()-screen_mode_timestamp;
// 		if (screen_timer_gap > 8000)
// 		{
// 			screen_mode = 1;
// 		}
// 	}

// 	//只要更新全局变量即可，一切自然会进入缓冲
// 	modbusrtu_dataprepare();
// 	mb.task();
// 	yield();

// 	// 蓝牙的循环
// 	//连接成功就更改标志位
// 	//不连接的时候及其的拖慢循环时间
// 	// if (doConnect == true) {
// 	// 	connectToServer();
// 	// 	doConnect = false;
// 	// }
	
// 	// //失去连接就开始重连
// 	// if (!connected) {
// 	// 	BLEDevice::getScan()->start(5); 
// 	// } 
// 	//临时注释掉，协助测试
// 	if (doConnect == true) {
// 		if (connectToServer()) {
// 			#ifdef DEBUG
// 			Serial.println("We are now connected to the BLE Server.");
// 			#endif
// 		} else {
// 			#ifdef DEBUG
// 			Serial.println("We have failed to connect to the server; there is nothin more we will do.");
// 			#endif
// 		}
// 	doConnect = false;
// 	}
//     if (!connected){
// 		BLEDevice::getScan()->start(5);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
// 	}


// 	// unsigned l.20ong running_time_stamp = micros();
// 	//用于使用MODBUS读取DC-DC的信息，得到输出功率，进而对比起最大值
// 	//读取功率数据
// 	//！！！！！特别注意，如果读之前刚刚写入过，读取就会非常慢，写入后延时然后再读取。
// 	//读取过快，功率显示闪动很大
// 	unsigned long outputpower_updatetime_gap = millis() - outputpower_updatetime_stamp;
// 	if(outputpower_updatetime_gap > 1000){
// 		result = node.readHoldingRegisters(4, 1);
// 		if (result == node.ku8MBSuccess)
// 		{
// 			output_power = int(node.getResponseBuffer(0)/10);
// 			if(output_power > max_output_power)
// 			{
// 				max_output_power = output_power;
// 			}
// 		}
// 		outputpower_updatetime_stamp = millis();
// 	}

// 	//为了测试，直接截断功率赋值
// 	// output_power = 200;

// 	// Serial.println(micros()-running_time_stamp);
// 	//功率大于100W开始计时
// 	unsigned long cumulative_time_gap = millis() - cumulative_time_stamp;
// 	if(cumulative_time_gap > 1000){
// 		if(output_power > 100)
// 		{
// 			cumulative_time ++;
// 			//在这里读取累计电量和累计时长，累加后写入NVS,实际记录WS，显示的时候转换为WH
// 			// preferences.getBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
// 			nvs_logger.cumulative_Ws = nvs_logger.cumulative_Ws + output_power;
// 			nvs_logger.cumulative_Seconds ++;
// 			preferences.putBytes("nvs-log", &nvs_logger, sizeof(nvs_logger));
// 		}
// 		cumulative_time_stamp = millis();
// 	}
// 	//找到最高转速
// 	if(cadence > max_cadence)
// 	{
// 		max_cadence = cadence;
// 	}
// 	//频率计数长时间不中断的时候，清零频率,从中断里不断更新时间戳
// 	unsigned long cadence_gap = millis() - cadence_time_stamp;
// 	if(cadence_gap > 5000){
// 		cadence = 0;
// 	}
// 	//找到最大心率
// 	if(heart_rate > max_heart_rate)
// 	{
// 		max_heart_rate = heart_rate;
// 	}


// }