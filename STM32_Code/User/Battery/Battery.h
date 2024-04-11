#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32f4xx.h"
 
 
#define RS485_RX_Len	100
 
#define         HighByte(n)   (((n)>>8)&0x00ff)
#define         LowByte(n)    ((n)&0x00ff)
 

typedef struct 
{
	uint16_t MonoOverVol;    //0  单体过压: 单位是 mV，单节过压保护值,单体最高大于该值则发生保护，关闭充电 MOS
	uint16_t OverRelVol;     //1  释放电压: 单位是 mV, 单体过压保护状态解除电压
	uint16_t MonoUnderVol;   //2  单体欠压: 单位是 mV, 单节欠压保护值,最低单体小于于该值则发生保护，关闭放电 MOS
	uint16_t UnderRelVol;    //3  释放电压: 单位是 mV, 单体欠压压保护状态解除电压
	uint16_t GroupOverVol;   //4  整组过压: 单位是 10mV, 整组过压设置值----注1:
	uint16_t GroOverRelVol;  //5  释放电压: 单位是 10mV, 整组欠压释放值
	uint16_t GroupUnderVol;  //6  整组欠压: 单位是 10mV, 整组欠压设置值
	uint16_t GroUnderRelVol; //7  释放电压: 单位是 10mV, 整组欠压释放值
	uint16_t ChargeHighTemp; //8  充电高温: 单位为 0.1k, 充电高温保护设置值----注2：具体计算方式如下：（发送值 – 2731）/10 = 实际温度
	uint16_t ChaHighRelTemp; //9  释放温度: 单位为 0.1k, 充电高温保护释放值
	uint16_t ChargeLowTemp;  //10 充电低温: 单位为 0.1k, 充电低温保护设置值
	uint16_t ChaLowRelTemp;  //11 释放温度: 单位为 0.1k, 充电低温保护释放值
	uint16_t DischaHighTemp; //12 放电高温: 单位为 0.1k, 放电高温保护设置值
	uint16_t DisHighRelTemp; //13 释放温度: 单位为 0.1k, 放电高温保护释放值
	uint16_t DischaLowTemp;  //14 放电低温: 单位为 0.1k, 放电低温保护设置值
	uint16_t DisLowRelTemp;  //15 释放温度: 单位为 0.1k, 放电低温保护释放值
	int16_t ChargeOverCur;  //16 充电过流: 单位 10mA, 充电过流保护值----注3
	int16_t DischaOverCur;  //17 放电过流: 单位 10mA, 放电过流保护值----注3
	uint16_t TotalVol;       //18 总电压: 单位 10mV，整组实际电压
	int16_t ChaDischaCur;   //19 充放电流: 单位 10mA----注3：电流单位采用 10mA，带符号位，充电为正，放电为负
	uint16_t SurplusCapacity;//20 剩余容量: 单位 10mAh----注4：
	uint16_t Battery1Vol;    //21 1号电池电压: 单位 mV
	uint16_t Battery2Vol;    //22 2号电池电压: 单位 mV
	uint16_t Battery3Vol;    //23 3号电池电压: 单位 mV
	uint16_t Battery4Vol;    //24 4号电池电压: 单位 mV
	uint16_t Battery5Vol;    //25 5号电池电压: 单位 mV
	uint16_t Battery6Vol;    //26 6号电池电压: 单位 mV
	uint16_t Battery7Vol;    //27 7号电池电压: 单位 mV
	uint16_t Battery8Vol;    //28 8号电池电压: 单位 mV
	uint16_t Battery9Vol;    //29 9号电池电压: 单位 mV
	uint16_t Battery10Vol;   //30 10号电池电压: 单位 mV
	uint16_t Battery11Vol;   //31 11号电池电压: 单位 mV
	uint16_t Battery12Vol;   //32 12号电池电压: 单位 mV
	uint16_t Battery13Vol;   //33 13号电池电压: 单位 mV
	uint16_t Battery14Vol;   //34 14号电池电压: 单位 mV
	uint16_t Battery15Vol;   //35 15号电池电压: 单位 mV
	uint16_t Battery16Vol;   //36 16号电池电压: 单位 mV
	uint16_t Battery17Vol;   //37 17号电池电压: 单位 mV
	uint16_t Battery18Vol;   //38 18号电池电压: 单位 mV
	uint16_t Battery19Vol;   //39 19号电池电压: 单位 mV
	uint16_t Battery20Vol;   //40 20号电池电压: 单位 mV
	uint16_t BatteryTemp1;   //41 电池温度1: 单位为 0.1k----注2
	uint16_t BatteryTemp2;   //42 电池温度1: 单位为 0.1k
	uint16_t BatteryTemp3;   //43 电池温度1: 单位为 0.1k
	uint16_t BatteryTemp4;   //44 电池温度1: 单位为 0.1k
	uint16_t NominalCapacity;//45 标称容量: 单位 10mAh----注4
	uint16_t RSOC;           //46 RSOC: 高字节预留，低字节表示剩余容量百分比 1 就表示 1%，最大 100。
	
	uint8_t flag; // 0 表示未接收到数据  1表示有数据  
}Battery;


 

void Battery_Init(void);
void Battery_Read_Reg(u8 bDevAddr, uint16_t wRegAddr, uint16_t wRegNum);//读寄存器
void Battery_Recived_Analy(void);
void Battery_Debug(void);
void Battery_Upload_Message(void);
unsigned short Battery_CRC16 (unsigned char *puchMsg,unsigned short usDataLen);
void battery_delay(unsigned int delayvalue);
//uint16_t Convert_16hex_to_10dec(uint16_t hex_num);



/* 高位字节的 CRC 值 */
static const unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40
} ;

/* 低位字节的 CRC 值 */
static const char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB,
0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2,
0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E,
0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27,
0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD,
0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4,
0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94,
0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D,
0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
0x41, 0x81, 0x80,
0x40
};
#endif

