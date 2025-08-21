#ifndef __ADE7753_H
#define __ADE7753_H
#include "stm32f0xx_hal.h"
// stm32
// Register addresses
//------Name----------------Address----No.Bits(Bytes)
#define ADE7753_WAVEFORM 0x01	// 24 (3)
#define ADE7753_AENERGY 0x02	// 24 (3)
#define ADE7753_RAENERGY 0x03	// 24 (3)
#define ADE7753_LAENERGY 0x04	// 24 (3)
#define ADE7753_VAENERGY 0x05	// 24 (3)
#define ADE7753_RVAENERGY 0x06	// 24 (3)
#define ADE7753_LVAENERGY 0x07	// 24 (3)
#define ADE7753_LVARENERGY 0x08 // 24 (3)
#define ADE7753_MODE 0x09		// 16 (2)
#define ADE7753_IRQEN 0x0A		// 16 (2)
#define ADE7753_STATUS 0x0B		// 16 (2)
#define ADE7753_RSTSTATUS 0x0C	// 16 (2)
#define ADE7753_CH1OS 0x0D		// 8  (1)
#define ADE7753_CH2OS 0x0E		// 8  (1)
#define ADE7753_GAIN 0x0F		// 8  (1)
#define ADE7753_PHCAL 0x10		// 6  (1)
#define ADE7753_APOS 0x11		// 16 (2)
#define ADE7753_WGAIN 0x12		// 12 (2)
#define ADE7753_WDIV 0x13		// 8  (1)
#define ADE7753_CFNUM 0x14		// 12 (2)
#define ADE7753_CFDEN 0x15		// 12 (2)
#define ADE7753_IRMS 0x16		// 24 (3)
#define ADE7753_VRMS 0x17		// 24 (3)
#define ADE7753_IRMSOS 0x18		// 12 (2)
#define ADE7753_VRMSOS 0x19		// 12 (2)
#define ADE7753_VAGAIN 0x1A		// 12 (2)
#define ADE7753_VADIV 0x1B		// 8  (1)
#define ADE7753_LINECYC 0x1C	// 16 (2)
#define ADE7753_ZXTOUT 0x1D		// 12 (2)
#define ADE7753_SAGCYC 0x1E		// 8  (1)
#define ADE7753_SAGLVL 0x1F		// 8  (1)
#define ADE7753_IPKLVL 0x20		// 8  (1)
#define ADE7753_VPKLVL 0x21		// 8  (1)
#define ADE7753_IPEAK 0x22		// 24 (3)
#define ADE7753_RSTIPEAK 0x23	// 24 (3)
#define ADE7753_VPEAK 0x24		// 24 (3)
#define ADE7753_RSTVPEAK 0x25	// 24 (3)
#define ADE7753_TEMP 0x26		// 8  (1)
#define ADE7753_PERIOD 0x27		// 16 (2)
#define ADE7753_TMODE 0x3D		// 8  (1)
#define ADE7753_CHKSUM 0x3E		// 6  (1)
#define ADE7753_DIEREV 0X3F		// 8  (1)

// Mode Register Mask
#define ADE7753_DISHPF 0x0001
#define ADE7753_DISLPF2 0x0002
#define ADE7753_DISCF 0x0004
#define ADE7753_DISSAG 0x0008
#define ADE7753_ASUSPEND 0x0010
#define ADE7753_TEMPSEL 0x0020
#define ADE7753_SWRST 0x0040
#define ADE7753_CYCMODE 0x0080
#define ADE7753_DISCH1 0x0100
#define ADE7753_DISCH2 0x0200
#define ADE7753_SWAP 0x0400
#define ADE7753_DTRT 0x1800
#define ADE7753_WAVSEL 0x6000
#define ADE7753_POAM 0x8000

// Interrupt Status Register Mask
#define ADE7753_AEHF 0x0001
#define ADE7753_SAG 0x0002
#define ADE7753_CYCEND 0x0004
#define ADE7753_WSMP 0x0008
#define ADE7753_ZX 0x0010
#define ADE7753_TEMPC 0x0020
#define ADE7753_RESET 0x0040
#define ADE7753_AEOF 0x0080
#define ADE7753_PKV 0x0100
#define ADE7753_PKI 0x0200
#define ADE7753_VAEHF 0x0400
#define ADE7753_VAEOF 0x0800
#define ADE7753_ZXTO 0x1000
#define ADE7753_PPOS 0x2000
#define ADE7753_PNEG 0x4000

// Channel 1 & 2 Offset Adjust and Gain Mask
#define ADE7753_OS_VAL 0x1F
#define ADE7753_OS_SIGN 0x20
#define ADE7753_ITGR_EN 0x80
#define ADE7753_CH1_GAIN 0x07
#define ADE7753_CH1_FS 0x18
#define ADE7753_CH2_GAIN 0xE0

// Configuration Constants
#define ADE7753_ITGR_OFF 0x00
#define ADE7753_ITGR_ON 0x80
#define ADE7753_GAIN_1 0x00
#define ADE7753_GAIN_2 0x21
#define ADE7753_GAIN_4 0x42
#define ADE7753_GAIN_8 0x63
#define ADE7753_GAIN_16 0x84
#define ADE7753_FS_0_5V 0x00
#define ADE7753_FS_0_25V 0x08
#define ADE7753_FS_0_125V 0x10
#define ADE7753_DR_1_128 0x0000
#define ADE7753_DR_1_256 0x0800
#define ADE7753_DR_1_512 0x1000
#define ADE7753_DR_1_1024 0x1800
#define ADE7753_WAV_PWR 0x0000
#define ADE7753_WAV_CH1 0x4000
#define ADE7753_WAV_CH2 0x6000

#define ADE7753_SPI_FREQ F_CPU / 4
#define ADE7753_TRA_DEL 4
// 0x28 -> 0x3C DON'T CARE
#define TMODE 0x3D
#define CHKSUM 0x3E
#define DIEREV 0x3F
//*** Dia chi cac thong so ghi trong EEPROM
// Thong so khoi tao ADE7758:
#define Flash_add 0x0000
#define IRMSOS_add 0 + Flash_add
#define VRMSOS_add 2 + Flash_add
#define APOS_add 4 + Flash_add
#define WGAIN_add 6 + Flash_add
#define VAGAIN_add 8 + Flash_add
#define WDIV_add 10 + Flash_add
#define VADIV_add 12 + Flash_add
#define CFNUM_add 14 + Flash_add
#define CFDEN_add 16 + Flash_add
// he so quy doi
#define Ku_add 18 + Flash_add
#define Ki_add 20 + Flash_add
#define Kp_add 22 + Flash_add
#define LINECYC_add 24 + Flash_add
#define ThressP_add 26 + Flash_add
#define VARCF_add 28 + Flash_add
#define Tarrif_add 30 + Flash_add
// Thanh ghi Nang luong:
#define Phientai_add 256 + Flash_add
#define P1giao_add 256 + Flash_add
#define P2giao_add 260 + Flash_add
#define P3giao_add 264 + Flash_add
#define P1nhan_add 268 + Flash_add
#define P2nhan_add 272 + Flash_add
#define P3nhan_add 276 + Flash_add
#define Q1_add 280 + Flash_add
#define Q2_add 284 + Flash_add
#define Q3_add 288 + Flash_add
#define Q4_add 292 + Flash_add
#define Pmaxgiao_add 300 + Flash_add
#define Pmaxnhan_add 312 + Flash_add
#define T_max_add 324 + Flash_add
#define WRITE 0x80
#define delay 10
#define ZX 0x10 // Indicates ZX event in STATUS REG
#define TEMPRDY 0x20
#define MODE 0x09 // 16B R/W U
typedef struct
{
	SPI_HandleTypeDef *SPI;
	GPIO_TypeDef *CS_PORT;
	uint16_t CS_PIN;
} ADE_Name;
///////////////////////////////////
void WriteADE(ADE_Name *ADE_7753, uint32_t value, uint8_t address, uint8_t length);
uint32_t ReadADE(ADE_Name *ADE_7753, uint8_t address, uint8_t length);
void ADE_init(ADE_Name *ADE_7753, SPI_HandleTypeDef *SPI_In, GPIO_TypeDef *CS_PORT, uint16_t CS_PIN);
void reset(ADE_Name *ADE_7753);
static uint32_t readRegister(ADE_Name *ADE_7753, uint8_t address, uint8_t length);
static void writeRegister(ADE_Name *ADE_7753, uint8_t address, uint16_t value, uint8_t length);
static void writeMaskRegister(ADE_Name *ADE_7753, uint8_t address, uint16_t value, uint8_t length, uint16_t mask);
static void transfer(ADE_Name *ADE_7753, uint8_t address, uint8_t *data, uint8_t length, uint8_t isRead);
// static void transfer(ADE_Name* ADE_7753,uint8_t address, uint8_t* data, uint8_t length);
uint32_t readRegisters(ADE_Name *ADE_7753, uint8_t address, uint8_t length);
// neww
long getResetInterruptStatus(ADE_Name *ADE_7753);
uint8_t read8bits(ADE_Name *ADE_7753, uint8_t reg);
int8_t read8bits_s(ADE_Name *ADE_7753, uint8_t reg);
uint16_t read16bits(ADE_Name *ADE_7753, uint8_t reg);
int16_t read16bits_s(ADE_Name *ADE_7753, uint8_t reg);
void write16(ADE_Name *ADE_7753, uint8_t reg, uint16_t data);
uint32_t read24bits(ADE_Name *ADE_7753, uint8_t reg);
int32_t read24bits_s(ADE_Name *ADE_7753, uint8_t reg);
uint16_t getPERIOD(ADE_Name *ADE_7753, int N);
int8_t getTEMP(ADE_Name *ADE_7753, int N);
//signed long getTEMP(ADE_Name *ADE_7753, int N);
long getVRMS(ADE_Name *ADE_7753, int N);
long getIRMS(ADE_Name *ADE_7753, int N);
uint32_t getE(ADE_Name *ADE_7753);
uint32_t getRE(ADE_Name *ADE_7753);
void gainSetup(ADE_Name *ADE_7753, uint8_t integrator, uint8_t scale, uint8_t PGA2, uint8_t PGA1);
void Test_ADE7753_Mode_Register(ADE_Name *ADE_7753);
#endif