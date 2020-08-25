/* --------0.Project information--------------------
 * I2C communication
 * Author : TRAN MINH THUAN
 * Date: July 23th, 2019
 * Project associate with TM4C123, CCS version 9.
---------------------------------------------------*/


/* --------1.System requirement---------------------
- Setup the TM4C123G as the Master to read data from sensors
---------------------------------------------------*/



////// ----------------2. Pre-processor Directives Section--------------------///////
#include <UserLibraries/Userlibs.h>
#include "UserLibraries/Delay_Systick.h"

#define SCL_PIN             GPIO_PIN_2
#define SDA_PIN             GPIO_PIN_3
#define I2C_PORT            I2C0_BASE
#define I2C_GPIO_PORT       GPIO_PORTB_BASE


#define L3G4200D_ADDR		                                                             0x68
#define WHO_AM_I                                                                         0x0F
#define CTRL_REG1                                                                        0x20
#define CTRL_REG2                                                                        0x21
#define CTRL_REG3                                                                        0x22
#define CTRL_REG4                                                                        0x23
#define CTRL_REG5                                                                        0x24
#define REFERENCE                                                                        0x25
#define OUT_TEMP                                                                         0x26
#define STATUS_REG                                                                       0x27
#define OUT_X_L                                                                          0xA8
#define OUT_X_H                                                                          0x29
#define OUT_Y_L                                                                          0x2A
#define OUT_Y_H                                                                          0x2B
#define OUT_Z_L                                                                          0x2C
#define OUT_Z_H                                                                          0x2D
#define FIFO_CTRL_REG                                                                    0x2E
#define FIFO_SRC_REG                                                                     0x2F
#define INT1_CFG                                                                         0x30
#define INT1_SRC                                                                         0x31
#define INT1_TSH_XH                                                                      0x32
#define INT1_TSH_XL                                                                      0x33
#define INT1_TSH_YH                                                                      0x34
#define INT1_TSH_YL                                                                      0x35
#define INT1_TSH_ZH                                                                      0x36
#define INT1_TSH_ZL                                                                      0x37
#define INT1_DURATION                                                                    0x38

#define OFFSETZ                                                                          21
//////------------------------------------------------------------------------///////

////// ----------------3.Global Declarations Section--------------------------///////
void        I2C_Init(void);
void        I2C_Write_Byte(uint8_t ui8Device_Addr, uint8_t ui8Reg_Addr, uint8_t ui8Data);
uint8_t     I2C_Read_Byte(uint8_t ui8Device_Addr, uint8_t ui8Reg_Addr);
int16_t     L3G4200D_Get_RawValue(void);

void        L3G4200D_Init(void);
float       L3G4200D_Cal_Offset(uint16_t ui16Sample_time);

unsigned long Tick=0;
unsigned long Tick_angle=0;
uint8_t data_recv[10]={0};
float fl_test[100]={0};
float fl_offset=0;
float fl_initangle=0;
uint16_t    test=0;
float fl_gyrorate=0;
float fl_cur_angle=0;
uint8_t ui8_integral=0;

//////------------------------------------------------------------------------///////

////// ----------------4. Subroutines Section---------------------------------///////
void main(void)
    {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
	I2C_Init();
	Systick_Init();
	L3G4200D_Init();
    SysCtlDelay(SysCtlClockGet()/300);
    fl_offset=L3G4200D_Cal_Offset(2000);
    while(1)
    {
        if(ui8_integral==1)  //2 ms
        {
            ui8_integral=0;
            fl_gyrorate= (L3G4200D_Get_RawValue()-fl_offset)*0.00875;
            fl_cur_angle+=fl_gyrorate*0.05;
            Tick_angle=0;
        }
//        for(test=0;test<100;test++)
//        {
//            fl_test[test]=(L3G4200D_Get_RawValue()-fl_offset)*0.00875;
//            delay_us(2000);
//        }
    }
}

void 		I2C_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    //GPIO Configure for I2C pins
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(I2C_GPIO_PORT, SCL_PIN);
    GPIOPinTypeI2C(I2C_GPIO_PORT, SDA_PIN);                         //Configuration SDA pin for I2C communication
    I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), false);         //Use System clock without fast mode (100Kbps) 
}

uint8_t 	I2C_Read_Byte(uint8_t ui8Device_Addr, uint8_t ui8Reg_Addr)
{
    uint8_t ui8_datarecv=0;
    I2CMasterSlaveAddrSet(I2C_PORT, ui8Device_Addr, false);         //Setup slave address and force Master to transmitter mode
    I2CMasterDataPut(I2C_PORT, ui8Reg_Addr);                        //Put data into Master Data Reg
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);         //Pump the data from Master Data Reg to I2C buses
    while(I2CMasterBusy(I2C_PORT));                                 //Wait for transmission complete
    I2CMasterSlaveAddrSet(I2C_PORT, L3G4200D_ADDR, true);           //Setup new communication but this time Master have receiver mode
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_RECEIVE);      //Receive data from I2C buses
    while(I2CMasterBusy(I2C_PORT));                                 //wait for receiving process complete
    ui8_datarecv=I2CMasterDataGet(I2C_PORT);                        //Read the data
    return ui8_datarecv;
}

void    	I2C_Write_Byte(uint8_t ui8Device_Addr, uint8_t ui8Reg_Addr, uint8_t ui8Data)
{
    I2CMasterSlaveAddrSet(I2C_PORT, ui8Device_Addr, false);
    I2CMasterDataPut(I2C_PORT, ui8Reg_Addr);
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C_PORT));
    I2CMasterDataPut(I2C_PORT, ui8Data);
    I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C_PORT));
}

void 		L3G4200D_Init(void)
{
    delay_us(10000);
    I2C_Write_Byte(L3G4200D_ADDR, CTRL_REG1, 0x0C);
    delay_us(1000);
    I2C_Write_Byte(L3G4200D_ADDR, CTRL_REG2, 0x00);
    delay_us(1000);
    I2C_Write_Byte(L3G4200D_ADDR, CTRL_REG3, 0x00);
    delay_us(1000);
    I2C_Write_Byte(L3G4200D_ADDR, CTRL_REG4, 0x08);
    delay_us(1000);
    I2C_Write_Byte(L3G4200D_ADDR, CTRL_REG5, 0x00);
    delay_us(1000);
}

int16_t     L3G4200D_Get_RawValue(void)
{
    uint16_t ui16_realdata=0;
    uint8_t ui8lowbyte=0,ui8highbyte=0;
    ui8lowbyte=I2C_Read_Byte(L3G4200D_ADDR, OUT_Z_L);
    delay_us(1000);
    ui8highbyte=I2C_Read_Byte(L3G4200D_ADDR, OUT_Z_H);
    ui16_realdata=(ui8highbyte<<8)|ui8lowbyte;
    return ui16_realdata;
}

float       L3G4200D_Cal_Offset(uint16_t ui16Sample_time)
{
    float fl_offset;
    uint32_t sumval=0;
    uint16_t ui16_count=0;
    for(ui16_count=0;ui16_count<ui16Sample_time;ui16_count++)
    {
        sumval+=L3G4200D_Get_RawValue();
        delay_us(2000);
    }
    fl_offset=sumval/ui16Sample_time;
    return fl_offset;
}


//////------------------------------------------------------------------------///////
