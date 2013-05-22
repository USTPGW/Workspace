// Unscented Kalman Filter File
// Created by Dan Kassen
// Hand-Converted to C from Matlab file written by previous senior design team

// USART Output Pin: PB10

#include <stm32f4xx.h>
#include <misc.h>			
#include <stm32f4xx_usart.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_flash.h>
#include "arm_math.h" 
#include "math_helper.h" 
#include "stm32f4_discovery.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

uint16_t PrescalerValue = 0;
__IO uint16_t ADC3ConvertedValue = 0;
double ADC3ConvertedVoltage = 0;

// function handles

void ADC3_CH12_Config(void);
void init_GPIO(void); // gpio for debugging
void init_SPI1(void);
uint16_t get_spidata(uint16_t data);
void cubedata(void);
void intcubedata(uint16_t counter);
void measureupdate(void);
void resetcounters(void);
void measureaverage(void);
double twoscomp(uint16_t meas);
void printvalue(uint16_t display);
void printpositions(void);
void gyrocalibration(void);
void PWM_Out(uint16_t pulse, uint16_t roll, uint16_t motor);
void USART3_Configuration(void);
void TIM_Config(void);
void EXTILine1_Config(void);
void EXTI1_IRQHandler(void);

// functions used in filter

double V(double u, double v, double w);
double beta(double u, double v, double w);
double qbar(double u, double v, double w);

double Cx(double u, double w, double i);
double Cy(double u, double v, double w, double p, double r);
double Cz(double u, double w, double i);

double Cl(double u, double v, double w, double p, double r);
double Cm(double u, double w, double q);
double Cn(double u, double v, double w, double p, double r);

double L(double u, double v, double w, double p, double r);
double M(double u, double v, double w, double q);
double N(double u, double v, double w, double p, double r);

double magZ(double phi, double theta, double psi);

double f(double *x, double *out);
double h(double *x, double *out);

//void ukf(double fstate, double *x, double *P, double hmeas, double *z, double *Q, double *R);
void ukf(double *x, double *P, double *z, double *Q, double *R);
void ukf2(double *x, double *P, double *z, double *Q, double *R2);
void sigmas(double *x, double *P, double c, double *X);
void ut1(double *X, double *Wm, double *Wc, double uL, double *Q, double *x1, double *X1, double *P1, double *X2);
void ut2(double *X1, double *Wm, double *Wc, double m, double *R, double *z1, double *Z1, double *P2, double *Z2);
void ut3(double *X1, double *Wm, double *Wc, double m, double *R2, double *z1, double *Z1, double *P2, double *Z2);
void ut4(double *X, double *Wm, double *Wc, double uL, double *Q, double *x1, double *X1, double *P1, double *X2);

//some "basic" matrix functions
void *cholesky(double *A, int n, double *B);
void mult(double *mat1, double *mat2, int row1, int col1, int row2, int col2, double *matout);
void inverse(double *matr, int n, double *inv);
void multt(double *mat1, int row1, int col1, double *matout);

void Delay(__IO uint32_t nCount) {
		while(nCount--) {
  }
}

/* These functions handle SPI transfers*/
void SPI1_send(uint16_t data){

	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
}

uint16_t SPI1_receive(void){
	
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

// USART send function
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		//while( !(USARTx->SR & 0x00000040) ); 
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); // Wait on Transmitter Empty
		USART_SendData(USARTx, *s);
		*s++;
	}
}

// variables for cube
int16_t xaccel;
int16_t yaccel;
int16_t zaccel;
int16_t xgyro;
int16_t ygyro;
int16_t zgyro;
int16_t xmagn;
int16_t ymagn;
int16_t zmagn;
int16_t temper;
int xac;
int yac;
int zac;
int xgc;
int ygc;
int zgc;
int xmc;
int ymc;
int zmc;
int tc;
uint16_t intcounter = 0;
int garbage;
int cubecount=0;
int receivedvalue=0;
double measout=0;
int16_t xgyrobias=0;
int16_t ygyrobias=0;
int16_t zgyrobias=0;
int16_t zmagnbias=0;
int32_t depthbias=0;
int16_t xaccelbias=0;
int16_t yaccelbias=0;

double gravity = 9.81;
double rho = 1.225; // 1026 = density of water
double mass = 3.402;
double buoyancy;
double motor = 0; // change to 1
double S = .182775;
double b = 1.32;
double c = 0.08;
double Ixx = .8742;
double Iyy = .9032;
double Izz = .355;
double Ixz = .056;
double dt = 0.2;	

// Error Covariance Matrix
double Q[576] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0.000001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
};

// Sensor Covariances
double R[100] = {
	0.00125, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0.00125, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0.00125, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0.0157, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0.0157, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0.0157, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 
};

double R2[169] = {
	0.00125, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0.00125, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0.00125, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0.009*9.81, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0.0157, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0.0157, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0.0157, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25
};

double i = 0; // 0.1047 for 6 degrees
uint16_t wingangle=0;
double di = 0;
double h_cg = 0.3;
double cbuoy = 0; // change to buoyancy of vehicle
double netbuoyancy;
double gamma;
double zmagwing;

double z[13] = {0.25, 0, 0, 0, 0, 9.81, 0, 0, 0, 14.223, 0, 0, 0}; //sensor observation matrix

double x[24] = {0}; // state matrix

int16_t xx[24] = {0}; // integer state matrix for storing

	double P[576] = {
0.000812390733,0.000059962809,-0.000129297918,0.000039929235,-0.000435555163,-0.000044676017,0.000000235908,0.000019663110,-0.000089381041,0.034007556666,0.015412594711,0.000005734016,-0.000231423799,0.000140121722,0.000003521545,-0.000000406178,0.000060060237,-0.002269507487,-0.000383640120,0.000145070451,-0.000000047549,-0.000010829536,-0.000012253503,-0.000000909149,
0.000059962809,0.008300541230,0.000002078858,0.000436308256,0.000052197414,-0.007001365821,-0.000026236655,-0.000000344734,-0.012559707362,0.000387430298,-0.009306138683,-0.000010114557,-0.000001302998,0.000000132859,0.000000042158,0.000000004587,0.000000981186,-0.000019068576,-0.000123835860,0.000001407998,0.000000040927,-0.000000096808,0.000020649603,0.000000480892,
-0.000129297918,0.000002078858,0.000020664004,-0.000004070696,0.000063556073,-0.000002565734,0.000000161063,-0.000003834319,-0.000003320803,-0.005430124221,-0.002472397554,-0.000000817093,0.000036937704,-0.000022367381,-0.000000562051,0.000000064889,-0.000009583730,0.000362209499,0.000060793494,-0.000023152758,0.000000007489,0.000001728702,0.000001975673,0.000000145871,
0.000039929235,0.000436308256,-0.000004070696,0.003150257263,-0.000010871135,-0.000167669909,0.000056760707,0.000000610523,-0.000500373574,0.002539693763,0.001533591192,-0.000001059834,-0.000013633504,0.000006104198,0.000000224322,0.000000029638,0.000005776129,-0.000159617529,-0.000398138694,0.000010735177,-0.000000115448,-0.000000763969,-0.000012234277,0.000000070632,
-0.000435555163,0.000052197414,0.000063556073,-0.000010871135,0.003070436383,-0.000049791514,-0.000000372653,-0.000007637969,-0.000084078463,-0.012981924057,-0.006416212681,-0.000049657568,0.000094943231,-0.000057466700,-0.000001438754,0.000000162708,-0.000024657646,0.000931344854,0.000159860006,-0.000059394785,0.000000020397,0.000004403549,0.000005367740,0.000000376217,
-0.000044676017,-0.007001365821,-0.000002565734,-0.000167669909,-0.000049791514,0.017821795741,0.000023524751,0.000000367072,0.010876392659,-0.000827362839,0.014940549847,0.000008689510,-0.000012775090,-0.000046139689,-0.000005245544,0.000037002081,0.000000950907,-0.000022058930,-0.000034450034,0.000001534387,-0.000000069892,-0.000000100077,-0.000023198686,-0.000000585177,
0.000000235908,-0.000026236655,0.000000161063,0.000056760707,-0.000000372653,0.000023524751,0.000685048405,0.000000071036,0.000052114763,0.000017000143,0.000037006290,0.000000030427,-0.000000118561,0.000000072674,0.000000001738,-0.000000000205,0.000000029733,-0.000001151620,0.000000005635,0.000000071785,-0.000000128705,-0.000000005487,-0.000000078861,-0.000000002654,
0.000019663110,-0.000000344734,-0.000003834319,0.000000610523,-0.000007637969,0.000000367072,0.000000071036,0.000142734097,0.000000550203,0.000825983493,0.000376022723,0.000000106471,-0.000005617788,0.000003401751,0.000000085478,-0.000000009870,0.000001457449,-0.000055084280,-0.000009238170,0.000003521058,-0.000000001163,-0.000000264460,-0.000000300293,-0.000000022233,
-0.000089381041,-0.012559707362,-0.000003320803,-0.000500373574,-0.000084078463,0.010876392659,0.000052114763,0.000000550203,0.020347610364,-0.000390363556,0.014251132222,0.000015332578,0.000001019029,0.000000153987,-0.000000047740,-0.000000002980,-0.000001005327,0.000016791873,0.000147213281,-0.000001297510,-0.000000081491,0.000000088784,-0.000033137100,-0.000000730648,
0.034007556666,0.000387430298,-0.005430124221,0.002539693763,-0.012981924057,-0.000827362839,0.000017000143,0.000825983493,-0.000390363556,12146.761674612500,-0.138733025487,-0.008447720097,20.243104695375,-0.001724027752,0.000360412062,-0.000371466124,0.003463732389,-0.130324928287,-0.025106458951,0.008179719900,0.000005776200,-0.000115993264,-0.000837667452,0.012584140156,
0.015412594711,-0.009306138683,-0.002472397554,0.001533591192,-0.006416212681,0.014940549847,0.000037006290,0.000376022723,0.014251132222,-0.138733025487,12141.444117451600,0.002234428910,-0.016709011046,20.222396590037,0.000131220240,0.000190099929,0.001692388834,-0.063932281875,-0.017425051366,0.004171418202,-0.000004399763,-0.000436459795,-0.000594575728,-0.003179980474,
0.000005734016,-0.000010114557,-0.000000817093,-0.000001059834,-0.000049657568,0.000008689510,0.000000030427,0.000000106471,0.000015332578,-0.008447720097,0.002234428910,16.591929367737,-0.000086291723,0.000021915078,-0.000000087730,0.000000037374,0.000000302741,-0.000011388881,-0.000000929649,0.000000683455,-0.000000096210,-0.000000577174,-0.000000398733,-16.545969995805,
-0.000231423799,-0.000001302998,0.000036937704,-0.000013633504,0.000094943231,-0.000012775090,-0.000000118561,-0.000005617788,0.000001019029,20.243104695375,-0.016709011046,-0.000086291723,0.200695653181,-0.000155782344,0.000000079001,-0.000003235935,-0.000025167213,0.000955880720,0.000135311046,-0.000062577031,0.000000105052,0.000009662793,0.000004158196,0.000125525437,
0.000140121722,0.000000132859,-0.000022367381,0.000006104198,-0.000057466700,-0.000046139689,0.000000072674,0.000003401751,0.000000153987,-0.001724027752,20.222396590037,0.000021915078,-0.000155782344,0.200279863676,0.000003575430,-0.000014694502,0.000014802686,-0.000578997833,-0.000060876812,0.000037626523,-0.000000032274,-0.000004063286,-0.000001826936,-0.000031412769,
0.000003521545,0.000000042158,-0.000000562051,0.000000224322,-0.000001438754,-0.000005245544,0.000000001738,0.000000085478,-0.000000047740,0.000360412062,0.000131220240,-0.000000087730,0.000000079001,0.000003575430,0.000002852040,-0.000001646473,0.000000352701,-0.000014137235,-0.000002221000,0.000000929756,-0.000000000336,-0.000000067592,-0.000000071638,0.000000148889,
-0.000000406178,0.000000004587,0.000000064889,0.000000029638,0.000000162708,0.000037002081,-0.000000000205,-0.000000009870,-0.000000002980,-0.000371466124,0.000190099929,0.000000037374,-0.000003235935,-0.000014694502,-0.000001646473,0.000011474981,0.000000028024,0.000001558806,-0.000000288538,-0.000000087195,0.000000000041,0.000000007608,-0.000000007028,-0.000000057177,
0.000060060237,0.000000981186,-0.000009583730,0.000005776129,-0.000024657646,0.000000950907,0.000000029733,0.000001457449,-0.000001005327,0.003463732389,0.001692388834,0.000000302741,-0.000025167213,0.000014802686,0.000000352701,0.000000028024,0.000008612576,-0.000243630678,-0.000057012294,0.000015525368,-0.000000007234,-0.000001169650,-0.000001875425,-0.000000102741,
-0.002269507487,-0.000019068576,0.000362209499,-0.000159617529,0.000931344854,-0.000022058930,-0.000001151620,-0.000055084280,0.000016791873,-0.130324928287,-0.063932281875,-0.000011388881,0.000955880720,-0.000578997833,-0.000014137235,0.000001558806,-0.000243630678,0.009383559721,0.001580577844,-0.000588794557,0.000000196885,0.000044326028,0.000051395507,0.000003773507,
-0.000383640120,-0.000123835860,0.000060793494,-0.000398138694,0.000159860006,-0.000034450034,0.000000005635,-0.000009238170,0.000147213281,-0.025106458951,-0.017425051366,-0.000000929649,0.000135311046,-0.000060876812,-0.000002221000,-0.000000288538,-0.000057012294,0.001580577844,0.003898560286,-0.000108697664,0.000000485276,0.000007564475,0.000124775549,-0.000000597458,
0.000145070451,0.000001407998,-0.000023152758,0.000010735177,-0.000059394785,0.000001534387,0.000000071785,0.000003521058,-0.000001297510,0.008179719900,0.004171418202,0.000000683455,-0.000062577031,0.000037626523,0.000000929756,-0.000000087195,0.000015525368,-0.000588794557,-0.000108697664,0.000185822604,-0.000000013581,-0.000002821285,-0.000003526649,-0.000000180176,
-0.000000047549,0.000000040927,0.000000007489,-0.000000115448,0.000000020397,-0.000000069892,-0.000000128705,-0.000000001163,-0.000000081491,0.000005776200,-0.000004399763,-0.000000096210,0.000000105052,-0.000000032274,-0.000000000336,0.000000000041,-0.000000007234,0.000000196885,0.000000485276,-0.000000013581,0.000026386312,0.000000001836,0.000000024271,0.000000141100,
-0.000010829536,-0.000000096808,0.000001728702,-0.000000763969,0.000004403549,-0.000000100077,-0.000000005487,-0.000000264460,0.000000088784,-0.000115993264,-0.000436459795,-0.000000577174,0.000009662793,-0.000004063286,-0.000000067592,0.000000007608,-0.000001169650,0.000044326028,0.000007564475,-0.000002821285,0.000000001836,0.000026451911,0.000000242979,0.000000787757,
-0.000012253503,0.000020649603,0.000001975673,-0.000012234277,0.000005367740,-0.000023198686,-0.000000078861,-0.000000300293,-0.000033137100,-0.000837667452,-0.000594575728,-0.000000398733,0.000004158196,-0.000001826936,-0.000000071638,-0.000000007028,-0.000001875425,0.000051395507,0.000124775549,-0.000003526649,0.000000024271,0.000000242979,0.000031438934,0.000000479035,
-0.000000909149,0.000000480892,0.000000145871,0.000000070632,0.000000376217,-0.000000585177,-0.000000002654,-0.000000022233,-0.000000730648,0.012584140156,-0.003179980474,-16.545969995805,0.000125525437,-0.000031412769,0.000000148889,-0.000000057177,-0.000000102741,0.000003773507,-0.000000597458,-0.000000180176,0.000000141100,0.000000787757,0.000000479035,24.322575887777
}; // covariance matrix

// matrix space defines



// space saving declares for functions
double X[1176] = {0}; // 24 x 49 // output of sigmas
double LL[576] = {0}; // used in cholesky
double A[576] = {0}; // used in sigmas
double Y[576] = {0}; // used in sigmas

double ztemp[13]; // 13 x 1
double x1[24]; // 24 x 1
double X1[1176]; // 24 x 49
double P1[576]; // 24 x 24
double X2[1176]; // 24 x 49
double z1[13]; // 13 x 1
double Z1[637]; // 13 x 49
double P2[169]; // 13 x 13
double P2inv[169]; // 13 x 13
double Z2[637]; // 13 x 49
double Z2T[637]; // 49 x 13
double P12[312]; // 24 x 13
double P12T[312]; // 13 x 24
double temp[1176]; // 24 x 49
double K[312]; // 24 x 13
double PT[576]; // 24 x 24
double fout[24] = {0}; // 24 x 1
double XF[24] = {0}; // 24 x 1
double XH[10] = {0};
double XH2[13] = {0};
double X2T[1176] = {0}; // 49 x 24
double hout[10] = {0}; // 10 x 1
double hout2[13] = {0}; // 13 x 1
double d[1] = {0};
double test[169] = {0}; // 13 x 13
double Wc[49] = {0};	
double Wm[49] = {0};
double invmat[169] = {0}; // 13 x 13
//double diag[2401] = {0}; // 49 x 49
double xtest[24] = {0};
	
double s = 0;
double uL = 24; // # of states
double m = 10; // # of measurements
double ualpha = .001; // default, tunable
double ki=0; // default, tunable
double ubeta=1; // default, tunable
double lambda = -24.00;
double uc = .000024;
double utL = 49;
double sum = 0;

int main(void) {
	int count;
	int printcount=0; // used to obtain data every so often
		
	init_GPIO(); // initializes general gpio, such as LEDs for debugging
	init_SPI1(); // enables spi
	USART3_Configuration(); // initialize usart for serial communcation
	ADC3_CH12_Config(); // initializes ADC3
	TIM_Config(); // initialize motor pwm timers
	
	ADC_SoftwareStartConv(ADC3); // convert pin value
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET); // wait for conversion to be complete 
	ADC3ConvertedValue = ADC_GetConversionValue(ADC3); // get analog value
	
	//define Wm, Wc
	Wm[0] = (-24/.000024)+1;
	Wc[0] = (-24/.000024)+3;
	for(count=1;count<49;count++){
		Wm[count] = (0.5/.000024);
		Wc[count] = (0.5/.000024);
	}
	
	x[0] = .49; // initial states have only one non-zero value
	
	netbuoyancy = mass*gravity + cbuoy; // netbuoyancy calculation
	
	gyrocalibration(); // takes about 1 second, calibrates gyros and magnetometer
	
	/* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
  EXTILine1_Config();

  /* Generate software interrupt: simulate a rising edge applied on EXTI1 line */
  EXTI_GenerateSWInterrupt(EXTI_Line1);
	
	ukf2(x, P, z, Q, R2); // execute initial filter, telling it that it is at origin
	
	gamma = (x[2]+x[1]*x[5]-x[0]*x[4])/sqrt(x[0]*x[0]*(x[4]*x[4]+1)+x[1]*x[1]*(1+x[4]*x[3]+x[3]*x[3])+x[2]*x[2]*(x[4]+x[3]+1)+x[2]*x[3]*x[4]*x[5]);
	wingangle = 1500 + 573*(.1047-gamma); // calculate wing angle value to output
		
	EXTI_DeInit(); // disable interrupts
		
	PWM_Out(wingangle,425,1); // output 
		
	count = 0;
		
	measureaverage(); // determine average of all measurements taken
	measureupdate(); // update z
	resetcounters(); // reset counters used in measurements
	
	EXTILine1_Config();
	EXTI_GenerateSWInterrupt(EXTI_Line1); // re-enable interrupts
	
	while(1){
		if(GPIOA->IDR & 0x0001){
			ukf2(x, P, z, Q, R2); // execute filter with "gps", updates P
			Delay(0x000FFFF); // debounce button
		}
		else{
			GPIO_SetBits(GPIOD, GPIO_Pin_12); // turns on led, used in timing testing
			ukf(x, P, z, Q, R); // execute filter
			GPIO_ResetBits(GPIOD, GPIO_Pin_12); // turns off led
			Delay(0x000FFFF); // debounce button
		}
		// gamma calculation for wing output: zdot/sqrt(xdot^2 + ydot^2 + zdot^2)
		gamma = (x[2]+x[1]*x[5]-x[0]*x[4])/sqrt(x[0]*x[0]*(x[4]*x[4]+1)+x[1]*x[1]*(1+x[4]*x[3]+x[3]*x[3])+x[2]*x[2]*(x[4]+x[3]+1)+x[2]*x[3]*x[4]*x[5]);
		wingangle = 1500 + 573*(.1047-gamma); // calculate wing angle value to output
		
		EXTI_DeInit(); // disable interrupts
		
		//PWM_Out(wingangle,425,1); // output using gamma (for actual filter)
		
		
		count = 0;
		
		measureaverage(); // determine average of all measurements taken
		measureupdate(); // update z
		resetcounters(); // reset counters used in measurements
		
		printcount++;
		if(printcount==10){
		printpositions(); // print positions to putty
		printcount=0;}
		
		zmagwing = 1500 + 2400*z[2]; // wing angle value for display
		PWM_Out(zmagwing,425,1); // output for display
		
		EXTILine1_Config();
		EXTI_GenerateSWInterrupt(EXTI_Line1); // re-enable interrupts
		}
		
}

/*double V(double u, double v, double w)
{return sqrt(u*u + v*v + w*w);}

double beta(double u, double v, double w)
{return v/V(u,v,w);}
	
double qbar(double u, double v, double w)
{return (rho*V(u,v,w)*V(u,v,w))/2;}*/
	
double Cx(double u, double w, double i)
{return 4.176*((w/u)+0.875*i)*(w/u) - (0.010652+0.0393*(4.176*((w/u)+0.875*i))*(4.176*((w/u)+0.875*i)));}

double Cy(double u, double v, double w, double p, double r)
{return 0.39569*(v/sqrt(u*u + v*v + w*w)) - 0.021877*p + 0.46644*r;}

double Cz(double u, double w, double i)
{return -4.176*((w/u)+0.875*i) - (0.010652+0.0393*(4.176*((w/u)+0.875*i)*(4.176*((w/u)+0.875*i))))*(w/u);}

double Cl(double u, double v, double w, double p, double r)
{return -0.52494*p + 0.00014753*(v/sqrt(u*u + v*v + w*w)) + 0.0013735*r + 64.7*(-di);}
	
double Cm(double u, double w, double q)
{return ((-2.21 + 3.06*i + 4*(h_cg-0.25))*((w/u)+0.875*i) + 0.9*i - 18.305*q);}
	
double Cn(double u, double v, double w, double p, double r)
{return -.22218*(v/sqrt(u*u + v*v + w*w)) + 0.012151*p - 0.26078*r;}
	
/*double L(double u, double v, double w, double p, double r)
{return 0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cl(u,v,w,p,r);}

double M(double u, double v, double w, double q)
{return 0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*c*Cm(u,w,q);}

double N(double u, double v, double w, double p, double r)
{return 0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cn(u,v,w,p,r);}*/

// function to handle magnetometer reading
double magZ(double phi, double theta, double psi)
{
	double a;
	double mb;
	a = .31*theta*cos(psi);
	mb = .31*phi*sin(psi);
	if((a+mb)>=0)
		{return sqrt(a*a+mb*mb);}
	else
		{return -sqrt(a*a+mb*mb);}
}

double f(double *x, double *out) // state transition equation, updated to reflect 0 indexing, may need to update method of outputing
{
	//double f[24] = {0};
/*u-dot*/			//out[0] = x[0] + dt*((0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*Cx(x[0],x[2],i)+motor)/mass - (gravity+(netbuoyancy/mass))*x[4]+x[8]*x[1] - x[7]*x[2]);
							// change 0 to -0.011 when using in vehicle, change motor to motor power
/*u-dot*/			out[0] = x[0] + dt*((0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*(0)+motor)/mass - (gravity-(netbuoyancy/mass))*x[4] + x[8]*x[1] - x[7]*x[2]);
/*v-dot*/			out[1] = x[1] + dt*(0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*Cy(x[0],x[1],x[2],x[6],x[8])/mass + (gravity - (netbuoyancy/mass))*x[3] - x[8]*x[0] - x[2]*x[6]);
/*w-dot*/			//out[2] = x[2] + dt*(0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*Cz(x[0],x[2],i)/mass + gravity - (netbuoyancy/mass) + x[7]*x[0] - x[6]*x[1]);
							// change 0 to -1 when using in vehicle
/*w-dot*/			out[2] = x[2] + dt*(0/mass + (gravity - (netbuoyancy/mass))*cos(x[3])*cos(x[4]) - x[7]*x[0] - x[6]*x[1]);
/*phi-dot*/		out[3] = x[3] + dt*(x[6] + (x[7]*x[3] + x[8])*x[4]);
/*theta-dot*/	out[4] = x[4] + dt*(x[7] - x[8]*x[3]); // - might need to be a +
/*psi-dot*/		out[5] = x[5] + dt*(x[7]*x[3] + x[8]);
/*p-dot*/			out[6] = x[6] + dt*((Ixx*0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cl(x[0],x[1],x[2],x[6],x[8]) + Ixz*0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cn(x[0],x[1],x[2],x[6],x[8]) - (Ixz*(Iyy-Ixx-Izz)*x[6] + (Ixx*Ixx + Izz*(Izz-Iyy))*x[8])*x[7])/(Ixx*Izz  -Ixz*Ixz));
/*q-dot*/			out[7] = x[7] + dt*((0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*c*Cm(x[0],x[2],x[7]) - (Ixx - Izz)*x[6]*x[8] - Ixz*((x[6]*x[6])-(x[8]*x[8])))/Iyy);
/*r-dot*/			out[8] = x[8] + dt*((Ixz*0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cl(x[0],x[1],x[2],x[6],x[8]) + Ixx*0.5*rho*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])*S*b*Cn(x[0],x[1],x[2],x[6],x[8]) - (Ixz*(Iyy-Ixx-Izz)*x[8] + (Ixz*Ixz + Ixx*(Ixx-Iyy))*x[6])*x[7])/(Ixx*Izz - Ixz*Ixz));
/*x-dot*/			out[9] = x[9] + dt*(x[0]*cos(x[5]) + (-sin(x[5]) + x[3]*x[4]*cos(x[5]))*x[1] + (x[3]*sin(x[5]) + x[4]*cos(x[5]))*x[2]+x[12]);
/*y-dot*/			out[10] = x[10] + dt*(sin(x[5])*x[0] + (-cos(x[5]) + x[3]*x[4]*sin(x[5]))*x[1] + (x[3]*cos(x[5]) + x[4]*sin(x[5]))*x[2] + x[13]);
/*z-dot*/			out[11] = x[11] + dt*(-x[4]*x[0] + x[3]*x[1] + x[2]);
							out[12] = x[12]; out[13] = x[13]; out[14] = x[14]; out[15] = x[15]; out[16] = x[16]; out[17] = x[17]; // biases
							out[18] = x[18]; out[19] = x[19]; out[20] = x[20]; out[21] = x[21]; out[22] = x[22]; out[23] = x[23];  // biases
	//return f[24];
}

double h(double *x, double *out) 
{ // observation update equation
	out[0] = .31*cos(x[5]) + x[14]; // magnetometer x
	out[1] = .31*-sin(x[5]) + x[15]; // magnetometer y
	out[2] = magZ(x[3],x[4],x[5]) + x[16]; // magnetometer z
	out[3] = -gravity*x[4] + x[17]; // accelerometer x
	out[4] = gravity*x[3]+x[18]; // accelerometer y
	out[5] = gravity + x[19]; // accelerometer z
	out[6] = x[6] + x[20]; // gyro p
	out[7] = x[7] + x[21]; // gyro q
	out[8] = x[8] + x[22]; // gyro r
	out[9] = x[11]*1.47 + 14.223 + x[23]; // pressure in PSI (depth)
}

double h2(double *x, double *out)
{ // observation update equation
	out[0] = .31*cos(x[5]) + x[14]; // magnetometer x
	out[1] = .31*-sin(x[5]) + x[15]; // magnetometer y
	out[2] = magZ(x[3],x[4],x[5]) + x[16]; // magnetometer z
	out[3] = -gravity*x[4] + x[17]; // accelerometer x
	out[4] = gravity*x[3]+x[18]; // accelerometer y
	out[5] = gravity + x[19]; // accelerometer z
	out[6] = x[6] + x[20]; // gyro p
	out[7] = x[7] + x[21]; // gyro q
	out[8] = x[8] + x[22]; // gyro r
	out[9] = x[11]*1.47 + 14.223 + x[23]; // pressure in PSI (depth)
	out[10] = x[9]; // x position
	out[11] = x[10]; // y position
	out[12] = x[11]; // z position
}

void ukf(double *x, double *P, double *z, double *Q, double *R)
{
	int count;
	int countrow;
	int countcol;
	
	/* scaling factors*/
	lambda = ualpha*ualpha*(uL+ki)-uL;
	uc = uL+lambda;
	uc = sqrt(uc);
	
	sigmas(x,P,uc,X);	// in: x, P, uc ;; out: X, computes sigmas for UKF
	
	ut1(X, Wm, Wc, uL, Q, x1, X1, P1, X2); // in: X, Wm, Wc, uL, Q ;; out: x1, X1, P1, X2, first unscented transform
	
	ut2(X1, Wm, Wc, m, R, z1, Z1, P2, Z2); // in: X1, Wm, Wc, uL, R ;; out: z1, Z1, P2, Z2, second unscented transform
	
	for(countrow=0;countrow<24;countrow++){
		X2[countrow*49] = X2[countrow*49]*Wc[0];
		for(countcol=1;countcol<49;countcol++){
			X2[countrow*49 + countcol] = X2[countrow*49 + countcol]*Wc[countcol];
		}
	}
			
	for(countrow=0;countrow<10;countrow++){
		for(countcol=0;countcol<49;countcol++){
			Z2T[countcol*10+countrow] = Z2[countrow*49+countcol]; // Z2' (tranpose), 49x10
		}
	}
	
	mult(X2,Z2T,24,49,49,10,P12); // P12 = X2*diag(Wc)*Z2';

	inverse(P2, 10, P2inv); // obtain inverse of P2
	
	mult(P12,P2inv,24,10,10,10,K);
	
	for(count=0;count<10;count++){
		ztemp[count]=z[count]-z1[count]; // z - z1
	}
	
	mult(K,ztemp,24,10,10,1,xtest); // K*(z-z1)
	
	for(count=0;count<24;count++){
		x[count]=xtest[count]+x1[count]; // x = x1+K*(z-z1)
	}
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=0;countcol<10;countcol++){
			P12T[countcol*24+countrow] = P12[countrow*10+countcol]; // P12' (tranpose), 10x24
		}
	}
	
	mult(K,P12T,24,10,10,24,P); // K*P12'
	
	for(count=0;count<576;count++){
		P[count] = P1[count] - P[count]; // P = P1 - K*P12'
	}
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=0;countcol<24;countcol++){
			PT[countcol*24+countrow] = P[countrow*24+countcol]; // P' (tranpose), 24x24
		}
	}
	
	for(count=0;count<576;count++){
		P[count] = (PT[count] + P[count])/2; // P = (P+P')/2
	}
}

void ukf2(double *x, double *P, double *z, double *Q, double *R2)
{
	int count;
	int countrow;
	int countcol;
	
	lambda = ualpha*ualpha*(uL+ki)-uL;
	uc = uL+lambda;
	uc = sqrt(uc);
	m = 13;
	
	sigmas(x,P,uc,X);	// in: x, P, uc ;; out: X
	
	ut1(X, Wm, Wc, uL, Q, x1, X1, P1, X2); // in: X, Wm, Wc, uL, Q ;; out: x1, X1, P1, X2
	
	ut3(X1, Wm, Wc, m, R2, z1, Z1, P2, Z2); // in: X1, Wm, Wc, uL, R ;; out: z1, Z1, P2, Z2
	
	for(countrow=0;countrow<24;countrow++){
		X2[countrow*49] = X2[countrow*49]*Wc[0];
		for(countcol=1;countcol<49;countcol++){
			X2[countrow*49 + countcol] = X2[countrow*49 + countcol]*Wc[countcol];
		}
	}
			
	for(countrow=0;countrow<13;countrow++){
		for(countcol=0;countcol<49;countcol++){
			Z2T[countcol*13+countrow] = Z2[countrow*49+countcol]; // Z2' (tranpose), 49x13
		}
	}
	
	mult(X2,Z2T,24,49,49,13,P12); // P12 = X2*diag(Wc)*Z2';

	inverse(P2, 13, P2inv); // obtain inverse of P2
	
	mult(P12,P2inv,24,13,13,13,K);
	
	for(count=0;count<13;count++){
		ztemp[count]=z[count]-z1[count]; // z - z1
	}
	
	mult(K,ztemp,24,13,13,1,x); // K*(z-z1)
	
	for(count=0;count<24;count++){
		x[count]=x[count]+x1[count]; // x = x1+K*(z-z1)
	}
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=0;countcol<13;countcol++){
			P12T[countcol*24+countrow] = P12[countrow*13+countcol]; // P12' (tranpose), 13x24
		}
	}
	
	mult(K,P12T,24,13,13,24,P); // K*P12'
	
	for(count=0;count<576;count++){
		P[count] = P1[count] - P[count]; // P = P1 - K*P12'
	}
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=0;countcol<24;countcol++){
			PT[countcol*24+countrow] = P[countrow*24+countcol]; // P' (tranpose), 24x24
		}
	}
	
	for(count=0;count<576;count++){
		P[count] = (PT[count] + P[count])/2; // P = (P+P')/2
	}
	
	m = 10;
}

void ut1(double *X, double *Wm, double *Wc, double uL, double *Q, double *x1, double *X1, double *P1, double *X2)
{
	int countrow;
	int countcol;
	int count;

	for(countcol=0;countcol<49;countcol++){
		for(count=0;count<24;count++){
			XF[count] = X[count*49+countcol];
		}
		f(XF,fout); // updated states for X1
		for(countrow=0;countrow<24;countrow++){
			X1[countrow*49 + countcol] = fout[countrow];	// X1(:,l) = f(X(:,k));
			//X1[(countrow+12)*49 + countcol] = X[(countrow+12)*49+countcol];	// X1(:,l) = f(X(:,k));
		}
	}
	
	for(count=0;count<24;count++){
		x1[count] = 0; // reset x1 to zeroes
	}
	
	for(countcol=0;countcol<49;countcol++){
		for(countrow=0;countrow<24;countrow++){ // x1 = x1+Wm(k)*Y(:,k)
			x1[countrow] = x1[countrow] + Wm[countcol]*X1[countrow*49+countcol];
	}
}

	for(countrow=0;countrow<24;countrow++){ // X2 = X1-x1(:,ones(1,utL));
		for(countcol=0;countcol<49;countcol++){
			X2[countrow*49+countcol] = X1[countrow*49+countcol] - x1[countrow];
		}
	}
	
	multt(X2,24,49,P1); // X2*diag(Wc)*X2'
	
	//mult(temp,X2T,24,49,49,24,P1);
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=0;countcol<24;countcol++){
			P1[countrow*24+countcol] = P1[countrow*24+countcol] + Q[countrow*24+countcol]; //P1 = X2*diag(Wc)*X2'+Q;
		}
	}
}

void ut2(double *X1, double *Wm, double *Wc, double m, double *R, double *z1, double *Z1, double *P2, double *Z2)
{
	int countrow;
	int countcol;
	int count;
	
	for(countcol=0;countcol<49;countcol++){
		for(count=0;count<24;count++){
			XH[count] = X1[count*49+countcol];
		}
		h(XH,hout); // updated states for X1
		for(countrow=0;countrow<10;countrow++){
			Z1[countrow*49 + countcol] = hout[countrow]; // Z1(:,k) = h(X(:,k));
		}
	}
	
	for(count=0;count<10;count++){
		z1[count] = 0; // reset z1 to zeroes
	}
	
	for(countcol=0;countcol<49;countcol++){
		for(countrow=0;countrow<10;countrow++){ // z1 = z1+Wm(k)*Z1(:,k)
			z1[countrow] = z1[countrow] + Wm[countcol]*Z1[countrow*49+countcol];
	}
}
	
	for(countrow=0;countrow<10;countrow++){ // Z2 = Z1-z1(:,ones(1,utL));
		for(countcol=0;countcol<49;countcol++){
			Z2[countrow*49+countcol] = Z1[countrow*49+countcol] - z1[countrow];
		}
	}
	
	multt(Z2,10,49,P2);
	
	//mult(temp,Z2T,10,49,49,10,P2);
	
	for(countrow=0;countrow<10;countrow++){
		for(countcol=0;countcol<10;countcol++){
			P2[countrow*10+countcol] = P2[countrow*10+countcol] + R[countrow*10+countcol]; //P2 = Z2*diag(Wc)*Z2'+R;
		}
	}
	
}

void ut3(double *X1, double *Wm, double *Wc, double m, double *R2, double *z1, double *Z1, double *P2, double *Z2)
{
	int countrow;
	int countcol;
	int count;
	
	for(countcol=0;countcol<49;countcol++){
		for(count=0;count<24;count++){
			XH2[count] = X1[count*49+countcol];
		}
		h2(XH2,hout2); // updated states for X1
		for(countrow=0;countrow<13;countrow++){
			Z1[countrow*49 + countcol] = hout2[countrow]; // Z1(:,k) = h(X(:,k));
		}
	}
	
	for(count=0;count<13;count++){
		z1[count] = 0; // reset z1 to zeroes
	}
	
	for(countcol=0;countcol<49;countcol++){
		for(countrow=0;countrow<13;countrow++){ // z1 = z1+Wm(k)*Z1(:,k)
			z1[countrow] = z1[countrow] + Wm[countcol]*Z1[countrow*49+countcol];
	}
}
	
	for(countrow=0;countrow<13;countrow++){ // Z2 = Z1-z1(:,ones(1,utL));
		for(countcol=0;countcol<49;countcol++){
			Z2[countrow*49+countcol] = Z1[countrow*49+countcol] - z1[countrow];
		}
	}
	
	multt(Z2,13,49,P2); // 13 x 49 * 49 x 13
	
	//mult(temp,Z2T,10,49,49,10,P2);
	
	for(countrow=0;countrow<13;countrow++){
		for(countcol=0;countcol<13;countcol++){
			P2[countrow*13+countcol] = P2[countrow*13+countcol] + R2[countrow*13+countcol]; //P2 = Z2*diag(Wc)*Z2'+R;
		}
	}
	
}

void sigmas(double *x, double *P, double c, double *X)
{
	int count;
	int countrow;
	int countcol;
	
	cholesky(P,24,A);
	
	for(count=0;count<576;count++)
	{A[count] = c*A[count];} // now A = c*chol(P)'
	
	for(countrow=0;countrow<24;countrow++){
		X[countrow*49] = x[countrow];
		for(countcol=0;countcol<24;countcol++){
			Y[countrow*24+countcol] = x[countrow];
		}
	}
	
	/*for(count=0;count<24;count++)
	{X[count*49] = x[count];}*/
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=1;countcol<25;countcol++){
			X[countrow*49+countcol] = (Y[(countrow*24)+countcol-1] + A[(countrow*24)+countcol-1]);
		}
	}
	
	for(countrow=0;countrow<24;countrow++){
		for(countcol=25;countcol<49;countcol++){
			X[countrow*49+countcol] = (Y[(countrow*24)+countcol-25] - A[(countrow*24)+countcol-25]);
		}
	}
}

void *cholesky(double *A, int n, double *B)
{
	int i;
	int j;
	int k;
	
  //double *L = (double*)calloc(n * n, sizeof(double));
 
    for (i=0; i<n; i++){
        for (j=0; j<(i+1); j++){
            s=0;
            for(k=0; k<j; k++)
                s+=LL[i*n+k]*LL[j*n+k];
            LL[i*n+j] = (i==j)?
                           sqrt(A[i*n+i]-s):
                           (1.0/LL[j*n+j]*(A[i*n+j]-s));
        }
			}
			
			for(i=0; i<n; i++){
        for(j=0; j<n; j++)
            B[i*n+j] = LL[i*n+j];
			}
}

void mult(double *mat1, double *mat2, int row1, int col1, int row2, int col2, double *matout)
{
	int c = 0;
	int d = 0; 
	int k = 0;
	
	for(c=0;c<row1;c++){
		for(d=0;d<col2;d++){
			for(k=0;k<row2;k++){
				sum = sum + mat1[c*row2+k]*mat2[k*col2+d];
			}
			
			matout[c*col2+d]=sum;
			sum=0;
		}
	}   
}

void multt(double *mat1, int row1, int col1, double *matout)
{
	int c = 0;
	int d = 0; 
	int k = 0;
	
	/*for(c=0;c<24;c++){
		for(d=0;d<49;d++){
			temp[c*49 + d] = mat1[c*49 + d]*Wc[d]; //X1*diag(Wc), output is rowx49
		}
	}*/
	
	for(c=0;c<row1;c++){
		for(d=0;d<col1;d++){
			temp[c*col1 + d] = mat1[c*col1 + d]*Wc[d]; //X1*diag(Wc), output is rowx49
		}
	}
	
	for(c=0;c<row1;c++){
		for(d=c;d<row1;d++){
			for(k=0;k<col1;k++){
					sum = sum + temp[c*col1+k]*mat1[d*col1+k];
			}
			
			matout[c*row1+d]=sum;
			matout[d*row1+c]=sum;
			sum=0;
		}
	}   
}

void inverse(double *matr, int n, double *inv)
{
	int i;
	int j;
	int k;
	
	// define temp matrix as input matrix for easy manipulation
	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			test[i*n+j] = matr[i*n+j];
		}
	}
	
	for(i=0;i<169;i++){
		invmat[i] = 0; // reset invmat
	}
	
	for(i=0;i<n;i++){
		invmat[i*n+i] = 1; // set invmat
	}
	
	//rref
	for(i=0;i<(n-1);i++){
		for(k=1;k<(n-i);k++){
			d[0] = test[(i+k)*n+i]/test[i*n+i];
			for(j=i;j<n;j++){
						test[(i+k)*n + j] = test[(i+k)*n+j] - d[0]*test[i*n+j];
			}
			for(j=0;j<n;j++){
				invmat[(i+k)*n + j] = invmat[(i+k)*n+j] - d[0]*invmat[i*n+j];
			}
		}
	}
	
	//turn to identity
	for(i=n-1;i>-1;i--){
		for(j=0;j<n;j++){
			invmat[i*n+j] = invmat[i*n+j]/test[i*n+i];
		}
			test[i*n+i] = test[i*n+i]/test[i*n+i];
		for(j=i;j>0;j--){
			d[0] = test[(j-1)*n+i];
			test[(j-1)*n+i] = test[(j-1)*n+i] - test[(j-1)*n+i]*test[i*n+i];
			for(k=0;k<n;k++){
				invmat[(j-1)*n+k] = invmat[(j-1)*n+k] - d[0]*invmat[i*n+k];
			}
		}
	}
	
	for(i=0;i<n*n;i++){
		inv[i] = invmat[i];
	}
}

uint16_t get_spidata(uint16_t data){
	  GPIOE->BSRRH |= GPIO_Pin_7; // set PE7 (CS) low
		SPI1_send(data); // transmit dummy byte and receive data
	  receivedvalue = SPI1_receive(); // receive data
		GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 (CS) high
		return receivedvalue;
}

void gyrocalibration(void)
{
	int count=0;
	int nn=100; // the higher the nn, the better the accuracy, but do not let go above 500 for risk of overflow
	
	int16_t xgyroave=0;
	int16_t ygyroave=0;
	int16_t zgyroave=0;
	int16_t zmagnave=0;
	int16_t xaccelave=0;
	int16_t yaccelave=0;
	
	while(count<nn){
		ADC_SoftwareStartConv(ADC3); // convert pin value
		while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET); // wait for conversion to be complete 
		depthbias = depthbias + ADC_GetConversionValue(ADC3); // get analog value
		
		cubedata(); // get all data from cube
		
		xgyroave = xgyroave + (xgyro<<2); // shifted by two because data is 14 bits
		ygyroave = ygyroave + (ygyro<<2);
		zgyroave = zgyroave + (zgyro<<2);
		xaccelave = xaccelave + (xaccel<<2);
		yaccelave = yaccelave + (yaccel<<2);
		zmagnave = zmagn<<2; // only need one reading for offset
		
		count++;
	}
	
	depthbias = depthbias/nn;
		
	// these handle the twos compliment and reshifting of the values
	if(xgyroave>0x7FFF)
		{xgyrobias = ~((xgyroave>>2)/nn);}
	else
		{xgyrobias = (xgyroave>>2)/nn;}
	
	if(ygyroave>0x7FFF)
		{ygyrobias = ~((ygyroave>>2)/nn);}
	else
		{ygyrobias = (ygyroave>>2)/nn;}
	
	if(zgyroave>0x7FFF)
		{zgyrobias = ~((zgyroave>>2)/nn);}
	else
		{zgyrobias = (zgyroave>>2)/nn;}
		
	if(xaccelave>0x7FFF)
		{xaccelbias = ~((xaccelave>>2)/nn);}
	else
		{xaccelbias = (xaccelave>>2)/nn;}
		
	if(yaccelave>0x7FFF)
		{yaccelbias = ~((yaccelave>>2)/nn);}
	else
		{yaccelbias = (yaccelave>>2)/nn;}
		
	if(zmagnbias>0x7FFF)
		{zmagnbias = ~(zmagnave>>2);}
	else
		{zmagnbias = zmagnave>>2;}
		
		xaccel = 0;
		yaccel = 0;
		zaccel = 0;
		xmagn = 0;
		ymagn = 0;
		zmagn = 0;
		xgyro = 0;
		ygyro = 0;
		zgyro = 0;
		temper = 0;
}

void cubedata(void){
	// obtains one of each reading when called
			get_spidata(0x0A00); // read data
			temper = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x0C00); // read data
			xaccel = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x0E00); // read data
			yaccel = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x0400); // read data
			zaccel = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x0600); // read data
			xgyro = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x0800); // read data
			ygyro = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x1000); // read data
			zgyro = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x1200); // read data
			xmagn = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x1400); // read data
			ymagn = (receivedvalue&0x3FFF);
			Delay(40000);
			
			get_spidata(0x1600); // read data
			zmagn = (receivedvalue&0x3FFF);
			Delay(40000);
			
			measureupdate();
	
}

void intcubedata(uint16_t counter){
	
	// if statements to determine which measurement to take
	// take sum of total measurements to point, use individual counters to keep track of sum
	// shift values to represent 16 bits (values are 14 from cube)
	if(counter%10 == 0){
			get_spidata(0x0A00); // read data
			temper = temper + ((receivedvalue&0x0FFF)<<4)&0xFFF0;
			tc++;
	}
	
	else if(counter%10 == 1){
			get_spidata(0x0C00); // read data
			xaccel = xaccel + (((receivedvalue-xaccelbias)&0x3FFF)<<2)&0xFFFC;
			xac++;
	}
	
	else if(counter%10 == 2){
			get_spidata(0x0E00); // read data
			yaccel = yaccel + (((receivedvalue-yaccelbias)&0x3FFF)<<2)&0xFFFC;
			yac++;
	}
	
	else if(counter%10 == 3){
			get_spidata(0x0400); // read data
			zaccel = zaccel + ((receivedvalue&0x3FFF)<<2)&0xFFFC;
			zac++;
	}
	
	else if(counter%10 == 4){
			get_spidata(0x0600); // read data
			xgyro = xgyro + (((receivedvalue-xgyrobias)&0x3FFF)<<2)&0xFFFC;
			xgc++;
	}
	
	else if(counter%10 == 5){	
			get_spidata(0x0800); // read data
			ygyro = ygyro + (((receivedvalue-ygyrobias)&0x3FFF)<<2)&0xFFFC;
			ygc++;	
	}
	
	else if(counter%10 == 6){
			get_spidata(0x1000); // read data
			zgyro = zgyro + (((receivedvalue-zgyrobias)&0x3FFF)<<2)&0xFFFC;
			zgc++;
	}
	
	else if (counter%10 == 7){	
			get_spidata(0x1200); // read data
			if(xmc%5==0){
			xmagn = xmagn + ((receivedvalue&0x3FFF)<<2)&0xFFFC;
			}
			xmc++;	
	}
	
	else if(counter%10 == 8){
			get_spidata(0x1400); // read data
			if(ymc%5==0){
			ymagn = ymagn + ((receivedvalue&0x3FFF)<<2)&0xFFFC;
			}
			ymc++;
	}
	
	else if(counter%10 == 9){
			get_spidata(0x1600); // read data
			if(zmc%5==0){
			zmagn = zmagn + (((receivedvalue-zmagnbias)&0x3FFF)<<2)&0xFFFC;
			}
			zmc++;
		}
	}

void measureaverage(void){
	// need to shift values back to 14 bits and average by dividing by their respective counter
	xaccel = ((xaccel>>2)/xac)&0x3FFF;
	yaccel = ((yaccel>>2)/yac)&0x3FFF;
	zaccel = ((zaccel>>2)/zac)&0x3FFF;
	xmagn = ((xmagn>>2)/(xmc/5))&0x3FFF;
	ymagn = ((ymagn>>2)/(ymc/5))&0x3FFF;
	zmagn = ((zmagn>>2)/(zmc/5))&0x3FFF;
	xgyro = ((xgyro>>2)/xgc)&0x3FFF;
	ygyro = ((ygyro>>2)/ygc)&0x3FFF;
	zgyro = ((zgyro>>2)/zgc)&0x3FFF;
	temper = ((temper>>4)/tc)&0x0FFF;
}	
	
void resetcounters(void){
	// resets counter values, needs to be done to prevent overflow on counters
	xac=0;
	yac=0;
	zac=0;
	xgc=0;
	ygc=0;
	zgc=0;
	xmc=0;
	ymc=0;
	zmc=0;
	tc=0;
	intcounter=0;
	xaccel = 0;
	yaccel = 0;
	zaccel = 0;
	xmagn = 0;
	ymagn = 0;
	zmagn = 0;
	xgyro = 0;
	ygyro = 0;
	zgyro = 0;
	temper = 0;
}

void measureupdate(void){
	// updates measurements with algorithm to convert between readings and values
	z[0] = twoscomp(xmagn)*.0005;
	z[1] = twoscomp(ymagn)*.0005;
	z[2] = twoscomp(zmagn)*.0005;
	//z[0] = .25;
	//z[1] = 0;
	//z[2] = 0;
	z[3] = twoscomp(xaccel)*.003333*-9.8;
	z[4] = twoscomp(yaccel)*.003333*-9.8;
	z[5] = twoscomp(zaccel)*.003333*-9.8;
	//z[3] = 0;
	//z[4] = 0;
	//z[5] = 9.8;
	z[6] = twoscomp(xgyro)*.05;
	z[7] = twoscomp(ygyro)*.05;
	z[8] = twoscomp(zgyro)*.05;
	//z[6] = 0;
  //z[7] = 0;
	//z[8] = 0;
	//z[9] = 12.7764;
	z[9] = 14.223;
	
	ADC_SoftwareStartConv(ADC3); // convert pin value
	while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET); // wait for conversion to be complete 
	ADC3ConvertedValue = ADC_GetConversionValue(ADC3) - depthbias; // get analog value
	
	if(ADC3ConvertedValue > 0x7FFF){
		ADC3ConvertedValue = 0;}
}

// twos compliment handler
double twoscomp(uint16_t meas){
	if (meas > 0x01FFF)
		{measout = -(~meas&0x1FFF);}
	else
	{measout = meas;}
		return measout;
}

// prints various states to putty, used in debugging and data collection
void printpositions(void){
	int countt=0;
	
	int16_t xpos;
	int16_t ypos;
	int16_t zpos;
	int16_t xspeed;
	int16_t yspeed;
	int16_t zspeed;
	int16_t phiangle;
	int16_t thetaangle;
	int16_t psiangle;
	int16_t gammaint;
	
	while(countt<12)
		{
			xx[countt] = x[countt]*256;
			countt++;
		}
		countt=0;
		
	xpos = xx[9];
	ypos = xx[10];
	zpos = xx[11];
	xspeed = xx[0];
	yspeed = xx[1];
	zspeed= xx[2];
	phiangle = xx[3];
	thetaangle = xx[4];
	psiangle = xx[5];
	gammaint = gamma*256;

	printvalue(xpos);
	printvalue(ypos);
	printvalue(zpos);
	printvalue(xspeed);
	printvalue(yspeed);
	printvalue(zspeed);
	printvalue(phiangle);
	printvalue(thetaangle);
	printvalue(psiangle);
	//printvalue(gammaint);
	
	USART_puts(USART3, "\r\n"); 
}

// USART print function
void printvalue(uint16_t display)
{
	int ccnt = 0; // character count for formatting
	int icnt = 0;
	int tempdisp;
	int storage;
	char strin[8] = 0;
	int zerocheck=1;
	
	if(display>0x7FFF)
	{
		USART_puts(USART3, "-");
		tempdisp = ((~display)&0x7FFF);
		storage = ((~display)&0x7FFF);
		ccnt++;
	}
	else
	{
		tempdisp = display;
		storage = display;
	}
		
		
	while(tempdisp!=0){
		tempdisp=tempdisp/10;
		icnt++;
	}
	
	while(icnt!=0){
		strin[icnt-1] = (storage%10) | 0x30;
		storage = storage/10;
		icnt--;
		zerocheck=0;
		ccnt++;
	}
	
	if(zerocheck==0){
		USART_puts(USART3, strin); 
		while((10-ccnt)!=0)
		{
			USART_puts(USART3, " ");
			ccnt++;
		}
	}
	else
	{
		USART_puts(USART3, "0");
		ccnt++;
		while((10-ccnt)!=0)
		{
			USART_puts(USART3, " ");
			ccnt++;
		}
	}
	ccnt=0;
	icnt=0;
}

void init_GPIO(void){

	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; //this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
	GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}

void init_SPI1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure the chip select pin
	   in this case we will use PE7 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 high

	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b; // one packet of data is 16 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;        // clock is high when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // SPI frequency is APB2 frequency / 64
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct); 
	
	// RCC_PCLK2Config(RCC_HCLK_Div16); // slows down clock speed

	SPI_Cmd(SPI1, ENABLE); // enable SPI1
}

void ADC3_CH12_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}

void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	/* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
		/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) and TIM3 CH3 (PC8) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	/* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
}

void PWM_Out(uint16_t pulse, uint16_t roll, uint16_t motor){
	if(pulse>2100){
		pulse=2100;} // 10 degrees, ensures wings don't go past stall -- 1600
		
	if(pulse<900){
		pulse=900;} // -10 degrees, ensures wings don't go past stall -- 1400
		
	// roll inputs, change as needed	
	if(roll<425){
		roll=425;}	
	if(roll>425){
		roll=425;}
		
	if(motor==1){
		motor=20000;} // motor on
		
	if(motor!=1){
		motor=0;} // motor off
		
	/* Compute the prescaler value */
	/* ((SystemCoreClock /2) / 33333) == 50 Hz Signal*/
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 33333) - 1;

  // Time base configuration
  TIM_TimeBaseStructure.TIM_Period = 20000-1; // 665 original value
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1; // (PrescalerValue); original value
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	// PWM1 Mode configuration: Channel 1 - Wing 1 - PC6
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (pulse + (roll - 425));
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// PWM1 Mode configuration: Channel 2  - Wing 2 - PC7
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (pulse - (roll - 425));
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// PWM1 Mode configuration: Channel 3 - Motor on or off - PC8
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (motor);
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// PWM1 Mode configuration: Channel 4 - 50% Duty for direction of motor - PC9
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10000; // 50% Duty
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	// TIM3 enable counter
  TIM_Cmd(TIM3, ENABLE);
}

void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* --------------------------- System Clocks Configuration -----------------*/
  /* USART3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
 
  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 9600 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(USART3, &USART_InitStructure);
 
  USART_Cmd(USART3, ENABLE);
}

void EXTILine1_Config(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line1 to PA1 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

  /* Configure EXTI Line1 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI1_IRQHandler(void)
{    
	
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
				intcubedata(intcounter);
				intcounter++;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
		
		
}

/*#ifdef  USE_FULL_ASSERT // used to catch hard fault errors

void assert_failed(uint8_t* file, uint32_t line)
{ 

  // Infinite loop 
  while (1)
  {
  }
}
#endif */