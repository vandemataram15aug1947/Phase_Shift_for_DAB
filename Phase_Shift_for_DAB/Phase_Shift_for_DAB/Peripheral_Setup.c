/*
 * Peripheral_Setup.c
 *
 *  Created on: 23 de jul de 2020
 *      Author: waner
 */
#include "Peripheral_Setup.h"


void Setup_GPIO(void){
    EALLOW;
    // LED 31 A, 2
    // LED 34 B, 1
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1;

    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1;

    //PWM 1A e 1B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;

    //PWM 2A e 2B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;

    //PWM 3A e 3B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;

    //PWM 4A e 4B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;
    EDIS;
}

void Setup_ePWM(void){
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    EPwm1Regs.TBPRD = 5000;							// Set timer period
    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD >> 1;

    EPwm1Regs.TBPHS.bit.TBPHS = 0;             		// Phase is 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        	// Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    	// Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;    	// Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR; 			// set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; 	// enable Dead-band module
    EPwm1Regs.DBFED.bit.DBFED = 100; 				// FED = 20 TBCLKs
    EPwm1Regs.DBRED.bit.DBRED = 100; 				// RED = 20 TBCLKs

    //Trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;          		// Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;  	// Dispara ADC no topo
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;         	// Trigger on every event


    // PWM 2A e 2B
    EPwm2Regs.TBPRD = EPwm1Regs.TBPRD;				// Set timer period
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD >> 1;
    EPwm2Regs.TBPHS.bit.TBPHS = EPwm2Regs.TBPRD;    // Phase is 180
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Count up/down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        	// Disable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;        	// Phase UP/DOWN
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       	// Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    	// Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

    EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; 			// set actions for EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    // PWM 3A e 3B
    EPwm3Regs.TBPRD = EPwm1Regs.TBPRD;              // Set timer period
    EPwm3Regs.CMPA.bit.CMPA = EPwm3Regs.TBPRD >> 1;
    EPwm3Regs.TBPHS.bit.TBPHS = 0;    // Phase is 180
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // Disable phase loading
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;           // Phase UP/DOWN
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

    EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM2A
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;


    // PWM 4A e 4B
    EPwm4Regs.TBPRD = EPwm1Regs.TBPRD;              // Set timer period
    EPwm4Regs.CMPA.bit.CMPA = EPwm4Regs.TBPRD >> 1;
    EPwm4Regs.TBPHS.bit.TBPHS = 0;    // Phase is 180
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm4Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up/down
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // Disable phase loading
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;           // Phase UP/DOWN
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // set actions for EPWM2A
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}


void Setup_ADC(void){
    Uint16 acqps;
    // determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14; 							// 75ns
    else 										// resolution is 16-bit
        acqps = 63; 							// 320ns

    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; 			// set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;		// Set pulse um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;			// power up the ADC
    DELAY_US(1000); 							// delay for 1ms to allow ADC time to power up

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; 			//sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA; //trigger on ePWM2 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = TRIG_SEL_ePWM1_SOCA;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x01; 		// end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   			// enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; 			// make sure INT1 flag is cleared

    EDIS;
}

