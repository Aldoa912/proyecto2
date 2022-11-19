/*
 * File:   proyecto2.c
 * Author: aldoa
 *
 * Created on 8 de noviembre de 2022, 09:16 AM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000      // Frecuencia interna del PIC
#define tmr0_value 102         // valor del timer0

//******************************************************************************
// Declaracion de variables
//******************************************************************************
unsigned int serv1;
unsigned int serv2;
unsigned int serv3;
unsigned int serv4;
unsigned int ADC;
unsigned int ADC2;
unsigned int ADC3;
unsigned int ADC4;
int cuenta;
uint8_t address = 0, cont = 0, cont_sleep = 0,data;
//******************************************************************************
//Prototipos de funciones
//******************************************************************************
void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);
void delay (unsigned int seg);
void servo1(int valor);
void servo2(int valor);
unsigned int map(uint8_t ADC, int entrada_min, int entrada_max, int salida_min, 
        int salida_max);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);
//******************************************************************************
// Interrupciones
//******************************************************************************

void __interrupt () isr (void){
    if(INTCONbits.T0IF){        //Se revisa la bandera del timer0
        TMR0 = tmr0_value; // se asigna el valor del timer0
        PORTCbits.RC3 = 1; // se enciende el puerto
        delay (serv3);      // nos dirigimos a la funcion de delay
        PORTCbits.RC3 = 0;  // despues de que pase el delay se apaga el puerto
        PORTCbits.RC4 = 1; 
        delay (serv4);
        PORTCbits.RC4 = 0; // Se repite lo mismo, solo que con otro puerto
        INTCONbits.T0IF = 0; // se limpia la bandera
    }
    
    if (INTCONbits.RBIF){
        if (PORTBbits.RB0 == 0){
            cont++;         // si se presiona RB0, se aumenta la variable cont
        }
        
        if (PORTBbits.RB1 == 0){
            address = address + 4;  //Si se presiona RB1, se aumenta el address
                                    // en cuatro
        }
        
        else if (PORTBbits.RB2 == 0)
            address = address - 4;  //Si se presiona RB2, se disminuye el 
                                    //address en 4
        
        else if (PORTBbits.RB3 == 0){
            write_EEPROM(address, serv1);
            write_EEPROM(address + 1, serv2);
            write_EEPROM(address + 2, serv3);
            write_EEPROM(address + 3, serv4);
            // si se presiona RB3 se guardara el valor de las variables serv1
            //serv2, serv3 y serv4 en el valor de address que este y los tres
            //espacios que esten debajdo de este
        }
        
        else if (cont == 1){ //Si la variable cont es = 1, se podran leer
                             // los datos que se tengan en la eeprom
            if (PORTBbits.RB4 == 0){
                serv1 = read_EEPROM(address);
                serv2 = read_EEPROM(address+1);
                serv3 = read_EEPROM(address+2);
                serv4 = read_EEPROM(address+3);
                // si se presiona RB4 se escribira el valor que este en la 
                //direccion dada por el adrress y las 3 posiciones debajo de 
                // esta en serv, serv2, serv3 y serv4
            }
            else if (PORTBbits.RB1 == 0){
                address = address + 4;
            }

            else if (PORTBbits.RB2 == 0)
                address = address - 4;
        }

        INTCONbits.RBIF = 0; // se apaga la bandera
    }
    return;
}
//******************************************************************************
// CÃ³digo Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupINTOSC();
    setupPWM();
    setupADC();
    while(1){
//******************************************************************************
// ADC
//******************************************************************************
        if (cont == 0){
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 0;   // se enciende la led que indica el modo 1
            ADCON0bits.CHS = 0b0000;        // usamos el canal 0
            __delay_us(100);
            ADCON0bits.GO = 1;  // enciendo la bandera
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // apago la bandera
            ADC = ADRESH;       // cargo el valor a ADC
            servo1 (ADC);        // llamo a la funcion e ingreso el valor de ADC
            CCPR1L = serv1;  // cargo el valor de la conversion a CCPR1L
            __delay_us(100);

            ADCON0bits.CHS = 0b0001; // usamos el canal 1
            __delay_us(100);
            ADCON0bits.GO = 1;  // enciendo la bandera
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // apago la bandera
            ADC2 = ADRESH;      // cargo el valor a ADC2
            servo2 (ADC2);       // llamo a la funcion e ingreso el valor de ADC2
                CCPR2L = serv2;  // cargo el valor de la conversion a CCPR2L
                __delay_us(100);

            ADCON0bits.CHS = 0b0010;    // usamos el canal 2
            __delay_us(100);    
            ADCON0bits.GO = 1;  // enciendo la bandera
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // apago la bandera
            ADC3 = ADRESH;      // cargo el valor a ADC3
            serv3 = map(ADC3, 0, 255, 5, 17);   // hago la conversion del ADC
            __delay_us(100);  
            
            ADCON0bits.CHS = 0b0011;    // usamos el canal 3
            __delay_us(100);    // 
            ADCON0bits.GO = 1;  // enciendo la bandera
            while(ADCON0bits.GO == 1){
                ;
            }
            ADIF = 0;           // apago la bandera
            ADC4 = ADRESH;      // cargo el valor a ADC4
            serv4 = map(ADC4, 0, 255, 5, 17); // hago la conversion del ADC
            __delay_us(100);  
        }
        
        
        if (cont == 1){ // si mi variable cont == 1, ya no podre leer los 
                        // ADC
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 1;
            PORTDbits.RD2 = 0;
            CCPR2L = serv2;
            CCPR1L = serv1;
            
        }
        
        if (cont == 2){
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 1;  
        }
        
        if (cont == 3){ // Si la variable cont = 3 esta se reinicia
            cont = 0;
        }
    }
    return;
}
//******************************************************************************
void servo1(int valor){
    serv1 = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
    // realizo la conversion para el servomotor
}

void servo2(int valor){
    serv2 = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
    // realizo la conversion para el servomotor
}
//******************************************************************************
// FunciÃ³n para configurar GPIOs
//******************************************************************************
void setup (void){
    ANSELH = 0;
    
    TRISB = 0b00011111;
    PORTB = 0;
    
    TRISA = 0;
    TRISC = 0;
    TRISD = 0;
    
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    cuenta = 0;
    
    OPTION_REGbits.nRBPU = 0;   // se apaga el bi nRBPU
    
    IOCB = 0b01111111;
    

    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;        // se enciente la interrupcion del puesto B
    INTCONbits.GIE = 1;         // habilitamos las interrupciones globales

   
    
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;           
    INTCONbits.TMR0IE = 1;          // activo la interrupcion del timer0
    INTCONbits.T0IF = 0;            // apago la bandera del timer0

    
    OPTION_REGbits.T0CS = 0;        // se selecciona el oscilador interno
    OPTION_REGbits.PSA = 0;         // se escoge el prescaler del timer 0
    OPTION_REGbits.PS = 0b011;
    
    TMR0 = tmr0_value;
}
//******************************************************************************
// FunciÃ³n para configurar ADC
//******************************************************************************
void setupADC(void){
    
    // Paso 1 Seleccionar puerto de entrada
    TRISAbits.TRISA0 = 1;
    ANSELbits.ANS0 = 1; 
    
    TRISAbits.TRISA1 = 1; 
    ANSELbits.ANS1 = 1; 
    
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1; 
    
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1; 
    
    // Paso 2 Configurar mÃ³dulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
        
    //Canal
    ADCON0bits.CHS = 0b0000;        // Canal AN1
    //ADCON0bits.CHS = 0b0010;       // Canal AN2
    //ADCON0bits.CHS = 0b0011;
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupPWM(void){
 
    // **********************************************************
    
    TRISCbits.TRISC2 = 1;           //encendemos el puerto del PWM
    TRISCbits.TRISC1 = 1;           //encendemos el puerto del PWM
    
    PR2 = 155;                      // Periodo de 20ms
    
    CCP1CONbits.P1M = 0b00;         
    
    
    CCP1CONbits.CCP1M = 0b1100;     

    
    CCP2CONbits.CCP2M = 0b1111;     
    
    //Calculos para 1.5ms de ancho de pulso
    CCP1CONbits.DC1B = 0b11;        //CCPxCON<5:4>
    CCPR1L = 11;                    //CCPR1L
    
    
    CCP2CONbits.DC2B1 = 0b1;        //CCPxCON<5:4>
    CCP2CONbits.DC2B0 = 0b1;
    CCPR2L = 11;                    //CCPR2L
    
    //Configutación del TMR2
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;        //Preescaler de 1:16
    T2CONbits.TMR2ON = 1;           //Se habilita TMR2
    
    while(!PIR1bits.TMR2IF){       
        ;
    }
    TRISCbits.TRISC2=0;             //Habilitamos la salida del PWM.
    TRISCbits.TRISC1=0;             //Habilitamos la salida del PWM.
}

uint8_t read_EEPROM (uint8_t address){
    while (WR||RD);
    
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
    
}

void write_EEPROM(uint8_t address, uint8_t data){
    uint8_t gieStatus;
    while (WR);
    
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    gieStatus = GIE;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    EECON1bits. WREN = 0;
    
    INTCONbits.GIE = gieStatus;
}
unsigned int map (uint8_t ADC, int entrada_min, int entrada_max, int salida_min, int salida_max){
    return ((ADC - entrada_min)*(salida_max-salida_min)) / ((entrada_max-entrada_min)+salida_min);
    // se convierten los valores del adc para que el servo pueda leerlos
}

void delay (unsigned int seg){ // se hace un delay para el pulso
    while (seg > 0){
        __delay_us(50);
        seg--;
    }
    return;
}