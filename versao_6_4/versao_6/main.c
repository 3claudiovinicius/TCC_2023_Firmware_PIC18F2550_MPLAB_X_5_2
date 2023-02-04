#include "myIncludes.h"
//<=============================================================================
void configADCON(unsigned char adcon0,unsigned char adcon1,unsigned char adcon2);
void configSampling(void);
unsigned int convAD(void);
void regInit(void);
void verifRes(void);
void normalize (int N, float X[]);
void bitrevsort (int N, float Re_x[], float Im_x[]);
void FFT (int N, float Re_X[], float Im_X[]);
//<===================================
void LCDGoto(char pos,char ln);
void LCD_Initialize(void);
void LCDPutChar(char ch);
void LCDPutCmd(char ch);
void LCDPutStr(const char *);
void LCDWriteNibble(char ch, char rs);
void SPI_config(void);
unsigned char SPI_transm(unsigned char dado);
void SPI_MCP41(unsigned char comando, unsigned char dado);
//<=============================================================================
void __interrupt(high_priority) samples(void){
    unsigned int sample;
    if(INTCONbits.TMR0IF){
        INTCONbits.TMR0IF=0;
        TMR0L=n_TMR0;
        sample=convAD();
        if(!canal){
            a[k]=sample;
        }else{
            b[k]=sample;
        }
        k++;
    }
}

void main(void) {
    regInit();
    LCD_Initialize();
    SPI_config();
    LCDPutStr("Iniciando...");
    __delay_ms(10000);
    LCDPutCmd(LCD_CLEAR);
    while(1){
        configSampling();
        
        for (k=0; k<No; k++){
            Re_X1[k] = a[k];}
        
        normalize(No, Re_X1);
        
        for (k=0; k<No; k++){
            Re_X[k] = Re_X1[k];}
        
        for (k=0; k<No; k++){
            Re_X1[k] = b[k];}

        normalize(No, Re_X1);
        
        for (k=0; k<No; k++){
            Im_X[k] = Re_X1[k];}
        
        bitrevsort (No, Re_X, Im_X);      

        FFT (No, Re_X, Im_X);
        
        Re_Xm1[0] = Re_X[0];
        Im_Xm1[0] = Im_X[0];
        
        for (k=1; k<No; k++){
            Re_Xm1[No-k]=Re_X[k];
            Im_Xm1[No-k]=Im_X[k];}
        
        for (k=0; k<No; k++){        
            Re_X1[k] = (Re_X[k] + Re_Xm1[k])/2;
            Im_X1[k] = (Im_X[k] - Im_Xm1[k])/2;

            Re_X1[k]=sqrt(Re_X1[k]*Re_X1[k] + Im_X1[k]*Im_X1[k]);
            a[k] = 100*Re_X1[k];

            Re_X1[k] = (Im_X[k] + Im_Xm1[k])/2;
            Im_X1[k] = (Re_Xm1[k] - Re_X[k])/2;

            Im_X1[k]=sqrt(Re_X1[k]*Re_X1[k] + Im_X1[k]*Im_X1[k]);
            b[k] = 100*Im_X1[k];}
        
        max1=0;
        max2=0;
        for (k=20; k<No; k++){
            if (max1 < a[k]){ 
                max1 = a[k];}
            if (max2 < b[k]){
                max2 = b[k];}}		

        float aux;
        aux=max1/max2;
        D2[e]=1000*log(aux)/log(2);
        verifRes();
    }
    return;
}

void regInit(void){
    TRISB=0x0D;
    OSCCON = 0x00;
    T0CON = 0b01001000;
    INTCON = 0b00100000;
    INTCON2 = 0b00000100;
    RCON = 0b10000000;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    
    INTCONbits.GIE = 0;
    
    __delay_ms(5);
    
    //passo=0x19;//0.5v
    //passo=0x05;//0.1v
    //passo=0x32;//1v
    passo=0x05;
    pot_value=0x1E;
    //erro=1e-2;//vbias=0.33~0.37v
    //erro=1e-4;//vbias=0.51v
    //erro=1e-5;//vbias nao encontrado
    //erro=1e-10;//vbias nao encontrado
    v=0;
    e=0;
    for(int j=0;j<20;j++){
        D2[j]=0;
        voltage[j]=0;
    }
    aux_D2=0;
    aux_Voff=0;
}

void configSampling(void){
    
    for(canal=0;canal<2;canal++){
      switch(canal){
          case 0: configADCON(CHAN_8,AN_8,DIV16_12TAD);
          break;
          case 1: configADCON(CHAN_9,AN_9,DIV16_12TAD);
          break;
      }
      INTCONbits.GIE=1;
      TMR0L=n_TMR0;
      k=0;
      while(k<No){
          T0CONbits.TMR0ON=1;
      }
      INTCONbits.GIE=0;
    }    
}

void verifRes(void){
    configADCON(CHAN_12,AN_12,DIV16_12TAD);
    ADRESH=0;
    ADRESL=0;
    ADCON0bits.ADON=1;
    ADCON0bits.GO=1;
    while(ADCON0bits.GO);
    v=((unsigned int)ADRESH<<8) | (ADRESL);
    voltage[e]=v*(float)vref/1023;
   
    sprintf(info,"Voff = %.2f V",voltage[e]);
    LCDPutStr(info);
    LCDGoto(0,1);
    sprintf(info,"D2 = %.2f",D2[e]);
    LCDPutStr(info);
    __delay_ms(3000);
    LCDPutCmd(LCD_CLEAR);
    
    pot_value=pot_value+passo;
    SPI_MCP41(command,pot_value);
    
    e+=1;
    v=0;

    if(pot_value==0x82){
        
        pot_value=0x1E;
        SPI_MCP41(command,pot_value);
        
        for(int i=0;i<20;i++){
            if(aux_D2<D2[i]){
                aux_D2=D2[i];
                aux_Voff=voltage[i];
            }
        }
        sprintf(info2,"V_bias = %.2f V",aux_Voff);
        LCDPutStr(info2);
        LCDGoto(0,1);
        sprintf(info2,"D2max = %.2f",aux_D2);
        LCDPutStr(info2);
        __delay_ms(1000);
        LCDPutCmd(LCD_CLEAR);
        
        LCDPutStr("FIM");
        __delay_ms(1000);
        LCDPutCmd(LCD_CLEAR);
        
        regInit();
    }
}

//<=============================================================================
void FFT (int N, float Re_X[], float Im_X[])
{
    int M, m, n, k, L, p;
    float tempR, tempI, uR, uI, sR, sI;

    M = log(N)/log(2); 

    for (p = 1; p <= M; p++) //Loop for each stage
    {
        //L = pow(2,p);
        L=1;
        for(k=0;k<p;k++){
            L=2*L;
        }
        
        uR = 1;
        uI = 0;
        sR = cos(2*M_PI/L); //Calculate sine & cosine values
        sI = -sin(2*M_PI/L);
    
        for (k = 0; k < L/2; k++) //Loop for each sub DFT
        {
            for (n = k; n < (N-1); n+=L) //Loop for each butterfly
            {
                m = n + L/2;
                tempR = Re_X[m]*uR - Im_X[m]*uI;   //Butterfly calculation
                tempI = Re_X[m]*uI + Im_X[m]*uR;
                Re_X[m] = Re_X[n] - tempR;
                Im_X[m] = Im_X[n] - tempI;
                Re_X[n] = Re_X[n] + tempR;
                Im_X[n] = Im_X[n] + tempI;
            }        
            tempR = uR;
            uR = tempR*sR - uI*sI;
            uI = tempR*sI + uI*sR;
        } 
    }
}

void bitrevsort (int N, float Re_x[], float Im_x[])
{
    int m, n, k;
	float tempR, tempI;
    
    m = N/2;

    for (n = 1; n <= N-2; n++) //Bit reversal sorting
    {
        if (n < m)
        {
            tempR = Re_x[m];
            tempI = Im_x[m];
            Re_x[m] = Re_x[n];
            Im_x[m] = Im_x[n];
            Re_x[n] = tempR;
            Im_x[n] = tempI;
        }
        
        k = N/2;    
        while (k <= m)
        {
            m = m-k;
            k = k/2;
        }   
        m = m+k;
    }
}    

void normalize (int N, float X[])
{
    char k;
    int media = 0;
    
    for (k=0; k<N; k++)
    {
        media = media + X[k];            
    }
    
    media = media/N;
    
    for (k=0; k<N; k++)
    {
        X[k] = X[k] - media;
        X[k] = X[k]/512;
    }
}

void configADCON(unsigned char adcon0,unsigned char adcon1,unsigned char adcon2){
    ADCON0=adcon0;
    ADCON1=adcon1;
    ADCON2=adcon2;
}

unsigned int convAD(void){
    unsigned int result;
    
    ADRESH=0;
    ADRESL=0;
    ADCON0bits.ADON=1;
    ADCON0bits.GO=1;
    
    while(ADCON0bits.GO);
    result = ((unsigned int)ADRESH<<8) | (ADRESL);
    return(result);
}

void LCDGoto(char pos,char ln)
{
// if incorrect line or column
    if ((ln > (NB_LINES-1)) || (pos > (NB_COL-1))){
    
// Just do nothing
        return;}
    LCDPutCmd((ln == 1) ? (0xC0 | pos) : (0x80 | pos));

    __delay_ms(LCD_delay);
}

void LCD_Initialize(void)
{
// clear latches before enabling TRIS bits
    LCD_PORT = 0;

    TRISA = 0x00;

// power up the LCD
//    LCD_PWR = 1;

// required by display controller to allow power to stabilize
    __delay_ms(LCD_Startup);

// required by display initialization
    LCDPutCmd(0x32);
    
    __delay_ms(4);

// set interface size, # of lines and font
    LCDPutCmd(FUNCTION_SET);

// turn on display and sets up cursor
    LCDPutCmd(DISPLAY_SETUP);

    DisplayClr();

// set cursor movement direction
    LCDPutCmd(ENTRY_MODE);
    
    __delay_ms(2); 
}

void LCDPutChar(char ch)
{
    __delay_ms(LCD_delay);

//Send higher nibble first
    LCDWriteNibble(ch,data);

//get the lower nibble
    ch = (ch << 4);

// Now send the low nibble
    LCDWriteNibble(ch,data);
}

void LCDPutCmd(char ch)
{
    __delay_ms(LCD_delay);

//Send the higher nibble
    LCDWriteNibble(ch,instr);

//get the lower nibble
    ch = (ch << 4);
    
//Now send the lower nibble
    LCDWriteNibble(ch,instr);
}

void LCDPutStr(const char *str)
{
    char i=0;

// While string has not been fully traveresed
    while (str[i])
        {
// Go display current char
        LCDPutChar(str[i]);
        i=i+1;
        }

}

void LCDWriteNibble(char ch, char rs)
{
// always send the upper nibble
    ch = (ch >> 4);

// mask off the nibble to be transmitted
    ch = (ch & 0x0F);

// clear the lower half of LCD_PORT
    LCD_PORT = (LCD_PORT & 0xF0);

// move the nibble onto LCD_PORT
    LCD_PORT = (LCD_PORT | ch);

// set data/instr bit to 0 = insructions; 1 = data
    LCD_RS = rs;

// RW - set write mode
//    LCD_RW = 0;
    
    __delay_ms(5);

// set up enable before writing nibble
    LCD_EN = 1;
    
// turn off enable after write of nibble
    LCD_EN = 0;
}

void SPI_config(void)
{
	SDO_DIR = 0;	//SDO como sa�da.
	SDI_DIR = 1;	//SDI como entrada.
	SCK_DIR = 0;	//SCK como sa�da.
	SS_DIR = 0;	//SS como saida.

	SS = 1;	//Desabilita Chip Select.

	SSPCON1 = 0b00100010;	/* Habilita modo SPI <5>.
								Iddle clock em n�vel baixo <4>.
								Modo mestre,
								bit rate = Fosc/64 <3:0>. */

	SSPSTAT = 0b01000000;	/* Recep��o na borda do meio do per�odo
								do clock <7>.
								A transmiss�o ocorre na transi��o
								do estado ativo para o ocioso do 
								clock <6>. 
								(Se SSPCON1<4>=1 -> SSPSTAT<6>=0).
							*/
     SPI_MCP41(command,pot_value);
}

unsigned char SPI_transm(unsigned char dado)
{
	SSPBUF = dado;			//Passa o dado para o buffer.

	while(!SSPSTATbits.BF)	//Espera o t�rmino da transmiss�o.
	{}
	
	dado = SSPBUF;			//Leitura burra.
	return dado;			//Retorna o dado lido.
}

void SPI_MCP41(unsigned char comando, unsigned char dado)
{
    SS = 0;
    SPI_transm(comando);
    SPI_transm(dado);
    SS = 1;
}



