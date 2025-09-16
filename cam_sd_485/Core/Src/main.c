/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* que funcione plox, tengo que chambeaaarr xdxd
 * cambio en la computadora
 * prueba de coommit en CubeIDE, carajoo
 *asdasdasdasdsadasdasdaadsadas
 *asdasdasdasdsadasdasdaadsadas
 *asdasdasdasdsadasdasdaadsadas
 *assdfsd
 *tmre
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include "IMG_20250425_000083.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- CONFIGURE THIS ---- */
#define LINK_UART_HANDLE  huart7          // <--- change to your UART handle (huart3, huart7, etc.)
#define SAVE_FILENAME     "im11confe.JPG"    // file on the SD card to send
/* ------------------------ */

#define CHUNK          256u
#define SOF0           0x55
#define SOF1           0xAA
#define ACK            0x06
#define NAK            0x15
#define UART_TMO_MS    2500


extern UART_HandleTypeDef LINK_UART_HANDLE;

static int generar_nombre_unico(char *out, size_t out_sz);
char filename[40];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart7_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

extern const uint8_t IMG_20250425_000083[];
extern const uint32_t IMG_20250425_000083_SIZE;

/* ---------- intentos.txt helpers ---------------------------------- */
static const char *ATTEMPT_FILE = "intentos.txt";
static uint32_t    imgNumber    = 0;          /* number for current image */



void myprintf(const char *fmt, ...);


/* fixed date like your sketch; swap for RTC later */
static void fecha_yyyymmdd(char *dst) { strcpy(dst, "20250101"); }

/* IMG_YYYYMMDD_NNNNNN.JPG with FatFS existence check */
static int generar_nombre_unico(char *out, size_t out_sz) {
    if (!out || out_sz < 32) return -1;
    char ymd[9]; fecha_yyyymmdd(ymd);
    FILINFO fno;
    for (unsigned i = 0; i < 1000000U; i++) {
        int n = snprintf(out, out_sz, "IMG_%s_%06u.JPG", ymd, i);
        if (n <= 0 || (size_t)n >= out_sz) return -2;
        FRESULT fr = f_stat(out, &fno);
        if (fr == FR_NO_FILE) return 0;      // available
        if (fr != FR_OK && fr != FR_EXIST) return -3;
    }
    return -4;
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


FATFS   FatFs;     /* FatFs handle      */
FIL     fil;       /* File handle (reuse for both .JPG and intentos.txt) */
FRESULT fres;

void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}



static uint32_t get_last_attempt(void)
{
    uint32_t last = 0;

    fres = f_open(&fil, ATTEMPT_FILE, FA_READ);
    if (fres == FR_OK) {
        char line[32];
        while (f_gets(line, sizeof line, &fil)) {
            uint32_t n;
            if (sscanf(line, "%lu", &n) == 1) last = n;
        }
        f_close(&fil);
    }   /* else: file not found – treat as zero  */

    return last;
}

/* --- append a number to intentos.txt ----------------------------------- */
static void log_attempt(uint32_t num)
{
    fres = f_open(&fil, ATTEMPT_FILE, FA_OPEN_APPEND | FA_WRITE);
    if (fres == FR_OK) {
        char line[16];
        UINT bw;
        int len = sprintf(line, "\r\n\r\n%lu\r\n", num);
        myprintf(line, "\r\n\r\n%lu\r\n", num);
        f_write(&fil, line, len, &bw);
        f_close(&fil);
    }
}

/* ===== CRC16-CCITT (poly 0x1021, init 0xFFFF) ===== */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ===== UART helpers ===== */
static int uart_send(const void *p, uint16_t n)
{
    return (HAL_UART_Transmit(&LINK_UART_HANDLE, (uint8_t*)p, n, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}
static bool uart_try_read(uint8_t *ch)  // non-blocking single byte
{
    return (HAL_UART_Receive(&LINK_UART_HANDLE, ch, 1, 0) == HAL_OK);
}

/* ===== ACK/NAK helper ===== */
static int send_ack(uint16_t seq, uint8_t code)
{
    uint8_t a[3] = { code, (uint8_t)seq, (uint8_t)(seq >> 8) };
    return uart_send(a, 3);
}

/* ===== Detect "SZ + size" header without blocking main loop ===== */
static bool detect_header_nonblocking(uint32_t *out_size)
{
    static uint8_t state = 0;
    static uint8_t sizebuf[4];
    static uint8_t idx = 0;
    uint8_t ch;

    //myprintf("Entro detect header \r\n");
    // Slurp any available bytes quickly
    while (uart_try_read(&ch)) {
        switch (state) {
        case 0:  state = (ch == 'S') ? 1 : 0; break;
        case 1:  state = (ch == 'Z') ? 2 : 0; idx = 0; break;
        case 2:
            sizebuf[idx++] = ch;
            if (idx == 4) {
                *out_size = (uint32_t)sizebuf[0]
                          | ((uint32_t)sizebuf[1] << 8)
                          | ((uint32_t)sizebuf[2] << 16)
                          | ((uint32_t)sizebuf[3] << 24);
                //uint16_t seq, len;
                //send_ack(seq, ACK);

                uint8_t ack = ACK; // tell sender we're ready
                //int e = uart_send(&ack, 1);
                int e = HAL_UART_Transmit(&LINK_UART_HANDLE, &ack, 1, HAL_MAX_DELAY);
                state = 0;
                myprintf("envio fue = %d \r\n", e);
                //myprintf("El envio fue 0x%02X\r\n", (unsigned)e);
                return true;

            }
            break;
        }
    }
    return false;
}

/* ===== Receive one frame (blocking small reads) ===== */
static int recv_frame(uint16_t *seq, uint8_t *payload, uint16_t *len)
{
    uint8_t b;

    // Hunt SOF
    do {
        if (HAL_UART_Receive(&LINK_UART_HANDLE, &b, 1, UART_TMO_MS) != HAL_OK) return -1;
    } while (b != SOF0);
    if (HAL_UART_Receive(&LINK_UART_HANDLE, &b, 1, UART_TMO_MS) != HAL_OK || b != SOF1) return -2;

    uint8_t hdr[4];
    if (HAL_UART_Receive(&LINK_UART_HANDLE, hdr, 4, UART_TMO_MS) != HAL_OK) return -3;

    uint16_t seq_le = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    uint16_t l      = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);
    if (l == 0 || l > CHUNK) return -4;

    if (HAL_UART_Receive(&LINK_UART_HANDLE, payload, l, UART_TMO_MS) != HAL_OK) return -5;

    uint8_t crcb[2];
    if (HAL_UART_Receive(&LINK_UART_HANDLE, crcb, 2, UART_TMO_MS) != HAL_OK) return -6;
    uint16_t rxcrc = (uint16_t)crcb[0] | ((uint16_t)crcb[1] << 8);

    uint16_t calcc = crc16_ccitt(payload, l);
    if (rxcrc != calcc) { *seq = seq_le; return -7; }

    *seq = seq_le; *len = l;
    return 0;
}

/* ===== Receive file AFTER header has been detected ===== */
int receive_frames_after_header(const char *save_path, uint32_t expected_size)
{
    FRESULT fr;
    FATFS   fs;
    FIL     f;
    UINT    bw;
    static uint8_t buf[CHUNK];


    //MX_FATFS_Init();
    //fr = f_mount(&fs, "", 1); if (fr != FR_OK) return -100;

    fr = f_open(&f, save_path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) { f_mount(NULL, "", 0); return -101; }

    uint32_t received = 0;
    uint16_t expect_seq = 0;
    myprintf("Abrio, antes de while  \r\n");
    //uint16_t seq, len;
    //send_ack(seq, ACK);





    while (received < expected_size) {

        uint16_t seq, len;
        //send_ack(seq, ACK);

        int r = recv_frame(&seq, buf, &len);
        myprintf("R = %d \r\n", r);
        HAL_Delay(500);
        if (r == 0 && seq == expect_seq) {
        	//myprintf("Recibio x3 \r\n");
            fr = f_write(&f, buf, len, &bw);
            if (fr != FR_OK || bw != len) {
            	myprintf("F \r\n");
                send_ack(seq, NAK);
                f_close(&f); f_mount(NULL,"",0);
                return -102;
            }
            received += len;
            send_ack(seq, ACK);
            expect_seq++;
        }
        /*
        else {
            uint16_t nak_seq = (r == 0) ? expect_seq : seq;
            send_ack(nak_seq, NAK);
            myprintf("Else NAK \r\n");
        }
        */
        else {
            // r == 0 but wrong seq  -> out-of-order/duplicate: NAK that seq
            // r == -7 (CRC fail)    -> we set *seq inside recv_frame: NAK that seq
            // r < 0 before seq known (timeout/parse error): DO NOT NAK
            if (r == 0) {
                send_ack(seq, NAK);
            } else if (r == -7) {
                send_ack(seq, NAK);
            } else {
            	myprintf("No NAK  \r\n");
            	//send_ack(seq, ACK);
                // just wait; don't spam NAKs when no frame was seen yet
                // optional: small delay to avoid hot loop
                //HAL_Delay(1);
            }
        }

        myprintf("received = %lu bytes\r\n", (unsigned long)received);
    }

    f_sync(&f);
    f_close(&f);
    myprintf("termino\r\n", (unsigned long)received);
    f_mount(NULL, "", 0);
    return 0;
}

/* ===== Call this from your main loop ===== */
static bool receiving = false;
static uint32_t pending_size = 0;

void user_loop_receiver(void)
{
    if (!receiving) {
        if (detect_header_nonblocking(&pending_size)) {
            receiving = true;
            myprintf("Recibio encabezado \r\n");
            myprintf("expected_size = %lu bytes\r\n", (unsigned long)pending_size);
            //HAL_Delay(500);
            (void)generar_nombre_unico(filename, sizeof(filename));
            (void)receive_frames_after_header(filename, pending_size); // blocks until done
            receiving = false;
        }
    }
}
/* ================== END H753 RECEIVER ================== */








/* **************************************** Programacion de comandos ********************************************/

/************************************************
 Declaracion de cabeceras, payload y variables
 ************************************************/

uint8_t TA; /*TA=TargetAddress (Address del Payload)*/
uint8_t SA; /*SA=SourceAddress (Address del OBC)*/

/*PPID=PayloadProtocolID
(Define las funciones
 Toma de foto camara 1: id=0x01
 Toma de foto camara 2: id=0x02)*/
uint8_t PPID;

/*PS=PayloadSize (Tamaño del payload)
 PayloadComando= 1-2bytes
 PayloadData= 1-255bytes*/
uint8_t PS;

/*Payload*/
uint8_t payload_total[255];
uint8_t payload_envio[]={};

/*CRC32*/
uint32_t CRC32;

/*Variable*/
uint8_t count;
uint8_t val;

/*Funcion generadora del CRC16 */
uint16_t gener_crc16 (uint8_t TA,uint8_t SA,uint8_t PPID,uint8_t PS){

	 /***************************************************
	  * Definicion de variables y algoritmo para el CRC16
	  * Con inversion de bits
	  * CRC-CCITT-BR (bit reverse): LSB first
	  * CRC16, G(x)=0x1021 (algoritmo)
	  * CRC initial: 0XFFFF
	  * Cabecera inicial: 0x88 0x05 0x01 0x01
	  * Cabecera invertida: 0x80 0x80 0xA0 0x11
	  ***************************************************/
	  uint16_t G_X = 0x1021; // Polynomial generator
	  uint16_t CRC16 = 0xFFFF;
	  uint8_t arreglo[4] = {TA,SA,PPID,PS};//Se aumento PPID,PS
	  uint8_t InvArreglo[4] = {};
	  uint8_t InvBits[4] ={}; //Arreglo de bits invertidos de la cabecera  (LSB first)
	  uint8_t Temp;
	  uint8_t Fin=0x00;
	  uint8_t n = sizeof(arreglo)/sizeof(arreglo[0]);

	  //Se invierten los elementos del array
	  //Cabecera actual: 0x01 0x01 0x05 0x88
	  for(int i=0; i<n; i++){
		  InvArreglo[i]=arreglo[n-i-1];
	  }

	  //InvArreglo[0]=0x35;

	  //Se invierten los bits de los elementos del array
	  //Cabecera LBS: 0x80 0x80 0xA0 0x11
	  for(int j=0; j<n;j++){
		  for(int i=0; i<8; i++){
			  Temp=InvArreglo[j]&(0x80);
			  //InvBits[0]=InvArreglo[1];
			  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
			  //HAL_Delay(1000);
			  if(Temp!=0x00){
				  Fin=(Fin>>1);
				  Fin=Fin|(0x80);
				  InvArreglo[j]=(InvArreglo[j]<<1);
				  // InvBits[0]=Fin;
				  // HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
				  // HAL_Delay(1000);
			  }else{
				  Fin=(Fin>>1);
				  InvArreglo[j]=(InvArreglo[j]<<1);
				  //InvBits[0]=InvArreglo[0];
				  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
				  //HAL_Delay(1000);
			  }
		  }
		  InvBits[j]=Fin;
		  Fin=0x00;
	  }

	  //Algoritmo para la generación del CRC16 de la cabecera invertida
	  for(int i=0; i<sizeof(InvBits)/sizeof(InvBits[0]); i++){
	  	  CRC16 ^= (uint16_t)(InvBits[i]<<8);

	  	  for (int i=0; i<8; i++){
	  		  if ((CRC16 & 0x8000) != 0){
	  			  CRC16 = (uint16_t)((CRC16 <<1)^G_X);
	  		  }else{
	  			  CRC16 <<=1;
	  		  }
	  	  }
	   }

	  return CRC16;

	  // El CRC16 de 0x8805 = 0xC294
	  // El CRC16 de 0x88050101 = 0x901B
	  // El CRC16 de 0x8080A011 (0x88050101 invertido) = 0x7DCC

	  /*uint8_t CRC16_0 = (uint8_t)(CRC16 & 0x00FF);
	  uint8_t CRC16_1 = (uint8_t)((CRC16 >> 8)& 0x00FF);*/
}
/*******************************************************************************************************************/

/*Funcion generadora del CRC32*/
uint32_t gener_crc32 (uint8_t TA,uint8_t SA,uint8_t PPID,uint8_t PS,uint8_t CRC16_1, uint8_t CRC16_0,uint8_t payload_envio_0){

	/***************************************************
	   * Definicion de variables y algoritmo para el CRC32
	   * Con inversion de bits
	   * CRC-CRC32-ISO3309, LSB first
	   * CRC32, G(x)= 0x04C11DB7 (algoritmo)
	   * CRC initial: 0xFFFFFFFF
	   * Prueba inicial: 0x88 05 01 01 7D CC 01
	  **************************************************/
	  uint32_t G_Y = 0x04C11DB7; // Polynomial generator
	  uint32_t CRC32 = 0xFFFFFFFF;
	  uint8_t arreglo2[7]={TA,SA,PPID,PS,CRC16_1,CRC16_0,payload_envio_0};

	  uint8_t InvArreglo2[7] = {};
	  uint8_t InvBits2[7] ={}; //Arreglo de bits invertidos de la cabecera  (LSB first)
	  //uint8_t InvBits3[1]={};
	  uint8_t Temp2;
	  uint8_t Fin2=0x00;
	  uint8_t n2 = sizeof(arreglo2)/sizeof(arreglo2[0]);

	  //Se invierten los elementos del array
	  //Frame actual: 0x01 0xCC 0x7D 0x01 0x01 0x05 0x88
	  for(int i=0; i<n2; i++){
		  InvArreglo2[i]=arreglo2[n2-i-1];
	  }

	  //Se invierten los bits de los elementos del array
	  //Frame LBS: 0x80 0x33 0xBE 0x80 0x80 0xA0 0x11
	  for(int j=0; j<n2;j++){
	  	  for(int i=0; i<8; i++){
	  		  Temp2=InvArreglo2[j]&(0x80);
	    	   	  //InvBits3[0]=InvArreglo2[j];
	    	   	  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	    	   	  //HAL_Delay(1000);
	  		  if(Temp2!=0x00){
	  			  Fin2=(Fin2>>1);
	  			  Fin2=Fin2|(0x80);
	  			  InvArreglo2[j]=(InvArreglo2[j]<<1);
	    	   		  //InvBits3[0]=Fin2;
	    	   		  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	    	   		  //HAL_Delay(1000);
	  		  }else{
	  			  Fin2=(Fin2>>1);
	  			  InvArreglo2[j]=(InvArreglo2[j]<<1);
	  			  //InvBits[0]=InvArreglo[0];
	    	   		  //HAL_UART_Transmit(&huart2,InvBits,sizeof(InvBits),100);// Sending in normal mode
	    	   		  //HAL_Delay(1000);
	  		  }
	  	  }
	  	  InvBits2[j]=Fin2;
	  	  Fin2=0x00;
	  }

	  //InvBits3[0]=InvBits2[6];
	  //HAL_UART_Transmit(&huart2,InvBits3,sizeof(InvBits3),100);// Sending in normal mode
	  //HAL_Delay(1000);

	  //Algoritmo para la generación del CRC32 de la cabecera invertida
	  for(int j=0; j<sizeof(InvBits2)/sizeof(InvBits2[0]); j++){
	  	  CRC32 ^= (uint32_t)(InvBits2[j]<<24);

	  	  for (int j=0; j<8; j++){
	  		  if ((CRC32 & 0x80000000) != 0){
	  			  CRC32 = (uint32_t)((CRC32 <<1)^G_Y);
	  		  }else{
	  			  CRC32 <<=1;
	  		  }
	  	  }
	  }

	  return CRC32;

	  // El CRC32 de 0x88050101 = 0x486ABA9C
	  // El CRC32 de 0x8033BE8080A011 = 0x81C46721
	  /*uint8_t CRC32_byte3 = (uint8_t)((CRC32 >> 24) & 0xFF);//MSB
	  uint8_t CRC32_byte2 = (uint8_t)((CRC32 >> 16) & 0xFF);
	  uint8_t CRC32_byte1 = (uint8_t)((CRC32 >> 8) & 0xFF);
	  uint8_t CRC32_byte0 = (uint8_t)(CRC32 & 0xFF);//LSB*/
}


/*******************************************************************************************************************/







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(2000); //a short delay is important to let the SD card settle

  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	myprintf("f_mount error (%i)\r\n", fres);
	while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	myprintf("f_getfree error (%i)\r\n", fres);
	while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin | LD3_Pin, GPIO_PIN_SET);   // PB0 + PB14
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,          GPIO_PIN_SET);   // PE1

  uint32_t t0 = HAL_GetTick();          /* non-blocking delay loop */
  while (HAL_GetTick() - t0 < 2000U) __NOP();

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,          GPIO_PIN_RESET);




  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin | LD3_Pin, GPIO_PIN_SET);   // PB0 + PB14
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,          GPIO_PIN_SET);   // PE1

  while (HAL_GetTick() - t0 < 2000U) __NOP();

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,          GPIO_PIN_RESET);


 // HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uartRxBuf, sizeof(uartRxBuf));
 // __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);   // we only need TC & IDLE
  myprintf("Ready waiting for 0x55 0xAA \r\n");
  HAL_Delay(1000);

  /*
  myprintf("Prueba_wardardo de imagen con h \r\n");

  f_open(&fil, "glolgoglgo.jpg", FA_WRITE | FA_CREATE_ALWAYS);
  UINT bw;

  f_write(&fil, IMG_20250425_000083, IMG_20250425_000083_SIZE, &bw);
  f_close(&fil);
  myprintf("FINALIZADO \r\n");'
  */

  /***********************************  Definicion de variables para el comandos ***********************************/

  /**************************
   Definicion de cabeceras I
  **************************/
  TA=0x88;
  SA=0x05;
  PPID=0x01;

  /**********************************
   Definicion payload de envío
   Comando de foto en camara 1: 0x01
  **********************************/
  switch(PPID){
   case 0x01:
  	payload_total[0]=0x01; /*TomaFotoSD*/
  	break;
   case 0x02:
  	payload_total[0]=0x02; /*EnvíoFotoPaySTM32*/
    break;
   case 0x03:
    payload_total[0]=0x03; /*TomaFotoALmacYenviaPayload*/
    break;
   default:
    payload_total[0]=0xFF;
  }

 /***************************
   Definicion de cabeceras II
  **************************/

  /*Define la cantidad de bytes (size) del payload*/
  for (int i=0; i<255; i++){
	  val = payload_total[i] & 0b11111111;
	  if (val != 0x00){
		  count=count+1;
	  }
  }

  /*cumple con darnos la cantidad de bytes(size) del payload*/
  PS=count;

  /*for(int i=0; i<count; i++){
   	   payload_envio[i]=payload_total[i];
  }*/
  payload_envio[0]=payload_total[0];

  /******************************
   * Hallando el CRC16 y CRC32
   ******************************/
  uint16_t CRC16 = gener_crc16(TA,SA,PPID,PS);

  uint8_t CRC16_0 = (uint8_t)(CRC16 & 0x00FF);
  uint8_t CRC16_1 = (uint8_t)((CRC16 >> 8)& 0x00FF);

  uint32_t CRC32 = gener_crc32(TA,SA,PPID,PS,CRC16_1,CRC16_0,payload_envio[0]);

  uint8_t CRC32_byte3 = (uint8_t)((CRC32 >> 24) & 0xFF);//MSB
  uint8_t CRC32_byte2 = (uint8_t)((CRC32 >> 16) & 0xFF);
  uint8_t CRC32_byte1 = (uint8_t)((CRC32 >> 8) & 0xFF);
  uint8_t CRC32_byte0 = (uint8_t)(CRC32 & 0xFF);//LSB



  /************************************************************************************************
   Armado de Protocolo |TA|SA|PPID|PS|CRC16_1|CRC16_0|Payload[0]|CRC32_3|CRC32_2|CRC32_1|CRC32_0|
   y envio por serial
   Cabecera 1: 0x88 0x05 0x01 0x01 0xCC 0x7D 0x01 0x21 0x67 0xC4 0x81
  ************************************************************************************************/
  //Para la web:
  //uint8_t cab[] = {TA,SA,PPID,PS,CRC16_1,CRC16_0,payload_envio[0],CRC32_byte3,CRC32_byte2,CRC32_byte1,CRC32_byte0};
  //Para Enduro: LSB primero
  uint8_t cab[] = {TA,SA,PPID,PS,CRC16_0,CRC16_1,payload_envio[0],CRC32_byte0,CRC32_byte1,CRC32_byte2,CRC32_byte3};

  myprintf("Enviando comando ... \n");
  //myprintf("%11X \n",cab);
  //uint8_t cab[] = {CRC32_byte3,CRC32_byte2,CRC32_byte1,CRC32_byte0};
  HAL_UART_Transmit(&huart2,cab,11,2500);// Sending in normal mode
  HAL_UART_Transmit(&huart7,cab,11,2500);// Sending in normal mode
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //user_loop_receiver();
	  //HAL_UART_Transmit(&huart7,cab,11,2500);// Sending in normal mode
	  //HAL_Delay(300);




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 2400;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
