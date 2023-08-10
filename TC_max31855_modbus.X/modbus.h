/*******************************************************************************
 * K�t�phane     : ModbusRTU Slave K�t�phanesi                                 *                 *
 * Yazar         : sigmoid                                                     *
 * Ba�lang��     : 1 A�ustos 2023                                              *
 * Versiyon      : 0.1                                                         *
 ******************************************************************************/

#define MODBUS_ADRES     2 // Modbus slave adresi
#define usart_hiz       9600
#define frame_ara_suresi    (1000000UL/(usart_hiz*10)) * 3.5F
#define GELEN_BUFFER_SIZE 128
#define FRAME_ADRES_SIZE  16     //buffer ayn� anda ka� frame saklayabilece�i

#define MB_FRAME_UZUNLUGU   64  // bir modbus frame max uzunlu�u

#define INPUT_DISCRETE_REG_SIZE 16      //Giri� biti adeti.
#define INPUT_DISCRETE_START_ADR 8      //giri� biti ba�lang�� adresi, bir nevi dizi i�in offset dizi[0] asl�nda START_ADDR oluyor.
#define COIL_REG_SIZE   16              //��k�� bobini adeti
#define COIL_REG_START_ADR  0           //��k�� bobini Ba�lang�� adresi
#define INPUT_REG_SIZE  16               //Giri� register� adeti 16bitlik
#define INPUT_REG_START_ADR 0           //Giri� register� ba�lang�� adresi
#define HOLDING_REG_SIZE    64         //Holding register adeti 16bitlik
#define HOLDING_REG_START_ADR   0       //Holding register ba�lang�� adresi

#define MB_DEBUG        //debug bilgilerini usart2 den yaz..


#define ModbusTMR_SetInterruptHandler(fonk) TMR3_SetInterruptHandler(fonk)
#define ModbusTMR_Start() TMR3_StartTimer()
#define ModbusTMRReloadVal timer3ReloadVal


uint8_t gelen_veri_pop();
void gelen_veri_push(uint8_t veri);
uint8_t frame_baslangic_pop();
void frame_baslangic_push(uint8_t adres);

void ModbusTMR_Clear();
void modbus_gelenFrame_Yazdir();
void modbus_data_clear();
void modbus_cevap_clear();

void modbus_gelen_aktif();
void modbus_giden_aktif();
//void modbus_gelen();
//void modbus_decode();
//void modbus_oninceleme();
void modbus_process();

void modbus_init();

void hex_yazdir(uint8_t veri);
void coil_durum_goster();
void usart2_yaz(char * t);

void mb_read_coils();
void mb_read_discrete_inputs();   
void mb_read_holding_registers();
void mb_read_input_registers();
void mb_write_single_coil();
void mb_write_single_register();
void mb_write_multiple_coils(); 
void mb_write_multiple_registers();

