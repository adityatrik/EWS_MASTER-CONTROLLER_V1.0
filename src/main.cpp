// =================================================== Memasukkan Library Program ======================================================== //
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <ModbusRtu.h>
#include <PCF8574.h>
// =============================================== Akhir Memasukkan Library Program ======================================================= //

// ======================================================== Memasukkan Variabel =========================================================== //
#define ADDR_LED_BUTTON 0x21
#define ADDR_SW_BUTTON 0x23
#define ADDR_LED_INDIKATOR 0x20

PCF8574 IO_LED_BUTTON(ADDR_LED_BUTTON);
PCF8574 IO_SW_BUTTON(ADDR_SW_BUTTON);
PCF8574 IO_LED_INDIKATOR(ADDR_LED_INDIKATOR);

byte
    FAST_PIN = 0,
    SKIP_PIN = 1,
    FASE_PIN = 2,
    DISPLAY_PIN = 3,
    SLOW_PIN = 4,
    HOLD_PIN = 5,
    FLASH_PIN = 6;

byte
    TICK_CW = 0,
    TICK_CCW = 0,
    TICK_ZERO = 0,
    t_index = 0,
    valTombol1 = 0,
    valTombol2 = 0,
    valTombol3 = 0,
    pinLedButton1 = PD13,
    pinLedButton2 = PD12,
    pinLedButton3 = PD11,
    pinSw1 = PF1, // Button - Selector Mode Manual / Auto
    pinSw2 = PF2, // Button - Selector Manual CW
    pinSw3 = PF4, // Button - Selector Manual CCW
    pinLED1 = PA3,
    pinLED2 = PA4,
    pinLED3 = PA5,
    tick = 0,
    detik = 0,
    menit = 0,
    jam = 0;

uint8_t
    u8state, // MODBUS - Index array pool
    u8query; // MODBUS - Index array sending

int
    gearPosition = 0,
    motorMode = 2,        // Motor - 0 = Manual || 1 = Auto
    motorEn = 1,          // Motor - 0 = Disable || 1 = Enable
    motorVelocity = 1000; // Motor - Speed 1-3000

int
    data_excitation[] = {0, 1},
    data_direct1[] = {48, 0, 0, 1000, 1000, 3000, 2000, 1},   // Input 1, RPM 1000, CW
    data_direct2[] = {48, 0, 0, 3000, 1000, 1000, 1000, 1},   // Input 2, RPM 3000, CW
    data_direct3[] = {48, 0, -1, -1000, 1000, 1000, 1000, 1}, // Input 3, RPM 1000, CCW
    data_direct4[] = {48, 0, -1, -3000, 1000, 1000, 1000, 1}, // Input 4, RPM 3000, CCW
    data_direct5[] = {0, 0, 0, 0, 1000, 1000, 1000, 1},       // Input 5, Stop
    start_address_encoder = 9,
    start_address_excitation[] =
        {
            124,
            125},
    start_address_direct_data[] =
        {
            90,  // Jenis data operasi
            91,  // Jenis data operasi
            92,  // Kecepatan awal
            93,  // Kecepatan awal
            94,  // Pengaturan kecepatan (RPM)
            95,  // Pengaturan kecepatan (RPM)
            96,  // Kecepatan akselrasi (Ms)
            97,  // Kecepatan akselrasi (Ms)
            98,  // Kecepatan deselrasi (Ms)
            99,  // Kecepatan deselrasi (Ms)
            100, // Torsi limit (1 = 0.1%)
            101, // Torsi limit (1 = 0.1%)
            102, // Direct data trigger (default 1)
            103  // Direct data trigger (default 1)
},
    start_address_dataspeed[] =
        {
            200,
            201,
            206,
            207,
},
    start_address_torque[] =
        {
            214,
            215},
    start_address_temperature[] =
        {
            250,
            251},
    start_address_position[] =
        {
            154,
            155};
// ====================================================== Akhir Memasukkan Variabel ======================================================= //

// ==================================================== Inisiasi Hardware Serial ========================================================== //
HardwareSerial UART_RS485_1(PC11, PC10);
HardwareSerial UART_RS485_2(PD6, PD5);
HardwareSerial UART_RS485_3(PB7, PB6);
Modbus master(0, UART_RS485_1, 1);
Modbus master_encoder(0, UART_RS485_3, 1);
modbus_t telegram[50];
uint16_t au16data[50];
// ================================================= Akhir Inisiasi Hardware Serial ======================================================= //

// ============================================================== Inisiasi RTOS ========================================================== //
void setupTelegram()
{
  for (byte i = 0; i < 2; i++)
  {
    telegram[i].u8id = 1;
    telegram[i].u8fct = 6;
    telegram[i].u16RegAdd = start_address_excitation[i];
    telegram[i].u16CoilsNo = 1;
    telegram[i].au16reg = au16data + i;
  }
  for (byte i = 2; i < 16; i++)
  {
    telegram[i].u8id = 1;
    telegram[i].u8fct = 6;
    telegram[i].u16RegAdd = start_address_direct_data[i - 2];
    telegram[i].u16CoilsNo = 1;
    telegram[i].au16reg = au16data + i;
  }
  for (byte i = 16; i < 20; i++)
  {
    telegram[i].u8id = 1;
    telegram[i].u8fct = 3;
    telegram[i].u16RegAdd = start_address_dataspeed[i - 16];
    telegram[i].u16CoilsNo = 1;
    telegram[i].au16reg = au16data + i;
  }
  for (byte i = 20; i < 22; i++)
  {
    telegram[i].u8id = 1;
    telegram[i].u8fct = 3;
    telegram[i].u16RegAdd = start_address_torque[i - 20];
    telegram[i].u16CoilsNo = 1;
    telegram[i].au16reg = au16data + i;
  }
  telegram[22].u8id = 1;
  telegram[22].u8fct = 3;
  telegram[22].u16RegAdd = start_address_encoder;
  telegram[22].u16CoilsNo = 1;
  telegram[22].au16reg = au16data + 22;
  for (byte i = 23; i < 25; i++)
  {
    telegram[i].u8id = 1;
    telegram[i].u8fct = 3;
    telegram[i].u16RegAdd = start_address_position[i - 23];
    telegram[i].u16CoilsNo = 1;
    telegram[i].au16reg = au16data + i;
  }
}
// =========================================================== Akhir Inisiasi RTOS ======================================================= //

// ============================================================== Inisiasi RTOS ========================================================== //
void TaskSerial(void *pvParameters);
void TaskProses(void *pvParameters);
// =========================================================== Akhir Inisiasi RTOS ======================================================= //

// ============================================================== SETUP PROGRAM ========================================================== //
void setup()
{
  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
  pinMode(pinLED3, OUTPUT);
  pinMode(pinLedButton1, OUTPUT);
  pinMode(pinLedButton2, OUTPUT);
  pinMode(pinLedButton3, OUTPUT);
  pinMode(pinSw1, INPUT);
  pinMode(pinSw2, INPUT);
  pinMode(pinSw3, INPUT);
  digitalWrite(pinLED1, HIGH);
  digitalWrite(pinLED2, HIGH);
  digitalWrite(pinLED3, HIGH);

  UART_RS485_1.begin(9600, SERIAL_8E1);
  UART_RS485_2.begin(9600, SERIAL_8N1);
  UART_RS485_3.begin(9600, SERIAL_8N1);
  UART_RS485_2.println("EWS Master Controller V1");

  Wire.setSCL(PB_8);
  Wire.setSDA(PB_9);
  Wire.begin();
  IO_LED_BUTTON.begin();
  IO_SW_BUTTON.begin();
  IO_LED_INDIKATOR.begin();

  master.start();
  master.setTimeOut(2000);
  master_encoder.start();
  master_encoder.setTimeOut(2000);
  u8state = u8query = 0;
  setupTelegram();

  xTaskCreate(
      TaskSerial, (const portCHAR *)"Serial" // A name just for humans
      ,
      1000 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 8 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL);

  xTaskCreate(
      TaskProses, (const portCHAR *)"Proses" // A name just for humans
      ,
      1500 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 5 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL);

  vTaskStartScheduler();
  while (1)
    ;
}
// =========================================================== Akhir SETUP PROGRAM ======================================================= //

void loop()
{
  // put your main code here, to run repeatedly:
}

// ============================================================== SERIAL TASK PROGRAM ========================================================== //
void TaskSerial(void *pvParameters)
{
  (void)pvParameters;
  int tickSerial = 0;
  for (;;)
  {
    tickSerial++;
    if (tickSerial >= 1000)
    {
      IO_LED_INDIKATOR.toggle(0);
      tickSerial = 0;
    }
    master_encoder.query(telegram[22]);
    master.poll();
    master_encoder.poll();
    if (master.getState() == COM_IDLE)
    {
    }
    if (master_encoder.getState() == COM_IDLE)
    {
      if (gearPosition != au16data[22])
      {
        UART_RS485_2.println(au16data[22]);
        gearPosition = au16data[22];
      }
    }
    valTombol1 = IO_SW_BUTTON.readButton(FASE_PIN);
    valTombol2 = IO_SW_BUTTON.readButton(SLOW_PIN);
    valTombol3 = IO_SW_BUTTON.readButton(SKIP_PIN);
    // valTombol1 = digitalRead(pinSw1);
    // valTombol2 = digitalRead(pinSw2);
    // valTombol3 = digitalRead(pinSw3);
    if (valTombol1 == 0)
    {
      motorMode = 0;
      IO_LED_BUTTON.write(FASE_PIN, LOW);
      IO_LED_INDIKATOR.write(5, LOW);
    }
    else
    {
      motorMode = 1;
      IO_LED_BUTTON.write(FASE_PIN, HIGH);
      IO_LED_INDIKATOR.write(5, HIGH);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
// ========================================================== AKHIR SERIAL TASK PROGRAM ====================================================== //

// ============================================================== PROSES TASK PROGRAM ========================================================== //
void TaskProses(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    tick++;
    if (tick % 5 == 0)
    {
      // IO_LED_INDIKATOR.toggle(3);
      // ========================================================== PENGULANGAN 100 MILIDETIK ============================================================== //
      if (motorEn == 1)
      {
        switch (t_index)
        {
        case 0:
          au16data[1] = data_excitation[1];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 1:
          au16data[3] = data_direct1[0];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 2:
          au16data[5] = data_direct1[1];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 1;
          break;

        case 3:
          au16data[6] = data_direct1[2];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 1;
          break;
        case 4:
          au16data[7] = data_direct1[3];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 5:
          au16data[9] = data_direct1[4];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 6:
          au16data[11] = data_direct1[5];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 7:
          au16data[13] = data_direct1[6];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 8:
          au16data[15] = data_direct1[7];
          master.query(telegram[u8query]);
          if (motorMode == 0)
          {
            if ((valTombol2 == 0) && (valTombol3 == 1))
            {
              if (TICK_CCW < 12)
              {
                IO_LED_BUTTON.write(SLOW_PIN, LOW);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(6, LOW);
                IO_LED_INDIKATOR.write(7, HIGH);
                data_direct1[2] = -1;
                data_direct1[3] = -300;
                motorEn = 1;
              }
              else
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(6, HIGH);
                IO_LED_INDIKATOR.write(7, HIGH);
                motorEn = 0;
              }
            }
            else if ((valTombol2 == 1) && (valTombol3 == 0))
            {
              if (TICK_CW < 12)
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, LOW);
                IO_LED_INDIKATOR.write(6, HIGH);
                IO_LED_INDIKATOR.write(7, LOW);
                data_direct1[2] = 0;
                data_direct1[3] = 300;
                motorEn = 1;
              }
              else
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(6, HIGH);
                IO_LED_INDIKATOR.write(7, HIGH);
                motorEn = 0;
              }
            }
            else
            {
              IO_LED_BUTTON.write(SLOW_PIN, HIGH);
              IO_LED_BUTTON.write(SKIP_PIN, HIGH);
              IO_LED_INDIKATOR.write(6, HIGH);
              IO_LED_INDIKATOR.write(7, HIGH);
              motorEn = 0;
            }
          }
          t_index = 0;
          u8query = 1;
          break;
        }
      }
      if (motorEn == 0)
      {
        switch (t_index)
        {
        case 0:
          au16data[1] = data_excitation[1];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 1:
          au16data[3] = data_direct5[0];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 2:
          au16data[5] = data_direct5[1];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 1;
          break;

        case 3:
          au16data[6] = data_direct5[2];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 1;
          break;

        case 4:
          au16data[7] = data_direct5[3];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 5:
          au16data[9] = data_direct5[4];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 6:
          au16data[11] = data_direct5[5];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 7:
          au16data[13] = data_direct5[6];
          master.query(telegram[u8query]);
          t_index++;
          u8query = u8query + 2;
          break;

        case 8:
          au16data[15] = data_direct5[7];
          master.query(telegram[u8query]);
          if (motorMode == 0)
          {
            if ((valTombol2 == 0) && (valTombol3 == 1))
            {
              if (TICK_CCW < 12)
              {
                IO_LED_BUTTON.write(SLOW_PIN, LOW);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(6, LOW);
                IO_LED_INDIKATOR.write(7, HIGH);
                data_direct1[2] = -1;
                data_direct1[3] = -300;
                motorEn = 1;
              }
              else
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(6, HIGH);
                IO_LED_INDIKATOR.write(7, HIGH);
                motorEn = 0;
              }
            }
            else if ((valTombol2 == 1) && (valTombol3 == 0))
            {
              if (TICK_CW < 12)
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, LOW);
                IO_LED_INDIKATOR.write(6, HIGH);
                IO_LED_INDIKATOR.write(7, LOW);
                data_direct1[2] = 0;
                data_direct1[3] = 300;
                motorEn = 1;
              }
              else
              {
                IO_LED_BUTTON.write(SLOW_PIN, HIGH);
                IO_LED_BUTTON.write(SKIP_PIN, HIGH);
                IO_LED_INDIKATOR.write(7, HIGH);
                IO_LED_INDIKATOR.write(6, HIGH);
                motorEn = 0;
              }
            }
            else
            {
              IO_LED_BUTTON.write(SLOW_PIN, HIGH);
              IO_LED_BUTTON.write(SKIP_PIN, HIGH);
              IO_LED_INDIKATOR.write(7, HIGH);
              IO_LED_INDIKATOR.write(6, HIGH);
              motorEn = 0;
            }
          }
          t_index = 0;
          u8query = 1;
          break;
        }
      }
      // ======================================================= AKHIR PENGULANGAN 100 MILIDETIK ============================================================ //
    }

    if (tick >= 100)
    {
      tick = 0;
      detik++;
      // ========================================================== PENGULANGAN 1 DETIK ============================================================== //
      digitalToggle(pinLED1);
      IO_LED_INDIKATOR.toggle(1);
      if (motorMode == 0)
      {
        if (motorEn == 1)
        {
          if (data_direct1[2] == 0)
          {
            TICK_CW++;
            if (TICK_CCW != 0)
            {
              TICK_CCW--;
            }
            if (TICK_CW >= 12)
            {
              t_index = 0;
              motorEn = 0;
            }
          }
          else
          {
            TICK_CCW++;
            if (TICK_CW != 0)
            {
              TICK_CW--;
            }
            if (TICK_CCW >= 12)
            {
              t_index = 0;
              motorEn = 0;
            }
          }
        }
      }
      else if (motorMode == 1)
      {
        if (motorEn == 1)
        {
          if (data_direct1[2] == 0)
          {
            TICK_CW++;
            if (TICK_CW >= 12)
            {
              motorEn = 0;
              TICK_ZERO = 0;
            }
          }
          else
          {
            TICK_CCW++;
            if (TICK_CCW >= 12)
            {
              motorEn = 0;
              TICK_ZERO = 0;
            }
          }
        }
        else
        {
          TICK_ZERO++;
          if (TICK_ZERO >= 2)
          {
            if (data_direct1[2] == 0)
            {
              data_direct1[2] = -1;
              data_direct1[3] = -300;
              TICK_CCW = 0;
              t_index = 0;
              motorEn = 1;
            }
            else
            {
              data_direct1[2] = 0;
              data_direct1[3] = 300;
              TICK_CW = 0;
              t_index = 0;
              motorEn = 1;
            }
          }
        }
      }
      // UART_RS485_2.println("===================================");
      // UART_RS485_2.println("CW Index: " + String(TICK_CW));
      // UART_RS485_2.println("CCW Index: " + String(TICK_CCW));
      // UART_RS485_2.println("STOP Index: " + String(TICK_ZERO));
      // UART_RS485_2.println("===================================");
      // UART_RS485_2.println();

      // ======================================================= AKHIR PENGULANGAN 1 DETIK ============================================================ //
      if (detik >= 60)
      {
        detik = 0;
        menit++;
        // ========================================================== PENGULANGAN 1 MENIT ============================================================== //

        // ======================================================= AKHIR PENGULANGAN 1 MENIT ============================================================ //
        if (menit >= 60)
        {
          menit = 0;
          jam++;
          // ========================================================== PENGULANGAN 1 JAM ============================================================== //

          // ======================================================= AKHIR PENGULANGAN 1 JAM ============================================================ //
          if (jam >= 24)
          {
            jam = 0;
            // ========================================================== PENGULANGAN 1 HARI ============================================================== //

            // ======================================================= AKHIR PENGULANGAN 1 HARI ============================================================ //
          }
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
// ========================================================== AKHIR PROSES TASK PROGRAM ====================================================== //
