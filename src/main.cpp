#include <Arduino.h>

#define DUMP_RAW 1
#define DUMP_HEX 2
#define ADDRESS_BUS_WIDTH   17
#define DATA_BUS_WIDTH       8

// PIN Definition
#define WE  12 // Write Enable
#define CE  11 // Chip Enable 
#define OE  10 // Output Enable
#define LED 13 // Led

// Pinos de Endereço e Dados
// A0 - A16
const int address_pins[ADDRESS_BUS_WIDTH] = { 37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53 };
// DQ0 - DQ7
const int dq_pins[DATA_BUS_WIDTH] = { 22,23,24,25,26,27,28,29 };

// Configuração da EEPROM
// TAMANHO DA PÁGINA da EEPROM
int PAGE_SIZE = 128;

// ULTIMO ENDEREÇO DA EEPROM
unsigned long END_ADDRESS = 0x20000;

// Drive the address bus with a specified value
void set_address(unsigned long addr) {  
  for (int i=0; i < ADDRESS_BUS_WIDTH; i++) {
    digitalWrite(address_pins[i], (addr & (1UL << i)) ? HIGH : LOW);
  }
}

// Drive the data bus with a specified value
void drive_data(byte data) {  
  for (int i=0; i < DATA_BUS_WIDTH; i++) {
    pinMode(dq_pins[i], OUTPUT);
    digitalWrite(dq_pins[i], (data & (1UL << i)) ? HIGH: LOW);
  }
}

// Set the data bus to tristate (undriven)
void tristate_data_pins(void) {
  
  for (int i=0; i < DATA_BUS_WIDTH; i++) {
    pinMode(dq_pins[i], INPUT);
  }
}


// Return the 8-bit value on the data bus
byte read_data(void) {
  byte val = 0;

  for (int i=0; i < DATA_BUS_WIDTH; i++) {
    if (digitalRead(dq_pins[i])) {
      val |= (1 << i);
    }
  }  
  return val;
}


// Perform a single read cycle on the bus and return the result.
// This implementation runs at Teensy bus speed, except for a
// single delay to allow for the memory device's access time.
byte bus_read_cycle(unsigned long address) {
  byte value;

  set_address(address);
  tristate_data_pins();
  digitalWrite(OE, LOW);
  digitalWrite(CE, LOW);
  delayMicroseconds(1);       // wait for the slow memory devices
  value = read_data();        // get the result
  digitalWrite(OE, HIGH);
  digitalWrite(CE, HIGH);

  return value;
}


// Perform a single write cycle on the bus.
void bus_write_cycle(unsigned long address, byte value) {

  set_address(address);
  digitalWrite(OE, HIGH);
  drive_data(value);
  digitalWrite(CE, LOW);
  digitalWrite(WE, LOW);
  digitalWrite(WE, HIGH);
  digitalWrite(CE, HIGH);
}


// Send a command to the device
void device_command(byte command) {
  bus_write_cycle(0x5555, 0xAA);
  bus_write_cycle(0x2AAA, 0x55);
  bus_write_cycle(0x5555, command);
}


// Tenta Realizar a Leitura do ID do Dispostivo
void identify_device(void) {
  byte family, device;

  digitalWrite(LED, HIGH);           // LED indicator that something is happening
 
  device_command(0x90);
  tristate_data_pins();
  set_address(0x0000);

  delayMicroseconds(10);            // T(IDA)

  // Not this. Not conventional bus reads. (?)
  //family = bus_read_cycle(0x0000);
  //device = bus_read_cycle(0x0001);

  digitalWrite(CE, LOW);
  digitalWrite(OE, LOW);

  delayMicroseconds(1);
  family = read_data();       // after T(AA)

  set_address(0x0001);

  delayMicroseconds(1);
  device = read_data();       // after T(AA)

  digitalWrite(CE, HIGH);

  // Exit product ID read mode
  device_command(0xF0);
  delayMicroseconds(10);            // T(IDA)
  digitalWrite(CE, LOW);
  delayMicroseconds(10);            // unspecified delay
  digitalWrite(CE, HIGH);           // return to idle state (not shown in timing diagram)
  
  digitalWrite(LED, LOW);

  Serial.print("DEVID:");
  Serial.print(family, HEX);
  Serial.print(" ");
  Serial.println(device, HEX);
}


// Envia o Comando para Apagar Toda a Memoria EEPROM
void erase_device(void) {
  digitalWrite(LED, HIGH);
  device_command(0x80);
  device_command(0x10);
  delay(20);      // T(SCE)
  Serial.println("ERASED");
  digitalWrite(LED, LOW);
}


/*
* Entrada no Processo de Programar o Dispositivo
*
*/
void program_device(void) {
  unsigned long block_address;
  uint8_t *buffer;
  int i;

  Serial.println("START_PROGRAM");

  for (block_address=0; block_address < END_ADDRESS; block_address += PAGE_SIZE) { // Navega entre os endereços disponiveis , pulando sempre o tamanho da página
    Serial.print("A:"); 
    Serial.println(block_address, HEX); // Solicita os Dados do Endereço para o Controlador

    // Realiza a Leitura da Página Inteiro para o BUFFER
    while (Serial.available() == 0)
    {
      delay(1); // Espera ter dados Disponiveis
    }
    
    Serial.readBytes(buffer, PAGE_SIZE); // Le a Página Inteira da Serial

    Serial.println("W"); // Informa que esta escrevendo na EEPROM
    
    device_command(0xA0);     // Desabilita o Write Protect da EEPROM 

    digitalWrite(LED, HIGH);

    for (i=0; i < PAGE_SIZE; i++) {
      bus_write_cycle(block_address+i, buffer[i]); // Escreve todos os Bytes da Página
    }

    delayMicroseconds(200);     // T(BLCO) 

    // Toggle Bit Polling method:
    // Keep reading the last address written until it comes back twice the same.
    // Only bit 6 is toggling, but it's easy to compare the whole byte.
    while (bus_read_cycle(block_address+127) != bus_read_cycle(block_address+127));

    digitalWrite(LED, LOW);

    Serial.println("F"); // Indica que finalizou o endereço..
  }

  Serial.println("END_PROGRAM");
}

/*
* Realiza o Dump do Conteudo da EEPROM, ou seja, copia todos os endereços da memória para o Serial
*/
void dump_device(int mode) {
  unsigned long addr;
  digitalWrite(LED, HIGH);
  int device_byte;

  for (addr=0; addr < END_ADDRESS; addr++) {
    device_byte = bus_read_cycle(addr);
    if(mode == DUMP_HEX) {
      Serial.print(device_byte, HEX);
      Serial.print(' ');
    } else if(mode == DUMP_RAW) {
      Serial.write(device_byte);
    }
  }  

  Serial.println();
  digitalWrite(LED, LOW);  
}

void set_device_settings(String args) {
  PAGE_SIZE = args.substring(0, args.indexOf(':')).toInt();
  long pages = args.substring(args.indexOf(':') + 1).toInt();
  
  Serial.print("CFG:");
  Serial.print(PAGE_SIZE);
  Serial.print(':');
  Serial.print(pages);
  
  long maxMemory = (((long)PAGE_SIZE) * pages); // Precisa do Casting para LONG em!
  END_ADDRESS = (unsigned long)maxMemory;

  Serial.print(':');
  Serial.println(maxMemory);
}

void setup() {
  Serial.begin(9600); 
  Serial.println("29EE010 EEPROM Programmer 0.1");
  
  // TORNA os PINOS LIGADOS ao DQ0-DQ7 da EEPROM SAIDA (E DRIVEN)
  // Ou seja passamos a controlar esses pinos e ele não ficam flutuando.
  for (int i=0; i < ADDRESS_BUS_WIDTH; i++) {
    pinMode(address_pins[i], OUTPUT);
  }

  // Setamos os Pinos WE/CE/OE para ALTO desabilitando tudo.
  pinMode(WE, OUTPUT);  digitalWrite(WE, HIGH);
  pinMode(CE, OUTPUT);  digitalWrite(CE, HIGH);
  pinMode(OE, OUTPUT);  digitalWrite(OE, HIGH);
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {    
  if(Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n'); // Busca o Proximo Comando

      if(cmd.equals("GETID")) {  // BUSCA O ID
        identify_device();
      } 
      else if(cmd.equals("DUMP:RAW")) { // REALIZA O DUMP DA EEPROM SEM CONVERSAO
        dump_device(DUMP_RAW);
      }
      else if(cmd.equals("DUMP:HEX")) { // REALIZA O DUMP DA EEPROM como HEX
        dump_device(DUMP_HEX);
      }
      else if(cmd.equals("CLEAR")) { // LIMPA A EEPROM
        erase_device();
      }
      else if(cmd.equals("PROGRAM")) { // PROGRAMA A EEPROM
        program_device();
      }
      else if(cmd.startsWith("SET")) {
        set_device_settings(cmd.substring(4));
      }
  }

  delay(100);
}
