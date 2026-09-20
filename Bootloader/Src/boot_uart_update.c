/*
 * OFW1 framed UARTを受信し、CRC検証済みimageをinactive slotへ書いてpending起動へ切り替える。
 */
#include "boot_uart_update.h"

#include "boot_crc32c.h"
#include "boot_flash.h"
#include "boot_image.h"
#include "stm32g474xx.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UART_HEADER_SIZE 12U
#define UART_TRAILER_SIZE 4U
#define UART_MAX_PAYLOAD 920U
#define UPDATE_CHUNK_SIZE 896U

enum { MSG_INFO=1, MSG_BEGIN, MSG_CHUNK, MSG_FINALIZE, MSG_REBOOT };
enum { STATUS_OK=0, STATUS_FRAME, STATUS_RANGE, STATUS_SEQUENCE, STATUS_CRC, STATUS_FLASH, STATUS_STATE };

static uint8_t frame[UART_HEADER_SIZE + UART_MAX_PAYLOAD + UART_TRAILER_SIZE];
static uint8_t response[UART_HEADER_SIZE + 8U + UART_TRAILER_SIZE];
static uint32_t image_size, image_crc, received, generation;
static boot_slot_t target_slot;
static bool begun;
volatile uint32_t boot_uart_rx_count;
volatile uint32_t boot_uart_last_byte;

static uint16_t load_u16(const uint8_t * p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8U); }
static uint32_t load_u32(const uint8_t * p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) | ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U); }
static void store_u16(uint8_t * p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8U); }
static void store_u32(uint8_t * p,uint32_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8U);p[2]=(uint8_t)(v>>16U);p[3]=(uint8_t)(v>>24U); }

static uint16_t crc16(const uint8_t * data,uint32_t length)
{
  uint16_t crc=UINT16_C(0xFFFF);
  for(uint32_t i=0;i<length;i++){crc^=(uint16_t)data[i]<<8U;for(uint32_t b=0;b<8U;b++)crc=(crc&UINT16_C(0x8000))?(uint16_t)((crc<<1U)^UINT16_C(0x1021)):(uint16_t)(crc<<1U);}
  return crc;
}

static void uart_init(void)
{
  RCC->CR |= RCC_CR_HSION | RCC_CR_HSIKERON;
  while((RCC->CR & RCC_CR_HSIRDY)==0U){}
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  GPIOB->MODER=(GPIOB->MODER&~((UINT32_C(3)<<6U)|(UINT32_C(3)<<8U)))|(UINT32_C(2)<<6U)|(UINT32_C(2)<<8U);
  GPIOB->AFR[0]=(GPIOB->AFR[0]&~((UINT32_C(0xF)<<12U)|(UINT32_C(0xF)<<16U)))|(UINT32_C(7)<<12U)|(UINT32_C(7)<<16U);
  GPIOB->OSPEEDR|=(UINT32_C(3)<<6U)|(UINT32_C(3)<<8U);
  GPIOB->PUPDR&=~((UINT32_C(3)<<6U)|(UINT32_C(3)<<8U));
  RCC->CCIPR=(RCC->CCIPR&~RCC_CCIPR_USART2SEL_Msk)|RCC_CCIPR_USART2SEL_1;
  RCC->APB1ENR1|=RCC_APB1ENR1_USART2EN;
  RCC->APB1RSTR1|=RCC_APB1RSTR1_USART2RST;RCC->APB1RSTR1&=~RCC_APB1RSTR1_USART2RST;
  USART2->BRR=16U;
  USART2->CR2=USART_CR2_SWAP;
  USART2->CR1=USART_CR1_TE|USART_CR1_RE|USART_CR1_FIFOEN;
  USART2->CR1|=USART_CR1_UE;
}

static void uart_send(const uint8_t * data,uint32_t length)
{
  for(uint32_t i=0;i<length;i++){while((USART2->ISR&USART_ISR_TXE_TXFNF)==0U){}USART2->TDR=data[i];}
  while((USART2->ISR&USART_ISR_TC)==0U){}
}

static uint8_t uart_read(void)
{
  while((USART2->ISR&USART_ISR_RXNE_RXFNE)==0U){
    if((USART2->ISR&(USART_ISR_ORE|USART_ISR_FE|USART_ISR_NE))!=0U)USART2->ICR=USART_ICR_ORECF|USART_ICR_FECF|USART_ICR_NECF;
  }
  const uint8_t value=(uint8_t)USART2->RDR;
  boot_uart_last_byte=value;
  boot_uart_rx_count++;
  return value;
}

static uint16_t receive_frame(uint8_t * type,uint16_t * sequence)
{
  static const uint8_t magic[4]={'O','F','W','1'};
  uint32_t matched=0U;
  while(matched<4U){const uint8_t value=uart_read();if(value==magic[matched])frame[matched++]=value;else matched=value==magic[0]?1U:0U;}
  for(uint32_t i=4U;i<UART_HEADER_SIZE;i++)frame[i]=uart_read();
  const uint16_t length=load_u16(&frame[8]);
  if(frame[4]!=1U||length>UART_MAX_PAYLOAD||crc16(frame,10U)!=load_u16(&frame[10]))return UINT16_MAX;
  for(uint32_t i=0U;i<(uint32_t)length+UART_TRAILER_SIZE;i++)frame[UART_HEADER_SIZE+i]=uart_read();
  if(boot_crc32c(frame,UART_HEADER_SIZE+length)!=load_u32(&frame[UART_HEADER_SIZE+length]))return UINT16_MAX;
  *type=frame[5];*sequence=load_u16(&frame[6]);return length;
}

static void send_result(uint8_t type,uint16_t sequence,uint8_t status,uint8_t slot,uint8_t state,uint8_t flags,uint32_t value)
{
  const uint8_t header[6]={'O','F','W','1',1U,(uint8_t)(type|UINT8_C(0x80))};
  memcpy(response,header,sizeof(header));store_u16(&response[6],sequence);store_u16(&response[8],8U);store_u16(&response[10],crc16(response,10U));
  response[12]=status;response[13]=slot;response[14]=state;response[15]=flags;store_u32(&response[16],value);store_u32(&response[20],boot_crc32c(response,20U));
  uart_send(response,sizeof(response));
}

static bool vector_is_valid(uint32_t base,uint32_t size)
{
  const uint32_t stack=*(const uint32_t *)base,reset=*(const uint32_t *)(base+4U),handler=reset&~UINT32_C(1);
  const bool stack_ok=((stack>=BOOT_SRAM1_BASE&&stack<=BOOT_SRAM1_END)||(stack>=BOOT_CCMRAM_BASE&&stack<=BOOT_CCMRAM_END))&&(stack&7U)==0U;
  return stack_ok&&(reset&1U)!=0U&&handler>=base&&handler<base+size;
}

static void handle(uint8_t type,uint16_t sequence,const uint8_t * payload,uint16_t length)
{
  boot_image_metadata_t metadata;
  const uint32_t base=boot_slot_base(target_slot);
  if(type==MSG_INFO){
    const bool a=boot_slot_is_valid(BOOT_SLOT_A,true,NULL),b=boot_slot_is_valid(BOOT_SLOT_B,true,NULL);
    send_result(type,sequence,STATUS_OK,(uint8_t)target_slot,0U,(uint8_t)((a?1U:0U)|(b?2U:0U)),generation);return;
  }
  if(type==MSG_BEGIN){
    if(length!=12U||payload[0]!=(uint8_t)target_slot){send_result(type,sequence,STATUS_RANGE,(uint8_t)target_slot,0U,0U,received);return;}
    image_size=load_u32(&payload[4]);image_crc=load_u32(&payload[8]);received=0U;begun=false;
    if(image_size<8U||image_size>BOOT_SLOT_A_SIZE){send_result(type,sequence,STATUS_RANGE,(uint8_t)target_slot,0U,0U,0U);return;}
    if(!boot_flash_erase_page(boot_slot_metadata_base(target_slot))){send_result(type,sequence,STATUS_FLASH,(uint8_t)target_slot,0U,0U,0U);return;}
    for(uint32_t offset=0U;offset<image_size;offset+=BOOT_FLASH_PAGE_SIZE)if(!boot_flash_erase_page(base+offset)){send_result(type,sequence,STATUS_FLASH,(uint8_t)target_slot,0U,0U,offset);return;}
    begun=true;send_result(type,sequence,STATUS_OK,(uint8_t)target_slot,BOOT_IMAGE_STATE_RECEIVING,0U,0U);return;
  }
  if(type==MSG_CHUNK){
    if(length<10U||!begun){send_result(type,sequence,STATUS_STATE,(uint8_t)target_slot,0U,0U,received);return;}
    const uint32_t offset=load_u32(payload),chunk_crc=load_u32(&payload[4]);const uint16_t chunk_length=load_u16(&payload[8]);
    if(offset!=received){send_result(type,sequence,STATUS_SEQUENCE,(uint8_t)target_slot,0U,0U,received);return;}
    if(chunk_length==0U||chunk_length>UPDATE_CHUNK_SIZE||length!=10U+chunk_length||offset+chunk_length>image_size){send_result(type,sequence,STATUS_RANGE,(uint8_t)target_slot,0U,0U,received);return;}
    if(boot_crc32c(&payload[10],chunk_length)!=chunk_crc){send_result(type,sequence,STATUS_CRC,(uint8_t)target_slot,0U,0U,received);return;}
    if(!boot_flash_program(base+offset,&payload[10],chunk_length)){send_result(type,sequence,STATUS_FLASH,(uint8_t)target_slot,0U,0U,received);return;}
    received+=chunk_length;send_result(type,sequence,STATUS_OK,(uint8_t)target_slot,BOOT_IMAGE_STATE_RECEIVING,0U,received);return;
  }
  if(type==MSG_FINALIZE){
    if(!begun||received!=image_size){send_result(type,sequence,STATUS_SEQUENCE,(uint8_t)target_slot,0U,0U,received);return;}
    if(!vector_is_valid(base,image_size)||boot_crc32c((const void *)base,image_size)!=image_crc){send_result(type,sequence,STATUS_CRC,(uint8_t)target_slot,0U,0U,received);return;}
    metadata=(boot_image_metadata_t){BOOT_IMAGE_METADATA_MAGIC,BOOT_IMAGE_METADATA_FORMAT,sizeof(metadata),generation,BOOT_IMAGE_STATE_PENDING,(uint32_t)target_slot,base,image_size,image_crc,0U};
    metadata.record_crc32c=boot_crc32c(&metadata,offsetof(boot_image_metadata_t,record_crc32c));
    if(!boot_metadata_write(target_slot,&metadata)||!boot_control_write(target_slot,generation,0U)){send_result(type,sequence,STATUS_FLASH,(uint8_t)target_slot,0U,0U,received);return;}
    send_result(type,sequence,STATUS_OK,(uint8_t)target_slot,BOOT_IMAGE_STATE_PENDING,0U,received);return;
  }
  if(type==MSG_REBOOT){send_result(type,sequence,STATUS_OK,(uint8_t)target_slot,BOOT_IMAGE_STATE_PENDING,0U,received);for(volatile uint32_t i=0U;i<100000U;i++){}NVIC_SystemReset();}
  send_result(type,sequence,STATUS_FRAME,(uint8_t)target_slot,0U,0U,0U);
}

void boot_uart_update(boot_slot_t running_slot)
{
  target_slot=running_slot==BOOT_SLOT_A?BOOT_SLOT_B:BOOT_SLOT_A;
  boot_image_metadata_t a={0},b={0};
  const bool va=boot_slot_is_valid(BOOT_SLOT_A,true,&a),vb=boot_slot_is_valid(BOOT_SLOT_B,true,&b);
  generation=(va?a.generation:0U);if(vb&&b.generation>=generation)generation=b.generation;generation++;
  uart_init();
  for(;;){uint8_t type;uint16_t sequence;const uint16_t length=receive_frame(&type,&sequence);if(length!=UINT16_MAX)handle(type,sequence,&frame[UART_HEADER_SIZE],length);}
}
