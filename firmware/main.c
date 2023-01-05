/*
* usart.c
*
* Created : 15-08-2020 07:24:45 PM
* Author  : Arnab Kumar Das
* Website : www.ArnabKumarDas.com
*/

#include <avr/io.h>      // Contains all the I/O Register Macros
#include <avr/interrupt.h>
//#include <util/delay.h>  // Generates a Blocking Delay
#include <stdio.h>

#define USART_BAUDRATE 57600 // Desired Baud Rate
//#define USART_BAUDRATE 115200 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ASYNCHRONOUS (0<<UMSEL00) // USART Mode Selection

#define DISABLED    (0<<UPM00)
#define EVEN_PARITY (2<<UPM00)
#define ODD_PARITY  (3<<UPM00)
#define PARITY_MODE  DISABLED // USART Parity Bit Selection

#define ONE_BIT (0<<USBS0)
#define TWO_BIT (1<<USBS0)
#define STOP_BIT ONE_BIT      // USART Stop Bit Selection

#define FIVE_BIT  (0<<UCSZ00)
#define SIX_BIT   (1<<UCSZ00)
#define SEVEN_BIT (2<<UCSZ00)
#define EIGHT_BIT (3<<UCSZ00)
#define DATA_BIT   EIGHT_BIT  // USART Data Bit Selection

#define UART_RX_BUFF_SIZE   128
#define UART_TX_BUFF_SIZE   1024

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
                                         _FDEV_SETUP_WRITE);


struct uart_tx {
    char data[UART_TX_BUFF_SIZE];
    volatile int head;
    volatile int len;
};

#if 1
/* frame bits including toggle bit */
#define RC6_FRAME_BITS  20

#define RC6_GET_ADDRESS(data)     ((data >> 8) && 0xff)
#define RC6_GET_COMMAND(data)     (data && 0xff)
#define RC6_GET_FIELD(data)       ((data >> 17) && 0x3)
#define RC6_IS_TOGGLE_SET(data)   (data && (1 << 16))

enum rc6_pulse {
    RC6_PULSE_2666, /* 1998 - 3330 */
    RC6_PULSE_1333, /* 1110 - 1998 */
    RC6_PULSE_889,  /*  666 - 1110 */
    RC6_PULSE_444,  /*  222 -  666 */
    RC6_PULSE_INVALID,
};

char rc6_pulse_lookup[] = {
    RC6_PULSE_444, // 222-444
    RC6_PULSE_444, // 444-666
    RC6_PULSE_889, // 666-889
    RC6_PULSE_889, // 889-1110
    RC6_PULSE_1333, // 1110-1332
    RC6_PULSE_1333, // 1332-1556
    RC6_PULSE_1333, // 1556-1776
    RC6_PULSE_1333, // 1776-1998
    RC6_PULSE_2666, // 1998-2220
    RC6_PULSE_2666, // 2220-2442
    RC6_PULSE_2666, // 2442-2664
    RC6_PULSE_2666, // 2664-2886
    RC6_PULSE_2666, // 2886-3108
    RC6_PULSE_2666, // 3108-3330
};

enum rc6_bit_state {
    RC6_IDLE = 0,
    RC6_LEADER,
    RC6_ONE_START,
    RC6_ONE_MIDDLE,
    RC6_ZERO_MIDDLE,
    RC6_ZERO_START,
    RC6_T_ZERO_MIDDLE,
    RC6_T_ONE_MIDDLE,
};

enum rc6_frame_state {
    RC6_FRAME_IDLE = 0,
    RC6_FRAME_LEADER,
    RC6_FRAME_ADDR,
    RC6_FRAME_DATA,
};

char rc6_bit_transitions[][5] = {
/*    2666,       1333,              889,               444,             INVALID */
    { RC6_LEADER, RC6_IDLE,          RC6_IDLE,          RC6_IDLE,        RC6_IDLE },     /* IDLE */
    { RC6_IDLE,   RC6_IDLE,          RC6_ONE_START,     RC6_IDLE,        RC6_IDLE },     /* LEADER */
    { RC6_IDLE,   RC6_IDLE,          RC6_T_ONE_MIDDLE,  RC6_ONE_MIDDLE,  RC6_IDLE },     /* ONE_START */
    { RC6_IDLE,   RC6_T_ZERO_MIDDLE, RC6_ZERO_MIDDLE,   RC6_ONE_START,   RC6_IDLE },     /* ONE_MIDDLE */
    { RC6_IDLE,   RC6_T_ONE_MIDDLE,  RC6_ONE_MIDDLE,    RC6_ZERO_START,  RC6_IDLE },     /* ZERO_MIDDLE */
    { RC6_IDLE,   RC6_T_ONE_MIDDLE,  RC6_T_ZERO_MIDDLE, RC6_ZERO_MIDDLE, RC6_IDLE },     /* ZERO_START */
    { RC6_IDLE,   RC6_ONE_MIDDLE,    RC6_ZERO_START,    RC6_IDLE,        RC6_IDLE },     /* T_ZERO_MIDDLE */
    { RC6_IDLE,   RC6_ZERO_MIDDLE,   RC6_ONE_START,     RC6_IDLE,        RC6_IDLE },     /* T_ONE_MIDDLE */
};

struct rc6_state {
    enum rc6_bit_state bitState;
    enum rc6_frame_state frameState;
    uint_least32_t data;
    char data_seen;
};

struct rc6_state decoder;

void rc6_frame_transition() {
    enum rc6_frame_state nextFrameState = RC6_FRAME_IDLE;
    enum rc6_bit_state bitState = decoder.bitState;

    switch(decoder.frameState) {
        case RC6_FRAME_IDLE:
            if (bitState == RC6_LEADER)
                decoder.data_seen = 0;
                nextFrameState = RC6_FRAME_LEADER;
            break;
        case RC6_FRAME_LEADER:
            if (bitState == RC6_ONE_MIDDLE)
                decoder.data = 1;
                nextFrameState = RC6_FRAME_ADDR;
            break;
        case RC6_FRAME_ADDR:
            if ((bitState == RC6_ONE_MIDDLE || bitState == RC6_ZERO_MIDDLE) && decoder.data_seen < 3) {
                printf("addr %d\r\n", bitState == RC6_ONE_MIDDLE);
                decoder.data = (decoder.data << 1) | (bitState == RC6_ONE_MIDDLE);
                nextFrameState = RC6_FRAME_ADDR;
                decoder.data_seen++;
            }
            if ((bitState == RC6_T_ONE_MIDDLE || bitState == RC6_T_ZERO_MIDDLE) && decoder.data_seen == 0) {
                decoder.data = (decoder.data << 1) | (bitState == RC6_T_ONE_MIDDLE);
                nextFrameState = RC6_FRAME_DATA;
                decoder.data_seen++;
            }
            break;
        case RC6_FRAME_DATA:
            if ((bitState == RC6_ONE_MIDDLE || bitState == RC6_ZERO_MIDDLE) && decoder.data_seen < 20) {
                printf("data %d\r\n", bitState == RC6_ONE_MIDDLE);
                decoder.data  = (decoder.data << 1) | (bitState == RC6_ONE_MIDDLE);
                nextFrameState = RC6_FRAME_DATA;
                decoder.data_seen++;
            }
            break;
    }

    printf("frameState %d -> %d (%d)\r\n", decoder.frameState, nextFrameState, decoder.data_seen);
    decoder.frameState = nextFrameState;
}

int rc6_transition(int elapsed_usec) {
    int duration_elems = elapsed_usec / 222;
    if (duration_elems > 14)
        return -1;

    enum rc6_pulse pulse = rc6_pulse_lookup[duration_elems - 1];

    char newBitState = rc6_bit_transitions[decoder.bitState][pulse];
    //printf("bitState %d -> %d\r\n", decoder.bitState, newBitState);
    decoder.bitState = newBitState;

    switch (decoder.bitState) {
        case RC6_IDLE:
        case RC6_LEADER:
        case RC6_T_ONE_MIDDLE:
        case RC6_T_ZERO_MIDDLE:
        case RC6_ONE_MIDDLE:
        case RC6_ZERO_MIDDLE:
            rc6_frame_transition();
            break;
        default:
            break;
    }

    return decoder.data_seen == RC6_FRAME_BITS ? decoder.data : 0;
}
#endif

struct uart_tx uartTx;

#define ROT_A_STATE (!(PIND & _BV(PD6)))
#define ROT_B_STATE (!(PIND & _BV(PD7)))

void USART_Init()
{
    uartTx.head = 0;
    uartTx.len = 0;

	// Set Baud Rate
	UBRR0H = BAUD_PRESCALER >> 8;
	UBRR0L = BAUD_PRESCALER;

	// Set Frame Format
	UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;

	// Enable Receiver and Transmitter and enable TX (data register ready) interrupt
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
}

void USART_TransmitPolling(uint8_t DataByte)
{
	while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
	UDR0 = DataByte;
}

int uart_putchar(char c, FILE *stream) {
    cli();
    if (uartTx.len < UART_TX_BUFF_SIZE)
    {
        uartTx.data[(uartTx.head + uartTx.len) % UART_TX_BUFF_SIZE] = c;
        uartTx.len++;
    } else {
        uartTx.data[0] = '_';
    }
    UCSR0B |= _BV(UDRIE0);
    sei();

    return 0;
}

ISR(USART_UDRE_vect) {
    if (uartTx.len) {
        UDR0 = uartTx.data[uartTx.head];
        uartTx.head++;
        uartTx.head = uartTx.head % UART_TX_BUFF_SIZE;
        uartTx.len--;
    } else {
        UCSR0B &= ~_BV(UDRIE0);
    }
}

enum enc_dir { ENC_STAY = 0, ENC_LEFT = 1, ENC_RGHT = 2, ENC_FAIL = 3};

enum enc_state {
    LL = 0,
    LH = 1,
    HL = 2,
    HH = 3,
};

char enc_trans[][4] = {
/*          LL        LH        HL        HH      */
/* LL */  { ENC_STAY, ENC_LEFT, ENC_RGHT, ENC_FAIL },
/* LH */  { ENC_RGHT, ENC_STAY, ENC_FAIL, ENC_LEFT },
/* HL */  { ENC_LEFT, ENC_FAIL, ENC_STAY, ENC_RGHT },
/* HH */  { ENC_FAIL, ENC_RGHT, ENC_LEFT, ENC_STAY },
};

enum enc_state encState;

enum enc_dir lastDir = ENC_STAY;
volatile int count;

volatile int newDir;

ISR(PCINT2_vect)
{
    enum enc_state curState = ROT_B_STATE | (ROT_A_STATE << 1);

    enum enc_dir dir = enc_trans[encState][curState];

    if (dir == ENC_LEFT || dir == ENC_RGHT)
    {
        if (lastDir == dir)
            count++;
        if (count > 0)
        {
            newDir = dir;
            count = 0;
        }

        lastDir = dir;
    }
    encState = curState;
}


long long unsigned ticks;


ISR (TIMER1_OVF_vect)
{
    TCNT1 = 65536 - 8;

 //   ticks++;

    PORTD ^= _BV(PD2);
}


int main()
{
	USART_Init();

    stdout = &mystdout;

    // ROT-A and ROT-B as input
    DDRD &= _BV(PD6);
    DDRD &= _BV(PD7);
    DDRD |= _BV(DDB2);

    // FIXME not needed
    PORTD &= ~_BV(PD2);
    PORTD |= _BV(PD2);

    cli();
    // and enable pin change interrupts for them
    PCICR |= _BV(PCIE2);
    PCMSK2 |= _BV(PCINT22) | _BV(PCINT23);

    TCNT1 = 65536 - 8;

	TCCR1A = 0x00;
	TCCR1B = _BV(CS11) | _BV(CS10);  // Timer mode with 1024 prescler
	TIMSK1 = _BV(TOIE1);   // Enable timer1 overflow interrupt(TOIE1)

    sei();

    encState = ROT_B_STATE | (ROT_A_STATE << 1);

    printf("Hello world!\r\n");
    printf("long long %d\r\n", sizeof(long long));


    rc6_transition(2700);
    rc6_transition(800);
    rc6_transition(400);

    rc6_transition(800);
    rc6_transition(400); //2

    rc6_transition(400);
    rc6_transition(400); //1

    rc6_transition(400);
    rc6_transition(1300); //0

    rc6_transition(1300);
    rc6_transition(400); //7

    rc6_transition(400);
    rc6_transition(400); //6

    rc6_transition(400);
    rc6_transition(400); //5

    rc6_transition(400);
    rc6_transition(400); //4

    rc6_transition(400);
    rc6_transition(800); //3

    rc6_transition(800); //2
    rc6_transition(400); //1

    rc6_transition(400);
    rc6_transition(400); //0


    rc6_transition(400);
    rc6_transition(400); //7

    rc6_transition(400);
    rc6_transition(400); //6

    rc6_transition(400);
    rc6_transition(400); //5

    rc6_transition(400);
    rc6_transition(400); //4

    rc6_transition(400);
    rc6_transition(800); //3

    rc6_transition(800); //2
    rc6_transition(400); //1

    rc6_transition(400);
    int ret = rc6_transition(400); //0

    printf("ret: %x\r\n", ret);

	while (1)
	{
        if (newDir) {
            printf("dir: %d\r\n", newDir);
            newDir = 0;
        }

    //if (!(ticks % 31250))
     //   printf("ticks: %lu\r\n", ticks);

    //    if (lastDir != 0) {
    //        printf("dir %d\r\n", lastDir);
    //        printf("state %d\r\n", encState);
    //        printf("count %d\r\n", count);
    //    }

        //if (count >= 1)
        //{
        //    printf("dir: %d\r\n", lastDir);
        //    count = 0;
        //}
//        if (count++ == 0)
        //    printf("nevim %u\r\n", count);

        //if (PIND & _BV(PD7)) {
		//    USART_TransmitPolling('1');
        //} else {
		//    USART_TransmitPolling('0');
        //}
		//USART_TransmitPolling(' ');
		//_delay_ms(10);
        //printf("nevim\r\n");
    }

	return 0;
}
