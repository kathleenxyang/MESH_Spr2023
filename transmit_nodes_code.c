/**
 * ADC to Pico to NeoCortec node transmission
 * 
 * Notes:
 * 1) sending chars DOES NOT WORK
 * 2)
 */


#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "NcApi.h"


/// \tag::hello_uart[]

#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_CTS_PIN 18
#define UART_RTS_PIN 19

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define YOUR_LENGTH 6 // Sensor payload Length, 2 bytes/sensor with 3 sensors = 6 bytes
#define ARRAY_LEN 6 // Do not change - 5

//ADC global variables
uint16_t data[3] = {0, 0, 0};

//NeoCortec Commands
uint8_t getNodeInfo = 0x08;
uint8_t getListReq = 0x09;
uint8_t getAckPacket = 0x03;
uint8_t getUnackPacket = 0x02;

// Global declarations expected by NcApi
tNcApi g_ncApi[1];
uint8_t g_numberOfNcApis = 1;
tNcApiRxHandlers ncRx;

// global variables that need not be changed unless sink node changes
uint8_t nodeId = 0x30; //sink node ID !!!IMPORTANT!!!
uint8_t destPort = 0x00; //any value between 0 and 4
uint8_t ackMsg[YOUR_LENGTH + ARRAY_LEN]; // total transmit length

// Application UART: Code     		Payload Length     		UART Payload
// Length (bytes):   1				m				   		n
// Structure: {code, payload len, nodeID MSB, nodeID LSB, destPort, Msg[n]} 	*as seen from integration manual 

/********Declarations********/
void picoSetup();
void getADCdata();
void emptyAckMsg();
void ExampleSetupNcApi();

/****************************/


int main() {

	picoSetup();
    ExampleSetupNcApi();

	emptyAckMsg();

	// construct the beginning of the array to be transmitted
	// this does not need to change with every iteration
	ackMsg[0] = getAckPacket;
	ackMsg[1] = YOUR_LENGTH+ARRAY_LEN-2;
	ackMsg[2] = 0x00;
	ackMsg[3] = nodeId;
	ackMsg[4] = destPort;

	//testMsg asking for node info for debugging
	//uint8_t testMsg[1] = {getNodeInfo};

	while(1)
	{
		
		getADCdata();

		// break up the 2 byte ADC values into 1 byte elements for transmission 
		ackMsg[5] = (uint8_t)((data[0]>>8) & (0x0F));
		ackMsg[6] = (uint8_t)(data[0]);
		ackMsg[7] = (uint8_t)((data[1]>>8) & (0x0F));
		ackMsg[8] = (uint8_t)(data[1]);
		ackMsg[9] = (uint8_t)((data[2]>>8) & (0x0F));
		ackMsg[10] = (uint8_t)(data[2]);
		ackMsg[11] = 0xff; // dummy transmission

		NcApiSupportTxData(0, ackMsg, YOUR_LENGTH+ARRAY_LEN);

		for(int i=0; i<6; i++)
		{
			printf("ackMsg[%d] = %x\n", i+5, ackMsg[5+i]);
		}

		for(int i=0; i<6; i++)
		{
			printf("\033[F");
		}
		
		//nodeInfo transmission for debugging purposes
		//NcApiSupportTxData(0, testMsg, 1);

		sleep_ms(9000); //9 second delay
		sleep_ms(6000); //additional 6 second delay
		
	}


}
/*************************************************/


void picoSetup()
{
	/************initialize UART*********/
	// Set up our UART with the required speed.
	stdio_init_all(); // NEED FOR COM PORT TO SHOW UP!!!!!!!!!!!!!!!!!!!
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RTS_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(uart0, true, true);

	//uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

	// Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

	/*************initialize ADC***********/

	adc_init();

	// ADC0 is soil sensor
    // ADC1 is Temp sensor
    // ADC2 is light sensor

	// Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

	// Select ADC input 0 (GPIO26)
    adc_select_input(0);

	// Set Round-Robin ADC reading scheme for ADC0-ADC2
    // This scheme will cycle through adc_select_input and select all of the ADC inputs
    adc_set_round_robin(0x7);


}

void getADCdata()
{
	//ADC has been initialized as round-robin, so it will automatically read the next ADC pin when called again
	data[0] = adc_read(); //soil
	sleep_ms(500);
    data[1] = adc_read(); //temp
    sleep_ms(500);
    data[2] = adc_read(); //light
	sleep_ms(500);
}

void emptyAckMsg()
{
	//empties ackMsg array after being initialized to ensure no random numbers
	for(int i = 0; i< YOUR_LENGTH+ARRAY_LEN; i++)
	{
		ackMsg[i] = 0;
	}
}


/**********************NcApi Functions***************************/

// Global functions expected by NcApi
NcApiErrorCodes NcApiSupportTxData(uint8_t n, uint8_t * finalMsg, uint8_t finalMsgLength)
{
	// Write to particular UART indexed by n
	for(int i = 0; i<finalMsgLength; i++)
	{
		uart_putc_raw(uart0, finalMsg[i]);
	}

	return NCAPI_OK;
}

void NcApiSupportMessageWritten(uint8_t n, void * callbackToken, uint8_t * finalMsg, uint8_t finalMsgLength)
{
	printf("Message successfully sent to UART %i", n);
}

void NcApiSupportMessageReceived(uint8_t n,void * callbackToken, uint8_t * msg, uint8_t msgLength)
{
	NcApiExecuteCallbacks(n,msg,msgLength);

	printf("Message successfully received by node");
}
 

// Handle CTS and RX interrupts
void ExamplePassthroughCts()
{
	NcApiCtsActive(0);
}

void ExamplePassthroughRx(uint8_t byte)
{
	NcApiRxData(0, byte);
	
}

void ExamplePassthroughNwu()
{
	NcApiCallbackNwuActive(0);
}


// Sample implementation of generic message received handler merely providing the bytes.
// Always called before any of the strongly typed message handlers are called.
void ExampleReadCallback(uint8_t n, uint8_t * msg, uint8_t msgLength)
{
	uint16_t i;
	printf("Raw UART data received: ");
	for (i=0; i<msgLength; i++)
		printf( "%02x ", msg[i] );
	printf("\r\n");
}

void ExampleHostAckCallback(uint8_t n, tNcApiHostAckNack * p)
{
	printf( "My previous package to node node %04x was successfully delivered\r\n", p->originId );
}

void ExampleHostNAckCallback(uint8_t n, tNcApiHostAckNack * p)
{
	printf( "My previous package to node node %04x was not delivered\r\n", p->originId );
}

void ExampleHostDataCallback(uint8_t n, tNcApiHostData * p)
{
	printf
	(
		"I have received %d bytes from node %04x. They are %dms old\r\n",
		p->payloadLength,
		p->originId,
		p->packageAge * 125
	);
}

void ExampleHostDataHapaCallback(uint8_t n, tNcApiHostDataHapa * p)
{
	printf
	(
		"I have received %d bytes from node %x4. They are %dms old\r\n",
		p->payloadLength,
		p->originId,
		((double)p->packageAge) / 524.288
	);
}

void ExampleWesSetupRequestCallback(uint8_t n, tNcApiWesSetupRequest * p)
{
	n;
}

void ExampleWesStatusCallback(uint8_t n, tNcApiWesStatus * p)
{
	n;
}

// Sample implementation of how to send one Unacknowledged command message
void ExampleSendUnacknowledged(uint16_t destNodeId, uint8_t port, uint16_t appSeqNo, uint8_t * payload, uint8_t payloadLen)
{
	tNcApiSendUnackParams args;
	NcApiErrorCodes apiStatus;
	args.msg.destNodeId = destNodeId;
	args.msg.destPort = port;
	args.msg.appSeqNo = appSeqNo;
	args.msg.payload = payload;
	args.msg.payloadLength = payloadLen;
	args.callbackToken = &g_ncApi;
	apiStatus = NcApiSendUnacknowledged(0, &args);
	if (apiStatus != NCAPI_OK)
	{
		; // Application specific
	}
}

// Sample implementation of how to send one Acknowledged command message
void ExampleSendAcknowledged(uint16_t destNodeId, uint8_t port, uint8_t * payload, uint8_t payloadLen)
{
	tNcApiSendAckParams args;
	NcApiErrorCodes apiStatus;
	args.msg.destNodeId = destNodeId;
	args.msg.destPort = port;
	args.msg.payload = payload;
	args.msg.payloadLength = payloadLen;
	args.callbackToken = &g_ncApi;
	apiStatus = NcApiSendAcknowledged(0, &args);
	if (apiStatus != NCAPI_OK)
	{
		; // Application specific
	}
}

// Sample implementation of how to send one WesCmd
void ExampleSendWesCmd(NcApiWesCmdValues cmd)
{
	tNcApiWesCmdParams args;
	args.msg.cmd = cmd;
	args.callbackToken = &g_ncApi;
	NcApiSendWesCmd( 0, &args );
}

// Sample implementation of how to send one WesResponse
void ExampleSendWesResponse(uint64_t uid, uint16_t nodeId)
{
	tNcApiWesResponseParams args;
	args.msg.uid[0] = (uid >> 32) & 0xff;
	args.msg.uid[1] = (uid >> 24) & 0xff;
	args.msg.uid[2] = (uid >> 16) & 0xff;
	args.msg.uid[3] = (uid >> 8) & 0xff;
	args.msg.uid[4] = uid & 0xff;
	args.msg.nodeId = nodeId;
	args.callbackToken = &g_ncApi;
	NcApiSendWesResponse( 0, &args );
}

// Sample implementation of how to setup global rx-handlers and initialize NcApi.
void ExampleSetupNcApi()
{
	tNcApiRxHandlers * rxHandlers = &ncRx;
	memset( rxHandlers, 0, sizeof(tNcApiRxHandlers));

	rxHandlers->pfnReadCallback = ExampleReadCallback;
	rxHandlers->pfnHostAckCallback = ExampleHostAckCallback;
	rxHandlers->pfnHostNAckCallback = ExampleHostNAckCallback;
	rxHandlers->pfnHostDataCallback = ExampleHostDataCallback;
	rxHandlers->pfnHostDataHapaCallback = ExampleHostDataHapaCallback;
	rxHandlers->pfnWesSetupRequestCallback = ExampleWesSetupRequestCallback;
	rxHandlers->pfnWesStatusCallback = ExampleWesStatusCallback;
	
	memset( (void*)&g_ncApi[0], 0, sizeof(tNcApi) );
	
	g_ncApi[0].NcApiRxHandlers=&ncRx;
}
