
/*
 * main_app.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#include "main_app.h"


#define CAN_QUEUE_CAPACITY 64

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t dlc;
	bool isExtended;
	bool isRemote;
} CanPacket;

static RoboMasters robomasters;

static Queue can1_txQueue;
static CanPacket can1_txBuffer[CAN_QUEUE_CAPACITY];
static Queue can2_txQueue;
static CanPacket can2_txBuffer[CAN_QUEUE_CAPACITY];

static void task1kHz();
static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);
static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote);

static void robomaster_transmitfeedbuck ();

static CanPacket can2_rxPacket = {0};

float wheel_val = 0;
float test_speed = 0;

uint32_t temp_data[3]= {0};
uint8_t fbsend_data[8] = {0};

RoboMaster *robomaster_temp;


uint32_t mailbox_checker = 0;
uint32_t can2_ErrorChecker = 0;
uint32_t can2_ModeChecker = 0;


void setup() {



	timer_startUs();

	RoboMasters_setTxFunc(&robomasters, can1_transmitQueue);
	RoboMasterConfig rmConf;
	rmConf.model = ROBOMASTER_M2006;
	PID_init(&rmConf.pidVelocity, ROBOMASTER_M2006_PID_VELOCITY_DEFAULT);
	PID_init(&rmConf.pidPosition, ROBOMASTER_M2006_PID_POSITION_DEFAULT);
	rmConf.controlTimeout = 200;	 // [ms]
	rmConf.feedbackTimeout = 200; // [ms]
	rmConf.rawPositionOffset = 0;
	rmConf.trapezoidAcceleration = 36.0f * 2.0f * PI;
	rmConf.trapezoidMaxAcceleration = rmConf.trapezoidAcceleration * 1.5f;
	rmConf.trapezoidDeadband = PI / 8.0f;
	for (uint8_t i = 0; i < 8; i++)
	{
		RoboMaster_init(&robomasters.robomaster[i], rmConf);
		RoboMaster_setCurrentLimit(&robomasters.robomaster[i],
				-ROBOMASTER_M2006_MAX_CURRENT, ROBOMASTER_M2006_MAX_CURRENT);
		RoboMaster_setVelocityLimit(&robomasters.robomaster[i],
				-360.0f * 2.0f * PI, 360.0f * 2.0f * PI); // [rad/s]
		RoboMaster_setPositionLimit(&robomasters.robomaster[i],
				-INFINITY, INFINITY); // [rad]
	}

	Queue_init(&can1_txQueue, can1_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, 0, disable_irq_nest, enable_irq_nest);
	Queue_init(&can2_txQueue, can2_txBuffer, sizeof(CanPacket), CAN_QUEUE_CAPACITY, 0, disable_irq_nest, enable_irq_nest);

	HAL_Delay(500);

	timer_set1kHzTask(task1kHz);
	timer_start1kHzTask();

	can1_setReceivedCallback(can1_rxCallback);
	can2_setReceivedCallback(can2_rxCallback);
	can1_start(&CAN1_FILTER_DEFAULT);
	can2_start(&CAN2_FILTER_DEFAULT);

//	UNUSED(can2_transmitQueue);
}

void loop() {

	CanPacket canPacket;
	if (Queue_size(&can1_txQueue) > 0 && can1_txAvailable() > 0)
	{
		if (Queue_pop(&can1_txQueue, &canPacket) == 0)
		{
			can1_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
	if (Queue_size(&can2_txQueue) > 0 && can2_txAvailable() > 0)
	{
		if (Queue_pop(&can2_txQueue, &canPacket) == 0)
		{
			can2_transmit(canPacket.id, canPacket.data, canPacket.dlc, canPacket.isExtended, canPacket.isRemote);
		}
	}
	mailbox_checker = can2_txAvailable();
	can2_ErrorChecker = HAL_CAN_GetError(&hcan2);
	can2_ModeChecker = hcan2.Init.Mode;
}

static void task1kHz() {
	static uint32_t tick = 0;

	for (uint8_t i = 0; i < 8; i++)
	{
		RoboMaster_calculateOutputCurrent(&robomasters.robomaster[i]);
	}
	RoboMaster_transmit(&robomasters);

	if(tick % 10 == 0) robomaster_transmitfeedbuck();//send ros_node



	gpio_setLedR((tick % 1000 < 500) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	tick++;

}

static void can1_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	RoboMaster_rxTask(&robomasters, id, data, dlc, isExtended, isRemote);
}

static void can2_rxCallback(uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	can2_rxPacket.id = id;
	memcpy(can2_rxPacket.data, data, dlc);
	can2_rxPacket.dlc = dlc;
	can2_rxPacket.isExtended = isExtended;
	can2_rxPacket.isRemote = isRemote;

	if (id == 0x100 && dlc == 8 && isExtended == false && isRemote == false) {
		memcpy(&wheel_val,&data[2],sizeof(float));

		switch(data[1]){
			case 0:
				RoboMaster_setTargetPosition(&robomasters.robomaster[data[0]],wheel_val);
				break;
			case 1:
				wheel_val = wheel_val * 36;
				RoboMaster_setTargetVelocity(&robomasters.robomaster[data[0]],wheel_val);
				break;
			case 2:
				RoboMaster_setTargetCurrent(&robomasters.robomaster[data[0]],wheel_val);
				break;
			default:
				break;
		}
	}
}

static void can1_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can1_txQueue, &packet);
}

static void can2_transmitQueue(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote) {
	if (dlc > 8) return;

	CanPacket packet;
	packet.id = id;
	memcpy(packet.data, data, dlc);
	packet.dlc = dlc;
	packet.isExtended = isExtended;
	packet.isRemote = isRemote;

	Queue_push(&can2_txQueue, &packet);
}



static void robomaster_transmitfeedbuck(){
	for(int id = 0; id < 8; id ++){
		robomaster_temp = &robomasters.robomaster[id];

		memcpy(&temp_data[0],&robomaster_temp->feedback.position,sizeof(float));
		memcpy(&temp_data[1],&robomaster_temp->feedback.velocity,sizeof(float));
		memcpy(&temp_data[2],&robomaster_temp->feedback.current,sizeof(float));

		fbsend_data [0] = id + 1;
				for(int i = 0; i < 3; i ++){
					fbsend_data[1] = i;
					for(int k = 0; k < 4 ; k++){
						fbsend_data[k + 2] = (temp_data[i] >> (8 * (3 - k))) & 0xff;
					}
					can2_transmitQueue(0x600,fbsend_data,8,false,false);
				}
			}
	}








