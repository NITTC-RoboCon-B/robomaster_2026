
/*
 * main_app.c
 *
 *  Created on: Oct 16, 2025
 *      Author: Doraemonjayo
 */

#include "main_app.h"


#define CAN_QUEUE_CAPACITY 64

#define robomaster_m3508_ratio 19
#define robomaster_m2006_ratio 36


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

static void usrbtnPressed();
static void usrbtnReleased();




static void robomaster_transmitfeedbuck();

#ifdef UseRobomastertest
static void robomaster_test(int mode,float value);
#endif

static CanPacket can2_rxPacket = {0};

float wheel_val = 0;
float test_speed = 0;

uint8_t fbsend_data[8] = {0};
uint32_t temp_data[3]= {0};
uint8_t temp_actuator[8] = {0};

RoboMaster *robomaster_temp;


uint32_t mailbox_checker = 0;
uint32_t can2_ErrorChecker = 0;
uint32_t can2_ModeChecker = 0;

uint8_t gatekey_id[2] = {0};
uint8_t temp_id[3] = {0};

uint8_t press_counter = 0;

//board set number
bool btnkey = 0;

uint32_t Board_id = 0;
//

//number of check necessary motor
float moter_val = 0;
int usemoter_count = 0;
int usemoter_ID[8]= {10,10,10,10,10,10,10,10};


void setup() {


	flash_read(&Board_id,1);
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

	gpio_setUsrBtnPressedCallback(usrbtnPressed);
	gpio_setUsrBtnReleasedCallback(usrbtnReleased);

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




static void usrbtnPressed(){
	btnkey = 1;
};

static void usrbtnReleased(){

};


static void task1kHz() {

	static uint32_t tick = 0;

	for (uint8_t i = 0; i < 8; i++)
	{
		RoboMaster_calculateOutputCurrent(&robomasters.robomaster[i]);
	}
	RoboMaster_transmit(&robomasters);

	if(tick % 100 == 0) robomaster_transmitfeedbuck();//send ros_node

	gpio_setLedR((tick % 1000 < 500) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	static uint32_t press_counter = 0;
	static uint32_t btninterval_checker = 0;

	if(btnkey == 1){
		press_counter ++;
		btnkey = 0;
		btninterval_checker = tick;
	}

	if(tick >= btninterval_checker + 1000 && press_counter != 0){
		Board_id = press_counter - 1;
		press_counter = 0;
		flash_write(&Board_id,1);
	}
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

//	uint8_t gatekey_id[2] = {0};

	gatekey_id[0] = (can2_rxPacket.id >> 24) & ((1 << 3) - 1);
	gatekey_id[1] = (can2_rxPacket.id >> 20) & ((1 << 4) - 1);

	if(gatekey_id[0] == 0 && gatekey_id[1] == Board_id){

		temp_id[0] =  can2_rxPacket.id        & ((1 << 8) - 1);
		temp_id[1] = (can2_rxPacket.id >> 8)  & ((1 << 8) - 1);
		temp_id[2] = (can2_rxPacket.id >> 16) & ((1 << 4) - 1);



		memcpy(&moter_val,&can2_rxPacket.data,sizeof(float));

		switch(temp_id[0]){
			case 0:
				RoboMaster_setTargetCurrent(&robomasters.robomaster[temp_id[1]],moter_val);
				break;
			case 1:
				if(temp_id[2] == 0) moter_val = moter_val * robomaster_m2006_ratio;
				else moter_val = moter_val * robomaster_m3508_ratio;
				RoboMaster_setTargetVelocity(&robomasters.robomaster[temp_id[1]],moter_val);
				break;
			case 2:
				if(temp_id[2] == 0) moter_val = moter_val * robomaster_m2006_ratio;
				else moter_val = moter_val * robomaster_m3508_ratio;
				RoboMaster_setTargetPosition(&robomasters.robomaster[temp_id[1]],moter_val);
				break;
			default:
				break;
		}

//		for(int i = 0;i <= usemoter_count; i ++){
//			if(usemoter_ID[i] != temp_id[1]){
//			usemoter_count ++;
//			}
//			temp_actuator[temp_id[1]] = temp_id[2];
//		}
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

	uint32_t send_id = 0;
	//id_set
	uint8_t priority  = 1;
	uint8_t type	  = 1;
	uint8_t board_id  = Board_id;
	uint8_t actuator  = 0;
	uint8_t device_id = 0;
	uint8_t mode      = 0;
	for(int id = 0; id < 8; id ++){
		device_id = id; // device_id
		actuator = temp_actuator[id]; // actuator
		send_id =
		 (priority << 27) |
		 (type << 24) 	  |
		 (board_id << 20) |
		 (actuator << 16) |
		 (device_id << 8) |
		 (mode);

		robomaster_temp = &robomasters.robomaster[id];

		uint8_t send_data[8] = {0};
		int16_t tempcast_val[2] = {0};
		float temp_position = robomaster_temp->feedback.position / 36;
		tempcast_val[0] =(int16_t)(robomaster_temp->feedback.velocity * 100 / 36);
		tempcast_val[1] =(int16_t)(robomaster_temp->feedback.current);

		memcpy(&send_data[0],&temp_position,sizeof(float));
		memcpy(&send_data[4],&tempcast_val[0],sizeof(int16_t));
		memcpy(&send_data[6],&tempcast_val[1],sizeof(int16_t));
		can2_transmitQueue(send_id,send_data,8,true,false);
	}
}




#ifdef UseRobomastertest

static void robomaster_test(int mode,float value){

	switch(mode){
		case 0:
			RoboMaster_setTargetPosition(&robomasters.robomaster[0],value);
			break;
		case 1:
			RoboMaster_setTargetVelocity(&robomasters.robomaster[0],value);
			break;
		case 2:
			RoboMaster_setTargetCurrent(&robomasters.robomaster[0],value);
			break;
		default:
			break;
		}

}

#endif


