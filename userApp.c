/***************************************************************************************************
* userApp.c
*
*  Created on: Oct 3, 2025
*  Author: James Donnellan
*
* Demo application to connect to a cloud based MQTT broker
* Data communication is un-encrypted using port 1883
*
* Before running the code: -
* 1. open /Core/Inc/WiFi_Credentials.h and enter WiFi access point SSID and password
* 2. open /Core/Inc/CloudBrokerCredentials.h and enter your broker credentials
* 3. Change the publish topic in the userApp() while(1) loop
* 4. Change the subscribe topic in the userApp() function
* 5. Change the subscribe topic in the subscribeMessageHandler() function
*
*
* Application Functionality
* 1. Fetches the epoch time from st.com and uses it to start the RTC with a 1-second wake-up interrupt
* 2. Connects to wireless access point and then to cloud DNS
* 3. Connects to MQTT broker
* 4. Subscribes to a single topic
* 5. Prints date and time every second
* 6. Publishes to a topic every time button is pressed
* 7. Subscribe callback function runs every time a publish message is received from the broker
********************************************************************************************************
 *
 */

#include "main.h"
#include "userApp.h"
#include "stm32l475e_iot01_accelero.h"
#include "vl53l0x_proximity.h"


void turnSOSOff(void);
void turnSOSOn(void);


extern UART_HandleTypeDef huart1;
extern int network_wr(Network* n, unsigned char* buffer, int len, int timeout_ms);
extern int network_rd(Network* n, unsigned char* buffer, int len, int timeout_ms);
extern int net_if_init(void * if_ctxt);
extern int net_if_deinit(void * if_ctxt);
extern int net_if_reinit(void * if_ctxt);
extern int wifi_net_if_init(void * if_ctxt);

uint8_t timeDisplay = 0;
uint8_t readSensor = 0, resetSystem = 0;
char eventType[12], lastEventType[12] = "safe";
int brakingSensitivity = 100;
net_hnd_t hnet;
Network network;
MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
MQTTMessage mqmsg;
MQTTClient client;


net_sockhnd_t socket;


typedef struct {
  char *HostName;
  char *HostPort;
  char *ConnSecurity;
  char *MQClientId;
  char *MQUserName;
  char *MQUserPwd;
#ifdef LITMUS_LOOP
  char *LoopTopicId;
#endif
} device_config_t;

/*------------------------------------------------------------------
 * RTC timer event interrupt callback function
 * Runs every 10 seconds
 * Tells my main loop to print the time and to read the sensors
 ------------------------------------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback (RTC_HandleTypeDef * hrtc)
{
	timeDisplay = 1;
	readSensor = 1;
}


/*------------------------------------------------------------------
 * GPIO Interrupt Handling,
 * GPIO_PIN_1 calls Wifi Module
 * BUTTON_EXTI13_Pin sets a flag to trigger a sensor reading
 ------------------------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
	case (GPIO_PIN_1):
	{
		SPI_WIFI_ISR();
		break;
	}

	case (BUTTON_EXTI13_Pin) :
	{
		printf("Resetting System...\r\n\n");
		resetSystem = 1;
		break;
	}

    default:
    {
      break;
    }
  }
}

/*--------------------------------------------------------------------------
 * Subscribe message callback function
 * Called every time a publish meassage is received from a subscribed topic
 --------------------------------------------------------------------------*/
void subscribeMessageHandler(MessageData* data)
{
	static char mqtt_msg[MQTT_MSG_BUFFER_SIZE], mqtt_topic[MQTT_TOPIC_BUFFER_SIZE];
	snprintf(mqtt_msg, data->message->payloadlen+1, "%s", (char *)data->message->payload);
	snprintf(mqtt_topic, data->topicName->lenstring.len+1, "%s", data->topicName->lenstring.data);
	printf("\r\nPublished message from MQTT broker\r\n");
	printf("Topic: %s, Payload: %s\r\n\n", mqtt_topic, mqtt_msg);


	if(strstr(mqtt_topic, "G00434671/feeds/sostoggle"))
	{
		if(strcmp(mqtt_msg, "ON") == 0)
		{
			turnSOSOn();

		}
		else
		{
			turnSOSOff();
		}
	}

	if(strstr(mqtt_topic, "G00434671/feeds/resetbutton"))
	{
		 if(strcmp(mqtt_msg, "1") == 0)   // only reset on press
		    {
		        resetSystem = 1;
		    }
	}


	if(strstr(mqtt_topic, "G00434671/feeds/brakingsensitivity"))
	{
		int sens = atoi(mqtt_msg); // Converts payload string to integer
		brakingSensitivity = sens;
		printf("Sensitivity: %d\r\n", brakingSensitivity);
	}
}


/*--------------------------------------------------------------------------
 * UserApp code
 --------------------------------------------------------------------------*/

void userApp()
{
	int32_t ret;
	// Accelerometer Variables //
	uint16_t distance_mm;
	int16_t XYZ[3];

	// Proximity Variables //
	const int crashThreshold = 900;

	// Initialize Sensors //
	VL53L0X_PROXIMITY_Init();
	BSP_ACCELERO_Init();

	//Network and MQTT variables
	char msgBuffer[25];

	// RTC variables
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	char timeBuffer[40]; // for timestamps

	printf("Starting user application\r\n\n");

	//LED off at initialization
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	//Connect to Adafruit IO MQTT broker
	brokerConnect(&client);

	// Subscribe Topics //

	// SOS //
	ret = MQTTSubscribe(&client, "G00434671/feeds/sostoggle", QOS0, (subscribeMessageHandler));
	if (ret != MQSUCCESS)
	{
		printf("\n\rSubscribe failed: %ld\r\n", ret);
	}
	else
	{
		printf("\n\rSubscribed to SosToggle\r\n");
		ret = MQTTYield(&client, 500);
	}
	HAL_Delay(1000);



	// Sensitivity Level //
	ret = MQTTSubscribe(&client, "G00434671/feeds/brakingsensitivity", QOS0, (subscribeMessageHandler));
	if (ret != MQSUCCESS)
	{
		printf("\n\rSubscribe failed: %ld\r\n", ret);
	}

	else
	{
		printf("\n\rSubscribed to BrakingSensitivity\r\n");
		ret = MQTTYield(&client, 500);
	}
	HAL_Delay(1000);




	// Reset Button //
	ret = MQTTSubscribe(&client, "G00434671/feeds/resetbutton", QOS0, (subscribeMessageHandler));
	if (ret != MQSUCCESS)
	{
		printf("\n\rSubscribe failed: %ld\r\n", ret);
	}
	else
	{
		printf("\n\rSubscribed to ResetButton\r\n");
		ret = MQTTYield(&client, 500);
	}
	HAL_Delay(1000);




	while(1)
	{
		// Display date/time every second //
		ret = MQTTYield(&client, 500);	//check for published messages from cloud - do not remove!


		if(timeDisplay)
		{
			timeDisplay = 0;

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			sprintf(timeBuffer, "%02d/%02d/%02d %02d:%02d:%02d\r\n", sDate.Date, sDate.Month, sDate.Year,
				sTime.Hours+1, sTime.Minutes, sTime.Seconds);
			printf("%s", timeBuffer);
		}



		if(readSensor) // Accelerometer Reading Every 10 Seconds //
		{
			readSensor = 0; // Set to 0 to avoid multiple readings unti       l next RTC

			// Read Sensors //
			BSP_ACCELERO_AccGetXYZ(XYZ);
			distance_mm = VL53L0X_PROXIMITY_GetDistance();

			// Braking Sensitivity //
			int hardBrakeThreshold = 20 + ((900 - 20) * (100 - brakingSensitivity)) /100;
			// 20 is minimum value, add this to the crash threshold minus min value
			// first half gets the threshold range.
			// multiply this by the max sensitivity possible (100) - the selected sensitivity
			// divide by 100 to get the % of the 880 range so sensitivity is affected

			// Determine Event Type //
			if(XYZ[0] >= crashThreshold)
			{
				printf("Emergency Services Notified\r\n");
				strcpy(eventType, "crash");
			}
			else if(XYZ[0] >= hardBrakeThreshold)
			{
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
				printf("[WARNING] Reduce Speed!\r\n");
				turnSOSOn();
				strcpy(eventType,"hard-brake");
			}
			else
			{
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
				turnSOSOff();
				strcpy(eventType,"safe");
			}

			if(strcmp(eventType, lastEventType) != 0)
			{
				// Event Type //
				mqmsg.payload = eventType;
				mqmsg.payloadlen = strlen(eventType);
				MQTTPublish(&client, "G00434671/feeds/eventtype", &mqmsg);
				strcpy(lastEventType, eventType);
			}
			if(distance_mm < 100)
			{
			    printf("WARNING Too Close");
			}
			else
			{
			    printf("Safe Distance");
			    turnSOSOff();
			}

			// Acceleration //
			sprintf(msgBuffer, "%d", XYZ[0]);
			mqmsg.qos = QOS0;
			mqmsg.payload = msgBuffer;
			mqmsg.payloadlen = strlen(msgBuffer);
			MQTTPublish(&client, "G00434671/feeds/accelerationx", &mqmsg);

			sprintf(msgBuffer, "%d", XYZ[1]);
			mqmsg.qos = QOS0;
			mqmsg.payload = msgBuffer;
			mqmsg.payloadlen = strlen(msgBuffer);
			MQTTPublish(&client, "G00434671/feeds/accelerationy", &mqmsg);
		}


		// Reset Button //
		if(resetSystem)
		{
			resetSystem = 0;


			strcpy(eventType, "safe");
			turnSOSOff();
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

			printf("Resetting System...\r\n");


			mqmsg.payload = eventType;
			mqmsg.payloadlen = strlen(eventType);
			MQTTPublish(&client, "G00434671/feeds/eventtype", &mqmsg);
			strcpy(lastEventType, "safe");
		}
	}
}

void brokerConnect(MQTTClient * client)
{
	int32_t ret;
	//Network and MQTT variables
	device_config_t MQTT_Config;
	static unsigned char mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];
	static unsigned char mqtt_read_buffer[MQTT_READ_BUFFER_SIZE];
	net_ipaddr_t ipAddr;
	net_macaddr_t macAddr;

	//Added
	//bool b_mqtt_connected           = false;
	//const char * connectionString   = NULL;
	//device_config_t * device_config = NULL;
#ifdef USE_MBED_TLS
	conn_sec_t connection_security  = CONN_SEC_UNDEFINED;
	const char * device_cert  = NULL;
	const char * device_key   = NULL;
#endif

	//Initialise MQTT broker structure
	//Fill in this section with MQTT broker credentials from header file
	MQTT_Config.HostName = CloudBroker_HostName;
	MQTT_Config.HostPort = CloudBroker_Port;
	MQTT_Config.ConnSecurity = "0";	//plain TCP connection with no security
	MQTT_Config.MQUserName = CloudBroker_Username;
	MQTT_Config.MQUserPwd = CloudBroker_Password;
	MQTT_Config.MQClientId = CloudBroker_ClientID;

	//Initialise WiFi network
	if (net_init(&hnet, NET_IF, (wifi_net_if_init)) != NET_OK) {
	//if (net_init(&hnet, NET_IF, (net_if_init)) != NET_OK) {
		printf("\n\rError");
	}
	else {
		printf("\n\rOK");
	}
	HAL_Delay(500);

	printf("\n\rRetrieving the IP address.");

	if (net_get_ip_address(hnet, &ipAddr) != NET_OK) {
		printf("\n\rError 2");
	}
	else
	{
		switch(ipAddr.ipv) {
			case NET_IP_V4:
				printf("\n\rIP address: %d.%d.%d.%d\n\r", ipAddr.ip[12], ipAddr.ip[13], ipAddr.ip[14], ipAddr.ip[15]);
				break;
			case NET_IP_V6:
			default:
				printf("\n\rError 3");
		}
	}

	if (net_get_mac_address(hnet, &macAddr) == NET_OK) {
		printf("\n\rMac Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
	               macAddr.mac[0], macAddr.mac[1], macAddr.mac[2], macAddr.mac[3], macAddr.mac[4], macAddr.mac[5]);
	}

	/*
	* Fetch the epoch time from st.com and use it to set the RTC time
	*/
	if (setRTCTimeDateFromNetwork(true) != TD_OK) {
		printf("Fail setting time\r\n");
	}
	else {
		printf("Time set, Starting RTC\r\n");
		//RTC started with a 10 -second wake-up interrupt
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20470, RTC_WAKEUPCLOCK_RTCCLK_DIV16); // changed to 20470 for 10 seconds
	}
#ifdef USE_MBED_TLS
	printf("Connecting to MQTT Broker Server using TLS\r\n\n");
	//Create network socket
	//ret = net_sock_create(hnet, &socket, NET_PROTO_TCP);
	connection_security = CONN_SEC_SERVERAUTH;
	//ret = net_sock_create(hnet, &socket, (connection_security == CONN_SEC_NONE) ? NET_PROTO_TCP :NET_PROTO_TLS);
	ret = net_sock_create(hnet, &socket, NET_PROTO_TLS);
#else
	printf("Connecting to MQTT Broker Server\r\n\n");
	ret = net_sock_create(hnet, &socket, NET_PROTO_TCP);
#endif
	if (ret != NET_OK)
	{
		printf("\n\rCould not create the socket.\r\n");
		printf("Check MQTT broker configuration settings.\r\n");
		while(1);
	}
	else
	{
#ifdef USE_MBED_TLS
		//ret |= net_sock_setopt(socket, "sock_noblocking", NULL, 0);
        switch(connection_security)
        {
          case CONN_SEC_MUTUALAUTH:
            ret |= ((checkTLSRootCA() != 0) && (checkTLSDeviceConfig() != 0) )
              || (getTLSKeys(&ca_cert, &device_cert, &device_key) != 0);
            ret |= net_sock_setopt(socket, "tls_server_name", (void *) CloudBroker_HostName, strlen(CloudBroker_HostName) + 1);
            ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
            ret |= net_sock_setopt(socket, "tls_dev_cert",    (void *) device_cert,             strlen(device_cert) + 1);
            ret |= net_sock_setopt(socket, "tls_dev_key",     (void *) device_key,              strlen(device_key) + 1);
            break;
          case CONN_SEC_SERVERNOAUTH:
            ret |= net_sock_setopt(socket, "tls_server_noverification", NULL, 0);
            ret |= (checkTLSRootCA() != 0)
              || (getTLSKeys(&ca_cert, NULL, NULL) != 0);
            ret |= net_sock_setopt(socket, "tls_server_name", (void *) CloudBroker_HostName, strlen(CloudBroker_HostName) + 1);
            ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
              break;
          case CONN_SEC_SERVERAUTH:
            ret |= net_sock_setopt(socket, "tls_server_name", (void *) CloudBroker_HostName, strlen(CloudBroker_HostName) + 1);
            ret |= net_sock_setopt(socket, "tls_ca_certs",    (void *) ca_cert,                 strlen(ca_cert) + 1);
            break;
          case CONN_SEC_NONE:
            break;
          default:
            msg_error("Invalid connection security mode. - %d\n", connection_security);
        }
#endif
        ret |= net_sock_setopt(socket, "sock_noblocking", NULL, 0);
        if (ret != NET_OK)
		{
        	printf("Could not retrieve the security connection settings and set the socket options.\n");
        	while(1);
		}
		else {


			ret = net_sock_open(socket, MQTT_Config.HostName, atoi(CloudBroker_Port), 0);
			if (ret != NET_OK)
			{
				printf("\n\rCould not open the socket.");
				while(1);
			}
			else {
#ifdef USE_MBED_TLS
				printf("\r\nTLS connection established to MQTT Broker Server\r\n\n");
#else
				printf("\r\nConnection established to MQTT Broker Server\r\n\n");
#endif
				HAL_Delay(1000);
			}

			network.my_socket = socket;
			network.mqttread = (network_rd);
			network.mqttwrite = (network_wr);

			MQTTClientInit(client, &network, MQTT_CMD_TIMEOUT, mqtt_send_buffer, MQTT_SEND_BUFFER_SIZE,
					mqtt_read_buffer, MQTT_READ_BUFFER_SIZE);

			/* MQTT connect */
			options.clientID.cstring = MQTT_Config.MQClientId;
			options.username.cstring = MQTT_Config.MQUserName;
			options.password.cstring = MQTT_Config.MQUserPwd;

			HAL_Delay(1000);

			printf("Connecting client to MQTT Broker\r\n\n");
			ret = MQTTConnect(client, &options);
			if (ret != 0)
			{
				printf("\n\rMQTTConnect() failed: %ld\n", ret);
				printf("Check MQTT client credential settings.\r\n");
				while(1);
			}
			else
			{
				printf("\n\rClient Connected to MQTT Broker\r\n");
				HAL_Delay(1000);
			}
		}
        HAL_Delay(1000);
	}
}


void turnSOSOff(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    mqmsg.qos = QOS0;
    mqmsg.payload = "0"; // 0 = off
    mqmsg.payloadlen = 1;
    MQTTPublish(&client, "G00434671/feeds/sosstatus", &mqmsg);
}

void turnSOSOn(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    mqmsg.qos = QOS0;
     mqmsg.payload = "1"; // 1 = on
    mqmsg.payloadlen = 1;
    MQTTPublish(&client, "G00434671/feeds/sosstatus", &mqmsg);

}

