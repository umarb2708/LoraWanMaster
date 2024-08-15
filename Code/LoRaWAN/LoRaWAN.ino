//----- Include Library.
#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h> 
//----------------------------------------

//--- LoRa Pin / GPIO configuration.
#define ss 15
#define rst 14
#define dio0 2
//----------------------------------------
//----Display Verbosity
//0-Low, 1- Medium 2-High 3-Debug 
int verbosity=0;
int disp_lora=1;
int disp_mqtt=1;
int disp_eeprom=0;
int disp_loop=0;
//--------------------
//--- Variable declaration to hold incoming and outgoing data.
String Incoming = "";
String Message = "";  
int n_slv=0;            //Number of slaves. Read from EEPROM 
int curr_slv=1;         //Current slave going to send msg
//-- LoRa data transmission configuration.
byte LocalAddress = 0x00;               //--> address of this device (Master Address).
byte Destination_ESP32_Slave= 0x01;  //--> destination to send to Send. Read from EEPROM  using curr_slv.
//---------------------------------------- 
int init_mem=0;
//---------------------------------------- Variable declaration for Millis/Timer.
unsigned long previousMillis_SendMSG = 0;
const long interval_SendMSG = 10000;
//---------------------------------------- 

//-----MQTT & WIFI---------------------
const char* ssid = "Sweet Home FTTH";
const char* password = "Umar@WIFI123#";
const char* mqtt_server = "innovize.local";  
const char *MQTT_USER = "innovize";
const char *MQTT_PASSWORD = "Admin@IS123#";   
String topic="";
String payload="";    
//----------------------------------------

WiFiClient espClient;
PubSubClient client(espClient);

//_______ Subroutines for sending data (LoRa Ra-02)_____________________________
void sendLoraMessage(String Outgoing, byte Destination) {
  Serial.println("------------LoRa Transmition----------------");
  Serial.println("Destination:"+String(Destination));
  Serial.println("LocalAddress:"+String(LocalAddress));
  Serial.println("Data Len:"+String(Outgoing.length()));
  Serial.println("Data:"+String(Outgoing));
  Serial.println("--------------------------------------------");

  LoRa.beginPacket();             //--> start packet
  LoRa.write(Destination);        //--> add destination address
  LoRa.write(LocalAddress);       //--> add sender address
  LoRa.write(Outgoing.length());  //--> add payload length
  LoRa.print(Outgoing);           //--> add payload
  LoRa.endPacket();               //--> finish packet and send it
}
//________________________________________________________________________________ 

//_________________Subroutines for receiving data (LoRa Ra-02)____________________
void onReceive(int packetSize) {
  if (packetSize == 0) return;  //--> if there's no packet, return

  //----read packet header bytes:
  int recipient = LoRa.read();        //--> recipient address
  byte sender = LoRa.read();          //--> sender address
  byte incomingLength = LoRa.read();  //--> incoming msg length
  //---------------------------------------- 

  // Clears Incoming variable data.
  Incoming = "";

  //-------------Get all incoming data.
  while (LoRa.available()) {
    Incoming += (char)LoRa.read();
  }
  //---------------------------------------- 

  //------ Check length for error.
  if (incomingLength != Incoming.length()) {
    Serial.println("error: message length does not match length");
    return; //--> skip rest of function
  }
  //---------------------------------------- 

  //-- Checks whether the incoming data or message for this device.
  if (recipient != LocalAddress) {
    Serial.println("This message is not for me.");
    return; //--> skip rest of function
  }
  //---------------------------------------- 

  //------if message is for this device, or broadcast, print details:
  Serial.println("--------------------LoRa Reception--------------------");
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + Incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("-----------------------------------------------------");
  //---------------------------------------- 
  String rcvData=String(sender,HEX)+"|"+Incoming;
  client.publish("HA/slave/data", rcvData.c_str());
}
//________________________________________________________________________________ 

//_________________________Connect WIFI for MQTT___________________________________
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//________________________________________________________________________________

//_____________________MQTT Message Reception_____________________________________
void callback(char* msg, byte* data, unsigned int length) {

  String data_s="";
  String msg_s=String(msg);//Converting char array to string for handeling 
  for (int i = 0; i < length; i++) {
    data_s=data_s+(char)data[i];
  }
  //Printing Topic and Payload for debugging
  Serial.println("----------MQTT Message Received--------");
  Serial.print("MQTT Message:");
  Serial.println(msg_s);
  Serial.print("MQTT data:");
  Serial.println(data_s);
  Serial.println();
  Serial.println("--------------------------------------");
  if(msg_s=="LoRaWAN/Topic")
  {
    //If subscribed to topic, Then store to topic variable
    topic=data_s;
  }
  else if (msg_s=="LoRaWAN/Payload")
  {
    //If subscribed to payload, Then store to payload variable
    payload=data_s;
  }
  else
  {
    //If subscribed to unknown, Then publish error 
    Serial.println("Unidentified Message!!!");
  }



  //Physical Identification of reception of payload 
  // Switch on the LED if a not Null payload recieved
  if (payload != "") 
  {
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
    digitalWrite(BUILTIN_LED, LOW);   
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

  //Execute the logic if Topic and Payload Recieved
  if (payload != "" && topic != "") 
  {
    exec_topic(topic,payload);

    //Clear variable after executing 
    payload="";
    topic="";
    //Turn of the LED after execution
    digitalWrite(BUILTIN_LED, HIGH);
  }



}

//____________________________MQTT Message Publishing_____________________________
void PublishMqttMsg(char* msg , char* data)
{
  Serial.println("----------MQTT Message Publish--------");
  Serial.print("MQTT Message:");
  Serial.println(msg);
  Serial.print("MQTT data:");
  Serial.println(data);
  Serial.println();
  Serial.println("--------------------------------------");
  client.publish(msg, data);
}
//________________________________________________________________________________
//_________________________Reconnect MQTT Server___________________________________
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "LoRaWAN-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      Serial.println("Publishing connection status....");
      // Once connected, publish an announcement...
      client.publish("LoRaWAN/Connection", "1");
      // ... and resubscribe
      Serial.println("Subscribing Topics....");
      client.subscribe("LoRaWAN/Topic");
      client.subscribe("LoRaWAN/Payload");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//________________________________________________________________________________

//______________________________EEPROM Handling___________________________________
void init_eeprom(){
  EEPROM.begin(64);
  /*
  Addr 0x0: LoRaWan Register
          bit 0   -> init_done
          bit 3:1 -> Master address
          bit 7:4 -> Number of clients
  Addr 0x1 - 0x63 : Slave register (Each slave data will be stored)
          bit 0   -> Slave enavle
          bit 3:1 -> Address of slave
          bit 7:4 -> Reserved for now

  */
  if(EEPROM.read(0)==255 | init_mem){
    Serial.println("Initializing EEPROM");
    EEPROM.write(0x0,0x11);
    //Initializing with one slave (0x01)
    EEPROM.write(0x1,0x2);
    if (EEPROM.commit()) {
      Serial.println("EEPROM successfully committed");
    } else {
      Serial.println("ERROR! EEPROM commit failed");
    } 
  }
}
void write_eeprom(int addr, int data){
  EEPROM.write(addr,data);
  if (EEPROM.commit()) {
      Serial.println("EEPROM successfully Written");
  } else {
      Serial.println("ERROR! EEPROM commit failed");
  }
}

int read_eeprom(int addr){
  int rddata=EEPROM.read(addr);
  if(disp_eeprom){
    Serial.println("---------EEPROM READ---------");
    Serial.println("Addr:"+String(addr));
    Serial.println("Data:"+String(rddata));
    Serial.println("-----------------------------");
  }
  return rddata;
}
//________________________________________________________________________________
//_________ VOID SETUP____________________________________________________________
void setup() {

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883 );
  client.setCallback(callback);

  //--- Settings and start Lora Ra-02.
  LoRa.setPins(ss, rst, dio0);
  Serial.println("Start LoRa init...");
  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 or 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");
  //----------------------------------------

  //---Initializing EE PROM
  init_eeprom(); 
  //------------------------
}
//________________________________________________________________________________ 

//____________________________________ VOID LOOP___________________________________
void loop() {
  // put your main code here, to run repeatedly:

  //---------------------------------------- Millis or Timer to send message / command data to slaves every 1 second (see interval_SendCMD variable).
  // Messages are sent every one second is alternately.
  
  unsigned long currentMillis_SendMSG = millis();
  
  if (currentMillis_SendMSG - previousMillis_SendMSG >= interval_SendMSG) {
    previousMillis_SendMSG = currentMillis_SendMSG;

    n_slv=byte_to_bit(read_eeprom(0),7,4);
    Serial.println("Num Slave="+String(n_slv));

    Destination_ESP32_Slave=byte_to_bit(read_eeprom(curr_slv),3,1);


    Message = "STATUS";

    //::::::::::::::::: Condition for sending message / command data to Slave

      sendLoraMessage(Message, Destination_ESP32_Slave);
    //:::::::::::::::::
  }
  //----------------------------------------
  //parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  //----------------------------------------

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
//________________________________________________________________________________ 

//_________________Execute the Topic and Payload received via MQTT______________
void exec_topic(String topic,String payload){
  String task="",subtask="";
  int slash=topic.indexOf('/');
  task = topic.substring(0,slash);
  subtask=topic.substring(slash+1);
  Serial.println("-------Exec Topic--------");
  Serial.print("Topic:");
  Serial.println(task);
  Serial.print("Sub Topic:");
  Serial.println(subtask);
  Serial.println("-------------------------");
  if(task == "config") config(subtask,payload);
  else if(task == "slaves") slaves(subtask,payload);
  else if(task == "control") control(subtask,payload);
  else Serial.println("Unknown Task");
}
//__________________________________________________________________________________
//Below are the task should done when a MQTT message recieved by LoRaWAN
void config(String subTopic,String Payload)
{
  Serial.println("----------CONFIG TASK-------------");

}
void slaves(String subTopic,String Payload)
{
  Serial.println("----------SLAVES TASK-------------");
}
void control(String subTopic,String Payload)
{
  Serial.println("----------CONTROL TASK-------------");
}



String find_task(String topic)
{
  String task="";
  int firstSlash=topic.indexOf('/');
  int secondSlash=topic.indexOf('/',firstSlash+1);
  task = topic.substring(firstSlash+1,secondSlash);
  return task;
}
String find_subtask(String topic)
{
  String subtask="";
  int firstSlash=topic.indexOf('/');
  int secondSlash=topic.indexOf('/',firstSlash+1);
  subtask = topic.substring(secondSlash+1);
  return subtask;
}

int byte_to_bit(byte data,int msb, int lsb){
  String result="";
  int ret_data=0;
  for(int i=msb;i>=lsb;i--){
    result=result+String(bitRead(data, i));
  }
  //Serial.println("DATA:"+String(data));
  //Serial.println("Result:"+result);
  unsigned long res = strtoul(result.c_str(), NULL, 2);
  ret_data=int(res);
  return ret_data;

}