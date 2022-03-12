#include "MicroBit.h"

#define DATA                 1
#define DATA_REQUEST         2
#define NEAR_BY_BOAT_REQUEST 3
#define NEAR_BY_BOAT_DATA    4
#define RSSI_DATA            5

#define CRASH_EVENT          10
#define POSSIBLE_CRASH       11
#define SHAKE_THRESHOLD      60
#define PERSON_IN_CAR        20
#define PERSON_IN_CAR_2      21
#define SPEED                30
#define ROADTYPE             32


MicroBit uBit;
ManagedString serial;
ManagedString emptySerialValue("0000");
int messageCount = 0;
int heartBeat = 0;

int microbitType = 0;//default as sensor

int recordTemp = 1; // 1 = record
int recordDirection = 1;
int recordAcc = 1;

const char * const oneWay ="\
    000,000,255,000,000\n\
    000,255,255,255,000\n\
    000,000,255,000,000\n\
    000,000,255,000,000\n\
    000,000,255,000,000\n";

const char * const twoWay ="\
    000,255,000,255,000\n\
    000,255,000,255,000\n\
    000,255,000,255,000\n\
    000,255,000,255,000\n\
    000,255,000,255,000\n";

static Pin *pin = &uBit.audio.virtualOutputPin;
static uint8_t pitchVolume = 0xff;

enum Note {
    C = 262,
    CSharp = 277,
    D = 294,
    Eb = 311,
    E = 330,
    F = 349,
    FSharp = 370,
    G = 392,
    GSharp = 415,
    A = 440,
    Bb = 466,
    B = 494,
    C3 = 131,
    CSharp3 = 139,
    D3 = 147,
    Eb3 = 156,
    E3 = 165,
    F3 = 175,
    FSharp3 = 185,
    G3 = 196,
    GSharp3 = 208,
    A3 = 220,
    Bb3 = 233,
    B3 = 247,
    C4 = 262,
    CSharp4 = 277,
    D4 = 294,
    Eb4 = 311,
    E4 = 330,
    F4 = 349,
    FSharp4 = 370,
    G4 = 392,
    GSharp4 = 415,
    A4 = 440,
    Bb4 = 466,
    B4 = 494,
    C5 = 523,
    CSharp5 = 555,
    D5 = 587,
    Eb5 = 622,
    E5 = 659,
    F5 = 698,
    FSharp5 = 740,
    G5 = 784,
    GSharp5 = 831,
    A5 = 880,
    Bb5 = 932,
    B5 = 988,
};

//request for temp, directional bearing.
    //on signal - request rssi (sensor to sensor)
    //on return signal - get rssi (sensor to infrastructure)

class MakeCodeMicrophoneTemplate {
  public:
    MIC_DEVICE microphone;
    LevelDetectorSPL level;
    MakeCodeMicrophoneTemplate() MIC_INIT { MIC_ENABLE; }
};
bool inRange(unsigned low, unsigned high, unsigned x)        
{        
 return (low <= x && x <= high);         
} 

ManagedString getDirection()
{
    int x =  uBit.compass.heading();
    if(inRange(349,360,x) || inRange(0,11,x)){
        return "N--";
    }
    else if(inRange(12,33,x)){
        return "NNE";
    }
    else if(inRange(34,56,x)){
        return "NE-";
    }
    else if(inRange(57,78,x)){
        return "ENE";
    }
    else if(inRange(79,101,x)){
        return "E--";
    }
    else if(inRange(102,123,x)){
        return "ESE";
    }
    else if(inRange(124,146,x)){
        return "SE-";
    }
    else if(inRange(147,168,x)){
        return "SSE";
    }
    else if(inRange(169,191,x)){
        return "S--";
    }
    else if(inRange(192,213,x)){
        return "SSW";
    }
    else if(inRange(214,236,x)){
        return "SW-";
    }
    else if(inRange(237,258,x)){
        return "WSW";
    }
    else if(inRange(259,281,x)){
        return "W--";
    }
    else if(inRange(282,303,x)){
        return "WNW";
    }
    else if(inRange(304,326,x)){
        return "NW-";
    }
    else if(inRange(327,348,x)){
        return "NNW";
    }
	return "---";
    //for loop of each section increase 
}
void analogPitch(int frequency, int ms) {
    if (frequency <= 0 || pitchVolume == 0) {
        pin->setAnalogValue(0);
    } else { 
        pin->setAnalogValue(100);
        pin->setAnalogPeriodUs(1000000/frequency);
    }
    if (ms > 0) {
        fiber_sleep(ms);
        pin->setAnalogValue(0);
        fiber_sleep(5);
    }
}
ManagedString convertTwo(char data, char data2)
{
    ManagedString sh1(data);
    ManagedString sh2(data2);

    ManagedString finalDirection(sh1+sh2);
    return finalDirection;
}
ManagedString convertDirection(char data, char data2, char data3)
{
    ManagedString sh1(data);
    ManagedString sh2(data2);
    ManagedString sh3(data3);

    ManagedString finalDirection(sh1+sh2+sh3);
    return finalDirection;
}	
ManagedString convertSerials(char data, char data2, char data3, char data4)
{
    ManagedString sh1(data); 
    ManagedString sh2(data2);
    ManagedString sh3(data3);
    ManagedString sh4(data4);
    
    ManagedString finalSerial(sh1+sh2+sh3+sh4);
    return finalSerial;
}
bool broadcast(PacketBuffer b)
{
    int resp = uBit.radio.datagram.send(b);
    if (resp == MICROBIT_OK) {
        //uBit.display.print(success);
    } else {
        //uBit.display.print(fail);
    }
    return resp;
}
void requestNearByBoats()
{
    PacketBuffer data(9);
    uint8_t *buf = data.getBytes();
    int msgType = NEAR_BY_BOAT_REQUEST;
    memcpy(buf, &msgType, 1);
    memcpy(buf+1, serial.toCharArray(), 4);
    memcpy(buf+5, emptySerialValue.toCharArray(), 4);

    broadcast(data);
}
int getAccerlation()
{
    int x = uBit.accelerometer.getX();
    int y = uBit.accelerometer.getY();
    int z = uBit.accelerometer.getZ();

    return sqrt((x^2) + (y^2) + (z^2));
}
void flash()
{
    uBit.display.disable();
    uBit.io.row1.setDigitalValue(1);
    uBit.io.row2.setDigitalValue(1);
    uBit.io.row3.setDigitalValue(1);
    uBit.io.row4.setDigitalValue(1);
    uBit.io.row5.setDigitalValue(1);

    for (int i = 0; i < 3; i++) 
    {
        uBit.io.col1.setDigitalValue(0);
        uBit.io.col2.setDigitalValue(0);
        uBit.io.col3.setDigitalValue(0);
        uBit.io.col4.setDigitalValue(0);
        uBit.io.col5.setDigitalValue(0);
        uBit.sleep(500);

        uBit.io.col1.setDigitalValue(1);
        uBit.io.col2.setDigitalValue(1);
        uBit.io.col3.setDigitalValue(1);
        uBit.io.col4.setDigitalValue(1);
        uBit.io.col5.setDigitalValue(1);
        uBit.sleep(500);
    }
    uBit.display.enable();
}
void playSOS() {
    const int beat = 250;
	const int beat2 = 500;
    analogPitch(Note::E, beat);
    uBit.sleep(250);
    analogPitch(Note::E, beat);
    uBit.sleep(250);
    analogPitch(Note::E, beat);
    uBit.sleep(250);
    analogPitch(Note::E, beat2);
    uBit.sleep(250);
    analogPitch(Note::E, beat2);
    uBit.sleep(250);
    analogPitch(Note::E, beat2);
    uBit.sleep(250);
    analogPitch(Note::E, beat);
    uBit.sleep(250);
    analogPitch(Note::E, beat);
    uBit.sleep(250);
    analogPitch(Note::E, beat);
  
}
void sendCrashSignal()
{		
	PacketBuffer data(9);
	uint8_t *buf = data.getBytes();
	int msgType = CRASH_EVENT;
	memcpy(buf, &msgType, 1);
	memcpy(buf+1, serial.toCharArray(), 4);
	memcpy(buf+5, emptySerialValue.toCharArray(), 4);	

	broadcast(data);
}
void checkCollision(MicroBitEvent e)
{	
    int acceleration = getAccerlation();
	if(acceleration>SHAKE_THRESHOLD)
    {
		sendCrashSignal();
	}  
}
void requestData()
{      
    PacketBuffer data(13);
    uint8_t *buf = data.getBytes();
    int msgType = DATA_REQUEST;
    memcpy(buf, &msgType, 1);
	memcpy(buf+1, serial.toCharArray(), 4);
	memcpy(buf+5, emptySerialValue.toCharArray(), 4);
    memcpy(buf+9, &heartBeat, 1);

    broadcast(data);
    //uBit.serial.printf("I'm hedddddddddddddre\n");
}
void receive(MicroBitEvent e)
{
    PacketBuffer p = uBit.radio.datagram.recv();
    
    if(microbitType == 0)
    {   
        //uBit.serial.printf("not here\n");
        //------------------
        if(p[0] == DATA_REQUEST && (convertSerials(p[5],p[6],p[7],p[8]) == "0000"))
        {
            ManagedString source(convertSerials(p[1],p[2],p[3],p[4]));
            ManagedString direction;
            ManagedString temp;
            ManagedString acc(getAccerlation());
            int hb = p[9];

            if(recordTemp == 1)
            {
                temp = uBit.thermometer.getTemperature();
            }
            if(recordDirection == 1)
            {
                direction = getDirection();
            }
            if(recordAcc == 0)
            {
                acc = "---";
                //uBit.serial.printf("not recording acc");
            }
            if(recordTemp == 0)
            {
                temp = "---";
                //uBit.serial.printf("not recording temp");
            }
            if(recordDirection == 0)
            {
                direction = "---";
                //uBit.serial.printf("not recording direction");
            }

            PacketBuffer data(17);
            uint8_t *buf = data.getBytes();
            int msgType = DATA;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, source.toCharArray(), 4);
            memcpy(buf+9, direction.toCharArray(), 3);
            memcpy(buf+12, temp.toCharArray(), 2);
            memcpy(buf+14, acc.toCharArray(), 2);
            memcpy(buf+16, &hb, 1);

            broadcast(data);

            requestNearByBoats();
        }
        else if(p[0] == NEAR_BY_BOAT_REQUEST)
        {
            ManagedString source(convertSerials(p[1],p[2],p[3],p[4]));//destination serial
                
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = NEAR_BY_BOAT_DATA;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, source.toCharArray(), 4);
            //memcpy(buf+9, pairedMbitSerial.toCharArray(),4);//for use on mBuoy
            broadcast(data);
        }
        else if(p[0] == NEAR_BY_BOAT_DATA && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            ManagedString NBBSerial(convertSerials(p[1],p[2],p[3],p[4]));//serial of nearby boat
            int rssiValue = p.getRSSI();
            ManagedString n(rssiValue);

            PacketBuffer data(16);
            uint8_t *buf = data.getBytes();
            int msgType = RSSI_DATA;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, emptySerialValue.toCharArray(), 4);
            memcpy(buf+9, NBBSerial.toCharArray(), 4);
            memcpy(buf+13, n.toCharArray(), 3);
                
            broadcast(data);
        }
        else if(p[0] == POSSIBLE_CRASH && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            create_fiber(playSOS);
            create_fiber(flash);
        }
        else if(p[0] == PERSON_IN_CAR && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            uBit.display.printAsync("X");
        }
        else if(p[0] == PERSON_IN_CAR_2 && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            uBit.display.printAsync(" ");
        }
        else if(p[0] == SPEED && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            ManagedString temp(convertTwo(p[9],p[10]));
            uBit.display.print(temp);
        }
        else if(p[0] == ROADTYPE && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
        {
            ManagedString temp(convertTwo(p[9],p[10]));
            MicroBitImage success(oneWay);
            MicroBitImage succesaaas(twoWay);
            ManagedString one(10);
            ManagedString two(20);
            uBit.display.print(success);
            if(temp == one)
            {
                uBit.display.print(success);
            }
            else if(temp == two)
            {
                uBit.display.print(succesaaas);
            }
        }
    }
    else if (microbitType == 1)
    {
        //uBit.serial.printf("I'm here\n");
        if ((p[0] == DATA) && (convertSerials(p[5],p[6],p[7],p[8]) == serial))
	    {
            ManagedString comma(",");
            ManagedString objSerial(convertSerials(p[1],p[2],p[3],p[4]));
            ManagedString Direction(convertDirection(p[9],p[10],p[11]));
            ManagedString temp(convertTwo(p[12],p[13]));
            ManagedString acc(convertTwo(p[14],p[15]));
            int hjk = p[16];
           
            ManagedString hb2(hjk);

            ManagedString header("Data");
            ManagedString end("\n");
            ManagedString ID(messageCount);
            int rssiValue = p.getRSSI();
            ManagedString n(rssiValue);
            
            

            ManagedString messagePart1 = ID + comma +header + comma + serial + comma + objSerial + comma + Direction;
            ManagedString messagePart2 = comma + temp + comma + acc ; // message to send to webserver
            ManagedString messagePart3 = comma + n + comma + hb2 + end;
            messageCount = messageCount +1;

            uBit.serial.printf(messagePart1.toCharArray());
            uBit.serial.printf(messagePart2.toCharArray());
            uBit.serial.printf(messagePart3.toCharArray());
        }
        else if ((p[0] == RSSI_DATA) && (convertSerials(p[5],p[6],p[7],p[8]) == "0000"))
	    {
            ManagedString comma(",");
            ManagedString objSerial(convertSerials(p[1],p[2],p[3],p[4]));
            ManagedString nearBoatSerial(convertSerials(p[9],p[10],p[11],p[12]));
            ManagedString rssiValue(convertDirection(p[13],p[14],p[15]));
            ManagedString header("NBBData");
            ManagedString end("\n");
            ManagedString ID(messageCount);
            ManagedString hb(heartBeat);

            ManagedString message = ID + comma + header + comma + objSerial + comma + nearBoatSerial + comma + rssiValue + comma + hb + comma + end;
            messageCount = messageCount +1;
            uBit.serial.printf(message.toCharArray());
        }
	else if (p[0] == CRASH_EVENT && (convertSerials(p[5],p[6],p[7],p[8]) == emptySerialValue))
	{
		ManagedString comma(",");
		ManagedString header("Crash");
		ManagedString boatSerial(convertSerials(p[1],p[2],p[3],p[4]));
		ManagedString ID(messageCount);
	        ManagedString end("\n");

		ManagedString message = ID + comma + header + comma + boatSerial + comma +end; // message to send to webserver

		messageCount = messageCount +1;

		uBit.serial.printf(message.toCharArray());

    	}
    }
}

void recieveLoop()
{
    ManagedString address;
    ManagedString end("\n");
    ManagedString x;
    ManagedString messageToSend;
    //uBit.serial.printf("I'm here\n");
    while(1)
    {
        x = uBit.serial.readUntil(end);
        int len =  x.length();
        address = x.substring(0,4);
        ManagedString v = x.substring(4,2);
        uBit.serial.printf(address.toCharArray());
        messageToSend = x.substring(6,2);
        uBit.serial.printf(v.toCharArray());
        uBit.serial.printf("\n");
        uBit.serial.printf(messageToSend.toCharArray());
        uBit.serial.printf("\n");

        if(v == 1){//record temp
            recordTemp = 1;
        }
        if(v == 2){//dont record temp
            recordTemp = 0;
        }
        if(v == 3){//record direc
            recordDirection = 1;
        }
        if(v == 4){//dont record direc
            recordDirection = 0;
        }
        if(v == 5){//record aCC
            recordAcc = 1;
        }
        if(v == 6){//dont record ACC
            recordAcc = 0;
        }
        if(v == 7){//record TYPE
            microbitType = 1;
            uBit.serial.printf("infra\n");
        }
        if(v == 8){//dont record TYPE
            microbitType = 0;
            uBit.serial.printf("sensor\n");
        }
        if(v == 10)//alert to possible crash
        {
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = POSSIBLE_CRASH;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, address.toCharArray(), 4);

            broadcast(data);
        }
        if(v == 50)//alert to possible crash
        {
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = PERSON_IN_CAR;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, address.toCharArray(), 4);

            broadcast(data);
        }
        if(v == 51)//alert to possible crash
        {
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = PERSON_IN_CAR_2;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, address.toCharArray(), 4);

            broadcast(data);
        }
        if(v == 40)//speed 
        {
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = SPEED;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, address.toCharArray(), 4);
            memcpy(buf+9, messageToSend.toCharArray(), 2);

            broadcast(data);
        }
        if(v == 45)//speed 
        {
            PacketBuffer data(13);
            uint8_t *buf = data.getBytes();
            int msgType = ROADTYPE;
            memcpy(buf, &msgType, 1);
            memcpy(buf+1, serial.toCharArray(), 4);
            memcpy(buf+5, address.toCharArray(), 4);
            memcpy(buf+9, messageToSend.toCharArray(), 1);

            broadcast(data);
        }
    
    }
}
int main()
{
    uBit.init();
    uBit.serial.setBaud(115200);
    uBit.compass.calibrate();
   
    ManagedString s(uBit.getSerial());
    serial = s.substring(9,4);
	
	uBit.messageBus.listen(MICROBIT_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, receive);
	//uBit.messageBus.listen(DEVICE_ID_BUTTON_A, DEVICE_BUTTON_EVT_CLICK, onButtonA);//set group
	//uBit.messageBus.listen(DEVICE_ID_BUTTON_B, DEVICE_BUTTON_EVT_CLICK, onButtonB);//set pin
    //uBit.messageBus.listen(DEVICE_ID_BUTTON_B, MICROBIT_BUTTON_EVT_HOLD, onButtonBHold);//confirm number
 	//uBit.messageBus.listen(DEVICE_ID_BUTTON_A, MICROBIT_BUTTON_EVT_LONG_CLICK, onButtonAHold);//display boat serial

	new MakeCodeMicrophoneTemplate(); 
    uBit.messageBus.listen(DEVICE_ID_MICROPHONE, LEVEL_THRESHOLD_HIGH,checkCollision);

    uBit.radio.enable();
	
	uBit.sleep(1000);
    //uBit.serial.printf(" here\n");

    create_fiber(recieveLoop);
    while(1)
    {	
		uBit.sleep(5000);
        if (microbitType == 1)
        {
            heartBeat++;
            requestData();  
            //uBit.serial.printf("requestData\n");
        }
        uBit.sleep(5000);
    }
}
	
