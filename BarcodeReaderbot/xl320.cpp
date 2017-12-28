#include "xl320.h"


XL320::XL320(QObject *parent) : QObject(parent)
{

}
void XL320::Init(QString portName,int baudRate)
{
 if(Media::MEDIA_ERROS_OK !=   _media.Init(portName,baudRate,2))
 {


     qDebug("error open device");
 }
 else{
 qDebug("port connect ok");}
}
void XL320::SetLed(unsigned char id, bool status)
{
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = LED_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = RAM_LED;
//    Instruction_Packet_Array[4] = Status;
    QByteArray packet;
    packet.push_back(id);
    packet.push_back(LED_LENGTH);
    packet.push_back(COMMAND_WRITE_DATA);
    packet.push_back(RAM_LED);
    packet.push_back(status);
    SendCommand(packet);


}

void XL320::SetTorqueEnable(unsigned char id, bool status)
{
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = LED_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = RAM_LED;
//    Instruction_Packet_Array[4] = Status;
    QByteArray packet;
    packet.push_back(id);
    packet.push_back(SET_HOLDING_TORQUE_LENGTH);
    packet.append((const char)0x00);
    packet.append(COMMAND_WRITE_DATA);
    packet.append(RAM_TORQUE_ENABLE);
    packet.append((const char)0x00);
    packet.push_back(status);
    SendCommand(packet);


}

unsigned int XL320::SetPosition(unsigned char id,  int speed){
    QByteArray packet;
    packet.append(id);
    packet.append(0x09);
    packet.append((const char)0x00);
    packet.append(0x03);
    packet.append(0x1E);
    packet.append((const char)0x00);




   // packet.append((speed & 0xff));
  //  packet.append((const char)0x00);
//packet.append(((speed>>8) & 0xff));
//packet.append((const char)0x00);
packet.append((const char)0x00);
packet.append((const char)0x00);
    packet.append(speed & 0xff);
    packet.append(((speed>>8) & 0xff));
    SendCommand(packet);
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
//    Instruction_Packet_Array[4] = byte(Position);
//    Instruction_Packet_Array[5] = byte((Position & 0x0F00) >> 8);
//    Instruction_Packet_Array[6] = byte(Speed);
//    Instruction_Packet_Array[7] = byte((Speed & 0x0F00) >> 8);

//    clearRXbuffer();

//    transmitInstructionPacket();


//    if (ID                                            == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
//        return (0x00);
//    }else{
//        readStatusPacket();
//        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//            return (Status_Packet_Array[0]);            // Return SERVO ID
//        }else{
//            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//        }
//    }

}

bool XL320::GetSensor(unsigned int *result){
    QString command = "sensor";
    QByteArray reply;

    _media.WriteCommand(command.toLatin1(),reply,6,8);
    //qDebug("reply: " + reply.trimmed());
    bool ok;
    int num = 0;
    num = reply.trimmed().toInt(&ok);
    if(!ok || reply.length() < 1)
        return false;

    *result = ((double)(num+18000)/100);
    return true;
    /*QByteArray packet,reply;
    packet.append(0x61);
    reply = SendCommand(packet);
    //QThread::msleep(1);
    reply = SendCommand(packet);
    //QThread::msleep(1);
    reply = SendCommand(packet);

    reply = SendCommand(packet);
    qDebug(reply);*/
}

void XL320::Close()
{
    _media.Disconnect();
}

unsigned int XL320::ReadPosition(unsigned char id){
    QByteArray packet,replay;
    packet.push_back(id);
    packet.push_back(READ_POS_LENGTH);
    packet.push_back(COMMAND_READ_DATA);
    packet.push_back(RAM_PRESENT_POSITION_L);
    packet.push_back(READ_TWO_BYTE_LENGTH);
    replay=SendCommand(packet);
if(replay.length()<3)return 0;
    if((int)(replay[2])!=id || replay.length()<7){

        return 0;
    }
    return replay[6]<<8+replay[5];

}
unsigned int XL320::ReadVoltage(int id){
    QByteArray packet,replay;
    packet.push_back(id);
    packet.push_back(READ_VOLT_LENGTH);
    packet.push_back((const char)NONE);
    packet.push_back(COMMAND_READ_DATA);
    packet.push_back(RAM_PRESENT_VOLTAGE);
    packet.push_back(READ_ONE_BYTE_LENGTH);
    replay=SendCommand(packet);
if(replay.length()<3)return 0;
if((int)(replay[2])!=id || replay.length()<6){

    return 0;
}


return replay[5];
}
unsigned int XL320::ReadTemperature(int id){
    QByteArray packet,replay;
    packet.push_back(id);
    packet.push_back(READ_TEMP_LENGTH);
    packet.push_back(COMMAND_READ_DATA);
    packet.push_back(RAM_PRESENT_TEMPERATURE);
    packet.push_back(READ_ONE_BYTE_LENGTH);
    replay=SendCommand(packet);
    if(replay.length()<3)return 0;
    if((int)(replay[2])!=id || replay.length()<6){

        return 0;
    }
return replay[5];

}
unsigned int XL320::Ping(int id){

    QByteArray packet;
    packet.append(id);
    packet.append(0x03);
    packet.append((const char)0x00);
    packet.append(COMMAND_PING);
    SendCommand(packet);
}
QByteArray XL320::SendCommand(QByteArray command)
{
 QByteArray packet;
 packet.push_back(HEADER);
 packet.push_back(HEADER);
 packet.push_back(0xFD);
 packet.push_back((const char)NONE);
 for(int i = 0; i < command.size();i++){
     packet.push_back(command[i]);
 }
 /*packet.push_back(command[0]);
 packet.push_back(command[1]);
 packet.push_back(command[2]);*/
/* unsigned int checksumPacket = command[0] + command[1] + command[2];
 for (unsigned char i = 3; i <= command[1]; i++){
      packet.push_back(command[i]);                                   // Write Instuction & Parameters (if there are any) to serial
      checksumPacket += command[i];
  }
  packet.push_back(~checksumPacket & 0xFF);*/
 int crc = update_crc(0,(unsigned char*)(packet.data()),packet.size());
 packet.push_back(crc & 0xff);
 packet.push_back((crc>>8) & 0xff);
 //QThread::msleep(50);
 //qDebug() <<packet.toHex();
  QByteArray replay;
  _media.WriteCommand(packet,replay,20,20);
  //qDebug() <<replay.toHex();
return replay;
}

         unsigned short XL320::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)

         {

             unsigned short i, j;

             unsigned short crc_table[256] = {

                 0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

                 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

                 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

                 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

                 0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

                 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

                 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

                 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

                 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

                 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

                 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

                 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

                 0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

                 0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

                 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

                 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

                 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

                 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

                 0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

                 0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

                 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

                 0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

                 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

                 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

                 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

                 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

                 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

                 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

                 0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

                 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

                 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

                 0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

             };



             for(j = 0; j < data_blk_size; j++)

             {

                 i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;

                 crc_accum = (crc_accum << 8) ^ crc_table[i];

             }



             return crc_accum;

         }
