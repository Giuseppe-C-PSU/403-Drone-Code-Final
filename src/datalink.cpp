/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Thanakorn Khamvilai (thanakorn.khamvilai@ttu.edu)
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/


#include "datalink.h"
#include "wifi.h"
#include "rc_pilot.h"


struct datalinkWork_ref obDatalinkWork = {
	0 , /* uint itime */
	0 , /* int badChecksums */
	0 , /* int badHeaderChecksums */
};

struct datalinkHeader_ref obDatalinkMessageHeader = {
	0xa3 , /* uchar sync1 */
	0xb2 , /* uchar sync2 */
	0xc1 , /* uchar sync3 */
	0 , /* uchar spare */
	0 , /* int messageID */
	0 , /* int messageSize */
	0 , /* uint hcsum */
	0 , /* uint csum */
};

struct datalinkMessage0_ref obDatalinkMessage0 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* char navStatus */
  0 , /* char gpsStatus */
  0 , /* char aglStatus */
  0 , /* uchar overrun */
  0 , /* char wow */
  0 , /* char autopilot */
  0 , /* char LaunchState */
  0 , /* uchar motor */
  0 , /* float time */
  {0,0,-2} , /* float pos[3] */
  {0,0,0}  , /* float vel[3] */
  {1,0,0,0} , /* float q[4] */
  2 , /* float altitudeAGL */
};

struct datalinkMessage1_ref obDatalinkMessage1 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float time */
  0 , /* char numberOfSats */
  0 , /* char datarecordStatus */
  0 , /* char safemode */
  0 , /* uchar type */
  {0,0,0} , /* float delm[3] */
  {0} , /* float delf[1] */
  {0} , /* float delt[1] */
  {0} , /* float delc[1] */
  12000  , /* int battery */
  0      , /* int current */
  0          , /* int rpm */
  0          , /* char tx */
  0        , /* char fuel */
  0   , /* char ycsStatus */
  0   , /* uchar uniqueID */
  0   , /* char yrdStatus */
  0   , /* char hubStatus */
  0 , /* char rangeFinderStatus */
  0 , /* char magnetStatus */
  {0,0,0}   , /* float traj_x[3] */
  {0,0,0}   , /* float traj_v[3] */
  {0,0,0}   , /* float traj_a[3] */
  {0,0,0,0} , /* float traj_q[4] */
  0         , /* float traj_psi */
  1.0     , /* float traj_vscale */
  0     , /* short traj_manIndex */
  {0} , /* uchar align[1] */
  0 , /* uchar actuatorInterfaceStatus */
  0 , /* uchar imuStatus */
  0 , /* uchar traj_status */
  0 , /* uchar visionStatus */
  0 , /* uchar missionStatus */
  0 , /* uchar otherStatus */
  0  , /* uchar cameraControlStatus */
  0  , /* uchar historyStatus */
  0 , /* uchar batteryStatus */
  {0,0} , /* uchar uplinkStatus[2] */
  0 , /* uchar lostComm */
  0 , /* uchar hokuyoLaserStatus */
  0 , /* uchar ubloxSNR */
  0 , /* uchar ubloxHacc */
  0 , /* uchar ubloxSacc */
  0 , /* uchar ubloxPDOP */
  0.0 , /* float pan */
  0.0 , /* float tilt */
  0.0 , /* float roll */
  58.5 , /* float fovy */
  1.0 , /* float G */
  {0.0,0.0,0.0} , /* float wind[3] */
  {30.0,0.0,0.0} , /* float pointPos[3] */
  33.659653f , /* float datumLat */
  -84.663333f , /* float datumLon */
  745.00f     , /* float datumAlt */
};

struct datalinkMessageUp0_ref obDatalinkMessageUp0 = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* int k */
  0 , /* float time */
  0 , /* float throttleLever */
  0 , /* float rollStick */
  0 , /* float pitchStick */
  0 , /* float rudderPedal */
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} , /* char button[16] */
};

struct datalinkMessagePWM_ref obDatalinkMessagePWM = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} , /* raw_pwm[17] */
  0, /* align */
};

struct datalinkMessageMotorCmd_ref obDatalinkMessageMotorCmd = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* discrete onboard time*/
  0 , /* onboard time */
  {0,0,0,0} , /* motor_cmd[4]*/
};

struct datalinkMessageAutopilotDels_ref obDatalinkMessageAutopilotDels = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float time */
  {0,0,0} , /* float c_delm[3] */
  {0} , /* float c_delf[1] */
  {0} , /* float c_delt[1] */
};

struct datalinkMessageTruth_ref obDatalinkMessageTruth = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* uint align  */
  {0,0,0} , /* float p_b_e_L[3]  */
  {0,0,0} , /* float v_b_e_L[3]  */
  {0,0,0,0} , /* float q[4]  */
};

struct datalinkMessageHITLSim2Onboard_ref obDatalinkMessageSim2Onboard = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float phiCmd */
  0 , /* float posDes_x */
  0 , /* float posDes_y */
  0 , /* float posDes_z */
  0 , /* float nav_p_x  */
  0 , /* float nav_p_y  */
  0 , /* float nav_p_z  */
  0 , /* float nav_v_x  */
  0 , /* float nav_v_y  */
  0 , /* float nav_v_z  */
  0 , /* float nav_w_x  */
  0 , /* float nav_w_y  */
  0 , /* float nav_w_z  */
  0 , /* float nav_phi  */
  0 , /* float nav_theta  */
  0 , /* float nav_psi  */
};

struct datalinkMessageHITLOnboard2Sim_ref obDatalinkMessageOnboard2Sim = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0 , /* float c_delf; */
  0 , /* float c_delm0; */
  0 , /* float c_delm1; */
  0 , /* float c_delm2; */
};

struct datalinkMessageOptitrack_ref obDatalinkMessageOptitrack = {
  0xa3 , /* uchar sync1 */
  0xb2 , /* uchar sync2 */
  0xc1 , /* uchar sync3 */
  0 , /* uchar spare */
  0 , /* int messageID */
  0 , /* int messageSize */
  0 , /* uint hcsum */
  0 , /* uint csum */
  0,
  0,
  0,
  0,
  0,
  0,
  0,
};

struct obDatalink_ref obDatalink = {
  &obDatalinkWork, /* dir work */
	&obDatalinkMessageHeader, /* dir header */
  &obDatalinkMessage0, /* dir m0 */
  &obDatalinkMessage1, /* dir m1 */
  &obDatalinkMessageUp0, /* dir up0 */
  &obDatalinkMessagePWM, /* dir pwm */
  &obDatalinkMessageAutopilotDels, /* dir autopilotDels */
  &obDatalinkMessageTruth, /* dir truthSim */
  &obDatalinkMessageSim2Onboard, /* dir sim2onboard */
  &obDatalinkMessageOnboard2Sim, /* dir onboard2sim */
  &obDatalinkMessageOptitrack, /* dir optitrack */
};


Dlink::Dlink(){

}

Dlink::~Dlink(){}

void Dlink::init(){
  // initialize wifi
  ether.init();
  // initialize datalink
  obDatalink.work = &obDatalinkWork;
  obDatalink.header = &obDatalinkMessageHeader;
  obDatalink.m0 = &obDatalinkMessage0;
  obDatalink.m1 = &obDatalinkMessage1;
  obDatalink.up0 = &obDatalinkMessageUp0;
  obDatalink.pwm = &obDatalinkMessagePWM;
  obDatalink.autopilotDels = &obDatalinkMessageAutopilotDels;
  obDatalink.truthSim = &obDatalinkMessageTruth;
  obDatalink.sim2onboard = &obDatalinkMessageSim2Onboard;
  obDatalink.onboard2sim = &obDatalinkMessageOnboard2Sim;
  obDatalink.optitrack = &obDatalinkMessageOptitrack;
  obDatalink.motors = &obDatalinkMessageMotorCmd;

  set_interval( DATALINK_INTERVALUP0, DATALINK_MESSAGE_UP0 );
  set_interval( DATALINK_INTERVALPWM, DATALINK_MESSAGE_PWM );
  set_interval( DATALINK_INTERVALAUTOPILOTDELS, DATALINK_MESSAGE_AUTOPILOTDELS );
  set_interval( DATALINK_INTERVALTRUTH, DATALINK_MESSAGE_TRUTH );
  set_interval( DATALINK_INTERVALHITL_SIM2ONBOARD, DATALINK_MESSAGE_HITL_SIM2ONBOARD );
  set_interval( DATALINK_INTERVALHITL_ONBOARD2SIM, DATALINK_MESSAGE_HITL_ONBOARD2SIM );
  set_interval( DATALINK_INTERVALOPTITRACK, DATALINK_MESSAGE_OPTITRACK );
}

void Dlink::set_interval( long intervalX, int type ){
  switch ( type ){
    case DATALINK_MESSAGE_UP0:
      obDatalink.up0->messageSize = sizeof( struct datalinkMessageUp0_ref );
      obDatalink.up0->messageID = DATALINK_MESSAGE_UP0;
      this->intervalUp0 = intervalX;
      this->previousMillisUp0 = 0;
      break;
    case DATALINK_MESSAGE_PWM:
      obDatalink.pwm->messageSize = sizeof( struct datalinkMessagePWM_ref );
      obDatalink.pwm->messageID = DATALINK_MESSAGE_PWM;
      this->intervalPWM = intervalX;
      this->previousMillisPWM = 0;
      break;
    case DATALINK_MESSAGE_AUTOPILOTDELS:
      obDatalink.autopilotDels->messageSize = sizeof( struct datalinkMessageAutopilotDels_ref );
      obDatalink.autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
      this->intervalAutopilotDels = intervalX;
      this->previousMillisAutopilotDels = 0;
      break;
    case DATALINK_MESSAGE_TRUTH:
      obDatalink.truthSim->messageSize = sizeof( struct datalinkMessageTruth_ref );
      obDatalink.truthSim->messageID = DATALINK_MESSAGE_TRUTH;
      this->intervalTruthSim = intervalX;
      this->previousMillisTruthSim = 0;
      break;
    case DATALINK_MESSAGE_HITL_SIM2ONBOARD:
      obDatalink.sim2onboard->messageSize = sizeof( struct datalinkMessageHITLSim2Onboard_ref );
      obDatalink.sim2onboard->messageID = DATALINK_MESSAGE_HITL_SIM2ONBOARD;
      this->intervalSim2Onboard = intervalX;
      this->previousMillisSim2Onboard = 0;
      break;
    case DATALINK_MESSAGE_HITL_ONBOARD2SIM:
      obDatalink.onboard2sim->messageSize = sizeof( struct datalinkMessageHITLOnboard2Sim_ref );
      obDatalink.onboard2sim->messageID = DATALINK_MESSAGE_HITL_ONBOARD2SIM;
      this->intervalOnboard2Sim = intervalX;
      this->previousMillisOnboard2Sim = 0;
      break;
    case DATALINK_MESSAGE_OPTITRACK:
      obDatalink.optitrack->messageSize = sizeof( struct datalinkMessageOptitrack_ref );
      obDatalink.optitrack->messageID = DATALINK_MESSAGE_OPTITRACK;
      this->intervalOptitrack = intervalX;
      this->previousMillisOptitrack = 0;
      break;
    default:
      Serial.println("Error: Invalid type for datalink interval");
      break;
  }
}

void Dlink::recv_update(){
  // messages to receive from GCS: TruthSim, Sim2Onboard, Optitrack
  // read datalink optitrack
  readDatalink( &ether.UdpGPS );
  // read datalink gcs
  readDatalink( &ether.UdpGCS );
}

void Dlink::send_update(){
  // messages to send to GCS: AutopiloDels, Up0, PWM, Onboard2Sim

  unsigned long currentMillis = millis();
  if (currentMillis - this->previousMillisAutopilotDels >= this->intervalAutopilotDels) {
    this->previousMillisAutopilotDels = currentMillis;
    writeAutopilotDels( &ether.UdpGCS, &cntrl);
  }

  currentMillis = millis();
  if (currentMillis - this->previousMillisUp0 >= this->intervalUp0) {
    this->previousMillisUp0 = currentMillis;
    writeUP0( &ether.UdpGCS, &rc );
  }

  currentMillis = millis();
  if (currentMillis - this->previousMillisPWM >= this->intervalPWM) {
    this->previousMillisPWM = currentMillis;
    writePWM( &ether.UdpGCS );
  }

  currentMillis = millis();
  if (currentMillis - this->previousMillisSim2Onboard >= this->intervalSim2Onboard) {
    this->previousMillisSim2Onboard = currentMillis;
    readDatalink( &ether.UdpGPS );
  }


}


uint32_t datalinkCheckSumCompute(unsigned char* buf, int32_t byteCount) {

	uint32_t sum1 = 0xffff;
	uint32_t sum2 = 0xffff;
	uint32_t tlen = 0;
	uint32_t shortCount = byteCount / sizeof(short);
	uint32_t oddLength = byteCount % 2;


	/* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

	while (shortCount)
	{
		/* 360 is the largest number of sums that can be performed without overflow */
		tlen = shortCount > 360 ? 360 : shortCount;
		shortCount -= tlen;
		do
		{
			sum1 += *buf++;
			sum1 += ((uint32_t)*buf++ << 8);
			sum2 += sum1;
		} while (--tlen);

		/* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
		if ((oddLength == 1) && (shortCount < 1))
		{
			sum1 += (uint32_t)*buf++;
			sum2 += sum1;
		}

		sum1 = (sum1 & (uint32_t)0xffff) + (sum1 >> 16);
		sum2 = (sum2 & (uint32_t)0xffff) + (sum2 >> 16);
	}

	/* Second reduction step to reduce sums to 16 bits */
	sum1 = (sum1 & (uint32_t)0xffff) + (sum1 >> 16);
	sum2 = (sum2 & (uint32_t)0xffff) + (sum2 >> 16);

	return(sum2 << 16 | sum1);
}

void datalinkCheckSumEncode(unsigned char* buf, uint32_t byteCount) {

	struct datalinkHeader_ref* h = (struct datalinkHeader_ref*)buf;

	h->sync1 = DATALINK_SYNC0;
	h->sync2 = DATALINK_SYNC1;
	h->sync3 = DATALINK_SYNC2;

	h->messageSize = byteCount;

	h->hcsum = datalinkCheckSumCompute(buf, sizeof(struct datalinkHeader_ref) - sizeof(int32_t) * 2);
	h->csum = datalinkCheckSumCompute(&(buf[sizeof(struct datalinkHeader_ref)]), byteCount - sizeof(struct datalinkHeader_ref));
}

void readDatalink( WiFiUDP* wf )
{
	struct obDatalink_ref* data = &obDatalink;

	int index, done;
	unsigned char* bf;
	void* dataPtr;
	int size;
	FILE* filep;

	int packetSize = wf->parsePacket();

	if ( packetSize == 0 ) return;

	done = 0;
	index = 0;

  int bytesread = wf->read(ether.buffer, BUFFERSIZE);

	while ( ( index <= bytesread - ( int ) sizeof( struct datalinkHeader_ref ) ) && !done )
	{
		if ( ( ether.buffer[index] == DATALINK_SYNC0 ) &&
				 ( ether.buffer[index + 1] == DATALINK_SYNC1 ) &&
				 ( ether.buffer[index + 2] == DATALINK_SYNC2 ) )
		{
			bf = &( ether.buffer[index] );
			memcpy( data->header, bf, sizeof( struct datalinkHeader_ref ) );

			if ( datalinkCheckSumCompute( bf, sizeof( struct datalinkHeader_ref ) - sizeof( int ) * 2 ) == data->header->hcsum &&
					 data->header->messageSize >= sizeof( struct datalinkHeader_ref ) &&
					 data->header->messageSize < BUFFERSIZE )
			{
				if ( data->header->messageSize + index <= bytesread )
				{
					/* have read in the entire message */

					/*((struct datalinkHeader_ref *)bf)->hcsum = 0;*/
					if ( datalinkCheckSumCompute( &bf[sizeof( struct datalinkHeader_ref )], data->header->messageSize - sizeof( struct datalinkHeader_ref ) ) == data->header->csum )
					{
            switch ( data->header->messageID )
						{
              case DATALINK_MESSAGE_OPTITRACK:
							if ( data->header->messageSize == sizeof( struct datalinkMessageOptitrack_ref ) )
							{
                memcpy( data->optitrack, bf, sizeof( struct datalinkMessageOptitrack_ref ) );
                // Serial.print(data->optitrack->pos_x);
                // Serial.print("\t ");
                // Serial.print(data->optitrack->pos_y);
                // Serial.print("\t ");
                // Serial.print(data->optitrack->pos_z);
                // Serial.print("\t ");
                // Serial.print(data->optitrack->qw);
                // Serial.print("\t ");
                // Serial.print(data->optitrack->qx);
                // Serial.print("\t ");
                // Serial.print(data->optitrack->qy);
                // Serial.print("\t ");
                // Serial.println(data->optitrack->qz);
							}
							break;

              case DATALINK_MESSAGE_TRUTH:
							if ( data->header->messageSize == sizeof( struct datalinkMessageTruth_ref ) )
							{
                memcpy( data->truthSim, bf, sizeof( struct datalinkMessageTruth_ref ) );

							}
							break;

              case DATALINK_MESSAGE_HITL_SIM2ONBOARD:
							if ( data->header->messageSize == sizeof( struct datalinkMessageHITLSim2Onboard_ref ) )
							{
                memcpy( data->sim2onboard, bf, sizeof( struct datalinkMessageHITLSim2Onboard_ref ) );

							}
							break;

							default:
							/* unrecognized message */
							break;
						}

						data->work->itime++;
					}
					else
					{ /* checksum bad */
						data->work->badChecksums++;
					}
					index += data->header->messageSize - 1;

				}
				else
				{ /* end of buffer includes a partial message - come back later... */
					index--;
					done = 1;
				}
			}
			else
			{ /* header checksum is bad */
				index += sizeof( struct datalinkHeader_ref ) - 1;
				data->work->badHeaderChecksums++;
			}
		}
		index++; /* start seq not found, go to next byte */

		if ( index < 0 ) index = BUFFERSIZE - 1;
	}
	// clearPort( port, index );
}

void writeM0( WiFiUDP* wf )
{

}

void writeM1( WiFiUDP* wf )
{

}

void writeUP0( WiFiUDP* wf, RC_PILOT* rc )
{
  struct obDatalink_ref* data = &obDatalink;

  data->up0->throttleLever = rc->rc_in.THR;
  data->up0->rollStick = rc->rc_in.ROLL;
  data->up0->pitchStick = rc->rc_in.PITCH;
  data->up0->rudderPedal = rc->rc_in.YAW;

  data->up0->button[0] = rc->rc_in.AUX;
  data->up0->button[1] = rc->rc_in.AUX2;

  data->up0->messageID = DATALINK_MESSAGE_UP0;
  datalinkCheckSumEncode ( ( unsigned char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->up0, sizeof ( struct datalinkMessageUp0_ref));
  wf->endPacket();
}

void writePWM( WiFiUDP* wf ){
  struct obDatalink_ref* data = &obDatalink;

  data->pwm->raw_pwm[0] = cntrl.pwmout_0; /* M1 */
  data->pwm->raw_pwm[1] = cntrl.pwmout_1; /* M2 */
  data->pwm->raw_pwm[2] = cntrl.pwmout_2; /* M3 */
  data->pwm->raw_pwm[3] = cntrl.pwmout_3; /* M4 */

  data->pwm->messageID = DATALINK_MESSAGE_UP0;
  datalinkCheckSumEncode ( ( unsigned char* ) data->pwm, sizeof ( struct datalinkMessagePWM_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->pwm, sizeof ( struct datalinkMessagePWM_ref));
  wf->endPacket();
}

void writeAutopilotDels( WiFiUDP* wf, Controller* cntrl )
{
  struct obDatalink_ref* data = &obDatalink;

  data->autopilotDels->c_delf[0] = cntrl->c_delf;
  data->autopilotDels->c_delm[0] = cntrl->c_delm0;
  data->autopilotDels->c_delm[1] = cntrl->c_delm1;
  data->autopilotDels->c_delm[2] = cntrl->c_delm2;
  data->autopilotDels->c_delt[0] = 1.0f;

  data->autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
	datalinkCheckSumEncode ( ( unsigned char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref));
  wf->endPacket();
}

void writeMotors(WiFiUDP* wf, Controller* cntrl)
{
  struct obDatalink_ref* data = &obDatalink;

  data->motors->motor_cmd[0] = cntrl->pwmout_0;
  data->motors->motor_cmd[1] = cntrl->pwmout_1;
  data->motors->motor_cmd[2] = cntrl->pwmout_2;
  data->motors->motor_cmd[3] = cntrl->pwmout_3;
  data->motors->k = 0;
  data->motors->time = 0.0f;

  data->motors->messageID = DATALINK_MESSAGE_MOTOR_CMD;
  datalinkCheckSumEncode ( ( unsigned char* ) data->motors, sizeof ( struct datalinkMessageMotorCmd_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->motors, sizeof ( struct datalinkMessageMotorCmd_ref));
  wf->endPacket();
}
