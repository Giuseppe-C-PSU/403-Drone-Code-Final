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
 * Dr. Thanakorn Khamvilai Email: thanakorn.khamvilai@ttu.edu
 * Dr. Vitor Valente       Email: vitor.valente@psu.edu
 *
 * EndCopyright
 ***/

#include "datalink.h"

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

struct datalinkMessageRCChannel_ref obDatalinkMessageRCChannel = {
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
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} , /* unsigned int rc_channels[16] */
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

struct obDatalink_ref obDatalink = {
  &obDatalinkWork, /* dir work */
	&obDatalinkMessageHeader, /* dir header */
  &obDatalinkMessage0, /* dir m0 */
  &obDatalinkMessage1, /* dir m1 */
  &obDatalinkMessageUp0, /* dir up0 */
  &obDatalinkMessageRCChannel, /* dir rcchannel */
  &obDatalinkMessageAutopilotDels, /* dir autopilotDels */
};

/**
 * @brief datalinkCheckSumCompute() calculates and returns the checksum of a
 * character buffer
 *
 * Calculates the checksum of a character buffer using a 32-bit Fletcher
 * checksum. Handles an odd number of bytes by calculating the checksum as if
 * there were an additional zero-byte appended to the end of the data.
 *
 * @param buf pointer to a character buffer array
 * @param byteCount the size in bytes of the character buffer
 */
uint32_t datalinkCheckSumCompute(unsigned char* buf, int32_t byteCount)
{

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

/**
 * @brief datalinkCheckSumEncode sets the header checksum and payload checksum of a
 * character buffer to be sent as a datalink message
 *
 * @param buf pointer to a character buffer
 * @param byteCount size of the character buffer in bytes
 */
void datalinkCheckSumEncode(unsigned char* buf, uint32_t byteCount)
{

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

  int bytesread = wf->read(buffer, BUFFERSIZE);

	while ( ( index <= bytesread - ( int ) sizeof( struct datalinkHeader_ref ) ) && !done )
	{
		if ( ( buffer[index] == DATALINK_SYNC0 ) &&
				 ( buffer[index + 1] == DATALINK_SYNC1 ) &&
				 ( buffer[index + 2] == DATALINK_SYNC2 ) )
		{
			bf = &( buffer[index] );

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
							case DATALINK_MESSAGE_UP0:
							if ( data->header->messageSize == sizeof( struct datalinkMessageUp0_ref ) )
							{
                memcpy( data->up0, bf, sizeof( struct datalinkMessageUp0_ref ) );

                if ( isfinite( data->up0->throttleLever ) &&
                      isfinite( data->up0->rollStick ) &&
                      isfinite( data->up0->pitchStick ) &&
                      isfinite( data->up0->rudderPedal ) )
                {
                  data->m1->yrdStatus = 1; // good receiver data

                  // save RC commands and add to message1 to send back to gcs
                  // ob->actuators->work->delm[0] = constrain( data->up0->rollStick, -1.0, 1.0 );
                  // ob->actuators->work->delm[1] = constrain( data->up0->pitchStick, -1.0, 1.0 );
                  // ob->actuators->work->delm[2] = constrain( data->up0->rudderPedal, -1.0, 1.0 );
                  // ob->actuators->work->delf[0] = constrain( data->up0->throttleLever, -1.0, 1.0 );
                }
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

void writeAutopilotDels( WiFiUDP* wf )
{
  struct obDatalink_ref* data = &obDatalink;

  data->autopilotDels->c_delf[0] = 0.1; //data->up0->throttleLever; // sample, need to be changed
  data->autopilotDels->c_delm[0] = 0.2; //data->up0->rollStick; // sample, need to be changed
  data->autopilotDels->c_delm[1] = 0.3; //data->up0->pitchStick; // sample, need to be changed
  data->autopilotDels->c_delm[2] = 0.4; //data->up0->rudderPedal; // sample, need to be changed

  data->autopilotDels->messageID = DATALINK_MESSAGE_AUTOPILOTDELS;
	datalinkCheckSumEncode ( ( unsigned char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->autopilotDels, sizeof ( struct datalinkMessageAutopilotDels_ref));
  wf->endPacket();
}

void writeRcChannels( WiFiUDP* wf , RC_PILOT* rc)
{
  struct obDatalink_ref* data = &obDatalink;

  data->rc->rc_channels[0] = rc->rc_in.ROLL;
  data->rc->rc_channels[1] = rc->rc_in.PITCH;
  data->rc->rc_channels[2] = rc->rc_in.THR;
  data->rc->rc_channels[3] = rc->rc_in.YAW;
  data->rc->rc_channels[4] = rc->rc_in.AUX;
  data->rc->rc_channels[5] = rc->rc_in.AUX2;

  for(int i=6;i<MAX_RC_CHANNELS;i++)
    data->rc->rc_channels[i] = 1500;

  data->rc->messageID = DATALINK_MESSAGE_RCCHANNEL;
  datalinkCheckSumEncode ( ( unsigned char* ) data->rc, sizeof ( struct datalinkMessageRCChannel_ref) );

  wf->beginPacket(wf->remoteIP(), wf->remotePort());
  wf->write(( char* ) data->rc, sizeof ( struct datalinkMessageRCChannel_ref));
  wf->endPacket();
}
