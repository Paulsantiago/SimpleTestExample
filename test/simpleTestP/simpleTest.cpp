// simple_test.cpp : �w�q�D���x���ε{�����i�J�I�C
//
#include "stdafx.h"

#include <string.h>
//#include <Mmsystem.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "osal.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

// #include "defConst.h"
// #include "OpenWave.h"

char *filename = "LostTime.wav";
//char *filename = "NonJeneregretterien.wav";
//char *filename = "2CELLOS.wav";
//char *filename = "GoT.wav";
//char *filename = "GoTLP.wav";

char IOmap[4096];
HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
volatile int rtcnt;
boolean inOP;
uint8 currentgroup = 0;
int32 CurrnetPosition[9];
//bool IsInOperation = 0;
bool bEnable = false;

uint16 wDS402State[9];
bool bMotionEnable[9] = {false};

LARGE_INTEGER   litmp;

void MC_Power(int nAxis);


uint16 *Controlword[10];
int32 *TargetPosition[10];
int32 *TargetVelocity[10];
uint16 *Statusword[10];
int32 *ActualPosition[10];
int32 *ActualVelocity[10];
int8 *AcutalMode[10];



void link_var(void)
{
	int offset = 0;
	for (int ax = 1; ax <= ec_slavecount; ax++)
	{
		offset = 0;
		Controlword[ax] = (uint16*)(ec_slave[ax].outputs + offset); offset += 2;
		TargetPosition[ax] = (int32*)(ec_slave[ax].outputs + offset); offset += 4;
		TargetVelocity[ax] = (int32*)(ec_slave[ax].outputs + offset); offset += 4;

		offset = 0;
		Statusword[ax] = (uint16*)(ec_slave[ax].inputs + offset); offset += 2;
		ActualPosition[ax] = (int32*)(ec_slave[ax].inputs + offset); offset += 4;
		ActualVelocity[ax] = (int32*)(ec_slave[ax].inputs + offset); offset += 4;
		//AcutalMode[ax] = (int8*)(ec_slave[ax].inputs + offset); offset += 1;
	}
}

/* most basic RT thread for process data, just does IO transfer */


int waveTime = 1;
void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{

	//for(int ax=1; ax<= ec_slavecount; ax++)
	for (int ax = 1; ax <= ec_slavecount; ax++)
	{
		MC_Power(ax);
	}

	ec_send_processdata();
	wkc = ec_receive_processdata(EC_TIMEOUTRET);
	rtcnt++;

	/* do RT control stuff here */

	if (bMotionEnable[1])
	{ 

		if ((uint32)(waveTime * 44.1) < samples)
		{
			int16 p1 = wavedata16[(uint32)(waveTime * 44.1)];
			int16 p2 = wavedata16[(uint32)((waveTime + 1) * 44.1)];
			*TargetVelocity[1] = (p2-p1) * 300.0;

			waveTime++;
		}

	}
}

int YasakawaSetup(uint16 slave)
{
	uint8  tempU8;
	uint16 tempU16;
    uint32 tempU32;
		

	tempU8 = 0;
	ec_SDOwrite(slave, 0x1c12, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	tempU16 = 0x1601;
	ec_SDOwrite(slave, 0x1c12, 0x1, FALSE, sizeof(tempU16), &tempU16, EC_TIMEOUTRXM);
	tempU8 = 1;
	ec_SDOwrite(slave, 0x1c12, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);

	tempU8 = 0;
	ec_SDOwrite(slave, 0x1c13, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	tempU16 = 0x1A01;
	ec_SDOwrite(slave, 0x1c13, 0x1, FALSE, sizeof(tempU16), &tempU16, EC_TIMEOUTRXM);
	tempU8 = 1;
	ec_SDOwrite(slave, 0x1c13, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);



	tempU8 = 0;
	ec_SDOwrite(slave, 0x1601, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	tempU32 = 0x60400010;
	ec_SDOwrite(slave, 0x1601, 0x1, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 6040h, Controlword
	tempU32 = 0x607A0020;
	ec_SDOwrite(slave, 0x1601, 0x2, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 607Ah, Target Position
	tempU32 = 0x60FF0020;
	ec_SDOwrite(slave, 0x1601, 0x3, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 60FFh, Target Velocity
	tempU8 = 3;
	ec_SDOwrite(slave, 0x1601, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	
	tempU8 = 0;
	ec_SDOwrite(slave, 0x1A01, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	tempU32 = 0x60410010;
	ec_SDOwrite(slave, 0x1A01, 0x1, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 6041h, Statusword
	tempU32 = 0x60640020;
	ec_SDOwrite(slave, 0x1A01, 0x2, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 6064h, Position Actual Value
	tempU32 = 0x606C0020;
	ec_SDOwrite(slave, 0x1A01, 0x3, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 606Ch, Velocity Actual Value
	//tempU32 = 0x60610008;
	//ec_SDOwrite(slave, 0x1A01, 0x3, FALSE, sizeof(tempU32), &tempU32, EC_TIMEOUTRXM); // 6060h, Mode
	tempU8 = 3;	
	ec_SDOwrite(slave, 0x1A01, 0x0, FALSE, sizeof(tempU8), &tempU8, EC_TIMEOUTRXM);
	

	
	uint8 mode;
	//mode = 8; //CSP
	mode = 9; //CSV
	ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

    while(EcatError) 
		printf("%s", ec_elist2string());

    //printf("YASKAWA slave %d set, retval = %d\n", slave, retval);
    return 1;
}

void MC_Power(int nAxis)
{
	uint16    wStatus , wControl;
	uint16 *pwStatus, *pwControl;

	//pwStatus = (uint16*)(ec_slave[nAxis].inputs + 10 * nAxis);
	pwStatus = (uint16*)(ec_slave[nAxis].inputs);
	wStatus = *pwStatus;
	
	//pwControl = (uint16*)(ec_slave[nAxis].outputs + 10 * nAxis);
	pwControl = (uint16*)(ec_slave[nAxis].outputs);
	wControl = *pwControl;

	if (ec_slave[nAxis].state == EC_STATE_OPERATIONAL )
    {

		switch(wDS402State[nAxis])
        {
        default:
        case DRV_DEV_STATE_NOT_READY:           /* Not ready to switch on */
            if(bEnable)            
            {
                   
                if(((wStatus & 0x4F) == 0) || ((wStatus & 0x4F) == DRV_STAT_SWITCH_ON_DIS))
                {
                    wDS402State[nAxis] = DRV_DEV_STATE_SWITCHON_DIS;
                }
                else if(wStatus & DRV_STAT_FAULT)
                {
                    wDS402State[nAxis] = DRV_DEV_STATE_MALFUNCTION;
                    //MC_SetAxisError(pAxis, MC_ERR_AXIS_FAULT);
                }
                else if((wStatus & DRV_STAT_RDY_SWITCH_ON) == DRV_STAT_RDY_SWITCH_ON)
                {
                    wDS402State[nAxis] = DRV_DEV_STATE_SWITCHON_DIS;
                }
            }
            break;

        case DRV_DEV_STATE_SWITCHON_DIS:        /* Optional state: Switch on disabled     */
            if(bEnable)            
            {
                if((wStatus & 0x4F) == DRV_STAT_SWITCH_ON_DIS)
                {
                    wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_SHUTDOWN);
                    wDS402State[nAxis] = DRV_DEV_STATE_READY_TO_SWITCHON;
                }
                else if((wStatus & 0x4F) == DRV_STAT_RDY_SWITCH_ON)
                {
                    wDS402State[nAxis] = DRV_DEV_STATE_READY_TO_SWITCHON;
                }
                else if(wStatus & DRV_STAT_FAULT)
                {
                    wDS402State[nAxis] = DRV_DEV_STATE_MALFUNCTION;
                }
            }
            break;

        case DRV_DEV_STATE_READY_TO_SWITCHON:   /* Ready to switch on    */
            if(bEnable)            
            {
                if((wStatus & 0x6F) == (DRV_STAT_QUICK_STOP | DRV_STAT_RDY_SWITCH_ON))
                {
                    wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_SWITCHON);
                    
                    wDS402State[nAxis] = DRV_DEV_STATE_SWITCHED_ON;
                    //EC_SET_FRM_DWORD(&pAxis->pPdoOut->lTargetPosition, EC_GET_FRM_DWORD(&pAxis->pPdoInp->lActualPosition));
                }
            }
            else
            {
                wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_DIS_VOLTAGE);
                 wDS402State[nAxis] = DRV_DEV_STATE_SWITCHON_DIS;
            }

            break;

        case DRV_DEV_STATE_SWITCHED_ON:         /* Switched on */
            //pPower->bStatus    = EC_FALSE;
            if(bEnable)
            {
                if((wStatus & 0x7F) == (DRV_STAT_QUICK_STOP | DRV_STAT_VOLTAGE_ENABLED | DRV_STAT_SWITCHED_ON | DRV_STAT_RDY_SWITCH_ON))
                {
                    wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_ENA_OPERATION);
                    wDS402State[nAxis] = DRV_DEV_STATE_OP_ENABLED;
                }
            }
            else
            {
                wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_SHUTDOWN);
                wDS402State[nAxis] = DRV_DEV_STATE_READY_TO_SWITCHON;
            }
            break;

        case DRV_DEV_STATE_OP_ENABLED:          /* Operation enabled  */
            if(bEnable)
            {
				bMotionEnable[nAxis] = true;
				
            }
            else
            {
				//ec_slave[nAxis].bMotionEnable = false;
				bMotionEnable[nAxis] = false;
                wControl = (WORD)((wControl & ~DRV_CTRL_CMD_MASK) | DRV_CTRL_CMD_DIS_OPERATION);
                wDS402State[nAxis] = DRV_DEV_STATE_SWITCHED_ON;
            }
			

            break;
			
			//IsInOperation = true;
			//ec_SDOwrite(, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

        case DRV_DEV_STATE_QUICK_STOP:          /* Optional state: Quick stop active  */
            /* todo */
            break;

        case DRV_DEV_STATE_MALFCT_REACTION:     /* Malfunction reaction active */
            if(wStatus & DRV_STAT_FAULT)
            {
                /* stay in this state */
            }
            else
            {
                wControl = (WORD)(wControl & ~DRV_CRTL_FAULT_RESET);
                wDS402State[nAxis] = DRV_DEV_STATE_NOT_READY;
            }
            break;

        case DRV_DEV_STATE_MALFUNCTION:         /* Malfunction                 */
            if(wStatus & DRV_STAT_FAULT)
            {
                wControl = (WORD)(wControl | DRV_CRTL_FAULT_RESET);
                wDS402State[nAxis] = DRV_DEV_STATE_MALFCT_REACTION;
            }
            //else
            //{
            //    pAxis->wDS402State = DRV_DEV_STATE_NOT_READY;
            //}
            break;
        }
    }
    else
    {
        ///* slave is not in OP */
        //pPower->bStatus         = EC_FALSE;         /* set all flags to disabled state */
        //wControl                = 0;                /* reset control word */
        //pAxis->wDS402State      = DRV_DEV_STATE_NOT_READY;
    }

   
	*pwControl = wControl;
	//*(ec_slave[0].outputs + 6*nCrrentAxis) = wControl;
}


void simpletest(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk, slc;
    UINT mmResult;

    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");
   
   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {   
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
       {
         printf("%d slaves found and configured.\n",ec_slavecount);

		 for(int ax=1; ax<= ec_slavecount; ax++)
			 ec_slave[ax].PO2SOconfig = &YasakawaSetup;
		 
         ec_config_map(&IOmap);

		 ec_configdc();

		 link_var();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
		 for (int ax = 0; ax < ec_slavecount; ax++)
			 ec_statecheck(ax, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes/* + ec_slave[2].Obytes*/;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) 
			 oloop = 1;
         
		 //if (oloop > 8) 
		//	 oloop = 8;

         iloop = ec_slave[0].Ibytes;
         
		 if ((iloop == 0) && (ec_slave[0].Ibits > 0)) 
			 iloop = 1;

         //if (iloop > 8) 
		//	 iloop = 8;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         
		 expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
		 //expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

         printf("Calculated workcounter %d\n", expectedWKC);

		 for (int ax = 0; ax < ec_slavecount; ax++)
			 ec_slave[ax].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);

         /* start RT thread as periodic MM timer */
         mmResult = timeSetEvent(1, 0, RTthread, 0, TIME_PERIODIC);

         /* request OP state for all slaves */
         ec_writestate(0);

         chk = 40;
         /* wait for all slaves to reach OP state */
		 for (int ax = 1; ax <= ec_slavecount; ax++)
		 {
			do
			{
				ec_statecheck(ax, EC_STATE_OPERATIONAL, 50000);
			}
			while (chk-- && (ec_slave[ax].state != EC_STATE_OPERATIONAL));

		 }
        
		 
         if ( (ec_slave[0].state == EC_STATE_OPERATIONAL) /*&& (ec_slave[2].state == EC_STATE_OPERATIONAL)*/ )
         {
            printf("Operational state reached for all slaves.\n");
            wkc_count = 0;
            inOP = TRUE;

             //usleep(100000); // wait for linux to sync on DC
             Sleep(100);
			 //ec_dcsync0(1, TRUE, SYNC0TIME, 0); // SYNC0 on slave 1
			 
			 /*for (int ax = 1; ax <= 8; ax++)
			 {
				 ec_dcsync0(ax, TRUE, SYNC0TIME, 0); // SYNC0 on slave ax
			 }*/
			 ec_dcsync0(1, TRUE, SYNC0TIME, 0); // SYNC0 on slave

			
			 bEnable = true;
            /* cyclic loop, reads data from RT thread */
            for(i = 1; i <= 5000; i++)
            {
                    if(wkc >= expectedWKC)
                    {
						printf(" TV:%011d", *TargetVelocity[1]);
						printf(" AV:%011d", *ActualVelocity[1]);

                        
                        needlf = TRUE;
                    }
                    //osal_usleep(50000);
					printf("\r");
					SleepEx(50,false);
                    
            }

			bEnable = false;
            inOP = FALSE;
         }
         else
         {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
         }           

         /* stop RT thread */
         timeKillEvent(mmResult);

         printf("\nRequest init state for all slaves\n");
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }   
}   

DWORD WINAPI ecatcheck( LPVOID lpParam ) 
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);                              
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);                           
                     }
                  } 
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);                           
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);                           
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);                           
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        //osal_usleep(10000);
		SleepEx(10,false);
    }   

    return 0;
}  

char ifbuf[1024];

int main(int argc, _TCHAR* argv[])
{


	//OpenWavFile(filename);
	//OpenWavFile("NonJeneregretterien.wav");
	
	QueryPerformanceFrequency(&litmp);

   ec_adaptert * adapter = NULL;   
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   //if(0)
   {      
      /* create thread to handle slave error handling in OP */
      thread1 = CreateThread( NULL, 0, ecatcheck, NULL, 0, NULL);
      
      strcpy(ifbuf, argv[1]);
      /* start cyclic part */
      simpletest(ifbuf);

	  //char *ifname = " \\Device\\NPF_{E88A7F0E-A377-4C9A-ADB9-9672C3628872}";
	  //char *ifname = " \\Device\\NPF_{43E82EA0-4C08-4239-8D74-9B3B64481B76}"; //broken	  
	  
	  //char *ifname = " \\Device\\NPF_{FC356536-9C99-4CA3-9DEE-8E43164B2A23}"; //broken	  
	  char *ifname = " \\Device\\NPF_{08B7F855-8780-4FFE-AED8-D8DF3D61213A}";
	  
	  simpletest(ifname);
   }
   else
   {
      printf("Usage: simple_test ifname1\n");
   	/* Print the list */
      printf ("Available adapters\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("Description : %s, Device to use for wpcap: %s\n", adapter->desc,adapter->name);
         adapter = adapter->next;
      }
   }   
   
   printf("End program\n");

   int a;
   scanf("%d" ,&a );
	return 0;
}

