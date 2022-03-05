/*
 * pw_support.h
 *
 *  Created on: 2 Oct 2019
 *      Author: pw
 */

#ifndef SRC_LTE_MODEL_PW_SUPPORT_H_
#define SRC_LTE_MODEL_PW_SUPPORT_H_

#include <stdio.h>
#include <ctime>

/**
 * PW init
 */
#define			PW_ENABLE			1
#define			PW_DISABLE			0

#define		 	PW_ACTIVE			PW_ENABLE


// eNB Mac Scheduler
enum ENB_SCHEDULER {
	RR_NS3_SCHEDULER = 1,
	PF_PWULLA_SCHEDULER,
	PW_SCHEDULER
};

// PW RADISYS SPECTRAL EFFICIENCY
enum PW_SPECTRAL_EFF {
	PW_SPECTRAL_EFF_MAXTP = 0,
	PW_SPECTRAL_EFF_CQI,
	PW_SPECTRAL_EFF_SINR
};

// eNB Antenna model
enum ENB_ANTENNAMODEL {
	ANT_NS3_PARABOLIC = 1,
	ANT_NS3_COSINE
};



//---------------------------------------------------------------------------------//
/***** NETWORK PARAMETER       *****/

#define	PWCONFIG_BWRB								50
#define PWCONFIG_SIMULATION_TIME_SEC				10.000				// Time Seconds
#define PWCONFIG_NOF_SIMRUN_SCENARIOS				100					// Nof of simulation scenario with diff seeds for UE pos
#define PWCONFIG_NUM_UE_PERCENTERENB				64					// No of UEs in center cell sector
#define PWCONFIG_ENB_COVERAGERADIUSM 				100					// in METERS

/* Network Parameter */
#define PWCONFIG_UE_RANDPOS_SETSEED					PW_DISABLE			// Set seed of randvar for UE position
#define PWCONFIG_SIMRUN_SCENARIOS_START				1					// Starting scenario

#define PWCONFIG_SIMMSG_INTERVAL_SEC				1.000				// Time Seconds
#define PWCONFIG_NUM_CENTERENBSITE					1
#define PWCONFIG_NUM_SECTOR_PERENB					3
#define	PWCONFIG_SECTORANT_OFFSET_M					0.5					// Sector Ant offset in meters

/* PW Interfering eNB */
/* NOTE: Enable PW Inf model by turning on PWCONFIG_PWINFMODEL flag */
#define PWCONFIG_PWINFMODEL							PW_DISABLE			// Enable/Disable PW Interference model
#define PWCONFIG_NUM_CENTERENBS						3					// Center eNB with 3 sectors
#define PWCONFIG_NUM_INF_TIER						2					// 0: 0; 1: 1-Tier (6);
#define PWCONFIG_NUM_TIER1_NEIGHENBSITE				6					// 1 Tier Surrounding Cell sites
#define PWCONFIG_NUM_TIER2_NEIGHENBSITE				12					// 2 Tier Surrounding Cell sites
#define PWCONFIG_NUM_NEIGHENBSITE					18 					// 1 and 2 Tier
#define PWCONFIG_NUMUE_PERNEIGHENB					2					// No of UEs in neighbor
#define PWCONFIG_NUMUE_PERNEIGHENB_TIER1			2					// No of UEs in neighbor eNB Tier 1
#define PWCONFIG_NUMUE_PERNEIGHENB_TIER2			2					// No of UEs in neighbor eNB Tier 2
#define PWCONFIG_ENB_HEIGHTM						10					// eNB height in meters
#define PWCONFIG_UE_HEIGHTM							1					// UE height in meters
#define	PWCONFIG_ENABLE_PLMODEL						PW_ACTIVE			// Enable/Disable Pathloss model
#define	PWCONFIG_TRANSMISSION_MODE					2					// *0=SISO 1=SISO *2=TxDiversity
																		//	3=MIMO_Spatial_Multiplexity OL
																		//	4=MIMO_Spatial_Multplexing_CL
#define PWCONFIG_INFMODEL_NEIGHLOADINGFACTOR		1.0					// NeighborCell Loading Factor (0.0-1.0)
#define PWCONFIG_INFMODEL_UE_CHANGEPHYPOSITION		PW_DISABLE			// Change UE node's physical position
// To disable PW_INFMODEL; DO Starttime > SimTime
#define PWCONFIG_INFMODEL_STARTPOSCHANGE_SEC		100.000				// PW inf model start time for infUE position changes
#define PWCONFIG_INFMODEL_POSCHANGE_INTERVAL_SEC	1.000				// PosChange interval seconds
//
#define PWCONFIG_INFMODEL_UE_UPDATEPOS				PW_ENABLE
#define PWCONFIG_INFMODEL_UE_UPDATEPOS_STARTSEC		2.000
#define PWCONFIG_INFMODEL_UE_UPDATEPOS_SETUEPHY		PW_DISABLE			// Enable/Disable change of Inf Ue in uePhy


/*** ENB SCHEDULER ***/
	#define PWCONFIG_ENB_SCHEDULER					PW_SCHEDULER			// ENB_SCHEDULER {
																			//		RR_NS3_SCHEDULER = 1,
																			//		PF_PWULLA_SCHEDULER,
																			//		PW_SCHEDULER};

	// VFT Modification - Change PW scheduler like RR_like
	#define PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE	PW_ENABLE				// PW_DISABLE / PW_ENABLE
	#define PWCONFIG_PWSCHEDULER_RRLIKE_EQUPDATE	PW_DISABLE				// PW_DISABLE / PW_ENABLE

	// PW_SCHEDULER: Throughput and Fairness Factors
	// Max(TP10FF0); QoSRR(TP0FF0)
	#define PWCONFIG_PWSCHEDULER_TP_FACTOR			0						// PW_EDIT Default: 10
	#define PWCONFIG_PWSCHEDULER_FAIRNESS_FACTOR	0						// PW_EDIT Default:  0

	#define PWCONFIG_ENB_ANTENNA_MODEL				ANT_NS3_COSINE			// ENB_ANTENNAMODEL {
																			//		ANT_NS3_PARABOLIC = 1,
																			//		ANT_NS3_COSINE
																			// };

	#define PWCONFIG_PW_SPECTRAL_EFF				PW_SPECTRAL_EFF_SINR	// PW_SPECTRAL_EFF {
																			//		PW_SPECTRAL_EFF_MAXTP = 0,
																			//		PW_SPECTRAL_EFF_CQI,
																			//		PW_SPECTRAL_EFF_SINR};
	#define PWCONFIG_ENB_SCHEDULER_ENABLE_UEPERTTI	PW_ENABLE				// ENB SCHEDULER - USE UE/TTI (PW_ENABLE, PW_DISABLE)
	#define PWCONFIG_ENB_SCHEDULER_UEPERTTI			2						// ENB_SCHEDULER UEs/TTI
																			// 	set in lte-enb-mac.h, rr-ff-mac-scheduler.cc

/*** Application ***/
#define PWCONFIG_TCPSOCKET_SEGMENTSIZE_BYTES	536						// ns3::TcpSocket TCP max segment size in bytes (default 536)
#define PWCONFIG_ENABLE_UL_TCP					PW_DISABLE				// Enable UL TCP Application
#define PWCONFIG_ENABLE_DL_TCP					PW_DISABLE				// Enable DL TCP Application
#define	PWCONFIG_ENABLE_UL_UDP					PW_ENABLE				// Enable UL UDP Application
//
#define PWCONFIG_SCHEDULER_LOGFILE				PW_ACTIVE


//---------------------------------------------------------------------------------//
// Working debug logs
#define PWCONFIG_WKINGLOG_AT_TTISEC				2.000					// Tmp: Start Tmp PW debug log at this TTI (in sec)
#define PWCONFIG_WKINGLOG_UL_SCH_PW				PW_DISABLE				// Tmp: working debug for UL PW Scheduler
#define PWCONFIG_WKINGLOG_DL_SCH_PW				PW_DISABLE				// Tmp: working debug for UL PW Scheduler
#define PWCONFIG_WKINGLOG_TCP_FLOWCONTROL		PW_DISABLE				// Tmp: working debug for UL TCP rate control
#define PWCONFIG_WKINGLOG_PWINFMODEL			PW_DISABLE				// Tmp: working debug for PW Interference model
#define PWCONFIG_WKINGLOG_UE_CHANGEPOSITION		PW_DISABLE				// Tmp: working debug for PWinf model UE changing position

#endif /* SRC_LTE_MODEL_PW_SUPPORT_H_ */
