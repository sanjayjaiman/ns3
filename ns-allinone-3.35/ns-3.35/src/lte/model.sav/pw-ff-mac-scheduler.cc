/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 */

#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/math.h>

#include <ns3/simulator.h>
#include <ns3/lte-amc.h>
#include <ns3/pw-ff-mac-scheduler.h>
#include <ns3/lte-vendor-specific-parameters.h>
#include <ns3/boolean.h>
#include <cfloat>
#include <set>
#include <ns3/double.h>
#include <fstream>
#include "up-link-config.h"


namespace ns3 {


// rg_sch_pfs.c
// U32 rgSchPfsScalingFactor[RG_SCH_PFS_MAX_FACTOR_LVLS]= {100,100,100,100,100,10,10,10,10,1,1};
// ns3 ULLA implementation
//uint8_t PfsScalingFactor[MAX_FACTOR_LVLS] = {1,100,100,100,100,10,10,10,10,1,1};

uint8_t PfsScalingFactor_UL[MAX_FACTOR_LVLS] 	= {100,100,100,100,100,10,10,10,10,1,1};
uint8_t PfsScalingFactor[MAX_FACTOR_LVLS] 		= {100,100,100,100,100,10,10,10,10,1,1};


// Throughput Factors
uint32_t PfsTpFactorSelector_UL[MAX_FACTOR_LVLS][PFS_TP_FACTOR_LVLS]={RG_SCH_PFS_TP_FACTOR_0_UL,RG_SCH_PFS_TP_FACTOR_1_UL,RG_SCH_PFS_TP_FACTOR_2_UL,
	     RG_SCH_PFS_TP_FACTOR_3_UL,RG_SCH_PFS_TP_FACTOR_4_UL,RG_SCH_PFS_TP_FACTOR_5_UL,
	     RG_SCH_PFS_TP_FACTOR_6_UL,RG_SCH_PFS_TP_FACTOR_7_UL,RG_SCH_PFS_TP_FACTOR_8_UL,
	     RG_SCH_PFS_TP_FACTOR_9_UL,RG_SCH_PFS_TP_FACTOR_10_UL};
uint32_t PfsTpFactorSelector[MAX_FACTOR_LVLS][PFS_TP_FACTOR_LVLS]=
		{RG_SCH_PFS_TP_FACTOR_0, RG_SCH_PFS_TP_FACTOR_1,RG_SCH_PFS_TP_FACTOR_2,
	     RG_SCH_PFS_TP_FACTOR_3, RG_SCH_PFS_TP_FACTOR_4, RG_SCH_PFS_TP_FACTOR_5,
	     RG_SCH_PFS_TP_FACTOR_6, RG_SCH_PFS_TP_FACTOR_7, RG_SCH_PFS_TP_FACTOR_8,
	     RG_SCH_PFS_TP_FACTOR_9, RG_SCH_PFS_TP_FACTOR_10};

// Fairness Factors
uint32_t PfsFairnessFactorSelector_UL[MAX_FACTOR_LVLS][PFS_FAIRNESS_FACTOR_LVLS]={RG_SCH_PFS_FAIR_FACTOR_0_UL,RG_SCH_PFS_FAIR_FACTOR_1_UL,RG_SCH_PFS_FAIR_FACTOR_2_UL,
	     RG_SCH_PFS_FAIR_FACTOR_3_UL,RG_SCH_PFS_FAIR_FACTOR_4_UL,RG_SCH_PFS_FAIR_FACTOR_5_UL,
	     RG_SCH_PFS_FAIR_FACTOR_6_UL,RG_SCH_PFS_FAIR_FACTOR_7_UL,RG_SCH_PFS_FAIR_FACTOR_8_UL,
	     RG_SCH_PFS_FAIR_FACTOR_9_UL,RG_SCH_PFS_FAIR_FACTOR_10_UL};
uint32_t PfsFairnessFactorSelector[MAX_FACTOR_LVLS][PFS_FAIRNESS_FACTOR_LVLS]={
		RG_SCH_PFS_FAIR_FACTOR_0, RG_SCH_PFS_FAIR_FACTOR_1, RG_SCH_PFS_FAIR_FACTOR_2,
	    RG_SCH_PFS_FAIR_FACTOR_3, RG_SCH_PFS_FAIR_FACTOR_4, RG_SCH_PFS_FAIR_FACTOR_5,
	    RG_SCH_PFS_FAIR_FACTOR_6, RG_SCH_PFS_FAIR_FACTOR_7, RG_SCH_PFS_FAIR_FACTOR_8_UL,
	    RG_SCH_PFS_FAIR_FACTOR_9, RG_SCH_PFS_FAIR_FACTOR_10};


uint32_t Qciprio[MAX_QCI] = {8,6,7,5,0,4,3,2,1,0,0,0,0,0,0,0,0,0,0};
uint32_t GBR_QCI_start = 0;
uint32_t GBR_QCI_end = 3;

uint32_t NGBR_QCI_start = 5;
uint32_t NGBR_QCI_end = 9;

NS_LOG_COMPONENT_DEFINE ("pw_PfFfMacScheduler");

/// PF type 0 allocation RBG
static const int PfType0AllocationRbg[4] = {
		10,       // RGB size 1
		26,       // RGB size 2
		63,       // RGB size 3
		110       // RGB size 4
};  // see table 7.1.6.1-1 of 36.213

//Convert number of RBs to dB
static const uint8_t PwrRbToPwrdBTbl[111] = { 0,    /* First entry is dummy */
		0,  3,  4,  6,  7,  7,  8,  9,  9,  10,
		10, 10, 11, 11, 11, 12, 12, 12, 12, 13,
		13, 13, 13, 13, 14, 14, 14, 14, 14, 14,
		15, 15, 15, 15, 15, 15, 15, 15, 16, 16,
		16, 16, 16, 16, 16, 16, 16, 16, 17, 17,
		17, 17, 17, 17, 17, 17, 17, 17, 17, 17,
		17, 18, 18, 18, 18, 18, 18, 18, 18, 18,
		18, 18, 18, 18, 18, 18, 18, 19, 19, 19,
		19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
		19, 19, 19, 19, 19, 19, 19, 20, 20, 20,
		20, 20, 20, 20, 20, 20, 20, 20, 20, 20
};

static const uint8_t UlSinrToItbsTbl[255] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
		2, 2, 3, 3, 4, 4, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 10, 10, 10, 10,
		11, 11, 12, 12, 13, 14, 15, 15, 16, 16, 17, 17, 18, 19, 19, 19, 20, 20, 21, 21,
		22, 23, 23, 23, 24, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
		26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
		26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
		26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26,
		26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26
};

static const uint32_t TbSzTbl[27][110] =
{
		{16,    32,    56,    88,    120,   152,   176,   208,   224,   256,   288,   328,   344,   376,   392,   424,   456,   488,   504,   536,   568,   600,   616,   648,   680,   712,   744,   776,   776,   808,   840,   872,   904,   936,   968,   1000,  1032,  1032,  1064,  1096,  1128,  1160,  1192,  1224,  1256,  1256,  1288,  1320,  1352,  1384,  1416,  1416,  1480,  1480,  1544,  1544,  1608,  1608,  1608,  1672,  1672,  1736,  1736,  1800,  1800,  1800,  1864,  1864,  1928,  1928,  1992,  1992,  2024,  2088,  2088,  2088,  2152,  2152,  2216,  2216,  2280,  2280,  2280,  2344,  2344,  2408,  2408,  2472,  2472,  2536,  2536,  2536,  2600,  2600,  2664,  2664,  2728,  2728,  2728,  2792,  2792,  2856,  2856,  2856,  2984,  2984,  2984,  2984,  2984,  3112},
		{24,    56,    88,    144,   176,   208,   224,   256,   328,   344,   376,   424,   456,   488,   520,   568,   600,   632,   680,   712,   744,   776,   808,   872,   904,   936,   968,   1000,  1032,  1064,  1128,  1160,  1192,  1224,  1256,  1288,  1352,  1384,  1416,  1416,  1480,  1544,  1544,  1608,  1608,  1672,  1736,  1736,  1800,  1800,  1864,  1864,  1928,  1992,  1992,  2024,  2088,  2088,  2152,  2152,  2216,  2280,  2280,  2344,  2344,  2408,  2472,  2472,  2536,  2536,  2600,  2600,  2664,  2728,  2728,  2792,  2792,  2856,  2856,  2856,  2984,  2984,  2984,  3112,  3112,  3112,  3240,  3240,  3240,  3240,  3368,  3368,  3368,  3496,  3496,  3496,  3496,  3624,  3624,  3624,  3752,  3752,  3752,  3752,  3880,  3880,  3880,  4008,  4008,  4008},
		{32,    72,    144,   176,   208,   256,   296,   328,   376,   424,   472,   520,   568,   616,   648,   696,   744,   776,   840,   872,   936,   968,   1000,  1064,  1096,  1160,  1192,  1256,  1288,  1320,  1384,  1416,  1480,  1544,  1544,  1608,  1672,  1672,  1736,  1800,  1800,  1864,  1928,  1992,  2024,  2088,  2088,  2152,  2216,  2216,  2280,  2344,  2344,  2408,  2472,  2536,  2536,  2600,  2664,  2664,  2728,  2792,  2856,  2856,  2856,  2984,  2984,  3112,  3112,  3112,  3240,  3240,  3240,  3368,  3368,  3368,  3496,  3496,  3496,  3624,  3624,  3624,  3752,  3752,  3880,  3880,  3880,  4008,  4008,  4008,  4136,  4136,  4136,  4264,  4264,  4264,  4392,  4392,  4392,  4584,  4584,  4584,  4584,  4584,  4776,  4776,  4776,  4776,  4968,  4968},
		{40,    104,   176,   208,   256,   328,   392,   440,   504,   568,   616,   680,   744,   808,   872,   904,   968,   1032,  1096,  1160,  1224,  1256,  1320,  1384,  1416,  1480,  1544,  1608,  1672,  1736,  1800,  1864,  1928,  1992,  2024,  2088,  2152,  2216,  2280,  2344,  2408,  2472,  2536,  2536,  2600,  2664,  2728,  2792,  2856,  2856,  2984,  2984,  3112,  3112,  3240,  3240,  3368,  3368,  3496,  3496,  3624,  3624,  3624,  3752,  3752,  3880,  3880,  4008,  4008,  4136,  4136,  4264,  4264,  4392,  4392,  4392,  4584,  4584,  4584,  4776,  4776,  4776,  4776,  4968,  4968,  4968,  5160,  5160,  5160,  5352,  5352,  5352,  5352,  5544,  5544,  5544,  5736,  5736,  5736,  5736,  5992,  5992,  5992,  5992,  6200,  6200,  6200,  6200,  6456,  6456},
		{56,    120,   208,   256,   328,   408,   488,   552,   632,   696,   776,   840,   904,   1000,  1064,  1128,  1192,  1288,  1352,  1416,  1480,  1544,  1608,  1736,  1800,  1864,  1928,  1992,  2088,  2152,  2216,  2280,  2344,  2408,  2472,  2600,  2664,  2728,  2792,  2856,  2984,  2984,  3112,  3112,  3240,  3240,  3368,  3496,  3496,  3624,  3624,  3752,  3752,  3880,  4008,  4008,  4136,  4136,  4264,  4264,  4392,  4392,  4584,  4584,  4584,  4776,  4776,  4968,  4968,  4968,  5160,  5160,  5160,  5352,  5352,  5544,  5544,  5544,  5736,  5736,  5736,  5992,  5992,  5992,  5992,  6200,  6200,  6200,  6456,  6456,  6456,  6456,  6712,  6712,  6712,  6968,  6968,  6968,  6968,  7224,  7224,  7224,  7480,  7480,  7480,  7480,  7736,  7736,  7736,  7992},
		{72,    144,   224,   328,   424,   504,   600,   680,   776,   872,   968,   1032,  1128,  1224,  1320,  1384,  1480,  1544,  1672,  1736,  1864,  1928,  2024,  2088,  2216,  2280,  2344,  2472,  2536,  2664,  2728,  2792,  2856,  2984,  3112,  3112,  3240,  3368,  3496,  3496,  3624,  3752,  3752,  3880,  4008,  4008,  4136,  4264,  4392,  4392,  4584,  4584,  4776,  4776,  4776,  4968,  4968,  5160,  5160,  5352,  5352,  5544,  5544,  5736,  5736,  5736,  5992,  5992,  5992,  6200,  6200,  6200,  6456,  6456,  6712,  6712,  6712,  6968,  6968,  6968,  7224,  7224,  7224,  7480,  7480,  7480,  7736,  7736,  7736,  7992,  7992,  7992,  8248,  8248,  8248,  8504,  8504,  8760,  8760,  8760,  8760,  9144,  9144,  9144,  9144,  9528,  9528,  9528,  9528,  9528},
		{328,    176,   256,   392,   504,   600,   712,   808,   936,   1032,  1128,  1224,  1352,  1480,  1544,  1672,  1736,  1864,  1992,  2088,  2216,  2280,  2408,  2472,  2600,  2728,  2792,  2984,  2984,  3112,  3240,  3368,  3496,  3496,  3624,  3752,  3880,  4008,  4136,  4136,  4264,  4392,  4584,  4584,  4776,  4776,  4968,  4968,  5160,  5160,  5352,  5352,  5544,  5736,  5736,  5992,  5992,  5992,  6200,  6200,  6456,  6456,  6456,  6712,  6712,  6968,  6968,  6968,  7224,  7224,  7480,  7480,  7736,  7736,  7736,  7992,  7992,  8248,  8248,  8248,  8504,  8504,  8760,  8760,  8760,  9144,  9144,  9144,  9144,  9528,  9528,  9528,  9528,  9912,  9912,  9912,  10296, 10296, 10296, 10296, 10680, 10680, 10680, 10680, 11064, 11064, 11064, 11448, 11448, 11448},
		{104,   224,   328,   472,   584,   712,   840,   968,   1096,  1224,  1320,  1480,  1608,  1672,  1800,  1928,  2088,  2216,  2344,  2472,  2536,  2664,  2792,  2984,  3112,  3240,  3368,  3368,  3496,  3624,  3752,  3880,  4008,  4136,  4264,  4392,  4584,  4584,  4776,  4968,  4968,  5160,  5352,  5352,  5544,  5736,  5736,  5992,  5992,  6200,  6200,  6456,  6456,  6712,  6712,  6712,  6968,  6968,  7224,  7224,  7480,  7480,  7736,  7736,  7992,  7992,  8248,  8248,  8504,  8504,  8760,  8760,  8760,  9144,  9144,  9144,  9528,  9528,  9528,  9912,  9912,  9912,  10296, 10296, 10296, 10680, 10680, 10680, 11064, 11064, 11064, 11448, 11448, 11448, 11448, 11832, 11832, 11832, 12216, 12216, 12216, 12576, 12576, 12576, 12960, 12960, 12960, 12960, 13536, 13536},
		{120,   256,   392,   536,   680,   808,   968,   1096,  1256,  1384,  1544,  1672,  1800,  1928,  2088,  2216,  2344,  2536,  2664,  2792,  2984,  3112,  3240,  3368,  3496,  3624,  3752,  3880,  4008,  4264,  4392,  4584,  4584,  4776,  4968,  4968,  5160,  5352,  5544,  5544,  5736,  5992,  5992,  6200,  6200,  6456,  6456,  6712,  6968,  6968,  7224,  7224,  7480,  7480,  7736,  7736,  7992,  7992,  8248,  8504,  8504,  8760,  8760,  9144,  9144,  9144,  9528,  9528,  9528,  9912,  9912,  9912,  10296, 10296, 10680, 10680, 10680, 11064, 11064, 11064, 11448, 11448, 11448, 11832, 11832, 12216, 12216, 12216, 12576, 12576, 12576, 12960, 12960, 12960, 13536, 13536, 13536, 13536, 14112, 14112, 14112, 14112, 14688, 14688, 14688, 14688, 15264, 15264, 15264, 15264},
		{136,   296,   456,   616,   776,   936,   1096,  1256,  1416,  1544,  1736,  1864,  2024,  2216,  2344,  2536,  2664,  2856,  2984,  3112,  3368,  3496,  3624,  3752,  4008,  4136,  4264,  4392,  4584,  4776,  4968,  5160,  5160,  5352,  5544,  5736,  5736,  5992,  6200,  6200,  6456,  6712,  6712,  6968,  6968,  7224,  7480,  7480,  7736,  7992,  7992,  8248,  8248,  8504,  8760,  8760,  9144,  9144,  9144,  9528,  9528,  9912,  9912,  10296, 10296, 10296, 10680, 10680, 11064, 11064, 11064, 11448, 11448, 11832, 11832, 11832, 12216, 12216, 12576, 12576, 12960, 12960, 12960, 13536, 13536, 13536, 13536, 14112, 14112, 14112, 14112, 14688, 14688, 14688, 15264, 15264, 15264, 15264, 15840, 15840, 15840, 16416, 16416, 16416, 16416, 16992, 16992, 16992, 16992, 17568},
		{144,   328,   504,   680,   872,   1032,  1224,  1384,  1544,  1736,  1928,  2088,  2280,  2472,  2664,  2792,  2984,  3112,  3368,  3496,  3752,  3880,  4008,  4264,  4392,  4584,  4776,  4968,  5160,  5352,  5544,  5736,  5736,  5992,  6200,  6200,  6456,  6712,  6712,  6968,  7224,  7480,  7480,  7736,  7992,  7992,  8248,  8504,  8504,  8760,  9144,  9144,  9144,  9528,  9528,  9912,  9912,  10296, 10296, 10680, 10680, 11064, 11064, 11448, 11448, 11448, 11832, 11832, 12216, 12216, 12576, 12576, 12960, 12960, 12960, 13536, 13536, 13536, 14112, 14112, 14112, 14688, 14688, 14688, 14688, 15264, 15264, 15264, 15840, 15840, 15840, 16416, 16416, 16416, 16992, 16992, 16992, 16992, 17568, 17568, 17568, 18336, 18336, 18336, 18336, 18336, 19080, 19080, 19080, 19080},
		{176,   376,   584,   776,   1000,  1192,  1384,  1608,  1800,  2024,  2216,  2408,  2600,  2792,  2984,  3240,  3496,  3624,  3880,  4008,  4264,  4392,  4584,  4776,  4968,  5352,  5544,  5736,  5992,  5992,  6200,  6456,  6712,  6968,  6968,  7224,  7480,  7736,  7736,  7992,  8248,  8504,  8760,  8760,  9144,  9144,  9528,  9528,  9912,  9912,  10296, 10680, 10680, 11064, 11064, 11448, 11448, 11832, 11832, 12216, 12216, 12576, 12576, 12960, 12960, 13536, 13536, 13536, 14112, 14112, 14112, 14688, 14688, 14688, 15264, 15264, 15840, 15840, 15840, 16416, 16416, 16416, 16992, 16992, 16992, 17568, 17568, 17568, 18336, 18336, 18336, 18336, 19080, 19080, 19080, 19080, 19848, 19848, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 21384, 21384, 22152, 22152, 22152},
		{208,   440,   680,   904,   1128,  1352,  1608,  1800,  2024,  2280,  2472,  2728,  2984,  3240,  3368,  3624,  3880,  4136,  4392,  4584,  4776,  4968,  5352,  5544,  5736,  5992,  6200,  6456,  6712,  6712,  6968,  7224,  7480,  7736,  7992,  8248,  8504,  8760,  8760,  9144,  9528,  9528,  9912,  9912,  10296, 10680, 10680, 11064, 11064, 11448, 11832, 11832, 12216, 12216, 12576, 12576, 12960, 12960, 13536, 13536, 14112, 14112, 14112, 14688, 14688, 15264, 15264, 15264, 15840, 15840, 16416, 16416, 16416, 16992, 16992, 17568, 17568, 17568, 18336, 18336, 18336, 19080, 19080, 19080, 19080, 19848, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 21384, 21384, 22152, 22152, 22152, 22920, 22920, 22920, 23688, 23688, 23688, 23688, 24496, 24496, 24496, 24496, 25456},
		{224,   488,   744,   1000,  1256,  1544,  1800,  2024,  2280,  2536,  2856,  3112,  3368,  3624,  3880,  4136,  4392,  4584,  4968,  5160,  5352,  5736,  5992,  6200,  6456,  6712,  6968,  7224,  7480,  7736,  7992,  8248,  8504,  8760,  9144,  9144,  9528,  9912,  9912,  10296, 10680, 10680, 11064, 11448, 11448, 11832, 12216, 12216, 12576, 12960, 12960, 13536, 13536, 14112, 14112, 14688, 14688, 14688, 15264, 15264, 15840, 15840, 16416, 16416, 16992, 16992, 16992, 17568, 17568, 18336, 18336, 18336, 19080, 19080, 19080, 19848, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 21384, 22152, 22152, 22152, 22920, 22920, 22920, 23688, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 25456, 25456, 26416, 26416, 26416, 26416, 27376, 27376, 27376, 27376, 28336, 28336},
		{256,   552,   840,   1128,  1416,  1736,  1992,  2280,  2600,  2856,  3112,  3496,  3752,  4008,  4264,  4584,  4968,  5160,  5544,  5736,  5992,  6200,  6456,  6968,  7224,  7480,  7736,  7992,  8248,  8504,  8760,  9144,  9528,  9912,  9912,  10296, 10680, 11064, 11064, 11448, 11832, 12216, 12216, 12576, 12960, 12960, 13536, 13536, 14112, 14112, 14688, 14688, 15264, 15264, 15840, 15840, 16416, 16416, 16992, 16992, 17568, 17568, 18336, 18336, 18336, 19080, 19080, 19848, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 22152, 22152, 22152, 22920, 22920, 22920, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 25456, 25456, 26416, 26416, 26416, 27376, 27376, 27376, 28336, 28336, 28336, 28336, 29296, 29296, 29296, 29296, 30576, 30576, 30576, 30576, 31704, 31704},
		{280,   600,   904,   1224,  1544,  1800,  2152,  2472,  2728,  3112,  3368,  3624,  4008,  4264,  4584,  4968,  5160,  5544,  5736,  6200,  6456,  6712,  6968,  7224,  7736,  7992,  8248,  8504,  8760,  9144,  9528,  9912,  10296, 10296, 10680, 11064, 11448, 11832, 11832, 12216, 12576, 12960, 12960, 13536, 13536, 14112, 14688, 14688, 15264, 15264, 15840, 15840, 16416, 16416, 16992, 16992, 17568, 17568, 18336, 18336, 18336, 19080, 19080, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 22152, 22152, 22152, 22920, 22920, 23688, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 25456, 26416, 26416, 26416, 27376, 27376, 27376, 28336, 28336, 28336, 29296, 29296, 29296, 29296, 30576, 30576, 30576, 30576, 31704, 31704, 31704, 31704, 32856, 32856, 32856, 34008, 34008},
		{328,   632,   968,   1288,  1608,  1928,  2280,  2600,  2984,  3240,  3624,  3880,  4264,  4584,  4968,  5160,  5544,  5992,  6200,  6456,  6712,  7224,  7480,  7736,  7992,  8504,  8760,  9144,  9528,  9912,  9912,  10296, 10680, 11064, 11448, 11832, 12216, 12216, 12576, 12960, 13536, 13536, 14112, 14112, 14688, 14688, 15264, 15840, 15840, 16416, 16416, 16992, 16992, 17568, 17568, 18336, 18336, 19080, 19080, 19848, 19848, 19848, 20616, 20616, 21384, 21384, 22152, 22152, 22152, 22920, 22920, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 25456, 26416, 26416, 26416, 27376, 27376, 27376, 28336, 28336, 28336, 29296, 29296, 29296, 30576, 30576, 30576, 30576, 31704, 31704, 31704, 31704, 32856, 32856, 32856, 34008, 34008, 34008, 34008, 35160, 35160, 35160, 35160},
		{336,   696,   1064,  1416,  1800,  2152,  2536,  2856,  3240,  3624,  4008,  4392,  4776,  5160,  5352,  5736,  6200,  6456,  6712,  7224,  7480,  7992,  8248,  8760,  9144,  9528,  9912,  10296, 10296, 10680, 11064, 11448, 11832, 12216, 12576, 12960, 13536, 13536, 14112, 14688, 14688, 15264, 15264, 15840, 16416, 16416, 16992, 17568, 17568, 18336, 18336, 19080, 19080, 19848, 19848, 20616, 20616, 20616, 21384, 21384, 22152, 22152, 22920, 22920, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 26416, 26416, 26416, 27376, 27376, 27376, 28336, 28336, 29296, 29296, 29296, 30576, 30576, 30576, 30576, 31704, 31704, 31704, 32856, 32856, 32856, 34008, 34008, 34008, 35160, 35160, 35160, 35160, 36696, 36696, 36696, 36696, 37888, 37888, 37888, 39232, 39232, 39232, 39232},
		{376,   776,   1160,  1544,  1992,  2344,  2792,  3112,  3624,  4008,  4392,  4776,  5160,  5544,  5992,  6200,  6712,  7224,  7480,  7992,  8248,  8760,  9144,  9528,  9912,  10296, 10680, 11064, 11448, 11832, 12216, 12576, 12960, 13536, 14112, 14112, 14688, 15264, 15264, 15840, 16416, 16416, 16992, 17568, 17568, 18336, 18336, 19080, 19080, 19848, 19848, 20616, 21384, 21384, 22152, 22152, 22920, 22920, 23688, 23688, 24496, 24496, 24496, 25456, 25456, 26416, 26416, 27376, 27376, 27376, 28336, 28336, 29296, 29296, 29296, 30576, 30576, 30576, 31704, 31704, 31704, 32856, 32856, 32856, 34008, 34008, 34008, 35160, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 37888, 37888, 39232, 39232, 39232, 40576, 40576, 40576, 40576, 42368, 42368, 42368, 42368, 43816, 43816},
		{408,   840,   1288,  1736,  2152,  2600,  2984,  3496,  3880,  4264,  4776,  5160,  5544,  5992,  6456,  6968,  7224,  7736,  8248,  8504,  9144,  9528,  9912,  10296, 10680, 11064, 11448, 12216, 12576, 12960, 13536, 13536, 14112, 14688, 15264, 15264, 15840, 16416, 16992, 16992, 17568, 18336, 18336, 19080, 19080, 19848, 20616, 20616, 21384, 21384, 22152, 22152, 22920, 22920, 23688, 24496, 24496, 25456, 25456, 25456, 26416, 26416, 27376, 27376, 28336, 28336, 29296, 29296, 29296, 30576, 30576, 30576, 31704, 31704, 32856, 32856, 32856, 34008, 34008, 34008, 35160, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 37888, 39232, 39232, 39232, 40576, 40576, 40576, 40576, 42368, 42368, 42368, 43816, 43816, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 46888},
		{440,   904,   1384,  1864,  2344,  2792,  3240,  3752,  4136,  4584,  5160,  5544,  5992,  6456,  6968,  7480,  7992,  8248,  8760,  9144,  9912,  10296, 10680, 11064, 11448, 12216, 12576, 12960, 13536, 14112, 14688, 14688, 15264, 15840, 16416, 16992, 16992, 17568, 18336, 18336, 19080, 19848, 19848, 20616, 20616, 21384, 22152, 22152, 22920, 22920, 23688, 24496, 24496, 25456, 25456, 26416, 26416, 27376, 27376, 28336, 28336, 29296, 29296, 29296, 30576, 30576, 31704, 31704, 31704, 32856, 32856, 34008, 34008, 34008, 35160, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 39232, 39232, 39232, 40576, 40576, 40576, 42368, 42368, 42368, 42368, 43816, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 46888, 48936, 48936, 48936, 48936, 48936, 51024, 51024, 51024},
		{488,   1000,  1480,  1992,  2472,  2984,  3496,  4008,  4584,  4968,  5544,  5992,  6456,  6968,  7480,  7992,  8504,  9144,  9528,  9912,  10680, 11064, 11448, 12216, 12576, 12960, 13536, 14112, 14688, 15264, 15840, 15840, 16416, 16992, 17568, 18336, 18336, 19080, 19848, 19848, 20616, 21384, 21384, 22152, 22920, 22920, 23688, 24496, 24496, 25456, 25456, 26416, 26416, 27376, 27376, 28336, 28336, 29296, 29296, 30576, 30576, 31704, 31704, 31704, 32856, 32856, 34008, 34008, 35160, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 39232, 39232, 39232, 40576, 40576, 40576, 42368, 42368, 42368, 43816, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 46888, 48936, 48936, 48936, 48936, 51024, 51024, 51024, 51024, 52752, 52752, 52752, 52752, 55056, 55056, 55056},
		{520,   1064,  1608,  2152,  2664,  3240,  3752,  4264,  4776,  5352,  5992,  6456,  6968,  7480,  7992,  8504,  9144,  9528,  10296, 10680, 11448, 11832, 12576, 12960, 13536, 14112, 14688, 15264, 15840, 16416, 16992, 16992, 17568, 18336, 19080, 19080, 19848, 20616, 21384, 21384, 22152, 22920, 22920, 23688, 24496, 24496, 25456, 25456, 26416, 27376, 27376, 28336, 28336, 29296, 29296, 30576, 30576, 31704, 31704, 32856, 32856, 34008, 34008, 34008, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 39232, 39232, 40576, 40576, 40576, 42368, 42368, 42368, 43816, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 48936, 48936, 48936, 48936, 51024, 51024, 51024, 51024, 52752, 52752, 52752, 55056, 55056, 55056, 55056, 57336, 57336, 57336, 57336, 59256, 59256, 59256},
		{552,   1128,  1736,  2280,  2856,  3496,  4008,  4584,  5160,  5736,  6200,  6968,  7480,  7992,  8504,  9144,  9912,  10296, 11064, 11448, 12216, 12576, 12960, 13536, 14112, 14688, 15264, 15840, 16416, 16992, 17568, 18336, 19080, 19848, 19848, 20616, 21384, 22152, 22152, 22920, 23688, 24496, 24496, 25456, 25456, 26416, 27376, 27376, 28336, 28336, 29296, 29296, 30576, 30576, 31704, 31704, 32856, 32856, 34008, 34008, 35160, 35160, 36696, 36696, 37888, 37888, 37888, 39232, 39232, 40576, 40576, 40576, 42368, 42368, 43816, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 48936, 48936, 48936, 51024, 51024, 51024, 51024, 52752, 52752, 52752, 55056, 55056, 55056, 55056, 57336, 57336, 57336, 57336, 59256, 59256, 59256, 59256, 61664, 61664, 61664, 61664, 63776},
		{584,   1192,  1800,  2408,  2984,  3624,  4264,  4968,  5544,  5992,  6712,  7224,  7992,  8504,  9144,  9912,  10296, 11064, 11448, 12216, 12960, 13536, 14112, 14688, 15264, 15840, 16416, 16992, 17568, 18336, 19080, 19848, 19848, 20616, 21384, 22152, 22920, 22920, 23688, 24496, 25456, 25456, 26416, 26416, 27376, 28336, 28336, 29296, 29296, 30576, 31704, 31704, 32856, 32856, 34008, 34008, 35160, 35160, 36696, 36696, 36696, 37888, 37888, 39232, 39232, 40576, 40576, 42368, 42368, 42368, 43816, 43816, 45352, 45352, 45352, 46888, 46888, 46888, 48936, 48936, 48936, 51024, 51024, 51024, 52752, 52752, 52752, 52752, 55056, 55056, 55056, 57336, 57336, 57336, 57336, 59256, 59256, 59256, 61664, 61664, 61664, 61664, 63776, 63776, 63776, 63776, 66592, 66592, 66592, 66592},
		{616,   1256,  1864,  2536,  3112,  3752,  4392,  5160,  5736,  6200,  6968,  7480,  8248,  8760,  9528,  10296, 10680, 11448, 12216, 12576, 13536, 14112, 14688, 15264, 15840, 16416, 16992, 17568, 18336, 19080, 19848, 20616, 20616, 21384, 22152, 22920, 23688, 24496, 24496, 25456, 26416, 26416, 27376, 28336, 28336, 29296, 29296, 30576, 31704, 31704, 32856, 32856, 34008, 34008, 35160, 35160, 36696, 36696, 37888, 37888, 39232, 39232, 40576, 40576, 40576, 42368, 42368, 43816, 43816, 43816, 45352, 45352, 46888, 46888, 46888, 48936, 48936, 48936, 51024, 51024, 51024, 52752, 52752, 52752, 55056, 55056, 55056, 55056, 57336, 57336, 57336, 59256, 59256, 59256, 61664, 61664, 61664, 61664, 63776, 63776, 63776, 63776, 66592, 66592, 66592, 66592, 68808, 68808, 68808, 71112},
		{712,   1480,  2216,  2984,  3752,  4392,  5160,  5992,  6712,  7480,  8248,  8760,  9528,  10296, 11064, 11832, 12576, 13536, 14112, 14688, 15264, 16416, 16992, 17568, 18336, 19080, 19848, 20616, 21384, 22152, 22920, 23688, 24496, 25456, 25456, 26416, 27376, 28336, 29296, 29296, 30576, 30576, 31704, 32856, 32856, 34008, 35160, 35160, 36696, 36696, 37888, 37888, 39232, 40576, 40576, 40576, 42368, 42368, 43816, 43816, 45352, 45352, 46888, 46888, 48936, 48936, 48936, 51024, 51024, 52752, 52752, 52752, 55056, 55056, 55056, 55056, 57336, 57336, 57336, 59256, 59256, 59256, 61664, 61664, 61664, 63776, 63776, 63776, 66592, 66592, 66592, 68808, 68808, 68808, 71112, 71112, 71112, 73712, 73712, 75376, 75376, 75376, 75376, 75376, 75376, 75376, 75376, 75376, 75376, 75376}

};

static const uint8_t PathLoss[4] = { 100, 105, 110, 115 };
static const uint8_t TargetCqi[5] = { 11, 10, 8, 7, 5 };

//Conver number dB pwr to number of RBs
static const uint8_t PwrToRbTbl[20+1] = {
		1, 1, 2, 2, 3, 4, 5, 6, 7, 9, 11,
		13, 17, 21, 26, 33, 41, 52, 65, 82, 103
};

static const double MCS_eff[29] = {
		0.1955,0.2533,0.3111,0.4,0.5066,0.6133,0.72,0.8644,0.9711,1.1133,
		1.22,1.22,1.38,1.5933,1.8033,1.9633,2.1233,2.2833,2.55,2.76,2.9733,2.9733,3.1866,3.5388,3.8055,
		3.9388, 4.25, 4.406, 5.1
};

//static const double ITBS_eff[27] = {
//		0.1955,0.2533,0.3111,0.4,0.5066,0.6133,0.72,0.8644,0.9711,1.1133,
//		1.22,1.38,1.5933,1.8033,1.9633,2.1233,2.2833,2.55,2.76,2.9733,3.1866,3.5388,3.8055,
//		3.9388, 4.25, 4.406, 5.1
//};

static const uint8_t sinr_target_per_cqi[29] = {117,118,120,122,124,126,127,130,132,134,135,139,140,
		142,144,145,146,148,150,152,153,154,156,158,160,161,164,165,169
};

static uint64_t tti_counter = 0;

/*
 * Just for testing
 */
static uint64_t success[65536] = {0};
static uint64_t miss[65536] = {0};



NS_OBJECT_ENSURE_REGISTERED (pw_PfFfMacScheduler);



pw_PfFfMacScheduler::pw_PfFfMacScheduler ()
: m_cschedSapUser (0),
  m_schedSapUser (0),
  m_pwEnbId (0),
  m_pwMaxTPprio (0),
  m_schedulerStatsFirstWrite (true),
  m_timeWindow (99.0),
  m_nextRntiUl (0),
  m_volteOn(false),
  m_ttiBundlingOn(false),
  m_ttiBundlingOffsetDB(6)
{
	m_amc = CreateObject <LteAmc> ();
	m_cschedSapProvider = new MemberCschedSapProvider<pw_PfFfMacScheduler> (this);
	m_schedSapProvider = new MemberSchedSapProvider<pw_PfFfMacScheduler> (this);
	m_ffrSapProvider = 0;
	m_ffrSapUser = new MemberLteFfrSapUser<pw_PfFfMacScheduler> (this);
}

pw_PfFfMacScheduler::~pw_PfFfMacScheduler ()
{
	NS_LOG_FUNCTION (this);
}

void
pw_PfFfMacScheduler::DoDispose ()
{
	NS_LOG_FUNCTION (this);
	m_dlHarqProcessesDciBuffer.clear ();
	m_dlHarqProcessesTimer.clear ();
	m_dlHarqProcessesRlcPduListBuffer.clear ();
	m_dlInfoListBuffered.clear ();
	m_ulHarqCurrentProcessId.clear ();
	m_ulHarqProcessesStatus.clear ();
	m_ulHarqProcessesDciBuffer.clear ();
	delete m_cschedSapProvider;
	delete m_schedSapProvider;
	delete m_ffrSapUser;
}

TypeId
pw_PfFfMacScheduler::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::pw_PfFfMacScheduler")
    								.SetParent<FfMacScheduler> ()
									.SetGroupName("Lte")
									.AddConstructor<pw_PfFfMacScheduler> ()
									.AddAttribute ("CqiTimerThreshold","The number of TTIs a CQI is valid (default 1000 - 1 sec.)",
											UintegerValue (1000),
											MakeUintegerAccessor (&pw_PfFfMacScheduler::m_cqiTimersThreshold),
											MakeUintegerChecker<uint32_t> ())
											.AddAttribute ("HarqEnabled",
													"Activate/Deactivate the HARQ [by default is active].",
													BooleanValue (true),
													MakeBooleanAccessor (&pw_PfFfMacScheduler::m_harqOn),
													MakeBooleanChecker ())
													.AddAttribute ("UlGrantMcs","The MCS of the UL grant, must be [0..15] (default 0)",
															UintegerValue (0),
															MakeUintegerAccessor (&pw_PfFfMacScheduler::m_ulGrantMcs),
															MakeUintegerChecker<uint8_t> ())
															.AddAttribute ("EfficiencyState",
																	"Work at Throughput mode or Radisys Spectral Efficiency",
																	UintegerValue (1),
																	MakeUintegerAccessor (&pw_PfFfMacScheduler::eff_state),
																	MakeUintegerChecker<uint8_t> ())
																	.AddAttribute ("IsVoLTE","enable volte schedular and link adaptation configuration",
																			BooleanValue (false),
																			MakeBooleanAccessor (&pw_PfFfMacScheduler::m_volteOn),
																			MakeBooleanChecker())
																			.AddAttribute("IsTTIBundling","Enable TTI bundling like operation",
																					BooleanValue (false),
																					MakeBooleanAccessor (&pw_PfFfMacScheduler::m_ttiBundlingOn),
																					MakeBooleanChecker())
																					.AddAttribute("TTIBundlingOffset","offset added in DB to get more RBs for same pwr when tti bundling enabled",
																							UintegerValue (6),
																							MakeUintegerAccessor (&pw_PfFfMacScheduler::m_ttiBundlingOffsetDB),
																							MakeUintegerChecker<uint16_t> ());
	return tid;
}



void
pw_PfFfMacScheduler::SetFfMacCschedSapUser (FfMacCschedSapUser* s)
{
	m_cschedSapUser = s;
}

void
pw_PfFfMacScheduler::SetFfMacSchedSapUser (FfMacSchedSapUser* s)
{
	m_schedSapUser = s;
}

FfMacCschedSapProvider*
pw_PfFfMacScheduler::GetFfMacCschedSapProvider ()
{
	return m_cschedSapProvider;
}

FfMacSchedSapProvider*
pw_PfFfMacScheduler::GetFfMacSchedSapProvider ()
{
	return m_schedSapProvider;
}

void
pw_PfFfMacScheduler::SetLteFfrSapProvider (LteFfrSapProvider* s)
{
	m_ffrSapProvider = s;
}

LteFfrSapUser*
pw_PfFfMacScheduler::GetLteFfrSapUser ()
{
	return m_ffrSapUser;
}

void
pw_PfFfMacScheduler::DoCschedCellConfigReq (const struct FfMacCschedSapProvider::CschedCellConfigReqParameters& params)
{
	PfsUlCell PfsCell;
	uint32_t minTpprio;
	uint32_t maxTpprio;
	uint32_t minFairprio;
	uint32_t maxFairprio;
	uint32_t minGBRprio;
	uint32_t maxGBRprio;
	uint32_t minAMBRprio;
	uint32_t maxAMBRprio;
	uint32_t minprio;
	uint32_t maxprio;
	NS_LOG_FUNCTION (this);
	// Read the subset of parameters used
	m_cschedCellConfig = params;
	m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
	FfMacCschedSapUser::CschedUeConfigCnfParameters cnf;
	cnf.m_result = SUCCESS;
	m_cschedSapUser->CschedUeConfigCnf (cnf);

	//Fill tables of weights with predefined tables
	if (PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE) {
		/* VFT Modification */
		PfsCell.tbls.tpPrios = FillTbl(PfsTpFactorSelector_UL[TP_FACTOR],
				PfsCell.tbls.tpPrios, 0, PFS_TP_FACTOR_LVLS, &minTpprio, &maxTpprio);
		PfsCell.tbls.fairPrios = FillTbl(PfsFairnessFactorSelector_UL[FAIRNESS_FACTOR],
				PfsCell.tbls.fairPrios,	0, PFS_FAIRNESS_FACTOR_LVLS, &minFairprio, &maxFairprio);

		//Calculate scaling for fairness and throughput weights
		PfsCell.priosScaleFact = PfsScalingFactor_UL[TP_FACTOR] * PfsScalingFactor_UL[FAIRNESS_FACTOR];

	} else {
		/* Radisys Implementation */
		PfsCell.tbls.tpPrios = FillTbl(PfsTpFactorSelector[TP_FACTOR],
				PfsCell.tbls.tpPrios, 0, PFS_TP_FACTOR_LVLS, &minTpprio, &maxTpprio);
		PfsCell.tbls.fairPrios = FillTbl(PfsFairnessFactorSelector[FAIRNESS_FACTOR],
				PfsCell.tbls.fairPrios, 0, PFS_FAIRNESS_FACTOR_LVLS, &minFairprio, &maxFairprio);

		//Calculate scaling for fairness and throughput weights
		PfsCell.priosScaleFact = PfsScalingFactor[TP_FACTOR] * PfsScalingFactor[FAIRNESS_FACTOR];

	}

	PfsCell.tbls.svcQciPrios = FillTbl(Qciprio, PfsCell.tbls.svcQciPrios,
			GBR_QCI_start, GBR_QCI_end - GBR_QCI_start + 1, &minGBRprio, &maxGBRprio);

	// Store maxTpprio to use later in calculating PfsPrio
	m_pwMaxTPprio = maxTpprio;

	minprio = (minTpprio*minFairprio*minGBRprio)/PfsCell.priosScaleFact;
	maxprio = (maxTpprio*maxFairprio*maxGBRprio)/PfsCell.priosScaleFact;
	//PfsCell.maxSrbDrbPrioval = (maxTpprio*maxFairprio*PfsCell.tbls.svcQciPrios)*PfsCell.priosScaleFact;
	PfsCell.ranges.gbrRange = FillRangeQueues(PfsCell.ranges.gbrRange, minprio, maxprio, ULNumGbrQueues);

	PfsCell.tbls.svcQciPrios = FillTbl(Qciprio, PfsCell.tbls.svcQciPrios, NGBR_QCI_start-1, NGBR_QCI_end-NGBR_QCI_start+1, &minAMBRprio,&maxAMBRprio);
	minprio = (minTpprio*minFairprio*minAMBRprio)/PfsCell.priosScaleFact;
	maxprio = (maxTpprio*maxFairprio*maxAMBRprio)/PfsCell.priosScaleFact;
	PfsCell.ranges.ambrRange = FillRangeQueues(PfsCell.ranges.ambrRange, minprio, maxprio, ULNumAmbrQueues);

	//Need to add MBR,AMBR.. and what's left in the function.
	//Currently PfsUlCell is global, need to see what will happen with more Enbs
	//Configure Max Ues to scheudle.
	PfsCell.MaxUesPerTti = params.m_vendorSpecificList.front().m_type;
	PfsCell.RePerRb = 12*(14-UL_DMRS);
	globalPfs = PfsCell;

	if (m_cschedCellConfig.m_ulBandwidth < 7)
		SbsSize = 0;
	else if (m_cschedCellConfig.m_ulBandwidth < 26)
		SbsSize = 2;
	else if (m_cschedCellConfig.m_ulBandwidth < 63)
		SbsSize = 3;
	else
		SbsSize = 4;

	CalcEffTbl();

	return;
}

void
pw_PfFfMacScheduler::CalcEffTbl(){
	for (int i = 0; i < MAX_ITBS+1; i++){
		TbSzEff[i] = 0;
		for (int j = 0; j < 110; j++){
			TbSzEff[i] += (TbSzTbl[i][j]*1024) / (globalPfs.RePerRb * (j+1));
		}
		TbSzEff[i] /= (uint32_t)110;


	}

}
PfsPrioRange
pw_PfFfMacScheduler::FillRangeQueues(PfsPrioRange range, uint8_t minprio, uint32_t maxprio, uint8_t NumQs){
	range.min = minprio;
	range.max = maxprio;
	range.maxDiff = maxprio - minprio + 1;
	range.NumQueus = NumQs;

	return range;

}
std::vector<uint32_t>
pw_PfFfMacScheduler::FillTbl(uint32_t *src, std::vector<uint32_t> dst,uint32_t start, uint8_t num_entries, uint32_t *min, uint32_t *max){
	uint32_t val;
	uint32_t minval;
	uint32_t maxval;

	uint32_t i;
	minval = src[start];
	maxval = src[start];
	for (i = start; i<start + num_entries;i++){
		val = src[i];
		if (val<minval && val!=0){
			minval = val;

		}
		if (val> maxval){
			maxval= val;
		}
		dst.push_back(val);
	}
	*min = minval;
	*max = maxval;

	return dst;

}
void
pw_PfFfMacScheduler::DoCschedUeConfigReq (const struct FfMacCschedSapProvider::CschedUeConfigReqParameters& params)
{
	NS_LOG_FUNCTION (this << " RNTI " << params.m_rnti << " txMode " << (uint16_t)params.m_transmissionMode);
	std::map <uint16_t,PfsUlUe>::iterator it_Pfs = PfsUlUes.find (params.m_rnti);
	std::map <uint16_t,uint8_t>::iterator it = m_uesTxMode.find (params.m_rnti);
	PfsUlUe PfsUe;

	if (it == m_uesTxMode.end ())
	{
		m_uesTxMode.insert (std::pair <uint16_t, double> (params.m_rnti, params.m_transmissionMode));
		// generate HARQ buffers
		m_dlHarqCurrentProcessId.insert (std::pair <uint16_t,uint8_t > (params.m_rnti, 0));
		DlHarqProcessesStatus_t dlHarqPrcStatus;
		dlHarqPrcStatus.resize (8,0);
		m_dlHarqProcessesStatus.insert (std::pair <uint16_t, DlHarqProcessesStatus_t> (params.m_rnti, dlHarqPrcStatus));
		DlHarqProcessesTimer_t dlHarqProcessesTimer;
		dlHarqProcessesTimer.resize (8,0);
		m_dlHarqProcessesTimer.insert (std::pair <uint16_t, DlHarqProcessesTimer_t> (params.m_rnti, dlHarqProcessesTimer));
		DlHarqProcessesDciBuffer_t dlHarqdci;
		dlHarqdci.resize (8);
		m_dlHarqProcessesDciBuffer.insert (std::pair <uint16_t, DlHarqProcessesDciBuffer_t> (params.m_rnti, dlHarqdci));
		DlHarqRlcPduListBuffer_t dlHarqRlcPdu;
		dlHarqRlcPdu.resize (2);
		dlHarqRlcPdu.at (0).resize (8);
		dlHarqRlcPdu.at (1).resize (8);
		m_dlHarqProcessesRlcPduListBuffer.insert (std::pair <uint16_t, DlHarqRlcPduListBuffer_t> (params.m_rnti, dlHarqRlcPdu));
		m_ulHarqCurrentProcessId.insert (std::pair <uint16_t,uint8_t > (params.m_rnti, 0));
		UlHarqProcessesStatus_t ulHarqPrcStatus;
		ulHarqPrcStatus.resize (8,0);
		m_ulHarqProcessesStatus.insert (std::pair <uint16_t, UlHarqProcessesStatus_t> (params.m_rnti, ulHarqPrcStatus));
		UlHarqProcessesDciBuffer_t ulHarqdci;
		ulHarqdci.resize (8);
		m_ulHarqProcessesDciBuffer.insert (std::pair <uint16_t, UlHarqProcessesDciBuffer_t> (params.m_rnti, ulHarqdci));
	}
	else
	{
		(*it).second = params.m_transmissionMode;
	}
	if (it_Pfs == PfsUlUes.end ()){
		PfsUe.cqi = UL_NUM_CQI;
		PfsUe.cqiPrio = PfsUlGetTpPrio(globalPfs, MAX_ITBS);
		PfsUe.effBsr = 0;
		//pfsUe->prioLnk.node = (PTR)ue;
		//pfsUe->pfsPrio = pfsCell->maxPrioValAmbr;
		//pfsUe.pfsPrio = 32 - pfsUe.lcgPrio * 4;
		//initfracprio need to configure the AMBR frac data rate
		PfsUe.fracPrioInfo = PfsinitFracPrioInfo(PfsUe.fracPrioInfo, 10000);

		PfsUlUes.insert (std::pair <uint16_t, PfsUlUe> (params.m_rnti, PfsUe));
		// ue->ul.cfgdAmbr = (ueCfg->ueQosCfg.ueBr * RG_SCH_CMN_REFRESH_TIME)/100;
		//Check rgSCHCmnRgrUeCfg
	}
	else
	{
		NS_LOG_FUNCTION (this << " RNTI " << params.m_rnti << " UE already in PfsUlUes ");

	}




	return;
}
uint8_t
pw_PfFfMacScheduler::PfsUlGetTpPrio(PfsUlCell PfsCell, uint8_t MAX_ITBS){
	return PfsCell.tbls.tpPrios[MAX_ITBS];


}

void
pw_PfFfMacScheduler::DoCschedLcConfigReq (const struct FfMacCschedSapProvider::CschedLcConfigReqParameters& params)
{
	NS_LOG_FUNCTION (this << " New LC, rnti: "  << params.m_rnti);
	uint8_t idx;



	std::map <uint16_t,PfsUlUe>::iterator it_Pfs;

	std::map <uint16_t, pw_pfsFlowPerf_t>::iterator it;

	for (uint16_t i = 0; i < params.m_logicalChannelConfigList.size (); i++)
	{
		it = m_flowStatsDl.find (params.m_rnti);
		it_Pfs = PfsUlUes.find (params.m_rnti);


		if (it == m_flowStatsDl.end ())
		{
			pw_pfsFlowPerf_t flowStatsDl;
			flowStatsDl.flowStart = Simulator::Now ();
			flowStatsDl.totalBytesTransmitted = 0;
			flowStatsDl.lastTtiBytesTrasmitted = 0;
			flowStatsDl.lastAveragedThroughput = 1;
			m_flowStatsDl.insert (std::pair<uint16_t, pw_pfsFlowPerf_t> (params.m_rnti, flowStatsDl));
			pw_pfsFlowPerf_t flowStatsUl;
			flowStatsUl.flowStart = Simulator::Now ();
			flowStatsUl.totalBytesTransmitted = 0;
			flowStatsUl.lastTtiBytesTrasmitted = 0;
			flowStatsUl.lastAveragedThroughput = 1;
			m_flowStatsUl.insert (std::pair<uint16_t, pw_pfsFlowPerf_t> (params.m_rnti, flowStatsUl));
		}
		//Currently LCs in m_logicalChannelConfigList arent by an increasing qci order like in radysis

		//Get QCI of the first LC in the LCG for LCG priority of the UE
		//(*it_Pfs).second.lcgPrio = globalPfs.tbls.svcQciPrios[params.m_logicalChannelConfigList.front().m_qci-1];
		//Add LC to LCG of UE
		idx = params.m_logicalChannelConfigList[i].m_logicalChannelGroup;
		(*it_Pfs).second.schedLcgInfo[idx].LcsInfo.push_back (params.m_logicalChannelConfigList[i]);
		//Update LCG priority from the first QCI of the LC in that LCG( should be in a sorted array of qci)
		(*it_Pfs).second.schedLcgInfo[idx].lcgPrio = globalPfs.tbls.svcQciPrios[params.m_logicalChannelConfigList.front().m_qci-1];
		if (params.m_logicalChannelConfigList[i].m_qosBearerType == LogicalChannelConfigListElement_s::QBT_GBR)
			(*it_Pfs).second.schedLcgInfo[idx].cfgdGbr += params.m_logicalChannelConfigList[i].m_eRabGuaranteedBitrateUl;
		else
			(*it_Pfs).second.schedLcgInfo[idx].cfgdGbr = 0;
		//rgSCHPfsInitFracPrioInfo(&pfsUe->schedLcgInfo[lcg->lcgId].fracPrioInfo, cmnLcg->cfgdGbr);


	}
	(*it_Pfs).second.schedLcgInfo[idx].fracPrioInfo = PfsinitFracPrioInfo((*it_Pfs).second.schedLcgInfo[idx].fracPrioInfo, (*it_Pfs).second.schedLcgInfo[idx].cfgdGbr);




	return;
}

void
pw_PfFfMacScheduler::DoCschedLcReleaseReq (const struct FfMacCschedSapProvider::CschedLcReleaseReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	for (uint16_t i = 0; i < params.m_logicalChannelIdentity.size (); i++)
	{
		std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it = m_rlcBufferReq.begin ();
		std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator temp;
		while (it!=m_rlcBufferReq.end ())
		{
			if (((*it).first.m_rnti == params.m_rnti) && ((*it).first.m_lcId == params.m_logicalChannelIdentity.at (i)))
			{
				temp = it;
				it++;
				m_rlcBufferReq.erase (temp);
			}
			else
			{
				it++;
			}
		}
	}
	return;
}

void
pw_PfFfMacScheduler::DoCschedUeReleaseReq (const struct FfMacCschedSapProvider::CschedUeReleaseReqParameters& params)
{
	NS_LOG_FUNCTION (this);

	m_uesTxMode.erase (params.m_rnti);
	m_dlHarqCurrentProcessId.erase (params.m_rnti);
	m_dlHarqProcessesStatus.erase  (params.m_rnti);
	m_dlHarqProcessesTimer.erase (params.m_rnti);
	m_dlHarqProcessesDciBuffer.erase  (params.m_rnti);
	m_dlHarqProcessesRlcPduListBuffer.erase  (params.m_rnti);
	m_ulHarqCurrentProcessId.erase  (params.m_rnti);
	m_ulHarqProcessesStatus.erase  (params.m_rnti);
	m_ulHarqProcessesDciBuffer.erase  (params.m_rnti);
	m_flowStatsDl.erase  (params.m_rnti);
	m_flowStatsUl.erase  (params.m_rnti);
	m_ceBsrRxed.erase (params.m_rnti);
	ueUl.erase(params.m_rnti);

	std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it = m_rlcBufferReq.begin ();
	std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator temp;
	while (it!=m_rlcBufferReq.end ())
	{
		if ((*it).first.m_rnti == params.m_rnti)
		{
			temp = it;
			it++;
			m_rlcBufferReq.erase (temp);
		}
		else
		{
			it++;
		}
	}
	if (m_nextRntiUl == params.m_rnti)
	{
		m_nextRntiUl = 0;
	}

	return;
}


void
pw_PfFfMacScheduler::DoSchedDlRlcBufferReq (const struct FfMacSchedSapProvider::SchedDlRlcBufferReqParameters& params)
{
	NS_LOG_FUNCTION (this << params.m_rnti << (uint32_t) params.m_logicalChannelIdentity);
	// API generated by RLC for updating RLC parameters on a LC (tx and retx queues)

	std::map <LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;

	LteFlowId_t flow (params.m_rnti, params.m_logicalChannelIdentity);

	it =  m_rlcBufferReq.find (flow);

	if (it == m_rlcBufferReq.end ())
	{
		m_rlcBufferReq.insert (std::pair <LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters> (flow, params));
	}
	else
	{
		(*it).second = params;
	}

	return;
}

void
pw_PfFfMacScheduler::DoSchedDlPagingBufferReq (const struct FfMacSchedSapProvider::SchedDlPagingBufferReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	NS_FATAL_ERROR ("method not implemented");
	return;
}

void
pw_PfFfMacScheduler::DoSchedDlMacBufferReq (const struct FfMacSchedSapProvider::SchedDlMacBufferReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	NS_FATAL_ERROR ("method not implemented");
	return;
}

int
pw_PfFfMacScheduler::GetRbgSize (int dlbandwidth)
{
	for (int i = 0; i < 4; i++)
	{
		if (dlbandwidth < PfType0AllocationRbg[i])
		{
			return (i + 1);
		}
	}

	return (-1);
}


unsigned int
pw_PfFfMacScheduler::LcActivePerFlow (uint16_t rnti)
{
	std::map <LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
	unsigned int lcActive = 0;
	for (it = m_rlcBufferReq.begin (); it != m_rlcBufferReq.end (); it++)
	{
		if (((*it).first.m_rnti == rnti) && (((*it).second.m_rlcTransmissionQueueSize > 0)
				|| ((*it).second.m_rlcRetransmissionQueueSize > 0)
				|| ((*it).second.m_rlcStatusPduSize > 0) ))
		{
			lcActive++;
		}
		if ((*it).first.m_rnti > rnti)
		{
			break;
		}
	}
	return (lcActive);

}


uint8_t
pw_PfFfMacScheduler::HarqProcessAvailability (uint16_t rnti)
{
	NS_LOG_FUNCTION (this << rnti);

	std::map <uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find (rnti);
	if (it == m_dlHarqCurrentProcessId.end ())
	{
		NS_FATAL_ERROR ("No Process Id found for this RNTI " << rnti);
	}
	std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find (rnti);
	if (itStat == m_dlHarqProcessesStatus.end ())
	{
		NS_FATAL_ERROR ("No Process Id Statusfound for this RNTI " << rnti);
	}
	uint8_t i = (*it).second;
	do
	{
		i = (i + 1) % HARQ_PROC_NUM;
	}
	while ( ((*itStat).second.at (i) != 0)&&(i != (*it).second));
	if ((*itStat).second.at (i) == 0)
	{
		return (true);
	}
	else
	{
		return (false); // return a not valid harq proc id
	}
}



uint8_t
pw_PfFfMacScheduler::UpdateHarqProcessId (uint16_t rnti)
{
	NS_LOG_FUNCTION (this << rnti);

	if (m_harqOn == false)
	{
		return (0);
	}


	std::map <uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find (rnti);
	if (it == m_dlHarqCurrentProcessId.end ())
	{
		NS_FATAL_ERROR ("No Process Id found for this RNTI " << rnti);
	}
	std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find (rnti);
	if (itStat == m_dlHarqProcessesStatus.end ())
	{
		NS_FATAL_ERROR ("No Process Id Statusfound for this RNTI " << rnti);
	}
	uint8_t i = (*it).second;
	do
	{
		i = (i + 1) % HARQ_PROC_NUM;
	}
	while ( ((*itStat).second.at (i) != 0)&&(i != (*it).second));
	if ((*itStat).second.at (i) == 0)
	{
		(*it).second = i;
		(*itStat).second.at (i) = 1;
	}
	else
	{
		NS_FATAL_ERROR ("No HARQ process available for RNTI " << rnti << " check before update with HarqProcessAvailability");
	}

	return ((*it).second);
}


void
pw_PfFfMacScheduler::RefreshHarqProcesses ()
{
	NS_LOG_FUNCTION (this);

	std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itTimers;
	for (itTimers = m_dlHarqProcessesTimer.begin (); itTimers != m_dlHarqProcessesTimer.end (); itTimers++)
	{
		for (uint16_t i = 0; i < HARQ_PROC_NUM; i++)
		{
			if ((*itTimers).second.at (i) == HARQ_DL_TIMEOUT)
			{
				// reset HARQ process

				NS_LOG_DEBUG (this << " Reset HARQ proc " << i << " for RNTI " << (*itTimers).first);
				std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find ((*itTimers).first);
				if (itStat == m_dlHarqProcessesStatus.end ())
				{
					NS_FATAL_ERROR ("No Process Id Status found for this RNTI " << (*itTimers).first);
				}
				(*itStat).second.at (i) = 0;
				(*itTimers).second.at (i) = 0;
			}
			else
			{
				(*itTimers).second.at (i)++;
			}
		}
	}

}


void
pw_PfFfMacScheduler::DoSchedDlTriggerReq (const struct FfMacSchedSapProvider::SchedDlTriggerReqParameters& params)
{
	NS_LOG_FUNCTION (this << " Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf));
	// API generated by RLC for triggering the scheduling of a DL subframe


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.368) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq "
				<< " DL - Frame no. " << (params.m_sfnSf >> 4)
				<< " subframe no. " << (0xF & params.m_sfnSf)
				<< " eNB " << m_pwEnbId
				//	<< " size " << params.m_ulInfoList.size ()
				//	<< " UEs/TTI " << (int)globalPfs.MaxUesPerTti
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	// evaluate the relative channel quality indicator for each UE per each RBG
	// (since we are using allocation type 0 the small unit of allocation is RBG)
	// Resource allocation type 0 (see sec 7.1.6.1 of 36.213)

	RefreshDlCqiMaps ();

	int rbgSize = GetRbgSize (m_cschedCellConfig.m_dlBandwidth);
	int rbgNum = m_cschedCellConfig.m_dlBandwidth / rbgSize;
	std::map <uint16_t, std::vector <uint16_t> > allocationMap; // RBs map per RNTI
	std::vector <bool> rbgMap;  // global RBGs map
	uint16_t rbgAllocatedNum = 0;
	std::set <uint16_t> rntiAllocated;
	rbgMap.resize (m_cschedCellConfig.m_dlBandwidth / rbgSize, false);

	rbgMap = m_ffrSapProvider->GetAvailableDlRbg ();
	for (std::vector<bool>::iterator it = rbgMap.begin (); it != rbgMap.end (); it++)
	{
		if ((*it) == true )
		{
			rbgAllocatedNum++;
		}
	}

	FfMacSchedSapUser::SchedDlConfigIndParameters ret;

	//   update UL HARQ proc id
	std::map <uint16_t, uint8_t>::iterator itProcId;
	for (itProcId = m_ulHarqCurrentProcessId.begin (); itProcId != m_ulHarqCurrentProcessId.end (); itProcId++)
	{
		(*itProcId).second = ((*itProcId).second + 1) % HARQ_PROC_NUM;
	}


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step1 - init"
				<< " rbgSize " << rbgSize
				<< " rbgNum " << rbgNum
				<< " rbgAllocatedNum " << rbgAllocatedNum
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////
	// RACH Allocation
	uint16_t rbAllocatedNum = 0;
	std::vector <bool> ulRbMap;
	ulRbMap.resize (m_cschedCellConfig.m_ulBandwidth, false);
	ulRbMap = m_ffrSapProvider->GetAvailableUlRbg ();
	uint8_t maxContinuousUlBandwidth = 0;
	uint8_t tmpMinBandwidth = 0;
	uint16_t ffrRbStartOffset = 0;
	uint16_t tmpFfrRbStartOffset = 0;
	uint16_t index = 0;

	for (std::vector<bool>::iterator it = ulRbMap.begin (); it != ulRbMap.end (); it++)
	{
		if ((*it) == true )
		{
			rbAllocatedNum++;
			if (tmpMinBandwidth > maxContinuousUlBandwidth)
			{
				maxContinuousUlBandwidth = tmpMinBandwidth;
				ffrRbStartOffset = tmpFfrRbStartOffset;
			}
			tmpMinBandwidth = 0;
		}
		else
		{
			if (tmpMinBandwidth == 0)
			{
				tmpFfrRbStartOffset = index;
			}
			tmpMinBandwidth++;
		}
		index++;
	}

	if (tmpMinBandwidth > maxContinuousUlBandwidth)
	{
		maxContinuousUlBandwidth = tmpMinBandwidth;
		ffrRbStartOffset = tmpFfrRbStartOffset;
	}

	m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
	uint16_t rbStart = 0;
	rbStart = ffrRbStartOffset;
	std::vector <struct RachListElement_s>::iterator itRach;
	for (itRach = m_rachList.begin (); itRach != m_rachList.end (); itRach++)
	{
		NS_ASSERT_MSG (m_amc->GetUlTbSizeFromMcs (m_ulGrantMcs, m_cschedCellConfig.m_ulBandwidth) > (*itRach).m_estimatedSize, " Default UL Grant MCS does not allow to send RACH messages");
		BuildRarListElement_s newRar;
		newRar.m_rnti = (*itRach).m_rnti;
		/*
		 * Allocate new ueInfo
		 */
		// create the new entry
		cfgUe(newRar.m_rnti);
		// DL-RACH Allocation
		// Ideal: no needs of configuring m_dci
		// UL-RACH Allocation
		newRar.m_grant.m_rnti = newRar.m_rnti;
		newRar.m_grant.m_mcs = m_ulGrantMcs;
		uint16_t rbLen = 1;
		uint16_t tbSizeBits = 0;
		// find lowest TB size that fits UL grant estimated size
		while ((tbSizeBits < (*itRach).m_estimatedSize) && (rbStart + rbLen < (ffrRbStartOffset + maxContinuousUlBandwidth)))
		{
			rbLen++;
			tbSizeBits = m_amc->GetUlTbSizeFromMcs (m_ulGrantMcs, rbLen);
		}
		if (tbSizeBits < (*itRach).m_estimatedSize)
		{
			// no more allocation space: finish allocation
			break;
		}
		newRar.m_grant.m_rbStart = rbStart;
		newRar.m_grant.m_rbLen = rbLen;
		newRar.m_grant.m_tbSize = tbSizeBits / 8;
		newRar.m_grant.m_hopping = false;
		newRar.m_grant.m_tpc = 0;
		newRar.m_grant.m_cqiRequest = false;
		newRar.m_grant.m_ulDelay = false;
		NS_LOG_INFO (this << " UL grant allocated to RNTI " << (*itRach).m_rnti << " rbStart " << rbStart << " rbLen " << rbLen << " MCS " << m_ulGrantMcs << " tbSize " << newRar.m_grant.m_tbSize);
		for (uint16_t i = rbStart; i < rbStart + rbLen; i++)
		{
			m_rachAllocationMap.at (i) = (*itRach).m_rnti;
		}

		if (m_harqOn == true)
		{
			// generate UL-DCI for HARQ retransmissions
			UlDciListElement_s uldci;
			uldci.m_rnti = newRar.m_rnti;
			uldci.m_rbLen = rbLen;
			uldci.m_rbStart = rbStart;
			uldci.m_mcs = m_ulGrantMcs;
			uldci.m_tbSize = tbSizeBits / 8;
			uldci.m_ndi = 1;
			uldci.m_cceIndex = 0;
			uldci.m_aggrLevel = 1;
			uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
			uldci.m_hopping = false;
			uldci.m_n2Dmrs = 0;
			uldci.m_tpc = 0; // no power control
			uldci.m_cqiRequest = false; // only period CQI at this stage
			uldci.m_ulIndex = 0; // TDD parameter
			uldci.m_dai = 1; // TDD parameter
			uldci.m_freqHopping = 0;
			uldci.m_pdcchPowerOffset = 0; // not used

			uint8_t harqId = 0;
			std::map <uint16_t, uint8_t>::iterator itProcId;
			itProcId = m_ulHarqCurrentProcessId.find (uldci.m_rnti);
			if (itProcId == m_ulHarqCurrentProcessId.end ())
			{
				NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << uldci.m_rnti);
			}
			harqId = (*itProcId).second;
			std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci = m_ulHarqProcessesDciBuffer.find (uldci.m_rnti);
			if (itDci == m_ulHarqProcessesDciBuffer.end ())
			{
				NS_FATAL_ERROR ("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI " << uldci.m_rnti);
			}
			(*itDci).second.at (harqId) = uldci;
		}

		rbStart = rbStart + rbLen;
		ret.m_buildRarList.push_back (newRar);
	}
	m_rachList.clear ();


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step2 - After RACH, noList " << m_rachList.size()
				<< " rbgAllocatedNum " << rbgAllocatedNum
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////
	// Process DL HARQ feedback
	RefreshHarqProcesses ();
	// retrieve past HARQ retx buffered
	if (m_dlInfoListBuffered.size () > 0)
	{
		if (params.m_dlInfoList.size () > 0)
		{
			NS_LOG_INFO (this << " Received DL-HARQ feedback");
			m_dlInfoListBuffered.insert (m_dlInfoListBuffered.end (), params.m_dlInfoList.begin (), params.m_dlInfoList.end ());
		}
	}
	else
	{
		if (params.m_dlInfoList.size () > 0)
		{
			m_dlInfoListBuffered = params.m_dlInfoList;
		}
	}
	if (m_harqOn == false)
	{
		// Ignore HARQ feedback
		m_dlInfoListBuffered.clear ();
	}
	std::vector <struct DlInfoListElement_s> dlInfoListUntxed;
	for (uint16_t i = 0; i < m_dlInfoListBuffered.size (); i++)
	{
		std::set <uint16_t>::iterator itRnti = rntiAllocated.find (m_dlInfoListBuffered.at (i).m_rnti);
		if (itRnti != rntiAllocated.end ())
		{
			// RNTI already allocated for retx
			continue;
		}
		uint8_t nLayers = m_dlInfoListBuffered.at (i).m_harqStatus.size ();
		std::vector <bool> retx;
		NS_LOG_INFO (this << " Processing DLHARQ feedback");
		if (nLayers == 1)
		{
			retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (0) == DlInfoListElement_s::NACK);
			retx.push_back (false);
		}
		else
		{
			retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (0) == DlInfoListElement_s::NACK);
			retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (1) == DlInfoListElement_s::NACK);
		}
		if (retx.at (0) || retx.at (1))
		{
			// retrieve HARQ process information
			uint16_t rnti = m_dlInfoListBuffered.at (i).m_rnti;
			uint8_t harqId = m_dlInfoListBuffered.at (i).m_harqProcessId;
			NS_LOG_INFO (this << " HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId);
			std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itHarq = m_dlHarqProcessesDciBuffer.find (rnti);
			if (itHarq == m_dlHarqProcessesDciBuffer.end ())
			{
				NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << rnti);
			}

			DlDciListElement_s dci = (*itHarq).second.at (harqId);
			int rv = 0;
			if (dci.m_rv.size () == 1)
			{
				rv = dci.m_rv.at (0);
			}
			else
			{
				rv = (dci.m_rv.at (0) > dci.m_rv.at (1) ? dci.m_rv.at (0) : dci.m_rv.at (1));
			}

			if (rv == 3)
			{
				// maximum number of retx reached -> drop process
				NS_LOG_INFO ("Maximum number of retransmissions reached -> drop process");
				std::map <uint16_t, DlHarqProcessesStatus_t>::iterator it = m_dlHarqProcessesStatus.find (rnti);
				if (it == m_dlHarqProcessesStatus.end ())
				{
					NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << m_dlInfoListBuffered.at (i).m_rnti);
				}
				(*it).second.at (harqId) = 0;
				std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
				if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
				{
					NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << m_dlInfoListBuffered.at (i).m_rnti);
				}
				for (uint16_t k = 0; k < (*itRlcPdu).second.size (); k++)
				{
					(*itRlcPdu).second.at (k).at (harqId).clear ();
				}
				continue;
			}
			// check the feasibility of retransmitting on the same RBGs
			// translate the DCI to Spectrum framework
			std::vector <int> dciRbg;
			uint32_t mask = 0x1;
			NS_LOG_INFO ("Original RBGs " << dci.m_rbBitmap << " rnti " << dci.m_rnti);
			for (int j = 0; j < 32; j++)
			{
				if (((dci.m_rbBitmap & mask) >> j) == 1)
				{
					dciRbg.push_back (j);
					NS_LOG_INFO ("\t" << j);
				}
				mask = (mask << 1);
			}
			bool free = true;
			for (uint8_t j = 0; j < dciRbg.size (); j++)
			{
				if (rbgMap.at (dciRbg.at (j)) == true)
				{
					free = false;
					break;
				}
			}
			if (free)
			{
				// use the same RBGs for the retx
				// reserve RBGs
				for (uint8_t j = 0; j < dciRbg.size (); j++)
				{
					rbgMap.at (dciRbg.at (j)) = true;
					NS_LOG_INFO ("RBG " << dciRbg.at (j) << " assigned");
					rbgAllocatedNum++;
				}

				NS_LOG_INFO (this << " Send retx in the same RBGs");
			}
			else
			{
				// find RBGs for sending HARQ retx
				uint8_t j = 0;
				uint8_t rbgId = (dciRbg.at (dciRbg.size () - 1) + 1) % rbgNum;
				uint8_t startRbg = dciRbg.at (dciRbg.size () - 1);
				std::vector <bool> rbgMapCopy = rbgMap;
				while ((j < dciRbg.size ())&&(startRbg != rbgId))
				{
					if (rbgMapCopy.at (rbgId) == false)
					{
						rbgMapCopy.at (rbgId) = true;
						dciRbg.at (j) = rbgId;
						j++;
					}
					rbgId = (rbgId + 1) % rbgNum;
				}
				if (j == dciRbg.size ())
				{
					// find new RBGs -> update DCI map
					uint32_t rbgMask = 0;
					for (uint16_t k = 0; k < dciRbg.size (); k++)
					{
						rbgMask = rbgMask + (0x1 << dciRbg.at (k));
						rbgAllocatedNum++;
					}
					dci.m_rbBitmap = rbgMask;
					rbgMap = rbgMapCopy;
					NS_LOG_INFO (this << " Move retx in RBGs " << dciRbg.size ());
				}
				else
				{
					// HARQ retx cannot be performed on this TTI -> store it
					dlInfoListUntxed.push_back (m_dlInfoListBuffered.at (i));
					NS_LOG_INFO (this << " No resource for this retx -> buffer it");
				}
			}
			// retrieve RLC PDU list for retx TBsize and update DCI
			BuildDataListElement_s newEl;
			std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
			if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
			{
				NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << rnti);
			}
			for (uint8_t j = 0; j < nLayers; j++)
			{
				if (retx.at (j))
				{
					if (j >= dci.m_ndi.size ())
					{
						// for avoiding errors in MIMO transient phases
						dci.m_ndi.push_back (0);
						dci.m_rv.push_back (0);
						dci.m_mcs.push_back (0);
						dci.m_tbsSize.push_back (0);
						NS_LOG_INFO (this << " layer " << (uint16_t)j << " no txed (MIMO transition)");
					}
					else
					{
						dci.m_ndi.at (j) = 0;
						dci.m_rv.at (j)++;
						(*itHarq).second.at (harqId).m_rv.at (j)++;
						NS_LOG_INFO (this << " layer " << (uint16_t)j << " RV " << (uint16_t)dci.m_rv.at (j));
					}
				}
				else
				{
					// empty TB of layer j
					dci.m_ndi.at (j) = 0;
					dci.m_rv.at (j) = 0;
					dci.m_mcs.at (j) = 0;
					dci.m_tbsSize.at (j) = 0;
					NS_LOG_INFO (this << " layer " << (uint16_t)j << " no retx");
				}
			}
			for (uint16_t k = 0; k < (*itRlcPdu).second.at (0).at (dci.m_harqProcess).size (); k++)
			{
				std::vector <struct RlcPduListElement_s> rlcPduListPerLc;
				for (uint8_t j = 0; j < nLayers; j++)
				{
					if (retx.at (j))
					{
						if (j < dci.m_ndi.size ())
						{
							NS_LOG_INFO (" layer " << (uint16_t)j << " tb size " << dci.m_tbsSize.at (j));
							rlcPduListPerLc.push_back ((*itRlcPdu).second.at (j).at (dci.m_harqProcess).at (k));
						}
					}
					else
					{ // if no retx needed on layer j, push an RlcPduListElement_s object with m_size=0 to keep the size of rlcPduListPerLc vector = 2 in case of MIMO
						NS_LOG_INFO (" layer " << (uint16_t)j << " tb size "<<dci.m_tbsSize.at (j));
						RlcPduListElement_s emptyElement;
						emptyElement.m_logicalChannelIdentity = (*itRlcPdu).second.at (j).at (dci.m_harqProcess).at (k).m_logicalChannelIdentity;
						emptyElement.m_size = 0;
						rlcPduListPerLc.push_back (emptyElement);
					}
				}

				if (rlcPduListPerLc.size () > 0)
				{
					newEl.m_rlcPduList.push_back (rlcPduListPerLc);
				}
			}
			newEl.m_rnti = rnti;
			newEl.m_dci = dci;
			(*itHarq).second.at (harqId).m_rv = dci.m_rv;
			// refresh timer
			std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer = m_dlHarqProcessesTimer.find (rnti);
			if (itHarqTimer== m_dlHarqProcessesTimer.end ())
			{
				NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)rnti);
			}
			(*itHarqTimer).second.at (harqId) = 0;
			ret.m_buildDataList.push_back (newEl);
			rntiAllocated.insert (rnti);
		}
		else
		{
			// update HARQ process status
			NS_LOG_INFO (this << " HARQ received ACK for UE " << m_dlInfoListBuffered.at (i).m_rnti);
			std::map <uint16_t, DlHarqProcessesStatus_t>::iterator it = m_dlHarqProcessesStatus.find (m_dlInfoListBuffered.at (i).m_rnti);
			if (it == m_dlHarqProcessesStatus.end ())
			{
				NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << m_dlInfoListBuffered.at (i).m_rnti);
			}
			(*it).second.at (m_dlInfoListBuffered.at (i).m_harqProcessId) = 0;
			std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (m_dlInfoListBuffered.at (i).m_rnti);
			if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
			{
				NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << m_dlInfoListBuffered.at (i).m_rnti);
			}
			for (uint16_t k = 0; k < (*itRlcPdu).second.size (); k++)
			{
				(*itRlcPdu).second.at (k).at (m_dlInfoListBuffered.at (i).m_harqProcessId).clear ();
			}
		}
	}
	m_dlInfoListBuffered.clear ();
	m_dlInfoListBuffered = dlInfoListUntxed;


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step3 - After HARQ "
				<< " rbgAllocatedNum " << rbgAllocatedNum
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////

	if (rbgAllocatedNum == rbgNum)
	{
		// all the RBGs are already allocated -> exit
		if ((ret.m_buildDataList.size () > 0) || (ret.m_buildRarList.size () > 0))
		{
			m_schedSapUser->SchedDlConfigInd (ret);
		}
		return;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////
	// RBG Allocation
	for (int i = 0; i < rbgNum; i++)
	{
		NS_LOG_INFO (this << " ALLOCATION for RBG " << i << " of " << rbgNum);
		if (rbgMap.at (i) == false)
		{
			std::map <uint16_t, pw_pfsFlowPerf_t>::iterator it;
			std::map <uint16_t, pw_pfsFlowPerf_t>::iterator itMax = m_flowStatsDl.end ();
			double rcqiMax = 0.0;
			for (it = m_flowStatsDl.begin (); it != m_flowStatsDl.end (); it++)
			{
				if ((m_ffrSapProvider->IsDlRbgAvailableForUe (i, (*it).first)) == false)
					continue;

				std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).first);
				if ((itRnti != rntiAllocated.end ())||(!HarqProcessAvailability ((*it).first)))
				{
					// UE already allocated for HARQ or without HARQ process available -> drop it
					if (itRnti != rntiAllocated.end ())
					{
						NS_LOG_DEBUG (this << " RNTI discared for HARQ tx" << (uint16_t)(*it).first);
					}
					if (!HarqProcessAvailability ((*it).first))
					{
						NS_LOG_DEBUG (this << " RNTI discared for HARQ id" << (uint16_t)(*it).first);
					}
					continue;
				}
				std::map <uint16_t,SbMeasResult_s>::iterator itCqi;
				itCqi = m_a30CqiRxed.find ((*it).first);
				std::map <uint16_t,uint8_t>::iterator itTxMode;
				itTxMode = m_uesTxMode.find ((*it).first);
				if (itTxMode == m_uesTxMode.end ())
				{
					NS_FATAL_ERROR ("No Transmission Mode info on user " << (*it).first);
				}
				int nLayer = TransmissionModesLayers::TxMode2LayerNum ((*itTxMode).second);
				std::vector <uint8_t> sbCqi;
				if (itCqi == m_a30CqiRxed.end ())
				{
					for (uint8_t k = 0; k < nLayer; k++)
					{
						sbCqi.push_back (1);  // start with lowest value
					}
				}
				else
				{
					sbCqi = (*itCqi).second.m_higherLayerSelected.at (i).m_sbCqi;
				}
				uint8_t cqi1 = sbCqi.at (0);
				uint8_t cqi2 = 0;
				if (sbCqi.size () > 1)
				{
					cqi2 = sbCqi.at (1);
				}

				if ((cqi1 > 0)||(cqi2 > 0)) // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
				{
					if (LcActivePerFlow ((*it).first) > 0)
					{
						// this UE has data to transmit
						double achievableRate = 0.0;
						uint8_t mcs = 0;
						for (uint8_t k = 0; k < nLayer; k++)
						{
							if (sbCqi.size () > k)
							{
								mcs = m_amc->GetMcsFromCqi (sbCqi.at (k));
							}
							else
							{
								// no info on this subband -> worst MCS
								mcs = 0;
							}
							achievableRate += ((m_amc->GetDlTbSizeFromMcs (mcs, rbgSize) / 8) / 0.001);   // = TB size / TTI
						}

						double rcqi = achievableRate / (*it).second.lastAveragedThroughput;
						NS_LOG_INFO (this << " RNTI " << (*it).first << " MCS " << (uint32_t)mcs << " achievableRate " << achievableRate << " avgThr " << (*it).second.lastAveragedThroughput << " RCQI " << rcqi);


						// PW_EDIT
						if ((Now().GetSeconds() >= 2.368) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
							std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
									<< " Step41 - RBG Allocation "
									<< " RNTI " << (*it).first
									<< " MCS " << (uint32_t)mcs
									<< " achievableRate " << achievableRate << " Bytes/Sec"
									<< " avgThr " << (*it).second.lastAveragedThroughput
									<< " RCQI(achievableRate/lastAverageThroughput) " << rcqi
									<< std::endl;
							std::cout << "Break" << std::endl;
						}


						if (rcqi > rcqiMax)
						{
							rcqiMax = rcqi;
							itMax = it;
						}
					}
				}   // end if cqi
			} // end for m_rlcBufferReq

			if (itMax == m_flowStatsDl.end ())
			{
				// no UE available for this RB
				NS_LOG_INFO (this << " any UE found");
			}
			else
			{
				rbgMap.at (i) = true;
				std::map <uint16_t, std::vector <uint16_t> >::iterator itMap;
				itMap = allocationMap.find ((*itMax).first);
				if (itMap == allocationMap.end ())
				{
					// insert new element
					std::vector <uint16_t> tempMap;
					tempMap.push_back (i);
					allocationMap.insert (std::pair <uint16_t, std::vector <uint16_t> > ((*itMax).first, tempMap));
				}
				else
				{
					(*itMap).second.push_back (i);
				}
				NS_LOG_INFO (this << " UE assigned " << (*itMax).first);


				// PW_EDIT
				if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
					std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
							<< " Step42 - RBG no. " << i << "of[0-" << rbgNum-1 << "]"
							<< " Select Allocation for RNTI " << (*itMax).first
							<< " with the HIGHEST RCQI(achievableRate/lastAverageThroughput)"
							<< std::endl;
					std::cout << "Break" << std::endl;
				}


			}
		} // end for RBG free
	} // end for RBGs

	// reset TTI stats of users
	std::map <uint16_t, pw_pfsFlowPerf_t>::iterator itStats;
	for (itStats = m_flowStatsDl.begin (); itStats != m_flowStatsDl.end (); itStats++)
	{
		(*itStats).second.lastTtiBytesTrasmitted = 0;
	}


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step5 - After RBG Allocation "
				<< " rbgAllocatedNum " << rbgAllocatedNum
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////
	// generate the transmission opportunities by grouping the RBGs of the same RNTI and
	// creating the correspondent DCIs
	std::map <uint16_t, std::vector <uint16_t> >::iterator itMap = allocationMap.begin ();
	while (itMap != allocationMap.end ())
	{
		// create new BuildDataListElement_s for this LC
		BuildDataListElement_s newEl;
		newEl.m_rnti = (*itMap).first;
		// create the DlDciListElement_s
		DlDciListElement_s newDci;
		newDci.m_rnti = (*itMap).first;
		newDci.m_harqProcess = UpdateHarqProcessId ((*itMap).first);

		uint16_t lcActives = LcActivePerFlow ((*itMap).first);
		NS_LOG_INFO (this << "Allocate user " << newEl.m_rnti << " rbg " << lcActives);
		if (lcActives == 0)
		{
			// Set to max value, to avoid divide by 0 below
			lcActives = (uint16_t)65535; // UINT16_MAX;
		}
		uint16_t RgbPerRnti = (*itMap).second.size ();
		std::map <uint16_t,SbMeasResult_s>::iterator itCqi;
		itCqi = m_a30CqiRxed.find ((*itMap).first);
		std::map <uint16_t,uint8_t>::iterator itTxMode;
		itTxMode = m_uesTxMode.find ((*itMap).first);
		if (itTxMode == m_uesTxMode.end ())
		{
			NS_FATAL_ERROR ("No Transmission Mode info on user " << (*itMap).first);
		}
		int nLayer = TransmissionModesLayers::TxMode2LayerNum ((*itTxMode).second);
		std::vector <uint8_t> worstCqi (2, 15);
		if (itCqi != m_a30CqiRxed.end ())
		{
			for (uint16_t k = 0; k < (*itMap).second.size (); k++)
			{
				if ((*itCqi).second.m_higherLayerSelected.size () > (*itMap).second.at (k))
				{
					NS_LOG_INFO (this << " RBG " << (*itMap).second.at (k) << " CQI " << (uint16_t)((*itCqi).second.m_higherLayerSelected.at ((*itMap).second.at (k)).m_sbCqi.at (0)) );
					for (uint8_t j = 0; j < nLayer; j++)
					{
						if ((*itCqi).second.m_higherLayerSelected.at ((*itMap).second.at (k)).m_sbCqi.size () > j)
						{
							if (((*itCqi).second.m_higherLayerSelected.at ((*itMap).second.at (k)).m_sbCqi.at (j)) < worstCqi.at (j))
							{
								worstCqi.at (j) = ((*itCqi).second.m_higherLayerSelected.at ((*itMap).second.at (k)).m_sbCqi.at (j));
							}
						}
						else
						{
							// no CQI for this layer of this suband -> worst one
							worstCqi.at (j) = 1;
						}
					}
				}
				else
				{
					for (uint8_t j = 0; j < nLayer; j++)
					{
						worstCqi.at (j) = 1; // try with lowest MCS in RBG with no info on channel
					}
				}
			}
		}
		else
		{
			for (uint8_t j = 0; j < nLayer; j++)
			{
				worstCqi.at (j) = 1; // try with lowest MCS in RBG with no info on channel
			}
		}
		for (uint8_t j = 0; j < nLayer; j++)
		{
			NS_LOG_INFO (this << " Layer " << (uint16_t)j << " CQI selected " << (uint16_t)worstCqi.at (j));
		}
		uint32_t bytesTxed = 0;
		for (uint8_t j = 0; j < nLayer; j++)
		{
			newDci.m_mcs.push_back (m_amc->GetMcsFromCqi (worstCqi.at (j)));
			int tbSize = (m_amc->GetDlTbSizeFromMcs (newDci.m_mcs.at (j), RgbPerRnti * rbgSize) / 8); // (size of TB in bytes according to table 7.1.7.2.1-1 of 36.213)
			newDci.m_tbsSize.push_back (tbSize);
			NS_LOG_INFO (this << " Layer " << (uint16_t)j << " MCS selected" << m_amc->GetMcsFromCqi (worstCqi.at (j)));
			bytesTxed += tbSize;


			// PW_EDIT
			if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
				std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
						<< " Step61 - Create DCI "
						<< " Layer" << (uint16_t)j
						<< " CQI selected " << (uint16_t)worstCqi.at (j)
						<< " MCS selected" << m_amc->GetMcsFromCqi (worstCqi.at (j))
						<< " tbSize " << tbSize
						<< " bytesTxed " << bytesTxed
						<< std::endl;
				std::cout << "Break" << std::endl;
			}


		}


		newDci.m_resAlloc = 0;  // only allocation type 0 at this stage
		newDci.m_rbBitmap = 0; // TBD (32 bit bitmap see 7.1.6 of 36.213)
		uint32_t rbgMask = 0;
		for (uint16_t k = 0; k < (*itMap).second.size (); k++)
		{
			rbgMask = rbgMask + (0x1 << (*itMap).second.at (k));
			NS_LOG_INFO (this << " Allocated RBG " << (*itMap).second.at (k));
		}
		newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)


		// PW_EDIT
		if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
			std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
					<< " *Step65 - Allocate RBG to UE in DCI* "
					<< " RNTI " << newDci.m_rnti
					<< " AllocationBitMap(convert to binary) " << newDci.m_rbBitmap
					<< std::endl;
			std::cout << "Break" << std::endl;
		}


		// create the rlc PDUs -> equally divide resources among actives LCs
		std::map <LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator itBufReq;
		for (itBufReq = m_rlcBufferReq.begin (); itBufReq != m_rlcBufferReq.end (); itBufReq++)
		{
			if (((*itBufReq).first.m_rnti == (*itMap).first)
					&& (((*itBufReq).second.m_rlcTransmissionQueueSize > 0)
							|| ((*itBufReq).second.m_rlcRetransmissionQueueSize > 0)
							|| ((*itBufReq).second.m_rlcStatusPduSize > 0) ))
			{
				std::vector <struct RlcPduListElement_s> newRlcPduLe;
				for (uint8_t j = 0; j < nLayer; j++)
				{
					RlcPduListElement_s newRlcEl;
					newRlcEl.m_logicalChannelIdentity = (*itBufReq).first.m_lcId;
					newRlcEl.m_size = newDci.m_tbsSize.at (j) / lcActives;
					NS_LOG_INFO (this << " LCID " << (uint32_t) newRlcEl.m_logicalChannelIdentity << " size " << newRlcEl.m_size << " layer " << (uint16_t)j);
					newRlcPduLe.push_back (newRlcEl);
					UpdateDlRlcBufferInfo (newDci.m_rnti, newRlcEl.m_logicalChannelIdentity, newRlcEl.m_size);
					if (m_harqOn == true)
					{
						// store RLC PDU list for HARQ
						std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find ((*itMap).first);
						if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
						{
							NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << (*itMap).first);
						}
						(*itRlcPdu).second.at (j).at (newDci.m_harqProcess).push_back (newRlcEl);
					}
				}
				newEl.m_rlcPduList.push_back (newRlcPduLe);
			}
			if ((*itBufReq).first.m_rnti > (*itMap).first)
			{
				break;
			}
		}
		for (uint8_t j = 0; j < nLayer; j++)
		{
			newDci.m_ndi.push_back (1);
			newDci.m_rv.push_back (0);
		}

		std::map <uint16_t,ueInfo>::iterator ue;
		ue = ueUl.find ((*itMap).first);


		/*
		 * sec before sending TPC to UE reduce the delta power UE gonna fix
		 * and reduce it from the total Pusch power the UE need to fix.
		 */
		if(eff_state != UE_EFF)
		{
			int8_t tempDelta;
			ue->second.remPuschPwr -= ue->second.delta;
			tempDelta = ue->second.fi + ue->second.delta;
			CHECK_AND_UPDATE_POWER_CORRECTION_FOR_PUSCH(tempDelta);
			if(tempDelta != ue->second.fi)
			{
				ue->second.fi = tempDelta;
			}
		}

		if(ue->second.isPhrAvail)
		{
			//update phr which used for min ue pwr and setting TPC
			ue->second.phr -= ue->second.delta;
			//3GPP phr lower bound
			ue->second.phr = ue->second.phr < -23 ? -23 : ue->second.phr;
		}

		if(eff_state == UE_EFF)
		{
			//			ue->second.avg_sinr += ue->second.delta*200;
		}

		if(ue->second.tpc != 1)
		{
			ue->second.lastTpcAppliedTick = tti_counter;
		}


		newDci.m_tpc = ue->second.tpc;

		newEl.m_dci = newDci;

		if (m_harqOn == true)
		{
			// store DCI for HARQ
			std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci = m_dlHarqProcessesDciBuffer.find (newEl.m_rnti);
			if (itDci == m_dlHarqProcessesDciBuffer.end ())
			{
				NS_FATAL_ERROR ("Unable to find RNTI entry in DCI HARQ buffer for RNTI " << newEl.m_rnti);
			}
			(*itDci).second.at (newDci.m_harqProcess) = newDci;
			// refresh timer
			std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =  m_dlHarqProcessesTimer.find (newEl.m_rnti);
			if (itHarqTimer== m_dlHarqProcessesTimer.end ())
			{
				NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
			}
			(*itHarqTimer).second.at (newDci.m_harqProcess) = 0;
		}

		// ...more parameters -> ignored in this version

		ret.m_buildDataList.push_back (newEl);
		// update UE stats
		std::map <uint16_t, pw_pfsFlowPerf_t>::iterator it;
		it = m_flowStatsDl.find ((*itMap).first);
		if (it != m_flowStatsDl.end ())
		{
			(*it).second.lastTtiBytesTrasmitted = bytesTxed;
			NS_LOG_INFO (this << " UE total bytes txed " << (*it).second.lastTtiBytesTrasmitted);


		}
		else
		{
			NS_FATAL_ERROR (this << " No Stats for this allocated UE");
		}

		itMap++;
	} // end while allocation
	ret.m_nrOfPdcchOfdmSymbols = 1;   /// \todo check correct value according the DCIs txed


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step7 - After RBG Allocation "
				<< " rbgAllocatedNum " << rbgAllocatedNum
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////
	// update UEs stats
	NS_LOG_INFO (this << " Update UEs statistics");
	for (itStats = m_flowStatsDl.begin (); itStats != m_flowStatsDl.end (); itStats++)
	{
		(*itStats).second.totalBytesTransmitted += (*itStats).second.lastTtiBytesTrasmitted;
		// update average throughput (see eq. 12.3 of Sec 12.3.1.2 of LTE – The UMTS Long Term Evolution, Ed Wiley)
		(*itStats).second.lastAveragedThroughput = ((1.0 - (1.0 / m_timeWindow)) * (*itStats).second.lastAveragedThroughput) + ((1.0 / m_timeWindow) * (double)((*itStats).second.lastTtiBytesTrasmitted / 0.001));
		NS_LOG_INFO (this << " UE total bytes " << (*itStats).second.totalBytesTransmitted);
		NS_LOG_INFO (this << " UE average throughput " << (*itStats).second.lastAveragedThroughput);
		(*itStats).second.lastTtiBytesTrasmitted = 0;


		// PW_EDIT
		if ((Now().GetSeconds() >= 2.011) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
			std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
					<< " Step8 - UE statistics"
					<< " UE " << (*itStats).first
					<< " totalBytes " << (*itStats).second.totalBytesTransmitted
					<< " avgThroughput " << (*itStats).second.lastAveragedThroughput
					<< std::endl;
			std::cout << "Break" << std::endl;
		}

	}

	m_schedSapUser->SchedDlConfigInd (ret);


	// PW_EDIT
	if ((Now().GetSeconds() >= 2.368) && PWCONFIG_WKINGLOG_DL_SCH_PW) {
		std::cout << "DEBUG_DLSCH " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedDlTriggerReq"
				<< " Step9 - Finish DL schedule"
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	return;
}

void
pw_PfFfMacScheduler::DoSchedDlRachInfoReq (const struct FfMacSchedSapProvider::SchedDlRachInfoReqParameters& params)
{
	NS_LOG_FUNCTION (this);

	m_rachList = params.m_rachList;

	return;
}

void
pw_PfFfMacScheduler::DoSchedDlCqiInfoReq (const struct FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	m_ffrSapProvider->ReportDlCqiInfo (params);

	for (unsigned int i = 0; i < params.m_cqiList.size (); i++)
	{
		if ( params.m_cqiList.at (i).m_cqiType == CqiListElement_s::P10 )
		{
			NS_LOG_LOGIC ("wideband CQI " <<  (uint32_t) params.m_cqiList.at (i).m_wbCqi.at (0) << " reported");
			std::map <uint16_t,uint8_t>::iterator it;
			uint16_t rnti = params.m_cqiList.at (i).m_rnti;
			it = m_p10CqiRxed.find (rnti);
			if (it == m_p10CqiRxed.end ())
			{
				// create the new entry
				m_p10CqiRxed.insert ( std::pair<uint16_t, uint8_t > (rnti, params.m_cqiList.at (i).m_wbCqi.at (0)) ); // only codeword 0 at this stage (SISO)
				// generate correspondent timer
				m_p10CqiTimers.insert ( std::pair<uint16_t, uint32_t > (rnti, m_cqiTimersThreshold));
			}
			else
			{
				// update the CQI value and refresh correspondent timer
				(*it).second = params.m_cqiList.at (i).m_wbCqi.at (0);
				// update correspondent timer
				std::map <uint16_t,uint32_t>::iterator itTimers;
				itTimers = m_p10CqiTimers.find (rnti);
				(*itTimers).second = m_cqiTimersThreshold;
			}
		}
		else if ( params.m_cqiList.at (i).m_cqiType == CqiListElement_s::A30 )
		{
			// subband CQI reporting high layer configured
			std::map <uint16_t,SbMeasResult_s>::iterator it;
			uint16_t rnti = params.m_cqiList.at (i).m_rnti;
			it = m_a30CqiRxed.find (rnti);
			if (it == m_a30CqiRxed.end ())
			{
				// create the new entry
				m_a30CqiRxed.insert ( std::pair<uint16_t, SbMeasResult_s > (rnti, params.m_cqiList.at (i).m_sbMeasResult) );
				m_a30CqiTimers.insert ( std::pair<uint16_t, uint32_t > (rnti, m_cqiTimersThreshold));
			}
			else
			{
				// update the CQI value and refresh correspondent timer
				(*it).second = params.m_cqiList.at (i).m_sbMeasResult;
				std::map <uint16_t,uint32_t>::iterator itTimers;
				itTimers = m_a30CqiTimers.find (rnti);
				(*itTimers).second = m_cqiTimersThreshold;
			}
		}
		else
		{
			NS_LOG_ERROR (this << " CQI type unknown");
		}
	}

	return;
}


double
pw_PfFfMacScheduler::EstimateUlSinr (uint16_t rnti, uint16_t rb)
{
	std::map <uint16_t, std::vector <double> >::iterator itCqi = m_ueCqi.find (rnti);
	if (itCqi == m_ueCqi.end ())
	{
		// no cqi info about this UE
		return (NO_SINR);

	}
	else
	{
		// take the average SINR value among the available
		double sinrSum = 0;
		unsigned int sinrNum = 0;
		for (uint32_t i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
		{
			double sinr = (*itCqi).second.at (i);
			if (sinr != NO_SINR)
			{
				sinrSum += sinr;
				sinrNum++;
			}
		}
		double estimatedSinr = (sinrNum > 0) ? (sinrSum / sinrNum) : DBL_MAX;
		// store the value
		(*itCqi).second.at (rb) = estimatedSinr;
		return (estimatedSinr);
	}
}
void
pw_PfFfMacScheduler::HarqCrcFail(uint16_t rnti){
	std::map <uint16_t,ueInfo>::iterator it;
	it = ueUl.find (rnti);
	if(SPECTRAL_EFF == eff_state)
	{
		it->second.delta_itbs -= 30; // on fail reduce 0.3 itbs
		if(it->second.avgItbs + it->second.delta_itbs < 0){
			it->second.delta_itbs = -(it->second.avgItbs); // fix to iTbs 0
		}
	}
	else
	{
		//Link edge check
		if(!(it->second.maxRb <= MIN_MAX_RBS && it->second.mcs_index == MIN_MCS))
		{
			it->second.delta_sinr = GET_MIN(4000,it->second.delta_sinr + it->second.sinr_margin_step_up); // why 4000? 4000*(1/(100*2)) ~ 20dB, StepUp ~ 90/200 = 0.45dB
		}
	}
}

void
pw_PfFfMacScheduler::HarqCrcOk(uint16_t rnti){
	std::map <uint16_t,ueInfo>::iterator it;
	it = ueUl.find (rnti);
	if(SPECTRAL_EFF == eff_state)
	{
		uint16_t maxiTbs = m_amc->GetItbsFromCqi(it->second.maxUlCqi)*100;
		it = ueUl.find (rnti);
		it->second.delta_itbs += 3; // on success raise 0.03 itbs
		if(it->second.avgItbs + it->second.delta_itbs > maxiTbs){
			it->second.delta_itbs = maxiTbs - it->second.avgItbs; // the part needed to get maxItbs
		}
	}
	else
	{
		it->second.delta_sinr = GET_MAX(-4000,it->second.delta_sinr - it->second.sinr_margin_step_down); // why 4000? 4000*(1/(100*2)) ~ 20dB, StepDown ~ 10/200 = 0.05dB
	}
}

uint8_t
pw_PfFfMacScheduler::GetPwrRbToPwrdB(uint8_t RBs){
	if(RBs > m_cschedCellConfig.m_ulBandwidth)
	{
		RBs = m_cschedCellConfig.m_ulBandwidth;
	}
	return PwrRbToPwrdBTbl[RBs];
}

void
pw_PfFfMacScheduler::DoSchedUlTriggerReq (const struct FfMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{
	NS_LOG_FUNCTION (this << " UL - Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf) << " size " << params.m_ulInfoList.size ());

	if ((Now().GetSeconds() >= 2.048) && PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
				<< " UL - Frame no. " << (params.m_sfnSf >> 4)
				<< " subframe no. " << (0xF & params.m_sfnSf)
				<< " eNB " << m_pwEnbId
				<< " size " << params.m_ulInfoList.size ()
				<< " UEs/TTI " << (int)globalPfs.MaxUesPerTti
				<< std::endl;
		std::cout << "Break" << std::endl;
	}

	tti_counter++;

	RefreshUlCqiMaps ();
	m_ffrSapProvider->ReportUlCqiInfo (m_ueCqi);

	uint8_t RemUes = globalPfs.MaxUesPerTti;

	// Generate RBs map
	FfMacSchedSapUser::SchedUlConfigIndParameters ret;
	std::vector <bool> rbMap;
	uint16_t rbAllocatedNum = 0;
	std::set <uint16_t> rntiAllocated;
	std::vector <uint16_t> rbgAllocationMap;
	// update with RACH allocation map
	rbgAllocationMap = m_rachAllocationMap;
	//rbgAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
	m_rachAllocationMap.clear ();
	m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);

	rbMap.resize (m_cschedCellConfig.m_ulBandwidth, false);
	rbMap = m_ffrSapProvider->GetAvailableUlRbg ();

	for (std::vector<bool>::iterator it = rbMap.begin (); it != rbMap.end (); it++)
	{
		if ((*it) == true )
		{
			rbAllocatedNum++;
		}
	}

	//uint8_t minContinuousUlBandwidth = m_ffrSapProvider->GetMinContinuousUlBandwidth ();
	uint8_t ffrUlBandwidth = m_cschedCellConfig.m_ulBandwidth - rbAllocatedNum;

	//Get Number of available subands in cell
	globalPfs.SubBandAvailable = utils::SCH_CEIL(ffrUlBandwidth,SbsSize)-1;


	// remove RACH allocation
	for (uint16_t i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
	{
		if (rbgAllocationMap.at (i) != 0)
		{
			rbMap.at (i) = true;
			NS_LOG_DEBUG (this << " Allocated for RACH " << i);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Process UL HARQ
	if (m_harqOn == true)
	{

		for (uint16_t i = 0; i < params.m_ulInfoList.size () && RemUes; i++)
		{
			uint16_t rnti = params.m_ulInfoList.at (i).m_rnti;
			if (params.m_ulInfoList.at (i).m_receptionStatus == UlInfoListElement_s::NotOk)
			{
				// retx correspondent block: retrieve the UL-DCI

				std::map <uint16_t,ueInfo>::iterator it;
				it = ueUl.find (rnti);
				if (!it->second.isTTIBundling)
				{
					this->HarqCrcFail(rnti);
				}
				miss[rnti]++;
				std::map <uint16_t, uint8_t>::iterator itProcId = m_ulHarqCurrentProcessId.find (rnti);
				if (itProcId == m_ulHarqCurrentProcessId.end ())
				{
					NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << rnti);
				}
				uint8_t harqId = (uint8_t)((*itProcId).second - HARQ_PERIOD) % HARQ_PROC_NUM;
				NS_LOG_INFO (this << " UL-HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId << " i " << i << " size "  << params.m_ulInfoList.size ());

				if ((Now().GetSeconds() >= 0.000) && PWCONFIG_WKINGLOG_UL_SCH_PW) {
					std::cout << "PW_SCHEDULER " << Now().GetSeconds()
															<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq UL-HARQ retx RNTI "
															<< rnti << " harqId " << (uint16_t)harqId
															<< " i " << i << " size "  << params.m_ulInfoList.size ()
															<< std::endl;
					std::cout << "Break" << std::endl;
				}

				std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itHarq = m_ulHarqProcessesDciBuffer.find (rnti);
				if (itHarq == m_ulHarqProcessesDciBuffer.end ())
				{
					NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << rnti);
					continue;
				}
				UlDciListElement_s dci = (*itHarq).second.at (harqId);
				std::map <uint16_t, UlHarqProcessesStatus_t>::iterator itStat = m_ulHarqProcessesStatus.find (rnti);
				if (itStat == m_ulHarqProcessesStatus.end ())
				{
					NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << rnti);
				}
				if ((*itStat).second.at (harqId) >= 3)
				{
					NS_LOG_INFO ("Max number of retransmissions reached (UL)-> drop process");
					if (it->second.isTTIBundling)
					{
						this->HarqCrcFail(rnti);
					}
					if ((Now().GetSeconds() >= 0.000)	&& PWCONFIG_WKINGLOG_UL_SCH_PW) {
						std::cout << "PW_SCHEDULER " << Now().GetSeconds()
																<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq RNTI " << rnti
																<< "Max number of retransmissions reached (UL)-> drop process"
																<< std::endl;
						std::cout << "Break" << std::endl;
					}

					continue;
				}
				bool free = true;
				for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
				{
					if (rbMap.at (j) == true)
					{
						free = false;
						NS_LOG_INFO (this << " BUSY " << j);
					}
				}
				if (free)
				{
					// retx on the same RBs
					for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
					{
						rbMap.at (j) = true;
						rbgAllocationMap.at (j) = dci.m_rnti;
						NS_LOG_INFO ("\tRB " << j);
						rbAllocatedNum++;
					}
					NS_LOG_INFO (this << " Send retx in the same RBs " << (uint16_t)dci.m_rbStart << " to " << dci.m_rbStart + dci.m_rbLen << " RV " << (*itStat).second.at (harqId) + 1);

					if ((Now().GetSeconds() >= 0.000)	&& PWCONFIG_WKINGLOG_UL_SCH_PW) {
						std::cout << "PW_SCHEDULER " << Now().GetSeconds()
																<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq RNTI " << rnti
																<< " Send retx in the same RBs " << (uint16_t)dci.m_rbStart
																<< " to " << dci.m_rbStart + dci.m_rbLen
																<< " RV " << (*itStat).second.at (harqId) + 1
																<< std::endl;
						std::cout << "Break" << std::endl;
					}

				}
				else
				{
					NS_LOG_INFO ("Cannot allocate retx due to RACH allocations for UE " << rnti);

					if ((Now().GetSeconds() >= 0.000)	&& PWCONFIG_WKINGLOG_UL_SCH_PW) {
						std::cout << "PW_SCHEDULER " << Now().GetSeconds()
																<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq RNTI " << rnti
																<< "Cannot allocate retx due to RACH allocations for UE "
																<< std::endl;
						std::cout << "Break" << std::endl;
					}

					continue;
				}
				dci.m_ndi = 0;
				// Update HARQ buffers with new HarqId
				(*itStat).second.at ((*itProcId).second) = (*itStat).second.at (harqId) + 1;
				(*itStat).second.at (harqId) = 0;
				(*itHarq).second.at ((*itProcId).second) = dci;
				ret.m_dciList.push_back (dci);

				// SET SCHEDULE UE
				RemUes--;
				rntiAllocated.insert (dci.m_rnti);

				if ((Now().GetSeconds() >= 0.000)	&& PWCONFIG_WKINGLOG_UL_SCH_PW) {
					std::cout << "PW_SCHEDULER " << Now().GetSeconds()
															<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq"
															<< " HARQ_DECISION - Schedule UE HARQ"
															<< " RNTI " << rnti
															<< " Remaining UEs to be Scheduled " << (int)RemUes
															<< std::endl;
					std::cout << "Break" << std::endl;
				}



			}
			else
			{
				NS_LOG_INFO (this << " HARQ-ACK feedback from RNTI " << params.m_ulInfoList.at (i).m_rnti);
				this->HarqCrcOk(rnti);
				success[rnti]++;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	int FlowsSchedule = 0;
	FlowsSchedule = getNumFlows(rntiAllocated);
	/*

  std::map <uint16_t,uint32_t>::iterator it;
  int nflows = 0;

  for (it = m_ceBsrRxed.begin (); it != m_ceBsrRxed.end (); it++)
    {
	  //Check if UEs are already allocated by checking if the rntiAllocated structure
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).first);

      // select UEs with queues not empty and not yet allocated for HARQ
      //Check if UE's BSR not empty
      if (((*it).second > 0)&&(itRnti == rntiAllocated.end ()))
        {
          nflows++;
        }
    }
	 */

	//Check also max Ues to schedule, need to count how many ues was allocated for ul retransmissions

	if (FlowsSchedule == 0){
		if (ret.m_dciList.size () > 0){
			m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));
			m_schedSapUser->SchedUlConfigInd (ret);
		}

		return;  // no flows to be scheduled

	}

	/*
  if (nflows == 0)
    {
      if (ret.m_dciList.size () > 0)
        {
          m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));
          m_schedSapUser->SchedUlConfigInd (ret);
        }

      return;  // no flows to be scheduled
    }

	 */
	std::map <uint16_t, pw_pfsFlowPerf_t>::iterator itStats;


	if (//(Now().GetSeconds() >= 8.00) &&
			PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
				<< " eNB " << m_pwEnbId << " RemUes " << unsigned(RemUes)
				<< " toBeSchedInfo.totBwReq " << toBeSchedInfo.totBwReq
				<< std::endl;
		std::cout << "break" << std::endl;
	}


	ScheduleUplinkUes(RemUes, rntiAllocated);
	toBeSchedInfo.totBwReq = globalPfs.SubBandAvailable;
	DisributeSbs();
	UlRbAlloc();


	if (//(Now().GetSeconds() >= 8.00) &&
			PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s After pw_PfFfMacScheduler::DoSchedUlTriggerReq "
				<< " eNB " << m_pwEnbId << " RemUes " << unsigned(RemUes)
				<< " toBeSchedInfo.totBwReq " << toBeSchedInfo.totBwReq
				<< std::endl;
		std::cout << "break" << std::endl;
	}


	/*

  // Divide the remaining resources equally among the active users starting from the subsequent one served last scheduling trigger
  //uint16_t tempRbPerFlow = (ffrUlBandwidth) / (nflows + rntiAllocated.size ());
  uint16_t rbPerFlow = (minContinuousUlBandwidth < tempRbPerFlow) ? minContinuousUlBandwidth : tempRbPerFlow;

  if (rbPerFlow < 3)
    {
      rbPerFlow = 3;  // at least 3 rbg per flow (till available resource) to ensure TxOpportunity >= 7 bytes
    }
	 */


	//////////////////////////////////////////////////////////////////////////////////////////////
	// *** Assign RB to the UE - New Data

	int rbAllocated = 0;
	std::vector<struct PfsUlUe>::iterator it_to_alloc;

	// Select first UE to schedule
	if (m_nextRntiUl != 0) {
		for (it_to_alloc = UlAllocInfo.UeLst.begin();
				it_to_alloc != UlAllocInfo.UeLst.end(); it_to_alloc++) {
			if ((*it_to_alloc).rnti == m_nextRntiUl) {
				break;
			}
		}
		if (it_to_alloc == UlAllocInfo.UeLst.end()) {
			NS_LOG_ERROR(this << " no user found");
		}
	}
	else {
		it_to_alloc = UlAllocInfo.UeLst.begin();
		m_nextRntiUl = (*it_to_alloc).rnti;
	}


	// Schedule UEs - Start
	do
	{

		if (  //(Now().GetSeconds() >= 2.625) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW
		) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedUlTriggerReq"
					<< " Start Scheduling UE: "
					<< " eNB " << m_pwEnbId << " RemUes " << unsigned(RemUes)
					<< " Schedule UE RNTI " << it_to_alloc->rnti
					<< " RB " << (int)it_to_alloc->nPrb
					<< " bytesAlloc " << (int)it_to_alloc->bytesAlloc
					<< std::endl;
			std::cout << "break" << std::endl;
		}


		///////////////////////////////////////////////////////////////////////////////////////////////
		// Ignore UE if HARQ is scheduled for this UE
		std::set<uint16_t>::iterator itRnti = rntiAllocated.find((*it_to_alloc).rnti);
		if ((itRnti != rntiAllocated.end())) {
			// UE already allocated for UL-HARQ -> skip it
			NS_LOG_DEBUG(
					this << " UE already allocated in HARQ -> discared, RNTI " << (*it_to_alloc).rnti);

			// PW_EDIT
			if (  //(Now().GetSeconds() >= 8.00) &&
					PWCONFIG_WKINGLOG_UL_SCH_PW) {
				std::cout << "PW_SCHEDULER " << Now().GetSeconds()
														<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
														<< " eNB " << m_pwEnbId << " UE RNTI "
														<< (*it_to_alloc).rnti
														<< " already allocated in HARQ -> DISCARD!"
														<< std::endl;
				std::cout << "break" << std::endl;
			}

			// Move to the next UE in the List
			if (it_to_alloc == UlAllocInfo.UeLst.end()) {
				// restart from the first
				it_to_alloc = UlAllocInfo.UeLst.begin();
			} else {
				// move to the next UE
				it_to_alloc++;
			}

			// Continue loop to the next UE
			continue;
		}


		///////////////////////////////////////////////////////////////////////////////////////////////
		// New Data
		UlDciListElement_s uldci;
		uldci.m_rnti = (*it_to_alloc).rnti;
		uldci.m_rbLen = it_to_alloc->nPrb;
		bool allocated = false;

		/*
		 * Set Power Per Rb@
		 */
		std::map<uint16_t, ueInfo>::iterator itUe;
		itUe = ueUl.find(uldci.m_rnti);

		// PW_EDIT Check if the ue is found in ueUl list
		if (itUe == ueUl.end())
		{
			NS_LOG_ERROR ("No UE Info found for UE RNTI " << uldci.m_rnti << " in uldci");
		}

		//itUe->second.RbAllocation = uldci.m_rbLen;

		// PW_EDIT tmp_debug_log
		if (  //(Now().GetSeconds() >= 2.625) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq " << " UE found "
													<< " eNB " << m_pwEnbId << " UE RNTI " << uldci.m_rnti
													<< " nPrbAllocated " << int(uldci.m_rbLen) << " itbs_index "
													<< int(itUe->second.itbs_index) << " rbAllocated "
													<< rbAllocated << " nPrb " << it_to_alloc->nPrb
													<< " m_ulBW " << (int) m_cschedCellConfig.m_ulBandwidth
													<< std::endl;
		}

		// PW_EDIT: Assign allocatedRBs to UEs
		while ((!allocated)
				&& ((rbAllocated + it_to_alloc->nPrb) < (unsigned) 1 + m_cschedCellConfig.m_ulBandwidth)
				&& (it_to_alloc->nPrb != 0)
				&& (itUe != ueUl.end())		// UE info found
		) {

			// PW_EDIT: TODO - BUG
			if (((*it_to_alloc).schedLcgInfo[0].LcsInfo.size() > 1) &&
					((*it_to_alloc).schedLcgInfo[3].LcsInfo.size() > 1)) {
				//
				std::cout << "ERROR - INVALID UE Logical Channel Config - " << Now().GetSeconds()
														<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
														<< " eNB " << m_pwEnbId << " UE RNTI " << it_to_alloc->rnti
														<< " LGC0size " << (*it_to_alloc).schedLcgInfo[0].LcsInfo.size()
														<< " LGC3size " << (*it_to_alloc).schedLcgInfo[3].LcsInfo.size()
														<< std::endl;

				// Skip UE
				break;
			}

			// check availability
			bool free = true;
			for (uint16_t j = rbAllocated; j < rbAllocated + it_to_alloc->nPrb;
					j++) {
				if (rbMap.at(j) == true) {
					free = false;
					break;
				}
				if ((m_ffrSapProvider->IsUlRbgAvailableForUe(j,
						(*it_to_alloc).rnti)) == false) {
					free = false;
					break;
				}
			}

			if (free) {
				NS_LOG_INFO(
						this << "RNTI: "<< (*it_to_alloc).rnti<< " RB Allocated " << rbAllocated << " rbPerFlow " << it_to_alloc->nPrb << " flows " << FlowsSchedule);

				// PW_EDIT
				if (  //(Now().GetSeconds() >= 8.00) &&
						PWCONFIG_WKINGLOG_UL_SCH_PW) {
					std::cout << "PW_SCHEDULER " << Now().GetSeconds()
															<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
															<< " eNB " << m_pwEnbId << " UE RNTI "
															<< (*it_to_alloc).rnti << " RBallocated "
															<< rbAllocated << " rbPerFlow " << it_to_alloc->nPrb
															<< " flows " << FlowsSchedule
															<< std::endl;
					std::cout << "break" << std::endl;
				}

				uldci.m_rbStart = rbAllocated;

				for (uint16_t j = rbAllocated;
						j < rbAllocated + it_to_alloc->nPrb; j++) {
					rbMap.at(j) = true;
					// store info on allocation for managing ul-cqi interpretation
					rbgAllocationMap.at(j) = (*it_to_alloc).rnti;
				}
				rbAllocated += it_to_alloc->nPrb;
				allocated = true;
				break;
			}
			rbAllocated++;

		}



		if (!allocated) {
			// PW_EDIT: Can't Allocate to this UE or Allocated RB is 0

			// PW_EDIT - tmp debug log
			if (  //(Now().GetSeconds() >= 8.00) &&
					PWCONFIG_WKINGLOG_UL_SCH_PW) {
				std::cout << "PW_SCHEDULER " << Now().GetSeconds()
														<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq *Unable to Allocate*"
														<< " eNB " << m_pwEnbId
														<< " UE RNTI " << it_to_alloc->rnti
														<< " PRB " << it_to_alloc->nPrb
														<< std::endl;
				std::cout << "break" << std::endl;
			}

			// Move to the next UE in the List
			if (it_to_alloc == UlAllocInfo.UeLst.end()) {
				// restart from the first
				it_to_alloc = UlAllocInfo.UeLst.begin();
			} else {
				// move to the next UE
				it_to_alloc++;
			}

			//
			if (rbAllocated == m_cschedCellConfig.m_ulBandwidth) {
				// Stop allocation: no more RBs
				m_nextRntiUl = (*it_to_alloc).rnti;
				break;
			}

			// Continue to the next UE
			continue;

		}

		// -------------------------------------------------------------------------------

		// PW_EDIT
		if (  //(Now().GetSeconds() >= 8.00) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds()
  													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq *GetUlTbSizeFromItbs"
													<< " eNB " << m_pwEnbId << " UE RNTI " << it_to_alloc->rnti
													<< " Allocated RB " << int(it_to_alloc->nPrb)
													<< " itbs_index " << int(itUe->second.itbs_index)
													<< std::endl;
			std::cout << "break" << std::endl;
		}

		uldci.m_tbSize = (m_amc->GetUlTbSizeFromItbs (itUe->second.itbs_index, it_to_alloc->nPrb) / 8);

		std::map <uint16_t, std::vector <double> >::iterator itCqi = m_ueCqi.find ((*it_to_alloc).rnti);
		std::map <uint16_t,ueInfo>::iterator ue;
		uint8_t cqi = 0;
		uint16_t rnti;


		// PW_EDIT
		if (  //(Now().GetSeconds() >= 8.00) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq *UE Allocation Possible*"
													<< " eNB " << m_pwEnbId << " UE RNTI " << it_to_alloc->rnti
													<< " Allocated RB " << int(it_to_alloc->nPrb)
													<< " itbs_index " << int(itUe->second.itbs_index)
													<< " tbSize " << uldci.m_tbSize
													<< std::endl;
			std::cout << "break" << std::endl;
		}


		double BLER,total,failu;
		if (itCqi == m_ueCqi.end ())
		{
			// no cqi info about this UE
			uldci.m_mcs = 0; // MCS 0 -> UL-AMC TBD
		}
		else
		{
			// translate SINR -> cqi: WILD ACK: same as DL
			rnti = (*it_to_alloc).rnti;
			ue = ueUl.find(rnti);

			/*
			 * Just for BLER testing
			 */
			total = miss[rnti] + success[rnti];
			failu = miss[rnti];
			if (total > 0) {
				BLER = failu / total;
			}

			if (ue->second.cqi_index == 0) {
				// Move to the next UE in the List
				if (it_to_alloc == UlAllocInfo.UeLst.end()) {
					// restart from the first
					it_to_alloc = UlAllocInfo.UeLst.begin();
				}
				else {
					// move to the next UE
					it_to_alloc++;
				}
				NS_LOG_DEBUG(this << " UE discared for CQI=0, RNTI " << uldci.m_rnti);

				// PW_EDIT
				if (  //(Now().GetSeconds() >= 8.00) &&
						PWCONFIG_WKINGLOG_UL_SCH_PW) {
					std::cout << "PW_SCHEDULER " << Now().GetSeconds()
															<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
															<< " eNB " << m_pwEnbId << " UE RNTI "
															<< uldci.m_rnti << " DISCARDED for CQI=0"
															<< std::endl;
					std::cout << "break" << std::endl;
				}

				// remove UE from allocation map
				for (uint16_t i = uldci.m_rbStart;
						i < uldci.m_rbStart + uldci.m_rbLen; i++) {
					rbgAllocationMap.at(i) = 0;
				}

				continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
			}
			uldci.m_mcs = ue->second.mcs_index;

		}
		NS_LOG_ERROR(this << "RNTI: " << rnti <<" BLER RATE: "<< BLER*100 << "%" << " rbPerFlow: " << it_to_alloc->nPrb <<" MCS: " << (int)uldci.m_mcs << " TrgCqi: " << (int)itUe->second.trgCqi);

		// PW_EDIT
		if ((Now().GetSeconds() >= 0.00) && PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
					<< " eNB " << m_pwEnbId
					<< " UE RNTI " << rnti << " BLER RATE " << BLER * 100
					<< " %" << " rbPerFlow " << it_to_alloc->nPrb << " MCS "
					<< (int) uldci.m_mcs << " TrgCqi "
					<< (int) itUe->second.trgCqi
					<< " schedLcgInfo0LcsInfoSize " << (*it_to_alloc).schedLcgInfo[0].LcsInfo.size()
					<< " schedLcgInfo3LcsInfoSize " << (*it_to_alloc).schedLcgInfo[3].LcsInfo.size()
					<< std::endl;
		}


		UpdateUlRlcBufferInfo (uldci.m_rnti, uldci.m_tbSize);
		uldci.m_ndi = 1;
		uldci.m_cceIndex = 0;
		uldci.m_aggrLevel = 1;
		uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
		uldci.m_hopping = false;
		uldci.m_n2Dmrs = 0;
		uldci.m_tpc = 0; // no power control
		uldci.m_cqiRequest = false; // only period CQI at this stage
		uldci.m_ulIndex = 0; // TDD parameter
		uldci.m_dai = 1; // TDD parameter
		uldci.m_freqHopping = 0;
		uldci.m_pdcchPowerOffset = 0; // not used
		ret.m_dciList.push_back (uldci);

		// store DCI for HARQ_PERIOD
		uint8_t harqId = 0;
		if (m_harqOn == true)
		{
			std::map <uint16_t, uint8_t>::iterator itProcId;
			itProcId = m_ulHarqCurrentProcessId.find (uldci.m_rnti);
			if (itProcId == m_ulHarqCurrentProcessId.end ())
			{
				NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << uldci.m_rnti);
			}
			harqId = (*itProcId).second;
			std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci = m_ulHarqProcessesDciBuffer.find (uldci.m_rnti);
			if (itDci == m_ulHarqProcessesDciBuffer.end ())
			{
				NS_FATAL_ERROR ("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI " << uldci.m_rnti);
			}
			(*itDci).second.at (harqId) = uldci;
			// Update HARQ process status (RV 0)
			std::map <uint16_t, UlHarqProcessesStatus_t>::iterator itStat = m_ulHarqProcessesStatus.find (uldci.m_rnti);
			if (itStat == m_ulHarqProcessesStatus.end ())
			{
				NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << uldci.m_rnti);
			}
			(*itStat).second.at (harqId) = 0;
		}

		NS_LOG_LOGIC (this << " UE Allocation RNTI " << (*it_to_alloc).rnti << " start PRB " << (uint32_t)uldci.m_rbStart << " nPRB " << (uint32_t)uldci.m_rbLen);
		NS_LOG_ERROR (this << " CQI " << (uint16_t)cqi << " MCS " << (uint32_t)uldci.m_mcs << " TBsize " << uldci.m_tbSize << " RbAlloc " << rbAllocated << " harqId " << (uint16_t)harqId);


		// PW_EDIT PW_WKING_VFT
		if (  (Now().GetSeconds() >= 2.0) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW
		) {
			std::cout << "PW_SCHEDULER DECISION * " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq " << " eNB "
													<< m_pwEnbId << " UE RNTI " << (*it_to_alloc).rnti
													<< " PRB[" << (uint32_t)uldci.m_rbStart << "-"
													<< ((uint32_t)uldci.m_rbStart + (uint32_t)uldci.m_rbLen - 1)
													<< "]" << " CQI(TODO) " << (uint16_t)cqi
													<< " MCS " << (uint32_t)uldci.m_mcs << " TBsize " << uldci.m_tbSize
													<< " BLER RATE " << BLER * 100 << "%"
													<< " TotalRbAllocTillNow " << rbAllocated << " harqId " << (uint16_t)harqId
													<< std::endl;
		}

		// PW_EDIT - Scheduler log to file
		if (PWCONFIG_SCHEDULER_LOGFILE) {
			ReportUlSchedulerStat((*it_to_alloc).rnti, (uint32_t)uldci.m_rbStart,
					(uint32_t)uldci.m_rbLen, (uint32_t)uldci.m_mcs, uldci.m_tbSize);
		}


		// update TTI  UE stats
		itStats = m_flowStatsUl.find (rnti);
		if (itStats != m_flowStatsUl.end ())
		{
			(*itStats).second.lastTtiBytesTrasmitted =  uldci.m_tbSize;
		}
		else
		{
			NS_LOG_DEBUG (this << " No Stats for this allocated UE");
		}

		std::map <uint16_t,PfsUlUe>::iterator it_pfs;
		it_pfs = PfsUlUes.find(rnti);

		// PW_EDIT
		if ((Now().GetSeconds() >= 0.00) && PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
					<< " eNB " << m_pwEnbId
					<< " UE RNTI " << rnti << " BLER RATE " << BLER * 100
					<< " %" << " rbPerFlow " << it_to_alloc->nPrb << " MCS "
					<< (int) uldci.m_mcs << " TrgCqi "
					<< (int) itUe->second.trgCqi
					<< " schedLcgInfo0LcsInfoSize " << (*it_to_alloc).schedLcgInfo[0].LcsInfo.size()
					<< " schedLcgInfo3LcsInfoSize " << (*it_to_alloc).schedLcgInfo[3].LcsInfo.size()
					<< std::endl;
		}

		// PW_EDIT: TODO
		if (((*it_to_alloc).schedLcgInfo[0].LcsInfo.size() > 1) &&
				((*it_to_alloc).schedLcgInfo[3].LcsInfo.size() > 1)) {
			//
			std::cout << "ERROR " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq "
													<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
													<< " LGC0size " << (*it_to_alloc).schedLcgInfo[0].LcsInfo.size()
													<< " LGC3size " << (*it_to_alloc).schedLcgInfo[3].LcsInfo.size()
													<< " are not Valid."
													<< std::endl;
		}

		FinalizeUlOnAlloc((*it_to_alloc));

		// Move to the next UE in the List
		if (it_to_alloc == UlAllocInfo.UeLst.end()) {
			// restart from the first
			it_to_alloc = UlAllocInfo.UeLst.begin();
		}
		else {
			// move to the next UE
			it_to_alloc++;
		}

		if ((rbAllocated == m_cschedCellConfig.m_ulBandwidth) || (it_to_alloc->nPrb == 0))
		{
			// Stop allocation: no more PRBs
			m_nextRntiUl = (*it_to_alloc).rnti;
			break;
		}
	}
	while ((*it_to_alloc).rnti != m_nextRntiUl);
	//while (((*it_to_alloc).rnti != m_nextRntiUl)&&(it_to_alloc->nPrb!=0));
	// Schedule UEs - End
	//////////////////////////////////////////////////////////////////////////////////////////////


	m_nextRntiUl = 0;


	// Update global UE stats
	// update UEs stats

	for (itStats = m_flowStatsUl.begin (); itStats != m_flowStatsUl.end (); itStats++)
	{
		(*itStats).second.totalBytesTransmitted += (*itStats).second.lastTtiBytesTrasmitted;
		// update average throughput (see eq. 12.3 of Sec 12.3.1.2 of LTE – The UMTS Long Term Evolution, Ed Wiley)
		(*itStats).second.lastAveragedThroughput = ((1.0 - (1.0 / m_timeWindow)) * (*itStats).second.lastAveragedThroughput) + ((1.0 / m_timeWindow) * (double)((*itStats).second.lastTtiBytesTrasmitted / 0.001));
		NS_LOG_LOGIC(this << " UE " << "rnti " << itStats->first << " total bytes " << (*itStats).second.totalBytesTransmitted);
		NS_LOG_LOGIC (this << " UE average throughput " << (*itStats).second.lastAveragedThroughput);


		// PW_EDIT PW_WKING_VFT
		if (  //(Now().GetSeconds() >= 8.00) &&
				PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::DoSchedUlTriggerReq " << " eNB "
													<< m_pwEnbId << " UE RNTI " << itStats->first
													<< " total bytes " << (*itStats).second.totalBytesTransmitted
													<< " UE average throughput " << (*itStats).second.lastAveragedThroughput
													<< std::endl;
			std::cout << "break" << std::endl;
		}


		(*itStats).second.lastTtiBytesTrasmitted = 0;
	}




	m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));
	m_schedSapUser->SchedUlConfigInd (ret);

	return;
}

void
pw_PfFfMacScheduler::FinalizeUlOnAlloc(PfsUlUe Ue){
	std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;

	it_buffers = m_ceBsr_LCG_Rxed.find(Ue.rnti);
	PfsRmvUeFromQueue(Ue.rnti);
	PfsUlMngUeInQ(it_buffers->second, Ue.rnti);

}



void
pw_PfFfMacScheduler::DoSchedUlNoiseInterferenceReq (const struct FfMacSchedSapProvider::SchedUlNoiseInterferenceReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	return;
}

void
pw_PfFfMacScheduler::DoSchedUlSrInfoReq (const struct FfMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	return;
}

void
pw_PfFfMacScheduler::UlRbAlloc(){
	std::vector<struct PfsUlUe>:: iterator it;
	uint32_t maxrb = 0;
	uint32_t nPrb;
	std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;
	uint32_t bytesAlloc = 0;


	for (it = UlAllocInfo.UeLst.begin() ; it < UlAllocInfo.UeLst.end(); ++it){
		std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(it->rnti);
		std::map <uint16_t,PfsUlUe>::iterator it_pfs;
		it_pfs = FindUe(it->rnti);
		maxrb =  GetMaxRbs(it->rnti);
		maxrb = std::min((*it).subbandShare*SbsSize, maxrb);
		//Calc Number of PRB required from effBsr(bytes required)
		//uint32_t PrbReq = it_pfs->second.effBsr / ITBS_eff[ue->second.itbs_index] / globalPfs.RePerRb;
		uint32_t PrbReq = it_pfs->second.effBsr*1024*8 / TbSzEff[ue->second.itbs_index] / globalPfs.RePerRb;

		if (PrbReq > maxrb)
			nPrb = maxrb;
		else
			nPrb = PrbReq;

		if (PrbReq == 0)
			PrbReq = 1;

		bytesAlloc = (m_amc->GetUlTbSizeFromItbs (ue->second.itbs_index, nPrb) / 8);
		//AddUetols of scheudled rgSCHCmnUlRbAllocAddUeToLst
		//rgSCHCmnUlRecordUeAlloc
		//rgSCHCmnUlUpdOutStndAlloc
		it_buffers = m_ceBsr_LCG_Rxed.find((*it).rnti);
		if ((*it_buffers).second.UlLcgsArray[0].bs > bytesAlloc)
			(*it_buffers).second.UlLcgsArray[0].bs -= bytesAlloc;
		else{
			(*it_buffers).second.UlLcgsArray[0].bs = 0;
		}
		it_pfs = PfsUlUes.find(it->rnti);
		it_pfs->second.effBsr -=bytesAlloc;

		//FindUe(it->rnti)->second.effBsr-= bytesAlloc;
		(*it).nPrb = nPrb;


		// PW_EDIT
		if (PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << "PW_SCHEDULER DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::UlRbAlloc "
					<< " eNB" << m_pwEnbId << " UE" << it->rnti
					<< " maxrb " << maxrb
					<< " PrbReq " << PrbReq
					<< " Allocated RB " << (*it).nPrb
					<< std::endl;
			std::cout << "Break" << std::endl;
		}


	}



}

//rgSCHCmnUlUpdOutStndAlloc
uint32_t pw_PfFfMacScheduler::DisributeSbs() {
	if (!toBeSchedInfo.UeLst.size())
		return 0;
	std::vector<struct PfsUlUe>::iterator it;
	uint32_t leftOverSbShare = 0;
	uint32_t leftOverSbShareFOrNextUe = 0;
	uint32_t usedSbs = 0;
	uint32_t freeSbs = globalPfs.SubBandAvailable;
	uint16_t UeCount = toBeSchedInfo.UeLst.size();
	uint32_t sbAvailForUes = globalPfs.SubBandAvailable;
	uint32_t totPfsPrio = toBeSchedInfo.TotPfsPrio;
	for (it = toBeSchedInfo.UeLst.begin(); it < toBeSchedInfo.UeLst.end();
			++it) {
		if (UeCount > 1) {

			// PW_EDIT: workaround - against divide by 0; totPfsPrio
			uint32_t sbSharePerUe;
			if (totPfsPrio > 0) {
				sbSharePerUe = (((*it).PfsPrio * sbAvailForUes)
						/ totPfsPrio) + leftOverSbShare
								+ leftOverSbShareFOrNextUe;
			}
			else // if totPfsPrio = 0; divide equally between UE to be sched
			{
				sbSharePerUe = (sbAvailForUes / toBeSchedInfo.UeLst.size())
														+ leftOverSbShare + leftOverSbShareFOrNextUe;

				std::cout << "PW_SCHEDULER " << Now().GetSeconds()
														<< "s pw_PfFfMacScheduler::DisributeSbs " << " eNB "
														<< m_pwEnbId << " nofUEs " << UeCount
														<< " totPfsPrio is 0"
														<< std::endl;
			}

			(*it).subbandShare = std::max(std::min(sbSharePerUe, (*it).SbsReq),
					(unsigned int) 1);
			leftOverSbShare += (sbSharePerUe - (*it).subbandShare)
													/ toBeSchedInfo.UeLst.size() - 1;
			leftOverSbShareFOrNextUe = (sbSharePerUe - (*it).subbandShare)
													% toBeSchedInfo.UeLst.size() - 1;

			// PW_EDIT
			if (PWCONFIG_WKINGLOG_UL_SCH_PW) {
				std::cout << "PW_SCHEDULER " << Now().GetSeconds()
														<< "s pw_PfFfMacScheduler::DisributeSbs " << " eNB "
														<< m_pwEnbId << " nofUEs " << UeCount << " UE"
														<< (*it).rnti << " sbAvailForUes " << sbAvailForUes
														<< " totPfsPrio " << totPfsPrio << " sbSharePerUe "
														<< sbSharePerUe << " SbsReq " << (*it).SbsReq
														<< " subbandShare " << (*it).subbandShare
														<< " leftOverSbShare " << leftOverSbShare
														<< " leftOverSbShareFOrNextUe "
														<< leftOverSbShareFOrNextUe << std::endl;
				std::cout << "break" << std::endl;
			}

		} else {
			(*it).subbandShare = std::min(freeSbs, (*it).SbsReq);
		}

		usedSbs += (*it).subbandShare;
		freeSbs -= (*it).subbandShare;

		SortAndInsertUeLstSbShare((*it), (*it).subbandShare);
		if (!(--UeCount))
			return usedSbs;

	}

	return usedSbs;

}

void
pw_PfFfMacScheduler::ScheduleUplinkUes(uint8_t RemUes, std::set <uint16_t> rntiAllocated){
	toBeSchedInfo.TotPfsPrio = 0;
	toBeSchedInfo.totBwReq = 0;
	UlAllocInfo.TotPfsPrio = 0;
	UlAllocInfo.totBwReq = 0;

	std::vector<struct PfsUlUe>:: iterator it;


	toBeSchedInfo.UeLst.clear();
	UlAllocInfo.UeLst.clear();

	std::vector<PfsUlUe> txQ;
	for (int i = ULNumGbrQueues-1; i >= 0 && RemUes; --i){
		txQ = globalPfs.Queues.GbrQueue[i];
		RemUes = PfsUlSchedData(txQ, RemUes, rntiAllocated);
	}
	for (int i = ULNumAmbrQueues-1; i >= 0 && RemUes; --i){
		txQ = globalPfs.Queues.AmbrQueue[i];
		RemUes = PfsUlSchedData(txQ, RemUes, rntiAllocated);
	}


	//Add rest of queues
	//DistributeSbs


	// PW_EDIT
	if (//(Now().GetSeconds() >= 8.00) &&
			PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s pw_PfFfMacScheduler::ScheduleUplinkUes "
				<< " eNB " << m_pwEnbId << " RemUes " << unsigned(RemUes)
				<< " rntiAllocated.size() " << rntiAllocated.size()
				<< " ULNumGbrQueues " << unsigned(ULNumGbrQueues)
				<< " ULNumAmbrQueues " << unsigned(ULNumAmbrQueues)
				<< std::endl;
		std::cout << "break" << std::endl;
	}

}
uint8_t
pw_PfFfMacScheduler::PfsUlSchedData(std::vector<PfsUlUe> Queue, uint8_t RemUes, std::set <uint16_t> rntiAllocated){
	std::vector<struct PfsUlUe>:: iterator it;
	PfsUlUe UeForScheudle;
	if (RemUes){
		for (it = Queue.begin() ; it < Queue.end() && RemUes; ++it ){
			UeForScheudle = (*it);
			PfsUlSchedUeForDat(UeForScheudle, rntiAllocated);
			--RemUes;
		}
	}
	return RemUes;


}



void
pw_PfFfMacScheduler::PfsUlSchedUeForDat(PfsUlUe UeForScheudle, std::set <uint16_t> rntiAllocated){
	uint32_t subBandReq;
	std::set <uint16_t>::iterator itRnti = rntiAllocated.find (UeForScheudle.rnti);
	if (itRnti != rntiAllocated.end()){
		NS_LOG_LOGIC (this << "UE already scheduled in this SF for Retx");
		return;

	}
	std::map <uint16_t,PfsUlUe>::iterator it_pfs;
	it_pfs = FindUe(UeForScheudle.rnti);

	// PW_EDIT Check if the UE is found in UeForSchedule
	if (it_pfs == PfsUlUes.end ())
	{
		std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s: pw_PfFfMacScheduler::PfsUlSchedUeForDat "
				<< " UE not found in PfsUlUes"
				<< std::endl;
	}

	subBandReq = GetNumSbs(it_pfs->second);
	toBeSchedInfo.TotPfsPrio+=it_pfs->second.PfsPrio;
	toBeSchedInfo.totBwReq+=subBandReq;
	UeForScheudle.SbsReq = subBandReq;
	it_pfs->second.SbsReq = subBandReq;
	if (subBandReq == 0)
		subBandReq++;
	SortAndInsertUeLst(UeForScheudle, subBandReq);


	// PW_EDIT
	if (PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds() << "s: pw_PfFfMacScheduler::PfsUlSchedUeForDat "
				<< " eNB" << m_pwEnbId << " UE" << UeForScheudle.rnti
				<< " rntiAllocated.size() " << rntiAllocated.size()
				<< " PfsPrio " << it_pfs->second.PfsPrio << " TotPfsPrio " << toBeSchedInfo.TotPfsPrio
				<< " subBandReq " << subBandReq << " totBwReq " << toBeSchedInfo.totBwReq
				<< " SbsReq " << it_pfs->second.SbsReq
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


}

void
pw_PfFfMacScheduler::SortAndInsertUeLstSbShare(PfsUlUe UeForScheudle, uint32_t subBandReq){
	std::vector<struct PfsUlUe>:: iterator it;
	for (it = UlAllocInfo.UeLst.begin() ; it < UlAllocInfo.UeLst.end(); ++it){

		if (UeForScheudle.subbandShare == 0){
			UlAllocInfo.UeLst.push_back(UeForScheudle);
			return;
		}
		if ((*it).subbandShare >  subBandReq ){
			UlAllocInfo.UeLst.insert(it,UeForScheudle);

			return;

		}
		if ((*it).SbsReq == 0){
			UlAllocInfo.UeLst.insert(it,UeForScheudle);


		}
		if (it == UlAllocInfo.UeLst.end()){
			UlAllocInfo.UeLst.push_back(UeForScheudle);
			return;
		}
	}
	UlAllocInfo.UeLst.insert(it,UeForScheudle);


}
void
pw_PfFfMacScheduler::SortAndInsertUeLst(PfsUlUe UeForScheudle, uint32_t subBandReq){
	std::vector<struct PfsUlUe>:: iterator it;

	for (it = toBeSchedInfo.UeLst.begin() ; it < toBeSchedInfo.UeLst.end(); ++it){
		if ((*it).SbsReq >  subBandReq ){
			toBeSchedInfo.UeLst.insert(it,UeForScheudle);

			return;

		}
		if (it == toBeSchedInfo.UeLst.end()){
			toBeSchedInfo.UeLst.push_back(UeForScheudle);

			return;
		}

	}
	toBeSchedInfo.UeLst.insert(it,UeForScheudle);




}

uint32_t
pw_PfFfMacScheduler::GetNumSbs(PfsUlUe UeForScheudle){
	uint32_t PrbReq;
	uint32_t NumSbs;
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(UeForScheudle.rnti);
	PrbReq = UeForScheudle.effBsr*1024*8 / TbSzEff[ue->second.itbs_index] / globalPfs.RePerRb;
	NumSbs = utils::SCH_CEIL(PrbReq, SbsSize);
	if (NumSbs == 0)
		NumSbs = 1;
	return NumSbs;


}

int
pw_PfFfMacScheduler::getNumFlows(std::set <uint16_t> rntiAllocated){

	int nflows = 0;

	std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;
	for (it_buffers = m_ceBsr_LCG_Rxed.begin (); it_buffers != m_ceBsr_LCG_Rxed.end (); it_buffers++){
		//Check if UEs are already allocated by checking if the rntiAllocated structure
		std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it_buffers).first);

		// select UEs with queues not empty and not yet allocated for HARQ
		//Check if UE's BSR not empty
		for (uint8_t lcg = 0; lcg < MAX_LCG_PER_UE; ++lcg)
		{
			uint32_t buffer_size = it_buffers->second.UlLcgsArray[lcg].bs;
			if (buffer_size > 0 && itRnti == rntiAllocated.end ()){
				nflows++;
				break;
			}
		}
	}

	return nflows;



}

void
pw_PfFfMacScheduler::DoSchedUlMacCtrlInfoReq (const struct FfMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;
	if (params.m_macCeList.size()){
		for (unsigned int i = 0; i< params.m_macCeList.size();i++){
			VendorSpecificListElement_s tmp;
			PfsUlLcgs buffers;
			tmp = params.m_vendorSpecificList.at(i);
			if ( params.m_macCeList.at (i).m_macCeType == MacCeListElement_s::BSR )
			{

				for (unsigned int j = 0; j < 4;j++){
					if (tmp.LCGS.UlLcgsArray[j].LcgId == j){
						buffers.UlLcgsArray[j].bs = tmp.LCGS.UlLcgsArray[j].bs;
						buffers.UlLcgsArray[j].cfgdGbr = tmp.LCGS.UlLcgsArray[j].cfgdGbr;
						buffers.UlLcgsArray[j].effGbr = tmp.LCGS.UlLcgsArray[j].effGbr;
						buffers.UlLcgsArray[j].deltaMbr = tmp.LCGS.UlLcgsArray[j].deltaMbr;
						buffers.UlLcgsArray[j].effDeltaMbr = tmp.LCGS.UlLcgsArray[j].effDeltaMbr;
						buffers.UlLcgsArray[j].LcgId = tmp.LCGS.UlLcgsArray[j].LcgId;
					}
					else{
						buffers.UlLcgsArray[j].bs = 0;
						buffers.UlLcgsArray[j].cfgdGbr = 0;
						buffers.UlLcgsArray[j].effGbr = 0;
						buffers.UlLcgsArray[j].deltaMbr = 0;
						buffers.UlLcgsArray[j].effDeltaMbr = 0;
						buffers.UlLcgsArray[j].LcgId = j;

					}
				}

				it_buffers = m_ceBsr_LCG_Rxed.find(tmp.rnti);
				if (it_buffers == m_ceBsr_LCG_Rxed.end ()){
					// create the new entry
					NS_LOG_LOGIC (this << " Didnt find RNTI=" << tmp.rnti << " in buffers list of UE");
					m_ceBsr_LCG_Rxed.insert ( std::pair<uint16_t, PfsUlLcgs> (tmp.rnti, buffers));

				}
				else{
					(*it_buffers).second = buffers;
				}

				UpdEffBsr(buffers, tmp.rnti);

				PfsUlMngUeInQ(buffers, tmp.rnti);

			}

			else if(params.m_macCeList.at (i).m_macCeType == MacCeListElement_s::PHR)
			{
				uint16_t rnti = params.m_macCeList.at (i).m_rnti;
				double phr = params.m_macCeList.at (i).m_macCeValue.m_phr;
				PwrUpdPhr(phr, rnti);
			}
		}



	}
	//Data indication recieved, updating BS of LCG
	else{
		VendorSpecificListElement_s tmp;
		PfsUlLcgs buffers;
		for (int i = 0; i < MAX_LCG_PER_UE; i++){
			buffers.UlLcgsArray[i].bs = 0;
			buffers.UlLcgsArray[i].BR = 0;
			buffers.UlLcgsArray[i].cfgdGbr = 0;
			buffers.UlLcgsArray[i].deltaMbr = 0;
			buffers.UlLcgsArray[i].effDeltaMbr = 0;
			buffers.UlLcgsArray[i].effGbr = 0;
			buffers.UlLcgsArray[i].reportedbs = 0;
			buffers.UlLcgsArray[i].LcgId = i;
		}
		tmp = params.m_vendorSpecificList.front();
		uint16_t lcgId = tmp.LCGS.UlLcgsArray[tmp.m_type].LcgId;
		buffers.UlLcgsArray[lcgId].BR = tmp.LCGS.UlLcgsArray[lcgId].BR;
		buffers.UlLcgsArray[lcgId].LcgId = lcgId;
		buffers.UlLcgsArray[lcgId].bs = tmp.LCGS.UlLcgsArray[lcgId].bs;
		buffers.UlLcgsArray[lcgId].cfgdGbr = tmp.LCGS.UlLcgsArray[lcgId].cfgdGbr;
		buffers.UlLcgsArray[lcgId].effGbr = tmp.LCGS.UlLcgsArray[lcgId].effGbr;
		buffers.UlLcgsArray[lcgId].deltaMbr = tmp.LCGS.UlLcgsArray[lcgId].deltaMbr;
		buffers.UlLcgsArray[lcgId].effDeltaMbr = tmp.LCGS.UlLcgsArray[lcgId].effDeltaMbr;
		PfsUlLcgUpd(buffers.UlLcgsArray[tmp.m_type], tmp.rnti);
		it_buffers = m_ceBsr_LCG_Rxed.find(tmp.rnti);
		if (it_buffers == m_ceBsr_LCG_Rxed.end ()){
			// create the new entry
			NS_LOG_LOGIC (this << "Didnt find RNTI=" << tmp.rnti << " in buffers list of UE");
			m_ceBsr_LCG_Rxed.insert ( std::pair<uint16_t, PfsUlLcgs> (tmp.rnti, buffers));

		}
		else{
			(*it_buffers).second.UlLcgsArray[lcgId] = buffers.UlLcgsArray[lcgId];
		}

		UpdEffBsr(buffers, tmp.rnti);

		PfsUlMngUeInQ(buffers, tmp.rnti);




	}






	return;
}






void
pw_PfFfMacScheduler::PwrUpdPhr(int8_t phr, uint16_t rnti){
	uint8_t rbPwr;
	std::map <uint16_t,ueInfo>::iterator it;
	it = ueUl.find (rnti);
	NS_LOG_DEBUG (this << " RNTI: " << rnti << " PHR: " << (uint16_t)phr);
	rbPwr = GetPwrRbToPwrdB(it->second.RbAllocation);
	it->second.phr = phr;
	it->second.pwrPerRb = MAX_UE_PWR - it->second.phr - rbPwr;
	SetTargetCqi(rnti);

	if(it->second.volteOn)
	{
		it->second.trgCqi = TargetCqi[4];
	}

	it->second.isPhrAvail = true;
}

void
pw_PfFfMacScheduler::UpdEffBsr(PfsUlLcgs m_bufferStatus, uint16_t rnti){
	//Addition needs to be added for after allocation updates.
	uint32_t BStotal = 0;
	std::map <uint16_t,PfsUlUe>::iterator it_pfs;
	it_pfs = FindUe(rnti);
	for (int i = 0; i < MAX_LCG_PER_UE; i++){
		BStotal += m_bufferStatus.UlLcgsArray[i].bs;
	}
	(*it_pfs).second.effBsr = BStotal;




}



void
pw_PfFfMacScheduler::PfsUlLcgUpd(PfsUlLcgInfo LCG, uint16_t rnti){
	std::map <uint16_t,PfsUlUe>::iterator it_pfs;
	it_pfs = FindUe(rnti);


	if (LCG.cfgdGbr){
		if (LCG.effGbr == 0 && LCG.deltaMbr!=0 && LCG.BR > LCG.deltaMbr -  LCG.effDeltaMbr){
			(*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo = PfsinitFracPrioInfo((*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo,  LCG.deltaMbr);
			(*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo = PfsUpdFracPrioInfo((*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo, LCG.deltaMbr-LCG.effDeltaMbr);

		}
		else{
			(*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo= PfsUpdFracPrioInfo((*it_pfs).second.schedLcgInfo[LCG.LcgId].fracPrioInfo, LCG.BR);
		}
	}
	else{
		(*it_pfs).second.fracPrioInfo = PfsUpdFracPrioInfo((*it_pfs).second.fracPrioInfo, LCG.BR);
	}



}


PfsFracPrioInfo
pw_PfFfMacScheduler::PfsUpdFracPrioInfo(PfsFracPrioInfo fracPrioInfo, uint32_t bytes){
	fracPrioInfo.fracPrioLvls += (fracPrioInfo.achvdFracDataRate + bytes) / fracPrioInfo.fracDataRate;
	fracPrioInfo.achvdFracDataRate = (fracPrioInfo.achvdFracDataRate + bytes) % fracPrioInfo.fracDataRate;
	if (fracPrioInfo.fracPrioLvls >= PFS_FAIRNESS_FACTOR_LVLS){
		fracPrioInfo.fracPrioLvls = PFS_FAIRNESS_FACTOR_LVLS-1;
	}

	return fracPrioInfo;


}

PfsFracPrioInfo
pw_PfFfMacScheduler::PfsinitFracPrioInfo(PfsFracPrioInfo fracPrioInfo, uint32_t cfgdRate ){

	fracPrioInfo.achvdFracDataRate = 0;
	fracPrioInfo.fracPrioLvls = 0;
	fracPrioInfo.fracDataRate = utils::SCH_CEIL(cfgdRate, PFS_FAIRNESS_FACTOR_LVLS);

	return fracPrioInfo;

}



void
pw_PfFfMacScheduler::PfsUlMngUeInQ (PfsUlLcgs m_bufferStatus, uint16_t rnti){

	bool Priochngd = false;
	int PrioLcg = -1;
	//Find PfsUe
	std::map <uint16_t,PfsUlUe>::iterator it_pfs;
	it_pfs = FindUe(rnti);


	// PW_EDIT
	if (PWCONFIG_WKINGLOG_UL_SCH_PW) {
		std::cout << Now().GetSeconds() << "s: PfsUlMngUeInQ: eNB " << m_pwEnbId << " RNTI " << rnti
				<< std::endl;
		std::cout << "Break" << std::endl;
	}


	//if(pfsUe->lstCp == &pfsCell->txQueues.contResLst)
	// {
	//   RETVOID;
	//}
	if (m_bufferStatus.UlLcgsArray[0].bs > 0 ){
		/* As long as UE has SRB data to be scheduled, highest
		 * prio is assigned to this UE */
		//pfsUe->pfsPrio = pfsCell->maxSrbDrb0PrioVal;
		// rgSCHPfsUlAddUeToSigQ(cell, ue);
		//LOG_MSG(MAC_SCH_PFS_UL_ADD_TO_Q, LOGINFO, MAC_SCHEDULER, glblTtiCnt, ue->ueId, pfsUe->srRcvd, pfsUe->pfsPrio, cmnLcg->bs,0,0,0,__func__,"sigLst");
		//RETVOID;

		(*it_pfs).second.PfsPrio = globalPfs.maxSrbDrbPrioval;
	}
	/**
	 else if((!cmnLcg->bs) && (!pfsUe->srRcvd))
	   {
	 * //Added the else-if conditional to do the handling of
	 * removing the UE from the sigLst (if it exists) if no SR grant is pending
	 * and BS for lcg[0] is 0.
	      if ((pfsUe->lstCp) && (pfsUe->lstCp == &pfsCell->txQueues.sigLst))
	      {
	         LOG_MSG(MAC_SCH_PFS_UL_ADD_TO_Q, LOGINFO, MAC_SCHEDULER, glblTtiCnt, ue->ueId, pfsUe->srRcvd, pfsUe->pfsPrio, cmnLcg->bs,0,0,0,__func__,"sigLst Removed");
	         rgSCHPfsUlRmvUeFrmQ(cell, ue);
	      }
	   }

	 **/


	for (uint8_t lcg = 1; lcg < MAX_LCG_PER_UE; ++lcg)
	{
		uint32_t buffer_size = m_bufferStatus.UlLcgsArray[lcg].bs;
		if (buffer_size > 0){
			if (m_bufferStatus.UlLcgsArray[lcg].cfgdGbr){
				if (m_bufferStatus.UlLcgsArray[lcg].effGbr){
					PrioLcg = lcg ;
					break;

				}
				else{
					PrioLcg = lcg ;
					continue;
				}
			}
			else{
				if (PrioLcg!=-1){
					break;
				}
				else{
					PrioLcg = lcg;
					break;
				}
			}

		}
	}
	if (PrioLcg!=-1){
		//pfsUe->lcgPrio = pfsUe->schedLcgInfo[prioLcg->lcgId].lcgPrio;
		(*it_pfs).second.lcgPrio =  (*it_pfs).second.schedLcgInfo[PrioLcg].lcgPrio;
		NS_LOG_LOGIC (this << " RNTI=" << rnti << " BSR report, calling PfsUlUpdPrio");
		PfsUlUpdPfsPrio(it_pfs, m_bufferStatus.UlLcgsArray[PrioLcg], &Priochngd);

		if (Priochngd){
			NS_LOG_LOGIC (this << " RNTI=" << rnti << " BSR report, calling PfsUlUpdPrio Priority changed");

		}
		PfsAddUeToPfsQ(&Priochngd, m_bufferStatus.UlLcgsArray[PrioLcg], it_pfs, (*it_pfs).first);



	}



}


/*
 * rgSCHPfsUlAddUeToPfsQ
 */
void
pw_PfFfMacScheduler::PfsAddUeToPfsQ(bool *Priochngd, PfsUlLcgInfo LCG, std::map <uint16_t,PfsUlUe>::iterator PfsUe, uint16_t rnti){
	uint8_t qId;
	if (PfsUe->second.pointQueue!= NULL){
		if (!(*Priochngd))
			//Also check if already scheudled UE
			return;
		PfsRmvUeFromQueue(PfsUe->second.rnti);
	}

	if (LCG.effGbr){
		//
		if (PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE) {
			// VFT Modification to change PW Scheduler to RR_like
			qId = UlGetQIdNew(PfsUe->second.PfsPrio, globalPfs.ranges.gbrRange);
		}
		else {
			// default - Radisys
			qId = UlGetQid(PfsUe->second.PfsPrio, globalPfs.ranges.gbrRange);
		}

		PfsUe->second.qid = qId;
		PfsUe->second.rnti = rnti;
		PfsUlAddUeToQ(PfsUe, &globalPfs.Queues.GbrQueue[qId]);
		PfsUe->second.pointQueue = &globalPfs.Queues.GbrQueue[qId];
	}
	else if (LCG.effDeltaMbr){
		//
		if (PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE) {
			// VFT Modification to change PW Scheduler to RR_like
			qId = UlGetQIdNew(PfsUe->second.PfsPrio, globalPfs.ranges.mbrRange);
		}
		else {
			// default - Radisys
			qId = UlGetQid(PfsUe->second.PfsPrio, globalPfs.ranges.mbrRange);
		}

		PfsUe->second.qid = qId;
		// pfsUe->lstCp = &pfsCell->txQueues.gbrLst[qId];
	}
	else
	{
		if (PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE) {
			// VFT Modification to change PW Scheduler to RR_like
			qId = UlGetQIdNew (PfsUe->second.PfsPrio, globalPfs.ranges.ambrRange);
		}
		else {
			// default - Radisys
			qId = UlGetQid(PfsUe->second.PfsPrio, globalPfs.ranges.ambrRange);
		}

		PfsUe->second.qid = qId;
		PfsUe->second.rnti = rnti;

		// PW_EDIT
		if (PWCONFIG_WKINGLOG_UL_SCH_PW) {
			std::cout << Now().GetSeconds() << "s: PfsUlUpdPfsPrio: eNB"
					<< m_pwEnbId << " LCG.LcgId " << (int)LCG.LcgId
					<< " qId " << (int)qId
					<< " PfsPrio " << (int)PfsUe->second.PfsPrio
					<< " range.min " << (int)globalPfs.ranges.ambrRange.min
					<< " range.max " << (int)globalPfs.ranges.ambrRange.max
					<< " range.NumQ " << (int)globalPfs.ranges.ambrRange.NumQueus
					<< " range.maxDiff " << (int)globalPfs.ranges.ambrRange.maxDiff
					<< std::endl;
			std::cout << "Break" << std::endl;
		}

		PfsUlAddUeToQ(PfsUe, &globalPfs.Queues.AmbrQueue[qId]);
		PfsUe->second.pointQueue = &globalPfs.Queues.AmbrQueue[qId];
		// pfsUe->lstCp = &pfsCell->txQueues.gbrLst[qId];

	}
}

void
pw_PfFfMacScheduler::PfsRmvUeFromQueue(uint16_t rnti){
	std::vector<PfsUlUe>:: iterator it;
	std::map <uint16_t,PfsUlUe>::iterator it_pfs;
	it_pfs = PfsUlUes.find(rnti);

	// PW_EDIT: Check if pointQ is null
	if ((*it_pfs).second.pointQueue != NULL) {

		for (it = (*it_pfs).second.pointQueue->begin();
				it < (*it_pfs).second.pointQueue->end(); ++it) {
			if ((*it).rnti == (*it_pfs).second.rnti)
				(*it_pfs).second.pointQueue->erase(it);

		}
		//
		(*it_pfs).second.pointQueue = NULL;
	}

}
/*
 * Insert UE to queue in a descending order of PfsPrio
 */
void
pw_PfFfMacScheduler::PfsUlAddUeToQ(std::map <uint16_t,PfsUlUe>::iterator PfsUe, std::vector<PfsUlUe> *UEsQueue){
	std::vector<struct PfsUlUe>:: iterator it;

	for (it = UEsQueue->begin() ; it < UEsQueue->end(); ++it){
		if ((*it).PfsPrio <  PfsUe->second.PfsPrio ){
			UEsQueue->insert(it,PfsUe->second);

			return;

		}

	}
	UEsQueue->insert(it,PfsUe->second);



}








/*
 * Returns corresponding queue index to place the UE in
 */
uint8_t
pw_PfFfMacScheduler::UlGetQid(uint32_t PfsPrio, PfsPrioRange range ){
	uint8_t Qid;
	Qid = ((PfsPrio- range.min)*range.NumQueus)/range.maxDiff;
	return Qid;

}

/**
 * Func: rgSCHPfsUlGetQIdNew
 * Desc: For UL, get QId for dataLst given PFS priority
 *
 */
uint8_t
pw_PfFfMacScheduler::UlGetQIdNew (uint32_t PfsPrio, PfsPrioRange range) {
	//
	uint8_t qId = PfsPrio%128;

	return qId;
}


void pw_PfFfMacScheduler::PfsUlUpdPfsPrio(
		std::map<uint16_t, PfsUlUe>::iterator it_pfs, PfsUlLcgInfo LCG,
		bool *Priochngd) {
	PfsUlUe pfsUe = (*it_pfs).second;
	uint32_t fairPrioIdx;

	if (LCG.cfgdGbr) {
		fairPrioIdx = pfsUe.schedLcgInfo[LCG.LcgId].fracPrioInfo.fracPrioLvls;
	} else {
		fairPrioIdx = pfsUe.fracPrioInfo.fracPrioLvls;
	}

	// PW_EDIT
	if (false) {
		std::cout << Now().GetSeconds() << "s: PfsUlUpdPfsPrio: eNB"
				<< m_pwEnbId << " lcgPrio " << pfsUe.lcgPrio << " cqiPrio "
				<< pfsUe.cqiPrio << " fairPrios(" << fairPrioIdx << ") "
				<< unsigned(globalPfs.tbls.fairPrios[fairPrioIdx])
				<< " priosScaleFact " << globalPfs.priosScaleFact << std::endl;
		std::cout << "Break" << std::endl;
	}

	uint32_t prevPfs = pfsUe.PfsPrio;

	//Check the fractional priority initializing and updates
	if (PWCONFIG_PWSCHEDULER_CHANGETO_RRLIKE) {
		// VFT Modification to change PW Scheduler to RR_like
		if (pfsUe.lcgPrio == 0 || pfsUe.lcgPrio == 9) {
			pfsUe.PfsPrio = 0;
		} else {
			if (PWCONFIG_PWSCHEDULER_RRLIKE_EQUPDATE) {
				// Using max of TP factor value instead of '29' maxTpprio
				pfsUe.PfsPrio = ((32 - pfsUe.lcgPrio * 4) + (m_pwMaxTPprio - pfsUe.cqiPrio)
						+ globalPfs.tbls.fairPrios[fairPrioIdx]);
			}
			else {
				pfsUe.PfsPrio = ((32 - pfsUe.lcgPrio * 4) + (29 - pfsUe.cqiPrio)
						+ globalPfs.tbls.fairPrios[fairPrioIdx]);
			}
		}
	}
	else
	{
		// Default
		pfsUe.PfsPrio = (pfsUe.lcgPrio * pfsUe.cqiPrio
				* globalPfs.tbls.fairPrios[fairPrioIdx])
												/ globalPfs.priosScaleFact;
	}

	if (prevPfs != pfsUe.PfsPrio) {
		*Priochngd = true;
		(*it_pfs).second.PfsPrio = pfsUe.PfsPrio;
	}

	// PW_EDIT
	if (false) {
		std::cout << Now().GetSeconds() << "s: PfsUlUpdPfsPrio[TP" << TP_FACTOR
				<< ",FAIR" << FAIRNESS_FACTOR << "] eNB " << m_pwEnbId
				<< " UE RNTI " << pfsUe.rnti << " lcgPrio " << pfsUe.lcgPrio
				<< " cqiPrio " << pfsUe.cqiPrio << " fairPrios(" << fairPrioIdx
				<< ") " << unsigned(globalPfs.tbls.fairPrios[fairPrioIdx])
				<< " priosScaleFact " << globalPfs.priosScaleFact
				<< " pfsUe.PfsPrio " << pfsUe.PfsPrio
				<< " (*it_pfs).second.PfsPrio " << (*it_pfs).second.PfsPrio
				<< std::endl;
		std::cout << "Break" << std::endl;
	}

}

void
pw_PfFfMacScheduler::DoSchedUlCqiInfoReq (const struct FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
	NS_LOG_FUNCTION (this);
	m_ffrSapProvider->ReportUlCqiInfo (params);
	std::map <uint16_t,uint16_t> rntivec;
	std::map <uint16_t,uint16_t>::iterator it;

	// retrieve the allocation for this subframe
	switch (m_ulCqiFilter)
	{
	case FfMacScheduler::SRS_UL_CQI:
	{
		// filter all the CQIs that are not SRS based
		if (params.m_ulCqi.m_type != UlCqi_s::SRS)
		{
			return;
		}
	}
	break;
	case FfMacScheduler::PUSCH_UL_CQI:
	{
		// filter all the CQIs that are not SRS based
		if (params.m_ulCqi.m_type != UlCqi_s::PUSCH)
		{
			return;
		}
	}
	case FfMacScheduler::ALL_UL_CQI:
		break;

	default:
		NS_FATAL_ERROR ("Unknown UL CQI type");
	}

	switch (params.m_ulCqi.m_type)
	{
	case UlCqi_s::PUSCH:
	{
		std::map <uint16_t, std::vector <uint16_t> >::iterator itMap;
		std::map <uint16_t, std::vector <double> >::iterator itCqi;
		NS_LOG_DEBUG (this << " Collect PUSCH CQIs of Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf));
		itMap = m_allocationMaps.find (params.m_sfnSf);
		if (itMap == m_allocationMaps.end ())
		{
			return;
		}
		for (uint32_t i = 0; i < (*itMap).second.size (); i++)
		{
			// convert from fixed point notation Sxxxxxxxxxxx.xxx to double
			double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (i));
			itCqi = m_ueCqi.find ((*itMap).second.at (i));
			if (itCqi == m_ueCqi.end ())
			{
				// create a new entry
				std::vector <double> newCqi;
				for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
				{
					if (i == j)
					{
						newCqi.push_back (sinr);
					}
					else
					{
						// initialize with NO_SINR value.
						newCqi.push_back (NO_SINR);
					}

				}
				m_ueCqi.insert (std::pair <uint16_t, std::vector <double> > ((*itMap).second.at (i), newCqi));
				// generate correspondent timer
				m_ueCqiTimers.insert (std::pair <uint16_t, uint32_t > ((*itMap).second.at (i), m_cqiTimersThreshold));
			}
			else
			{
				// update the value
				(*itCqi).second.at (i) = sinr;
				it = rntivec.find(itMap->second.at (i));
				if(it == rntivec.end() && itMap->second.at (i) != 0)
				{
					rntivec.insert ( std::pair<uint16_t, uint16_t > (itMap->second.at (i), itMap->second.at (i)));
				}
				NS_LOG_DEBUG (this << " RNTI " << (*itMap).second.at (i) << " RB " << i << " SINR " << sinr);
				// update correspondent timer
				std::map <uint16_t, uint32_t>::iterator itTimers;
				itTimers = m_ueCqiTimers.find ((*itMap).second.at (i));
				(*itTimers).second = m_cqiTimersThreshold;

			}

		}
		std::map<uint16_t,uint16_t>::iterator it;

		for(it = rntivec.begin(); it != rntivec.end() ; it++)
		{
			updItbsUlcqi(it->second,0,m_cschedCellConfig.m_ulBandwidth-2);
			std::map <uint16_t,PfsUlUe>::iterator it_pfs;
			std::map <uint16_t,ueInfo>::iterator ue;
			ue = ueUl.find (it->second);
			it_pfs = PfsUlUes.find(it->second);
			it_pfs->second.cqiPrio = PfsUlGetTpPrio(globalPfs, ue->second.itbs_index);
			std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;
			it_buffers = m_ceBsr_LCG_Rxed.find(it->second);
			PfsUlMngUeInQ(it_buffers->second, it->second);
		}

		// remove obsolete info on allocation
		m_allocationMaps.erase (itMap);
	}

	//   iTbs  = rgSCHCmnUlGetITbs(cell, ue, cell->isCpUlExtend); Get Itbs based on ULLA and CQI indications
	//    pfsUe->cqiPrio = rgSCHPfsUlGetTpPrio(pfsCell, iTbs); Update CQI priority based on the new ITbs
	//UpdCqiPrio()



	break;
	case UlCqi_s::SRS:
	{
		NS_LOG_DEBUG (this << " Collect SRS CQIs of Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf));
		// get the RNTI from vendor specific parameters
		uint16_t rnti = 0;
		NS_ASSERT (params.m_vendorSpecificList.size () > 0);
		for (uint16_t i = 0; i < params.m_vendorSpecificList.size (); i++)
		{
			if (params.m_vendorSpecificList.at (i).m_type == SRS_CQI_RNTI_VSP)
			{
				Ptr<SrsCqiRntiVsp> vsp = DynamicCast<SrsCqiRntiVsp> (params.m_vendorSpecificList.at (i).m_value);
				rnti = vsp->GetRnti ();
			}
		}
		std::map <uint16_t, std::vector <double> >::iterator itCqi;
		itCqi = m_ueCqi.find (rnti);



		// PW_EDIT
		if (  //(Now().GetSeconds() >= 8.00) &&
				false) {
			std::cout << "PW_SCHEDULER " << Now().GetSeconds()
    													<< "s pw_PfFfMacScheduler::updItbsUlcqi "
														<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
														//<< " minSinr " << (*itCqi).second.at (0)
														<< std::endl;
			std::cout << "break -3" << std::endl;
		}




		if (itCqi == m_ueCqi.end ())
		{
			// create a new entry
			std::vector <double> newCqi;
			for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
			{
				double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (j));
				newCqi.push_back (sinr);
				NS_LOG_INFO (this << " RNTI " << rnti << " new SRS-CQI for RB  " << j << " value " << sinr);


				// PW_EDIT
				if ((Now().GetSeconds() >= 0.00)
						&& (m_pwEnbId == 10)
						&& false) {
					std::cout << "PW_SCHEDULER " << Now().GetSeconds()
            													<< "s pw_PfFfMacScheduler::DoSchedUlCqiInfoReq "
																<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
																<< " sinr " << sinr
																<< std::endl;
				}



			}
			m_ueCqi.insert (std::pair <uint16_t, std::vector <double> > (rnti, newCqi));
			// generate correspondent timer
			m_ueCqiTimers.insert (std::pair <uint16_t, uint32_t > (rnti, m_cqiTimersThreshold));
		}
		else
		{
			// update the values
			for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
			{
				double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (j));
				(*itCqi).second.at (j) = sinr;
				NS_LOG_INFO (this << " RNTI " << rnti << " update SRS-CQI for RB  " << j << " value " << sinr);
			}
			// update correspondent timer
			std::map <uint16_t, uint32_t>::iterator itTimers;
			itTimers = m_ueCqiTimers.find (rnti);
			(*itTimers).second = m_cqiTimersThreshold;

		}
		updItbsUlcqi(rnti,0,m_cschedCellConfig.m_ulBandwidth-2);
		std::map <uint16_t,PfsUlUe>::iterator it_pfs;
		std::map <uint16_t,ueInfo>::iterator ue;
		ue = ueUl.find (rnti);
		it_pfs = PfsUlUes.find(rnti);
		it_pfs->second.cqiPrio = PfsUlGetTpPrio(globalPfs, ue->second.itbs_index);
		std::map <uint16_t, PfsUlLcgs>::iterator it_buffers;
		it_buffers = m_ceBsr_LCG_Rxed.find(rnti);

		// PW_EDIT: Check if RNTI exists
		if (it_buffers != m_ceBsr_LCG_Rxed.end())
		{
			PfsUlMngUeInQ(it_buffers->second, rnti);
		}
		else
		{
			NS_LOG_LOGIC (this << " RNTI " << rnti << " not found!");
		}

		// PW_EDIT: debugging bug here:
		if (false) {
			std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s: pw_PfFfMacScheduler::DoSchedUlCqiInfoReq:"
					<< " eNB " << m_pwEnbId << " RNTI " << rnti
					<< " m_ceBsr_LCG_Rxed empty? " << m_ceBsr_LCG_Rxed.empty()
					<< " size " << m_ceBsr_LCG_Rxed.size()
					<< "size UlLcgsArray[0].LcsInfo.size() = "
					<< (((it_buffers->second).UlLcgsArray[0].LcsInfo)).size()
					<< std::endl;
		}

	}


	break;
	case UlCqi_s::PUCCH_1:
	case UlCqi_s::PUCCH_2:
	case UlCqi_s::PRACH:
	{
		NS_FATAL_ERROR ("pw_PfFfMacScheduler supports only PUSCH and SRS UL-CQIs");
	}
	break;
	default:
		NS_FATAL_ERROR ("Unknown type of UL-CQI");
	}





	return;
}


std::map <uint16_t,PfsUlUe>::iterator
pw_PfFfMacScheduler::FindUe (uint16_t rnti){

	return (PfsUlUes.find(rnti));

}

void
pw_PfFfMacScheduler::RefreshDlCqiMaps (void)
{
	// refresh DL CQI P01 Map
	std::map <uint16_t,uint32_t>::iterator itP10 = m_p10CqiTimers.begin ();
	while (itP10 != m_p10CqiTimers.end ())
	{
		NS_LOG_INFO (this << " P10-CQI for user " << (*itP10).first << " is " << (uint32_t)(*itP10).second << " thr " << (uint32_t)m_cqiTimersThreshold);
		if ((*itP10).second == 0)
		{
			// delete correspondent entries
			std::map <uint16_t,uint8_t>::iterator itMap = m_p10CqiRxed.find ((*itP10).first);
			NS_ASSERT_MSG (itMap != m_p10CqiRxed.end (), " Does not find CQI report for user " << (*itP10).first);
			NS_LOG_INFO (this << " P10-CQI expired for user " << (*itP10).first);
			m_p10CqiRxed.erase (itMap);
			std::map <uint16_t,uint32_t>::iterator temp = itP10;
			itP10++;
			m_p10CqiTimers.erase (temp);
		}
		else
		{
			(*itP10).second--;
			itP10++;
		}
	}

	// refresh DL CQI A30 Map
	std::map <uint16_t,uint32_t>::iterator itA30 = m_a30CqiTimers.begin ();
	while (itA30 != m_a30CqiTimers.end ())
	{
		NS_LOG_INFO (this << " A30-CQI for user " << (*itA30).first << " is " << (uint32_t)(*itA30).second << " thr " << (uint32_t)m_cqiTimersThreshold);
		if ((*itA30).second == 0)
		{
			// delete correspondent entries
			std::map <uint16_t,SbMeasResult_s>::iterator itMap = m_a30CqiRxed.find ((*itA30).first);
			NS_ASSERT_MSG (itMap != m_a30CqiRxed.end (), " Does not find CQI report for user " << (*itA30).first);
			NS_LOG_INFO (this << " A30-CQI expired for user " << (*itA30).first);
			m_a30CqiRxed.erase (itMap);
			std::map <uint16_t,uint32_t>::iterator temp = itA30;
			itA30++;
			m_a30CqiTimers.erase (temp);
		}
		else
		{
			(*itA30).second--;
			itA30++;
		}
	}

	return;
}


void
pw_PfFfMacScheduler::RefreshUlCqiMaps (void)
{
	// refresh UL CQI  Map
	std::map <uint16_t,uint32_t>::iterator itUl = m_ueCqiTimers.begin ();
	while (itUl != m_ueCqiTimers.end ())
	{
		NS_LOG_INFO (this << " UL-CQI for user " << (*itUl).first << " is " << (uint32_t)(*itUl).second << " thr " << (uint32_t)m_cqiTimersThreshold);
		if ((*itUl).second == 0)
		{
			// delete correspondent entries
			std::map <uint16_t, std::vector <double> >::iterator itMap = m_ueCqi.find ((*itUl).first);
			NS_ASSERT_MSG (itMap != m_ueCqi.end (), " Does not find CQI report for user " << (*itUl).first);
			NS_LOG_INFO (this << " UL-CQI exired for user " << (*itUl).first);
			(*itMap).second.clear ();
			m_ueCqi.erase (itMap);
			std::map <uint16_t,uint32_t>::iterator temp = itUl;
			itUl++;
			m_ueCqiTimers.erase (temp);
		}
		else
		{
			(*itUl).second--;
			itUl++;
		}
	}

	return;
}

void
pw_PfFfMacScheduler::UpdateDlRlcBufferInfo (uint16_t rnti, uint8_t lcid, uint16_t size)
{
	std::map<LteFlowId_t, FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
	LteFlowId_t flow (rnti, lcid);
	it = m_rlcBufferReq.find (flow);
	if (it != m_rlcBufferReq.end ())
	{
		NS_LOG_INFO (this << " UE " << rnti << " LC " << (uint16_t)lcid << " txqueue " << (*it).second.m_rlcTransmissionQueueSize << " retxqueue " << (*it).second.m_rlcRetransmissionQueueSize << " status " << (*it).second.m_rlcStatusPduSize << " decrease " << size);
		// Update queues: RLC tx order Status, ReTx, Tx
		// Update status queue
		if (((*it).second.m_rlcStatusPduSize > 0) && (size >= (*it).second.m_rlcStatusPduSize))
		{
			(*it).second.m_rlcStatusPduSize = 0;
		}
		else if (((*it).second.m_rlcRetransmissionQueueSize > 0) && (size >= (*it).second.m_rlcRetransmissionQueueSize))
		{
			(*it).second.m_rlcRetransmissionQueueSize = 0;
		}
		else if ((*it).second.m_rlcTransmissionQueueSize > 0)
		{
			uint32_t rlcOverhead;
			if (lcid == 1)
			{
				// for SRB1 (using RLC AM) it's better to
				// overestimate RLC overhead rather than
				// underestimate it and risk unneeded
				// segmentation which increases delay
				rlcOverhead = 4;
			}
			else
			{
				// minimum RLC overhead due to header
				rlcOverhead = 2;
			}
			// update transmission queue
			if ((*it).second.m_rlcTransmissionQueueSize <= size - rlcOverhead)
			{
				(*it).second.m_rlcTransmissionQueueSize = 0;
			}
			else
			{
				(*it).second.m_rlcTransmissionQueueSize -= size - rlcOverhead;
			}
		}
	}
	else
	{
		NS_LOG_ERROR (this << " Does not find DL RLC Buffer Report of UE " << rnti);
	}
}

void
pw_PfFfMacScheduler::UpdateUlRlcBufferInfo (uint16_t rnti, uint16_t size)
{

	size = size - 2; // remove the minimum RLC overhead
	std::map <uint16_t,uint32_t>::iterator it = m_ceBsrRxed.find (rnti);
	if (it != m_ceBsrRxed.end ())
	{
		NS_LOG_INFO (this << " UE " << rnti << " size " << size << " BSR " << (*it).second);
		if ((*it).second >= size)
		{
			(*it).second -= size;
		}
		else
		{
			(*it).second = 0;
		}
	}
	else
	{
		NS_LOG_ERROR (this << " Does not find BSR report info of UE " << rnti);
	}

}

void
pw_PfFfMacScheduler::TransmissionModeConfigurationUpdate (uint16_t rnti, uint8_t txMode)
{
	NS_LOG_FUNCTION (this << " RNTI " << rnti << " txMode " << (uint16_t)txMode);
	FfMacCschedSapUser::CschedUeConfigUpdateIndParameters params;
	params.m_rnti = rnti;
	params.m_transmissionMode = txMode;
	m_cschedSapUser->CschedUeConfigUpdateInd (params);
}
void
pw_PfFfMacScheduler::SetRemPuschDeltaSinr(uint8_t sinr, uint16_t rnti)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);

	/*
	 * TRUE if received at list one PHR report from the UE
	 * If not, we can't ask UE to increase power, only to reduce
	 * as we don't know its current state
	 */
	if(ue->second.isPhrAvail)
	{
		// maxPwrPerRb = maxUePwr - PwrRbToPwrdBTbl[MinimumRBs == 3]
		ue->second.maxPwrDeltaByPhr = (MAX_UE_PWR - PwrRbToPwrdBTbl[MIN_MAX_RBS]) - ue->second.pwrPerRb;
	}
	else
	{
		ue->second.maxPwrDeltaByPhr = 0;
	}

	//If we have no delta power for RB and we need to increase UE power.
	if(ue->second.maxPwrDeltaByPhr < 0 && (ue->second.trgSinr - sinr) > 0)
	{
		ue->second.remPuschPwr = 0;
		return;
	}

	ue->second.remPuschPwr = GET_MIN(ue->second.maxPwrDeltaByPhr,(ue->second.trgSinr - sinr)/2);
}
void
pw_PfFfMacScheduler::SetRemPuschDelta(uint8_t cqi, uint16_t rnti)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);

	/*
	 * TRUE if received at list one PHR report from the UE
	 * If not, we can't ask UE to increase power, only to reduce
	 * as we don't know its current state
	 */
	if(ue->second.isPhrAvail)
	{
		// maxPwrPerRb = maxUePwr - PwrRbToPwrdBTbl[MinimumRBs == 3]
		ue->second.maxPwrDeltaByPhr = (MAX_UE_PWR - PwrRbToPwrdBTbl[3]) - ue->second.pwrPerRb;
	}
	else
	{
		ue->second.maxPwrDeltaByPhr = 0;
	}

	//If we have no delta power for RB and we need to increase UE power.
	if(ue->second.maxPwrDeltaByPhr < 0 && ((ue->second.trgCqi - ue->second.trgCqiDelta) - cqi)*2 > 0)
	{
		ue->second.remPuschPwr = 0;
		return;
	}

	/*
	 * Correct the minimum between the two as we can't
	 * fix more than maxPwrDeltaByPhr if needed
	 */
	ue->second.remPuschPwr = GET_MIN(ue->second.maxPwrDeltaByPhr,((ue->second.trgCqi - ue->second.trgCqiDelta) - cqi)*2);
}
void
pw_PfFfMacScheduler::SetTpc(uint16_t rnti, int8_t availPwr){

	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);

	uint8_t            tpcs[3]   = {1, 2, 2};
	uint8_t            deltas[3] = {0, 1, 1};
	int8_t delta;

	//COOL_OFF_PERIOD_DURATION passed mean UE already updated with last TPC
	if(tti_counter >= ue->second.lastTpcAppliedTick + COOL_OFF_PERIOD_DURATION)
	{
		//if ue->second.remPuschPwr > availPwr we can fix more than we have
		delta = GET_MIN(ue->second.remPuschPwr,availPwr);
		//The UE transmit on lowest power already, we don't need to reduce more
		if(ue->second.phr >= 40 && delta <= -1)
		{
			ue->second.tpc = 1;
			ue->second.delta = 0;
		}
		if(delta <= -1)
		{
			ue->second.tpc = 0;
			ue->second.delta = -1;
		}
		else if(delta >= 3)
		{
			ue->second.tpc = 3;
			ue->second.delta = 3;
		}
		else
		{
			ue->second.tpc = tpcs[delta];
			ue->second.delta = deltas[delta];
		}
	}
	//Wait for UE to update last TPC
	else
	{
		ue->second.tpc = 1;
		ue->second.delta = 0;
	}

	if (false) {
		std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::SetTpc "
				<< " eNB " << m_pwEnbId	<< " RNTI " << rnti
				<< " availPwr " << int(availPwr)
				<< " tpc " << unsigned(ue->second.tpc) << " delta " << int(ue->second.delta)
				<< std::endl;
	}

}
uint8_t pw_PfFfMacScheduler::GetPwrGetMaxRb(int8_t availPwr,uint16_t offsetDB = 0)
{
	uint8_t minRB;
	if(availPwr <= 0)
	{
		return PwrToRbTbl[0];
	}

	if(availPwr >= PwrRbToPwrdBTbl[m_cschedCellConfig.m_ulBandwidth])
	{
		return m_cschedCellConfig.m_ulBandwidth;
	}
	//when tti bundling enabled offset will be added in the function interface
	availPwr+=offsetDB;
	minRB = m_cschedCellConfig.m_ulBandwidth <= PwrToRbTbl[availPwr] ? m_cschedCellConfig.m_ulBandwidth : PwrToRbTbl[availPwr];
	return minRB;
}

void
pw_PfFfMacScheduler::cfgUe(uint16_t rnti)
{
	ueInfo ue;
	ue.delta_itbs = 0;
	// RAR Itbs
	ue.avgItbs = m_amc->GetItbsFromMcsUl(m_ulGrantMcs);//m_amc->GetUlMcsToItbs(m_ulGrantMcs);
	// No power to correct at init
	ue.phr = 40; //as PW
	// Set maximum power per RB at the init
	ue.pwrPerRb = MAX_UE_PWR - PwrRbToPwrdBTbl[MIN_MAX_RBS];
	// Max RBs allocation
	ue.RbAllocation = m_cschedCellConfig.m_ulBandwidth;
	//no power correction for the beginning
	ue.delta = 0;
	ue.tpc = 1;
	ue.remPuschPwr = 0;
	//start from less RB till first PHR is recieved
	ue.isPhrAvail = false;
	//set for TPC cool down
	ue.lastTpcAppliedTick = tti_counter;
	ue.fi = 0;
	ue.avg_sinr = 0;
	ue.maxPwrDeltaByPhr = 0;
	ue.mcs_index = 0;
	ue.itbs_index = 0;
	ue.cqi_index = 0;
	ue.delta_sinr = 0;
	//default 16Qam uplink support
	ue.enable64Qam = Is64Qam(UP_64_QAM_PRECENTAGE);
	if(!ue.enable64Qam)
	{
		ue.maxUlCqi = MAX_UL_CQI_16Q; // CQI = 11
		ue.trgSinr = UL_16QAM_TARGET_SINR;
	}
	else
	{
		ue.maxUlCqi = UL_NUM_CQI - 2; // CQI = 14
		ue.trgSinr = UL_64QAM_TARGET_SINR;
	}
	ue.trgCqi = ue.maxUlCqi;
	ue.trgCqiDelta = 0;
	uint8_t targerBler = TARGET_BLER;
	if (m_volteOn)
	{
		ue.volteOn = true;
		targerBler = 1; //for VoLTE we set target BLER to 1
	}
	else
	{
		ue.volteOn = false;
	}

	if (m_ttiBundlingOn)
	{
		ue.isTTIBundling = true;
	}
	else
	{
		ue.isTTIBundling = false;
	}

	uint8_t ratio_up = 100 - targerBler;
	ue.sinr_margin_step_down = MARGIN_STEP_DOWN*2; // divide by 100 to get dB, multiply by 200 for not dB steps. [0,..,100]

	if(!targerBler)
	{
		ue.sinr_margin_step_up = 200;
		ue.sinr_margin_step_down = 1;
	}
	else if (targerBler == 1)
	{
		ue.sinr_margin_step_up = 198;
		ue.sinr_margin_step_down = 2;
	}
	else
	{
		ue.sinr_margin_step_up = (ue.sinr_margin_step_down*ratio_up)/targerBler;
	}

	ueUl.insert ( std::pair<uint16_t, ueInfo > (rnti, ue));
}

int8_t
pw_PfFfMacScheduler::GetUeAvailPwr(uint16_t rnti)
{
	int8_t availPwr;
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);

	//If PHR report received already
	if(ue->second.isPhrAvail)
	{
		availPwr = MAX_UE_PWR - ue->second.pwrPerRb;
	}
	else
	{
		availPwr = 3;
	}

	if(eff_state == UE_EFF)
	{
		uint16_t best_EFF_bit,EFF_bit;
		uint8_t max_mcs,idx, availIdx,maxRB,Rb,mcs_Index = 0,tpc;
		int8_t temp_avail_pwr,delta;
		idx = MIN_MCS;

		max_mcs = ue->second.enable64Qam ? 28 : 23;
		ue->second.remPuschPwr = ue->second.gain_diff_per_tb_avg[idx];
		this->SetTpc(rnti,availPwr);
		tpc = ue->second.tpc;
		temp_avail_pwr = availPwr - ue->second.delta;
		if (ue->second.isTTIBundling)
		{
			maxRB = GetPwrGetMaxRb(temp_avail_pwr,this->m_ttiBundlingOffsetDB);
		}
		else
		{
			maxRB = GetPwrGetMaxRb(temp_avail_pwr,0);
		}
		maxRB = GET_MAX(maxRB,MIN_MAX_RBS);
		best_EFF_bit = MCS_eff[idx]*maxRB;
		idx++;
		for(; idx <= max_mcs ; idx++)
		{
			if((availPwr < ue->second.gain_diff_per_tb_avg[idx]) || ue->second.gain_diff_per_tb_avg[idx]>3)
			{
				break;
			}
		}

		for(availIdx=1;availIdx<idx;availIdx++){
			ue->second.remPuschPwr = ue->second.gain_diff_per_tb_avg[availIdx];
			this->SetTpc(rnti, availPwr);
			temp_avail_pwr = availPwr - ue->second.delta;
			Rb = GetPwrGetMaxRb(temp_avail_pwr);
			Rb = GET_MAX(Rb,MIN_MAX_RBS);
			EFF_bit = Rb*MCS_eff[availIdx];
			if(best_EFF_bit<EFF_bit){
				best_EFF_bit = EFF_bit;
				maxRB = Rb;
				mcs_Index = availIdx;
				delta = ue->second.delta;
				tpc = ue->second.tpc;
			}
		}

		ue->second.tpc = tpc;
		ue->second.delta = delta;
		ue->second.mcs_index = mcs_Index;
		availPwr -= delta;
		return availPwr;
	}
	else
	{
		this->SetTpc(rnti,availPwr);
		availPwr -= ue->second.delta;
		//reduce,increase availPwr as TPC sent to fix
		if(ue->second.enable64Qam && eff_state != NEW_SPECTRAL_EFF)
		{
			ue->second.trgCqiDelta = m_amc->GetTrgCqiDelta(availPwr);
			availPwr += ue->second.trgCqiDelta*2;
		}

		if (false) {
			std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::GetUeAvailPwr "
					<< " TPC sent to reduce/increase availPwr "
					<< " eNB " << m_pwEnbId	<< " RNTI " << rnti
					<< " availPwr " << int(availPwr) << " (MAX_UE_PWR=" << MAX_UE_PWR << ") pwPerRb " << ue->second.pwrPerRb
					<< std::endl;
		}

		return availPwr;
	}

	if (false) {
		std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::GetUeAvailPwr "
				<< " eNB " << m_pwEnbId	<< " RNTI " << rnti
				<< " availPwr " << availPwr << " MAX_UE_PWR " << MAX_UE_PWR << " pwPerRb " << ue->second.pwrPerRb
				<< std::endl;
	}

}

void
pw_PfFfMacScheduler::SetTargetCqi(uint16_t rnti)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);
	int16_t path_loss = 0;
	uint8_t idx = 0, deltaTFAndTenLogM = 10*log10(1.0*ue->second.RbAllocation);

	path_loss = (MAX_UE_PWR - ue->second.phr - P_O_NOMINAL_PUSCH - deltaTFAndTenLogM - ue->second.fi);

	NS_LOG_INFO (Now().GetMilliSeconds()<<" UE " << rnti << " eNB " << (uint16_t)m_pwEnbId << " path_loss "<<path_loss);
	if((path_loss >= 0 && path_loss <= PathLoss[idx]))
	{
		ue->second.trgCqi = TargetCqi[idx];
	}
	else if( path_loss <= PathLoss[++idx])
	{
		ue->second.trgCqi = TargetCqi[idx];
	}
	else if(path_loss <= PathLoss[++idx])
	{
		ue->second.trgCqi = TargetCqi[idx];
	}
	else if(path_loss <= PathLoss[++idx])
	{
		ue->second.trgCqi = TargetCqi[idx];
	}
	else
	{
		ue->second.trgCqi = TargetCqi[idx];
	}

	if(ue->second.enable64Qam)
	{
		ue->second.trgCqi += 3;
	}
	NS_LOG_ERROR(this << " PATHLOSS: " << path_loss);
}

bool
pw_PfFfMacScheduler::Is64Qam(uint8_t precent)
{
	uint8_t rand = (random()%100) + 1;
	if(rand <= precent)
	{
		return true;
	}
	return false;
}

uint8_t
pw_PfFfMacScheduler::UlGetItbs(uint16_t rnti)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);
	uint8_t iTbs,maxItbs;
	iTbs = (ue->second.avgItbs+ue->second.delta_itbs)/100;
	maxItbs = m_amc->GetItbsFromCqi(ue->second.maxUlCqi);


	if (false) {
		std::cout << "DEBUG_ULLA " << Now().GetSeconds() << "s pw_PfFfMacScheduler::UlGetItbs "
				<< " eNB " << m_pwEnbId	<< " RNTI " << rnti
				<< " UlCQI " << ue->second.maxUlCqi << " maxItbs " << maxItbs
				<< std::endl;
	}

	return GET_MIN(iTbs,maxItbs);
}

uint8_t
pw_PfFfMacScheduler::GetMaxRbs(uint16_t rnti)
{
	//Check TPC for TARGET_CQI
	int8_t availPwr;
	uint16_t rbPerFlow;
	availPwr = GetUeAvailPwr(rnti);
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);

	// If there is a power change, start to count 4 TTIs for UE update
	//if(ue->second.tpc != 1)
	//{
	//ue->second.lastTpcAppliedTick = tti_counter;
	//}

	// If reducing power, update the pwrPerRb
	if(ue->second.tpc == 0 && eff_state != UE_EFF)
	{
		ue->second.pwrPerRb += ue->second.delta;
	}

	ue->second.maxRb = this->GetPwrGetMaxRb(availPwr);
	rbPerFlow = ue->second.maxRb;

	//As in Radysis
	rbPerFlow = GET_MIN(rbPerFlow,m_cschedCellConfig.m_ulBandwidth-2);
	rbPerFlow = GET_MAX(rbPerFlow,MIN_MAX_RBS);
	return rbPerFlow;
}
void
pw_PfFfMacScheduler::MaxThroughput(uint16_t rnti, double minSinr)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);
	uint8_t max_mcs,idx,sinr_no_db;
	sinr_no_db = (minSinr*2)+128;
	if(!ue->second.avg_sinr)
	{
		ue->second.avg_sinr = sinr_no_db*100;
	}
	if(sinr_no_db > ue->second.avg_sinr){
		ue->second.avg_sinr = (((FACTOR-ALFA_HIGH)*ue->second.avg_sinr) + (ALFA_HIGH*sinr_no_db*100))/FACTOR;
	}
	else
	{
		ue->second.avg_sinr = (((FACTOR-ALFA_LOW)*ue->second.avg_sinr) + (ALFA_LOW*sinr_no_db*100))/FACTOR;
	}

	//ue->second.avg_sinr = (ue->second.avg_sinr*0.8 + sinr_no_db*20);

	max_mcs = ue->second.enable64Qam ? 28 : 23;

	for(idx=0; idx <= max_mcs; idx++)
	{
		ue->second.gain_diff_per_tb_avg[idx] = (sinr_target_per_cqi[idx]*100 - ue->second.avg_sinr + ue->second.delta_sinr)/200;
	}
	ue->second.itbs_index = m_amc->GetItbsFromMcsUl(ue->second.mcs_index);
	ue->second.cqi_index = m_amc->GetCqiFromMcs(ue->second.mcs_index);

	// PW_EDIT
	if (  //(Now().GetSeconds() >= 8.00) &&
			false) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::MaxThroughput "
													<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
													<< " minSinr " << minSinr
													<< " ue->second.mcs_index " << int(ue->second.mcs_index)
													<< " ue->second.itbs_index " << int(ue->second.itbs_index)
													<< " ue->second.cqi_index " << int(ue->second.cqi_index)
													<< std::endl;
		std::cout << "break -1" << std::endl;
	}

}

void
pw_PfFfMacScheduler::SpectralEfficiencyCqi(uint16_t rnti, double minSinr)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);
	ue->second.mcs_index = m_amc->GetMcsFromSinr(minSinr);
	ue->second.cqi_index = m_amc->GetCqiFromMcs(ue->second.mcs_index);

	if(ue->second.maxRb < 3)
	{
		if(ue->second.maxPwrDeltaByPhr < 0)
		{
			if(ue->second.cqi_index + ue->second.maxPwrDeltaByPhr < 1)
			{
				ue->second.cqi_index = 1;
			}
		}
		else
		{
			ue->second.cqi_index += ue->second.maxPwrDeltaByPhr;
		}
	}

	ue->second.itbs_index = m_amc->GetItbsFromCqi(ue->second.cqi_index);

	// for first time.
	if(!ue->second.avgItbs)
		ue->second.avgItbs = ue->second.itbs_index*100;

	// average itbs
	ue->second.avgItbs = (ue->second.avgItbs*80 + ue->second.itbs_index*20*100)/100; // + delta);// + delta;

	ue->second.itbs_index = UlGetItbs(ue->first);
	ue->second.mcs_index = m_amc->GetMcsFromItbs(ue->second.itbs_index,ue->second.enable64Qam);
	SetRemPuschDelta(ue->second.cqi_index, ue->first);


	// PW_EDIT
	if (  //(Now().GetSeconds() >= 8.00) &&
			false) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::SpectralEfficiencyCqi "
													<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
													<< " minSinr " << minSinr
													<< " ue->second.mcs_index " << int(ue->second.mcs_index)
													<< " ue->second.itbs_index " << int(ue->second.itbs_index)
													<< " ue->second.cqi_index " << int(ue->second.cqi_index)
													<< std::endl;
		std::cout << "break -2" << std::endl;
	}


}
void
pw_PfFfMacScheduler::updItbsUlcqi(uint16_t rnti, uint8_t rbStart, uint8_t rbEnd)
{
	std::map <uint16_t, std::vector <double> >::iterator itCqi = m_ueCqi.find (rnti);
	double minSinr = (*itCqi).second.at (rbStart);
	if (minSinr == NO_SINR)
	{
		minSinr = EstimateUlSinr (rnti, rbStart);
	}
	for (uint16_t i = rbStart; i < rbEnd; i++)
	{
		double sinr = (*itCqi).second.at (i);
		if (sinr == NO_SINR)
		{
			sinr = EstimateUlSinr (rnti, i);
		}
		if (sinr < minSinr && sinr != -4096)
		{
			minSinr = sinr;
		}
	}


	// PW_EDIT
	if ((Now().GetSeconds() >= 0.00)
			&& (m_pwEnbId == 10)
			&&	false) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::updItbsUlcqi "
													<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
													<< " minSinr " << minSinr
													<< " rbStart " << int(rbStart)
													<< " rbEnd " << int(rbEnd)
													<< std::endl;
		std::cout << "break -3" << std::endl;
	}


	switch(eff_state)
	{
	case UE_EFF:
		MaxThroughput(rnti, minSinr);
		break;
	case SPECTRAL_EFF:
		SpectralEfficiencyCqi(rnti, minSinr);
		break;
	case NEW_SPECTRAL_EFF:
		SpectralEfficiencySinr(rnti, minSinr);
		break;
	}
}

void
pw_PfFfMacScheduler::SpectralEfficiencySinr(uint16_t rnti, double minSinr)
{
	std::map <uint16_t,ueInfo>::iterator ue = ueUl.find(rnti);
	int32_t effectiveSinr;
	uint8_t sinr_no_db;
	sinr_no_db = (minSinr*2)+128;
	if(!ue->second.avg_sinr)
	{
		ue->second.avg_sinr = sinr_no_db*100;
	}
	if(sinr_no_db > ue->second.avg_sinr){
		ue->second.avg_sinr = (((FACTOR-ALFA_HIGH)*ue->second.avg_sinr) + (ALFA_HIGH*sinr_no_db*100))/FACTOR;
	}
	else
	{
		ue->second.avg_sinr = (((FACTOR-ALFA_LOW)*ue->second.avg_sinr) + (ALFA_LOW*sinr_no_db*100))/FACTOR;
	}

	effectiveSinr = (ue->second.avg_sinr - ue->second.delta_sinr) /100;
	ue->second.itbs_index = UlSinrToItbsTbl[effectiveSinr];

	if(!ue->second.enable64Qam && ue->second.itbs_index > UL_16QAM_MAX_ITBS)
	{
		ue->second.itbs_index = UL_16QAM_MAX_ITBS;
	}

	//Benny force robust MCS
	if(ue->second.itbs_index > 10 && ue->second.volteOn)
	{
		ue->second.itbs_index = 10;
	}

	ue->second.mcs_index = m_amc->GetMcsFromItbs(ue->second.itbs_index, ue->second.enable64Qam);
	ue->second.cqi_index = m_amc->GetCqiFromMcs(ue->second.mcs_index);
	SetRemPuschDeltaSinr(effectiveSinr, ue->first);

	// PW_EDIT
	if ((Now().GetSeconds() >= 0.00)
			&& (m_pwEnbId == 10)
			&& false) {
		std::cout << "PW_SCHEDULER " << Now().GetSeconds()
													<< "s pw_PfFfMacScheduler::SpectralEfficiencySinr "
													<< " eNB " << m_pwEnbId << " UE RNTI " << rnti
													<< " minSinr " << minSinr
													<< " ue->second.mcs_index " << (int)ue->second.mcs_index
													<< " ue->second.itbs_index " << (int)ue->second.itbs_index
													<< " ue->second.cqi_index " << (int)ue->second.cqi_index
													<< std::endl;
		std::cout << "break -3" << std::endl;
	}



}


// PW_EDIT
void
pw_PfFfMacScheduler::DoSetPwParams (uint16_t pwEnbId)
{
	m_pwEnbId = pwEnbId;
}


// PW Scheduler Stats File
void pw_PfFfMacScheduler::ReportUlSchedulerStat (uint16_t rnti, uint32_t rbStart,
		uint32_t rbAllocated, uint32_t mcsIndex, uint16_t tbSize)
{
	std::string fileName = "ulSchedulerStats.txt";
	std::ofstream outFile;

	if (m_schedulerStatsFirstWrite == true)
	{
		outFile.open (fileName.c_str());
		if (!outFile.is_open())
		{
			NS_LOG_ERROR ("Can't open file " << fileName.c_str());
			return;
		}

		m_schedulerStatsFirstWrite = false;
		outFile << "% PW Radisys Scheduler" << std::endl;
		outFile << "% time\tcellId\tRNTI\tallocatedRBs\tstartRB\tendRB\tmcsIndex\ttbSize";
		outFile << std::endl;
	}
	else
	{
		outFile.open(fileName.c_str(), std::ios_base::app);
		if (!outFile.is_open())
		{
			NS_LOG_ERROR ("Can't open file " << fileName.c_str());
			return;
		}
	}

	//
	outFile << Simulator::Now().GetSeconds () << "\t";
	outFile << m_pwEnbId << "\t";
	outFile << rnti << "\t";
	outFile << rbAllocated << "\t";
	outFile << rbStart << "\t";
	outFile << (rbStart + rbAllocated - 1) << "\t";
	outFile << mcsIndex << "\t";
	outFile << tbSize << "\t";
	outFile << std::endl;
	outFile.close();


	// PW_EDIT PW_WKING_VFT
	if (  //(Now().GetSeconds() >= 8.00) &&
			false) {
		std::cout << "PW_SCHEDULER_ReportStat " << Now().GetSeconds()
													<< "s " << " eNB "
													<< m_pwEnbId << " UE RNTI " << rnti
													<< " PRBs " << rbAllocated << " [" << rbStart << "-"
													<< (rbStart + rbAllocated - 1) << "]"
													<< " MCS " << mcsIndex << " TBsize " << tbSize
													<< std::endl;
		std::cout << "break" << std::endl;
	}


}



}
