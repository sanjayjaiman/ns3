/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
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
 * Original Author: Giuseppe Piro  <g.piro@poliba.it>
 * Modified by:     Marco Miozzo   <mmiozzo@cttc.es>
 *                  Nicola Baldo   <nbaldo@cttc.es>
 */


#include <ns3/lte-amc.h>
#include <ns3/log.h>
#include <ns3/assert.h>
#include <ns3/math.h>
#include <vector>
#include <ns3/spectrum-value.h>
#include <ns3/double.h>
#include "ns3/enum.h"
#include <ns3/lte-mi-error-model.h>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteAmc");

NS_OBJECT_ENSURE_REGISTERED (LteAmc);

/**
 * Table of CQI index and its spectral efficiency. Taken from 3GPP TSG-RAN WG1
 * [R1-081483 Conveying MCS and TB size via PDCCH]
 * (http://www.3gpp.org/ftp/tsg_ran/WG1_RL1/TSGR1_52b/Docs/R1-081483.zip)
 * file `TBS_support.xls` tab "MCS Table" (rounded to 2 decimal digits).
 * The index of the vector (range 0-15) identifies the CQI value.
 */
static const double SpectralEfficiencyForCqi[16] = {
  0.0, // out of range
  0.15, 0.23, 0.38, 0.6, 0.88, 1.18,
  1.48, 1.91, 2.41,
  2.73, 3.32, 3.9, 4.52, 5.12, 5.55
};

#if 0 // currently unused
/**
 * Table of MCS index (IMCS) and its TBS index (ITBS). Taken from 3GPP TS
 * 36.213 v8.8.0 Table 7.1.7.1-1: _Modulation and TBS index table for PDSCH_.
 * The index of the vector (range 0-31; valid values 0-28) identifies the MCS
 * index. Note that this is similar to the one in R1-081483, but:
 * - a few values are different; and
 * - in R1-081483, a valid MCS index is in the range of 1-30 (not 0-28).
 */
static const int ModulationSchemeForMcs[32] = {
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
  4, 4, 4, 4, 4, 4, 4,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  2,  // reserved
  4,  // reserved
  6,  // reserved
};
#endif

/**
 * Table of MCS index and its spectral efficiency. Taken from 3GPP TSG-RAN WG1
 * [R1-081483 Conveying MCS and TB size via PDCCH]
 * (http://www.3gpp.org/ftp/tsg_ran/WG1_RL1/TSGR1_52b/Docs/R1-081483.zip)
 * file `TBS_support.xls` tab "MCS Table" (rounded to 2 decimal digits).
 * The index of the vector (range 0-31) corresponds to the MCS index according
 * to the convention in TS 36.213 (i.e., the MCS index reported in R1-081483
 * minus one)
 */
static const double SpectralEfficiencyForMcs[32] = {
  0.15, 0.19, 0.23, 0.31, 0.38, 0.49, 0.6, 0.74, 0.88, 1.03, 1.18,
  1.33, 1.48, 1.7, 1.91, 2.16, 2.41, 2.57,
  2.73, 3.03, 3.32, 3.61, 3.9, 4.21, 4.52, 4.82, 5.12, 5.33, 5.55,
  0, 0, 0
};

/**
 * Table of MCS index (IMCS) and its TBS index (ITBS). Taken from 3GPP TS
 * 36.213 v8.8.0 Table 7.1.7.1-1: _Modulation and TBS index table for PDSCH_.
 * The index of the vector (range 0-28) identifies the MCS index.
 */
static const int McsToItbsDl[29] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 15, 16, 17, 18,
  19, 20, 21, 22, 23, 24, 25, 26
};

/**
 * Table of MCS index (IMCS) and its TBS index (ITBS). Taken from 3GPP TS
 * 36.213 v8.8.0 Table 8.6.1-1: _Modulation, TBS index and redundancy version table for PUSCH_.
 * The index of the vector (range 0-28) identifies the MCS index.
 */
static const int McsToItbsUl[29] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18,
  19, 19, 20, 21, 22, 23, 24, 25, 26
};

/*
 * Map UL_CQI [0,..,255] to MCS [0,..,28] according to Radysis
 * can be found at lteclms/ys_ms_ul.c
 */
static int SinrToMcs[256];

/*
 * Map MCS [0,..,28] to CQI [0,..,15] according to Radysis
 * can be found at lteclms/ys_ms_ul.c
 */
static int McsToCqi[29];

/*
 * Map CQI [0,..,15] to ITBS [0,..,26] according to Radysis
 * can be found at ltemac/rg_sch_cmn.c build under the function rgSCHCmnUlInit
 */
static uint8_t UlCqiToiTbsTbl[16];

/*
 * UL_EFF_CQI
 * can be found at ltemac/rg_sch_cmn.c rgSchCmnUlCqiTbl
 */
static const int EFF[16] = {              //3402
  0,156,240,386,616,898,1204,1512,1960,2464,3402,3800,4102,4342,5238,5728
};


/**
 * Table of number of physical resource blocks (NPRB), TBS index (ITBS), and
 * their associated transport block size. Taken from 3GPP TS 36.213 v8.8.0
 * Table 7.1.7.2.1-1: _Transport block size table (dimension 27×110)_.
 * \note For NPRB = 1 and ITBS = 6 the standard returns 328, but it is not
 *       consistent with the other values, therefore we use 88 obtained by
 *       following the sequence of NPRB = 1 values.
 */
static const int TransportBlockSizeTable [110][27] = {
  /* NPRB 001*/ { 16, 24, 32, 40, 56, 72, 88, 104, 120, 136, 144, 176, 208, 224, 256, 280, 328, 336, 376, 408, 440, 488, 520, 552, 584, 616, 712},
  /* NPRB 002*/ { 32, 56, 72, 104, 120, 144, 176, 224, 256, 296, 328, 376, 440, 488, 552, 600, 632, 696, 776, 840, 904, 1000, 1064, 1128, 1192, 1256, 1480},
  /* NPRB 003*/ { 56, 88, 144, 176, 208, 224, 256, 328, 392, 456, 504, 584, 680, 744, 840, 904, 968, 1064, 1160, 1288, 1384, 1480, 1608, 1736, 1800, 1864, 2216},
  /* NPRB 004*/ { 88, 144, 176, 208, 256, 328, 392, 472, 536, 616, 680, 776, 904, 1000, 1128, 1224, 1288, 1416, 1544, 1736, 1864, 1992, 2152, 2280, 2408, 2536, 2984},
  /* NPRB 005*/ { 120, 176, 208, 256, 328, 424, 504, 584, 680, 776, 872, 1000, 1128, 1256, 1416, 1544, 1608, 1800, 1992, 2152, 2344, 2472, 2664, 2856, 2984, 3112, 3752},
  /* NPRB 006*/ { 152, 208, 256, 328, 408, 504, 600, 712, 808, 936, 1032, 1192, 1352, 1544, 1736, 1800, 1928, 2152, 2344, 2600, 2792, 2984, 3240, 3496, 3624, 3752, 4392},
  /* NPRB 007*/ { 176, 224, 296, 392, 488, 600, 712, 840, 968, 1096, 1224, 1384, 1608, 1800, 1992, 2152, 2280, 2536, 2792, 2984, 3240, 3496, 3752, 4008, 4264, 4392, 5160},
  /* NPRB 008*/ { 208, 256, 328, 440, 552, 680, 808, 968, 1096, 1256, 1384, 1608, 1800, 2024, 2280, 2472, 2600, 2856, 3112, 3496, 3752, 4008, 4264, 4584, 4968, 5160, 5992},
  /* NPRB 009*/ { 224, 328, 376, 504, 632, 776, 936, 1096, 1256, 1416, 1544, 1800, 2024, 2280, 2600, 2728, 2984, 3240, 3624, 3880, 4136, 4584, 4776, 5160, 5544, 5736, 6712},
  /* NPRB 010*/ { 256, 344, 424, 568, 696, 872, 1032, 1224, 1384, 1544, 1736, 2024, 2280, 2536, 2856, 3112, 3240, 3624, 4008, 4264, 4584, 4968, 5352, 5736, 5992, 6200, 7480},
  /* NPRB 011*/ { 288, 376, 472, 616, 776, 968, 1128, 1320, 1544, 1736, 1928, 2216, 2472, 2856, 3112, 3368, 3624, 4008, 4392, 4776, 5160, 5544, 5992, 6200, 6712, 6968, 8248},
  /* NPRB 012*/ { 328, 424, 520, 680, 840, 1032, 1224, 1480, 1672, 1864, 2088, 2408, 2728, 3112, 3496, 3624, 3880, 4392, 4776, 5160, 5544, 5992, 6456, 6968, 7224, 7480, 8760},
  /* NPRB 013*/ { 344, 456, 568, 744, 904, 1128, 1352, 1608, 1800, 2024, 2280, 2600, 2984, 3368, 3752, 4008, 4264, 4776, 5160, 5544, 5992, 6456, 6968, 7480, 7992, 8248, 9528},
  /* NPRB 014*/ { 376, 488, 616, 808, 1000, 1224, 1480, 1672, 1928, 2216, 2472, 2792, 3240, 3624, 4008, 4264, 4584, 5160, 5544, 5992, 6456, 6968, 7480, 7992, 8504, 8760, 10296},
  /* NPRB 015*/ { 392, 520, 648, 872, 1064, 1320, 1544, 1800, 2088, 2344, 2664, 2984, 3368, 3880, 4264, 4584, 4968, 5352, 5992, 6456, 6968, 7480, 7992, 8504, 9144, 9528, 11064},
  /* NPRB 016*/ { 424, 568, 696, 904, 1128, 1384, 1672, 1928, 2216, 2536, 2792, 3240, 3624, 4136, 4584, 4968, 5160, 5736, 6200, 6968, 7480, 7992, 8504, 9144, 9912, 10296, 11832},
  /* NPRB 017*/ { 456, 600, 744, 968, 1192, 1480, 1736, 2088, 2344, 2664, 2984, 3496, 3880, 4392, 4968, 5160, 5544, 6200, 6712, 7224, 7992, 8504, 9144, 9912, 10296, 10680, 12576},
  /* NPRB 018*/ { 488, 632, 776, 1032, 1288, 1544, 1864, 2216, 2536, 2856, 3112, 3624, 4136, 4584, 5160, 5544, 5992, 6456, 7224, 7736, 8248, 9144, 9528, 10296, 11064, 11448, 13536},
  /* NPRB 019*/ { 504, 680, 840, 1096, 1352, 1672, 1992, 2344, 2664, 2984, 3368, 3880, 4392, 4968, 5544, 5736, 6200, 6712, 7480, 8248, 8760, 9528, 10296, 11064, 11448, 12216, 14112},
  /* NPRB 020*/ { 536, 712, 872, 1160, 1416, 1736, 2088, 2472, 2792, 3112, 3496, 4008, 4584, 5160, 5736, 6200, 6456, 7224, 7992, 8504, 9144, 9912, 10680, 11448, 12216, 12576, 14688},
  /* NPRB 021*/ { 568, 744, 936, 1224, 1480, 1864, 2216, 2536, 2984, 3368, 3752, 4264, 4776, 5352, 5992, 6456, 6712, 7480, 8248, 9144, 9912, 10680, 11448, 12216, 12960, 13536, 15264},
  /* NPRB 022*/ { 600, 776, 968, 1256, 1544, 1928, 2280, 2664, 3112, 3496, 3880, 4392, 4968, 5736, 6200, 6712, 7224, 7992, 8760, 9528, 10296, 11064, 11832, 12576, 13536, 14112, 16416},
  /* NPRB 023*/ { 616, 808, 1000, 1320, 1608, 2024, 2408, 2792, 3240, 3624, 4008, 4584, 5352, 5992, 6456, 6968, 7480, 8248, 9144, 9912, 10680, 11448, 12576, 12960, 14112, 14688, 16992},
  /* NPRB 024*/ { 648, 872, 1064, 1384, 1736, 2088, 2472, 2984, 3368, 3752, 4264, 4776, 5544, 6200, 6968, 7224, 7736, 8760, 9528, 10296, 11064, 12216, 12960, 13536, 14688, 15264, 17568},
  /* NPRB 025*/ { 680, 904, 1096, 1416, 1800, 2216, 2600, 3112, 3496, 4008, 4392, 4968, 5736, 6456, 7224, 7736, 7992, 9144, 9912, 10680, 11448, 12576, 13536, 14112, 15264, 15840, 18336},
  /* NPRB 026*/ { 712, 936, 1160, 1480, 1864, 2280, 2728, 3240, 3624, 4136, 4584, 5352, 5992, 6712, 7480, 7992, 8504, 9528, 10296, 11064, 12216, 12960, 14112, 14688, 15840, 16416, 19080},
  /* NPRB 027*/ { 744, 968, 1192, 1544, 1928, 2344, 2792, 3368, 3752, 4264, 4776, 5544, 6200, 6968, 7736, 8248, 8760, 9912, 10680, 11448, 12576, 13536, 14688, 15264, 16416, 16992, 19848},
  /* NPRB 028*/ { 776, 1000, 1256, 1608, 1992, 2472, 2984, 3368, 3880, 4392, 4968, 5736, 6456, 7224, 7992, 8504, 9144, 10296, 11064, 12216, 12960, 14112, 15264, 15840, 16992, 17568, 20616},
  /* NPRB 029*/ { 776, 1032, 1288, 1672, 2088, 2536, 2984, 3496, 4008, 4584, 5160, 5992, 6712, 7480, 8248, 8760, 9528, 10296, 11448, 12576, 13536, 14688, 15840, 16416, 17568, 18336, 21384},
  /* NPRB 030*/ { 808, 1064, 1320, 1736, 2152, 2664, 3112, 3624, 4264, 4776, 5352, 5992, 6712, 7736, 8504, 9144, 9912, 10680, 11832, 12960, 14112, 15264, 16416, 16992, 18336, 19080, 22152},
  /* NPRB 031*/ { 840, 1128, 1384, 1800, 2216, 2728, 3240, 3752, 4392, 4968, 5544, 6200, 6968, 7992, 8760, 9528, 9912, 11064, 12216, 13536, 14688, 15840, 16992, 17568, 19080, 19848, 22920},
  /* NPRB 032*/ { 872, 1160, 1416, 1864, 2280, 2792, 3368, 3880, 4584, 5160, 5736, 6456, 7224, 8248, 9144, 9912, 10296, 11448, 12576, 13536, 14688, 15840, 16992, 18336, 19848, 20616, 23688},
  /* NPRB 033*/ { 904, 1192, 1480, 1928, 2344, 2856, 3496, 4008, 4584, 5160, 5736, 6712, 7480, 8504, 9528, 10296, 10680, 11832, 12960, 14112, 15264, 16416, 17568, 19080, 19848, 20616, 24496},
  /* NPRB 034*/ { 936, 1224, 1544, 1992, 2408, 2984, 3496, 4136, 4776, 5352, 5992, 6968, 7736, 8760, 9912, 10296, 11064, 12216, 13536, 14688, 15840, 16992, 18336, 19848, 20616, 21384, 25456},
  /* NPRB 035*/ { 968, 1256, 1544, 2024, 2472, 3112, 3624, 4264, 4968, 5544, 6200, 6968, 7992, 9144, 9912, 10680, 11448, 12576, 14112, 15264, 16416, 17568, 19080, 19848, 21384, 22152, 25456},
  /* NPRB 036*/ { 1000, 1288, 1608, 2088, 2600, 3112, 3752, 4392, 4968, 5736, 6200, 7224, 8248, 9144, 10296, 11064, 11832, 12960, 14112, 15264, 16992, 18336, 19080, 20616, 22152, 22920, 26416},
  /* NPRB 037*/ { 1032, 1352, 1672, 2152, 2664, 3240, 3880, 4584, 5160, 5736, 6456, 7480, 8504, 9528, 10680, 11448, 12216, 13536, 14688, 15840, 16992, 18336, 19848, 21384, 22920, 23688, 27376},
  /* NPRB 038*/ { 1032, 1384, 1672, 2216, 2728, 3368, 4008, 4584, 5352, 5992, 6712, 7736, 8760, 9912, 11064, 11832, 12216, 13536, 15264, 16416, 17568, 19080, 20616, 22152, 22920, 24496, 28336},
  /* NPRB 039*/ { 1064, 1416, 1736, 2280, 2792, 3496, 4136, 4776, 5544, 6200, 6712, 7736, 8760, 9912, 11064, 11832, 12576, 14112, 15264, 16992, 18336, 19848, 21384, 22152, 23688, 24496, 29296},
  /* NPRB 040*/ { 1096, 1416, 1800, 2344, 2856, 3496, 4136, 4968, 5544, 6200, 6968, 7992, 9144, 10296, 11448, 12216, 12960, 14688, 15840, 16992, 18336, 19848, 21384, 22920, 24496, 25456, 29296},
  /* NPRB 041*/ { 1128, 1480, 1800, 2408, 2984, 3624, 4264, 4968, 5736, 6456, 7224, 8248, 9528, 10680, 11832, 12576, 13536, 14688, 16416, 17568, 19080, 20616, 22152, 23688, 25456, 26416, 30576},
  /* NPRB 042*/ { 1160, 1544, 1864, 2472, 2984, 3752, 4392, 5160, 5992, 6712, 7480, 8504, 9528, 10680, 12216, 12960, 13536, 15264, 16416, 18336, 19848, 21384, 22920, 24496, 25456, 26416, 30576},
  /* NPRB 043*/ { 1192, 1544, 1928, 2536, 3112, 3752, 4584, 5352, 5992, 6712, 7480, 8760, 9912, 11064, 12216, 12960, 14112, 15264, 16992, 18336, 19848, 21384, 22920, 24496, 26416, 27376, 31704},
  /* NPRB 044*/ { 1224, 1608, 1992, 2536, 3112, 3880, 4584, 5352, 6200, 6968, 7736, 8760, 9912, 11448, 12576, 13536, 14112, 15840, 17568, 19080, 20616, 22152, 23688, 25456, 26416, 28336, 32856},
  /* NPRB 045*/ { 1256, 1608, 2024, 2600, 3240, 4008, 4776, 5544, 6200, 6968, 7992, 9144, 10296, 11448, 12960, 13536, 14688, 16416, 17568, 19080, 20616, 22920, 24496, 25456, 27376, 28336, 32856},
  /* NPRB 046*/ { 1256, 1672, 2088, 2664, 3240, 4008, 4776, 5736, 6456, 7224, 7992, 9144, 10680, 11832, 12960, 14112, 14688, 16416, 18336, 19848, 21384, 22920, 24496, 26416, 28336, 29296, 34008},
  /* NPRB 047*/ { 1288, 1736, 2088, 2728, 3368, 4136, 4968, 5736, 6456, 7480, 8248, 9528, 10680, 12216, 13536, 14688, 15264, 16992, 18336, 20616, 22152, 23688, 25456, 27376, 28336, 29296, 35160},
  /* NPRB 048*/ { 1320, 1736, 2152, 2792, 3496, 4264, 4968, 5992, 6712, 7480, 8504, 9528, 11064, 12216, 13536, 14688, 15840, 17568, 19080, 20616, 22152, 24496, 25456, 27376, 29296, 30576, 35160},
  /* NPRB 049*/ { 1352, 1800, 2216, 2856, 3496, 4392, 5160, 5992, 6968, 7736, 8504, 9912, 11064, 12576, 14112, 15264, 15840, 17568, 19080, 21384, 22920, 24496, 26416, 28336, 29296, 31704, 36696},
  /* NPRB 050*/ { 1384, 1800, 2216, 2856, 3624, 4392, 5160, 6200, 6968, 7992, 8760, 9912, 11448, 12960, 14112, 15264, 16416, 18336, 19848, 21384, 22920, 25456, 27376, 28336, 30576, 31704, 36696},
  /* NPRB 051*/ { 1416, 1864, 2280, 2984, 3624, 4584, 5352, 6200, 7224, 7992, 9144, 10296, 11832, 12960, 14688, 15840, 16416, 18336, 19848, 22152, 23688, 25456, 27376, 29296, 31704, 32856, 37888},
  /* NPRB 052*/ { 1416, 1864, 2344, 2984, 3752, 4584, 5352, 6456, 7224, 8248, 9144, 10680, 11832, 13536, 14688, 15840, 16992, 19080, 20616, 22152, 24496, 26416, 28336, 29296, 31704, 32856, 37888},
  /* NPRB 053*/ { 1480, 1928, 2344, 3112, 3752, 4776, 5544, 6456, 7480, 8248, 9144, 10680, 12216, 13536, 15264, 16416, 16992, 19080, 21384, 22920, 24496, 26416, 28336, 30576, 32856, 34008, 39232},
  /* NPRB 054*/ { 1480, 1992, 2408, 3112, 3880, 4776, 5736, 6712, 7480, 8504, 9528, 11064, 12216, 14112, 15264, 16416, 17568, 19848, 21384, 22920, 25456, 27376, 29296, 30576, 32856, 34008, 40576},
  /* NPRB 055*/ { 1544, 1992, 2472, 3240, 4008, 4776, 5736, 6712, 7736, 8760, 9528, 11064, 12576, 14112, 15840, 16992, 17568, 19848, 22152, 23688, 25456, 27376, 29296, 31704, 34008, 35160, 40576},
  /* NPRB 056*/ { 1544, 2024, 2536, 3240, 4008, 4968, 5992, 6712, 7736, 8760, 9912, 11448, 12576, 14688, 15840, 16992, 18336, 20616, 22152, 24496, 26416, 28336, 30576, 31704, 34008, 35160, 40576},
  /* NPRB 057*/ { 1608, 2088, 2536, 3368, 4136, 4968, 5992, 6968, 7992, 9144, 9912, 11448, 12960, 14688, 16416, 17568, 18336, 20616, 22920, 24496, 26416, 28336, 30576, 32856, 35160, 36696, 42368},
  /* NPRB 058*/ { 1608, 2088, 2600, 3368, 4136, 5160, 5992, 6968, 7992, 9144, 10296, 11832, 12960, 14688, 16416, 17568, 19080, 20616, 22920, 25456, 27376, 29296, 31704, 32856, 35160, 36696, 42368},
  /* NPRB 059*/ { 1608, 2152, 2664, 3496, 4264, 5160, 6200, 7224, 8248, 9144, 10296, 11832, 13536, 15264, 16992, 18336, 19080, 21384, 23688, 25456, 27376, 29296, 31704, 34008, 36696, 37888, 43816},
  /* NPRB 060*/ { 1672, 2152, 2664, 3496, 4264, 5352, 6200, 7224, 8504, 9528, 10680, 12216, 13536, 15264, 16992, 18336, 19848, 21384, 23688, 25456, 28336, 30576, 32856, 34008, 36696, 37888, 43816},
  /* NPRB 061*/ { 1672, 2216, 2728, 3624, 4392, 5352, 6456, 7480, 8504, 9528, 10680, 12216, 14112, 15840, 17568, 18336, 19848, 22152, 24496, 26416, 28336, 30576, 32856, 35160, 36696, 39232, 45352},
  /* NPRB 062*/ { 1736, 2280, 2792, 3624, 4392, 5544, 6456, 7480, 8760, 9912, 11064, 12576, 14112, 15840, 17568, 19080, 19848, 22152, 24496, 26416, 29296, 31704, 34008, 35160, 37888, 39232, 45352},
  /* NPRB 063*/ { 1736, 2280, 2856, 3624, 4584, 5544, 6456, 7736, 8760, 9912, 11064, 12576, 14112, 16416, 18336, 19080, 20616, 22920, 24496, 27376, 29296, 31704, 34008, 36696, 37888, 40576, 46888},
  /* NPRB 064*/ { 1800, 2344, 2856, 3752, 4584, 5736, 6712, 7736, 9144, 10296, 11448, 12960, 14688, 16416, 18336, 19848, 20616, 22920, 25456, 27376, 29296, 31704, 34008, 36696, 39232, 40576, 46888},
  /* NPRB 065*/ { 1800, 2344, 2856, 3752, 4584, 5736, 6712, 7992, 9144, 10296, 11448, 12960, 14688, 16992, 18336, 19848, 21384, 23688, 25456, 28336, 30576, 32856, 35160, 37888, 39232, 40576, 48936},
  /* NPRB 066*/ { 1800, 2408, 2984, 3880, 4776, 5736, 6968, 7992, 9144, 10296, 11448, 13536, 15264, 16992, 19080, 20616, 21384, 23688, 26416, 28336, 30576, 32856, 35160, 37888, 40576, 42368, 48936},
  /* NPRB 067*/ { 1864, 2472, 2984, 3880, 4776, 5992, 6968, 8248, 9528, 10680, 11832, 13536, 15264, 16992, 19080, 20616, 22152, 24496, 26416, 29296, 31704, 34008, 36696, 37888, 40576, 42368, 48936},
  /* NPRB 068*/ { 1864, 2472, 3112, 4008, 4968, 5992, 6968, 8248, 9528, 10680, 11832, 13536, 15264, 17568, 19848, 20616, 22152, 24496, 27376, 29296, 31704, 34008, 36696, 39232, 42368, 43816, 51024},
  /* NPRB 069*/ { 1928, 2536, 3112, 4008, 4968, 5992, 7224, 8504, 9528, 11064, 12216, 14112, 15840, 17568, 19848, 21384, 22152, 24496, 27376, 29296, 31704, 35160, 36696, 39232, 42368, 43816, 51024},
  /* NPRB 070*/ { 1928, 2536, 3112, 4136, 4968, 6200, 7224, 8504, 9912, 11064, 12216, 14112, 15840, 18336, 19848, 21384, 22920, 25456, 27376, 30576, 32856, 35160, 37888, 40576, 42368, 43816, 52752},
  /* NPRB 071*/ { 1992, 2600, 3240, 4136, 5160, 6200, 7480, 8760, 9912, 11064, 12576, 14112, 16416, 18336, 20616, 22152, 22920, 25456, 28336, 30576, 32856, 35160, 37888, 40576, 43816, 45352, 52752},
  /* NPRB 072*/ { 1992, 2600, 3240, 4264, 5160, 6200, 7480, 8760, 9912, 11448, 12576, 14688, 16416, 18336, 20616, 22152, 23688, 26416, 28336, 30576, 34008, 36696, 39232, 40576, 43816, 45352, 52752},
  /* NPRB 073*/ { 2024, 2664, 3240, 4264, 5160, 6456, 7736, 8760, 10296, 11448, 12960, 14688, 16416, 19080, 20616, 22152, 23688, 26416, 29296, 31704, 34008, 36696, 39232, 42368, 45352, 46888, 55056},
  /* NPRB 074*/ { 2088, 2728, 3368, 4392, 5352, 6456, 7736, 9144, 10296, 11832, 12960, 14688, 16992, 19080, 21384, 22920, 24496, 26416, 29296, 31704, 34008, 36696, 40576, 42368, 45352, 46888, 55056},
  /* NPRB 075*/ { 2088, 2728, 3368, 4392, 5352, 6712, 7736, 9144, 10680, 11832, 12960, 15264, 16992, 19080, 21384, 22920, 24496, 27376, 29296, 32856, 35160, 37888, 40576, 43816, 45352, 46888, 55056},
  /* NPRB 076*/ { 2088, 2792, 3368, 4392, 5544, 6712, 7992, 9144, 10680, 11832, 13536, 15264, 17568, 19848, 22152, 23688, 24496, 27376, 30576, 32856, 35160, 37888, 40576, 43816, 46888, 48936, 55056},
  /* NPRB 077*/ { 2152, 2792, 3496, 4584, 5544, 6712, 7992, 9528, 10680, 12216, 13536, 15840, 17568, 19848, 22152, 23688, 25456, 27376, 30576, 32856, 35160, 39232, 42368, 43816, 46888, 48936, 57336},
  /* NPRB 078*/ { 2152, 2856, 3496, 4584, 5544, 6968, 8248, 9528, 11064, 12216, 13536, 15840, 17568, 19848, 22152, 23688, 25456, 28336, 30576, 34008, 36696, 39232, 42368, 45352, 46888, 48936, 57336},
  /* NPRB 079*/ { 2216, 2856, 3496, 4584, 5736, 6968, 8248, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 24496, 25456, 28336, 31704, 34008, 36696, 39232, 42368, 45352, 48936, 51024, 57336},
  /* NPRB 080*/ { 2216, 2856, 3624, 4776, 5736, 6968, 8248, 9912, 11064, 12576, 14112, 16416, 18336, 20616, 22920, 24496, 26416, 29296, 31704, 34008, 36696, 40576, 43816, 45352, 48936, 51024, 59256},
  /* NPRB 081*/ { 2280, 2984, 3624, 4776, 5736, 7224, 8504, 9912, 11448, 12960, 14112, 16416, 18336, 20616, 22920, 24496, 26416, 29296, 31704, 35160, 37888, 40576, 43816, 46888, 48936, 51024, 59256},
  /* NPRB 082*/ { 2280, 2984, 3624, 4776, 5992, 7224, 8504, 9912, 11448, 12960, 14688, 16416, 19080, 21384, 23688, 25456, 26416, 29296, 32856, 35160, 37888, 40576, 43816, 46888, 51024, 52752, 59256},
  /* NPRB 083*/ { 2280, 2984, 3752, 4776, 5992, 7224, 8760, 10296, 11448, 12960, 14688, 16992, 19080, 21384, 23688, 25456, 27376, 30576, 32856, 35160, 39232, 42368, 45352, 46888, 51024, 52752, 61664},
  /* NPRB 084*/ { 2344, 3112, 3752, 4968, 5992, 7480, 8760, 10296, 11832, 13536, 14688, 16992, 19080, 21384, 24496, 25456, 27376, 30576, 32856, 36696, 39232, 42368, 45352, 48936, 51024, 52752, 61664},
  /* NPRB 085*/ { 2344, 3112, 3880, 4968, 5992, 7480, 8760, 10296, 11832, 13536, 14688, 16992, 19080, 22152, 24496, 26416, 27376, 30576, 34008, 36696, 39232, 42368, 45352, 48936, 52752, 55056, 61664},
  /* NPRB 086*/ { 2408, 3112, 3880, 4968, 6200, 7480, 9144, 10680, 12216, 13536, 15264, 17568, 19848, 22152, 24496, 26416, 28336, 30576, 34008, 36696, 40576, 43816, 46888, 48936, 52752, 55056, 63776},
  /* NPRB 087*/ { 2408, 3240, 3880, 5160, 6200, 7736, 9144, 10680, 12216, 13536, 15264, 17568, 19848, 22152, 25456, 26416, 28336, 31704, 34008, 37888, 40576, 43816, 46888, 51024, 52752, 55056, 63776},
  /* NPRB 088*/ { 2472, 3240, 4008, 5160, 6200, 7736, 9144, 10680, 12216, 14112, 15264, 17568, 19848, 22920, 25456, 27376, 28336, 31704, 35160, 37888, 40576, 43816, 46888, 51024, 52752, 55056, 63776},
  /* NPRB 089*/ { 2472, 3240, 4008, 5160, 6456, 7736, 9144, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 25456, 27376, 29296, 31704, 35160, 37888, 42368, 45352, 48936, 51024, 55056, 57336, 66592},
  /* NPRB 090*/ { 2536, 3240, 4008, 5352, 6456, 7992, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 25456, 27376, 29296, 32856, 35160, 39232, 42368, 45352, 48936, 51024, 55056, 57336, 66592},
  /* NPRB 091*/ { 2536, 3368, 4136, 5352, 6456, 7992, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 23688, 26416, 28336, 29296, 32856, 36696, 39232, 42368, 45352, 48936, 52752, 55056, 57336, 66592},
  /* NPRB 092*/ { 2536, 3368, 4136, 5352, 6456, 7992, 9528, 11448, 12960, 14688, 16416, 18336, 21384, 23688, 26416, 28336, 30576, 32856, 36696, 39232, 42368, 46888, 48936, 52752, 57336, 59256, 68808},
  /* NPRB 093*/ { 2600, 3368, 4136, 5352, 6712, 8248, 9528, 11448, 12960, 14688, 16416, 19080, 21384, 23688, 26416, 28336, 30576, 34008, 36696, 40576, 43816, 46888, 51024, 52752, 57336, 59256, 68808},
  /* NPRB 094*/ { 2600, 3496, 4264, 5544, 6712, 8248, 9912, 11448, 12960, 14688, 16416, 19080, 21384, 24496, 27376, 29296, 30576, 34008, 37888, 40576, 43816, 46888, 51024, 55056, 57336, 59256, 68808},
  /* NPRB 095*/ { 2664, 3496, 4264, 5544, 6712, 8248, 9912, 11448, 13536, 15264, 16992, 19080, 21384, 24496, 27376, 29296, 30576, 34008, 37888, 40576, 43816, 46888, 51024, 55056, 57336, 61664, 71112},
  /* NPRB 096*/ { 2664, 3496, 4264, 5544, 6968, 8504, 9912, 11832, 13536, 15264, 16992, 19080, 22152, 24496, 27376, 29296, 31704, 35160, 37888, 40576, 45352, 48936, 51024, 55056, 59256, 61664, 71112},
  /* NPRB 097*/ { 2728, 3496, 4392, 5736, 6968, 8504, 10296, 11832, 13536, 15264, 16992, 19848, 22152, 25456, 28336, 29296, 31704, 35160, 37888, 42368, 45352, 48936, 52752, 55056, 59256, 61664, 71112},
  /* NPRB 098*/ { 2728, 3624, 4392, 5736, 6968, 8760, 10296, 11832, 13536, 15264, 16992, 19848, 22152, 25456, 28336, 30576, 31704, 35160, 39232, 42368, 45352, 48936, 52752, 57336, 59256, 61664, 73712},
  /* NPRB 099*/ { 2728, 3624, 4392, 5736, 6968, 8760, 10296, 12216, 14112, 15840, 17568, 19848, 22920, 25456, 28336, 30576, 31704, 35160, 39232, 42368, 46888, 48936, 52752, 57336, 61664, 63776, 73712},
  /* NPRB 100*/ { 2792, 3624, 4584, 5736, 7224, 8760, 10296, 12216, 14112, 15840, 17568, 19848, 22920, 25456, 28336, 30576, 32856, 36696, 39232, 43816, 46888, 51024, 55056, 57336, 61664, 63776, 75376},
  /* NPRB 101*/ { 2792, 3752, 4584, 5992, 7224, 8760, 10680, 12216, 14112, 15840, 17568, 20616, 22920, 26416, 29296, 30576, 32856, 36696, 40576, 43816, 46888, 51024, 55056, 57336, 61664, 63776, 75376},
  /* NPRB 102*/ { 2856, 3752, 4584, 5992, 7224, 9144, 10680, 12576, 14112, 16416, 18336, 20616, 23688, 26416, 29296, 31704, 32856, 36696, 40576, 43816, 46888, 51024, 55056, 59256, 61664, 63776, 75376},
  /* NPRB 103*/ { 2856, 3752, 4584, 5992, 7480, 9144, 10680, 12576, 14688, 16416, 18336, 20616, 23688, 26416, 29296, 31704, 34008, 36696, 40576, 43816, 48936, 51024, 55056, 59256, 63776, 66592, 75376},
  /* NPRB 104*/ { 2856, 3752, 4584, 5992, 7480, 9144, 10680, 12576, 14688, 16416, 18336, 21384, 23688, 26416, 29296, 31704, 34008, 37888, 40576, 45352, 48936, 52752, 57336, 59256, 63776, 66592, 75376},
  /* NPRB 105*/ { 2984, 3880, 4776, 6200, 7480, 9144, 11064, 12960, 14688, 16416, 18336, 21384, 23688, 27376, 30576, 31704, 34008, 37888, 42368, 45352, 48936, 52752, 57336, 59256, 63776, 66592, 75376},
  /* NPRB 106*/ { 2984, 3880, 4776, 6200, 7480, 9528, 11064, 12960, 14688, 16992, 18336, 21384, 24496, 27376, 30576, 32856, 34008, 37888, 42368, 45352, 48936, 52752, 57336, 61664, 63776, 66592, 75376},
  /* NPRB 107*/ { 2984, 3880, 4776, 6200, 7736, 9528, 11064, 12960, 15264, 16992, 19080, 21384, 24496, 27376, 30576, 32856, 35160, 39232, 42368, 46888, 48936, 52752, 57336, 61664, 66592, 68808, 75376},
  /* NPRB 108*/ { 2984, 4008, 4776, 6200, 7736, 9528, 11448, 12960, 15264, 16992, 19080, 22152, 24496, 27376, 30576, 32856, 35160, 39232, 42368, 46888, 51024, 55056, 59256, 61664, 66592, 68808, 75376},
  /* NPRB 109*/ { 2984, 4008, 4968, 6456, 7736, 9528, 11448, 13536, 15264, 16992, 19080, 22152, 24496, 28336, 31704, 34008, 35160, 39232, 43816, 46888, 51024, 55056, 59256, 61664, 66592, 68808, 75376},
  /* NPRB 110*/ { 3112, 4008, 4968, 6456, 7992, 9528, 11448, 13536, 15264, 17568, 19080, 22152, 25456, 28336, 31704, 34008, 35160, 39232, 43816, 46888, 51024, 55056, 59256, 63776, 66592, 71112, 75376}

};


LteAmc::LteAmc ()
{
	/*
		 * Fill SinrToMcs,McsToCqi,CqiToItbs tables.
		 */
		FillSinrToMcs();
}


LteAmc::~LteAmc ()
{ 
}

TypeId
LteAmc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LteAmc")
  .SetParent<Object> ()
  .SetGroupName("Lte")
  .AddConstructor<LteAmc> ()
  .AddAttribute ("BerUl",
                   "The requested Uplink BER in assigning MCS (default is 0.00005).",
                   DoubleValue (0.00005),
                   MakeDoubleAccessor (&LteAmc::m_berUl),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BerDl",
  				 "The requested Downlink BER in assigning MCS (default is 0.00005).",
  				 DoubleValue (0.00005),
  				 MakeDoubleAccessor (&LteAmc::m_berDl),
  				 MakeDoubleChecker<double> ())
  .AddAttribute ("AmcModel",
                "AMC model used to assign CQI",
                 EnumValue (LteAmc::MiErrorModel),
                 MakeEnumAccessor (&LteAmc::m_amcModel),
                 MakeEnumChecker (LteAmc::MiErrorModel, "Vienna",
                                  LteAmc::PiroEW2010, "PiroEW2010"));
  return tid;
}

std::vector<LteAmc::McsPrbInfo>
LteAmc::GetUlMcsNprbInfoFromTbs (int tbs, int max_nprb, int max_mcs)
{
    NS_LOG_FUNCTION (this);
    NS_ASSERT_MSG (max_nprb <= 110, "BW_PRB=" << max_nprb);
    NS_ASSERT_MSG (max_mcs < 29, "MCS=" << max_mcs);
        
    std::vector<LteAmc::McsPrbInfo> McsPrbVector;
    int itbs = 0;
    int grantTbs = 0; //The TBS obtained from MCS and PRB combination
    //Find tbs >= in TransportBlockSizeTable
    for (int inprb = 0; inprb < max_nprb; inprb++)
    {
        for (int mcs = 0; mcs <= max_mcs; mcs++)
        {
            itbs = McsToItbsUl[mcs];
            grantTbs = TransportBlockSizeTable[inprb][itbs];
            if (grantTbs >= tbs)
            {
              LteAmc::McsPrbInfo grantInfo;
              grantInfo.mcs = mcs;
              grantInfo.nbRb = inprb + 1;
              grantInfo.tbs = grantTbs;
              McsPrbVector.push_back(grantInfo);
              break;
            }
        }
    }
    return McsPrbVector;
}

int
LteAmc::GetCqiFromSpectralEfficiency (double s)
{
  NS_LOG_FUNCTION (s);
  NS_ASSERT_MSG (s >= 0.0, "negative spectral efficiency = " << s);
  int cqi = 0;
  while ((cqi < 15) && (SpectralEfficiencyForCqi[cqi + 1] < s))
    {
      ++cqi;
    }
  NS_LOG_LOGIC ("cqi = " << cqi);
  return cqi;
}


int
LteAmc::GetMcsFromCqi (int cqi)
{
  NS_LOG_FUNCTION (cqi);
  NS_ASSERT_MSG (cqi >= 0 && cqi <= 15, "CQI must be in [0..15] = " << cqi);
  double spectralEfficiency = SpectralEfficiencyForCqi[cqi];
  int mcs = 0;
  while ((mcs < 28) && (SpectralEfficiencyForMcs[mcs + 1] <= spectralEfficiency))
    {
      ++mcs;
    }
  NS_LOG_LOGIC ("mcs = " << mcs);
  return mcs;
}

int
LteAmc::GetDlTbSizeFromMcs (int mcs, int nprb)
{
  NS_LOG_FUNCTION (mcs);

  NS_ASSERT_MSG (mcs < 29, "MCSs=" << mcs);
  NS_ASSERT_MSG (nprb < 111, "NPRB=" << nprb);

  int itbs = McsToItbsDl[mcs];
  return (TransportBlockSizeTable[nprb - 1][itbs]);
}
int
LteAmc::GetUlTbSizeFromItbs (int itbs, int nprb)
{
  NS_LOG_FUNCTION (itbs);

  NS_ASSERT_MSG (itbs < 27, "MCS=" << itbs);
  NS_ASSERT_MSG (nprb < 111, "NPRB=" << nprb);
  return (TransportBlockSizeTable[nprb - 1][itbs]);
}


int
LteAmc::GetUlTbSizeFromMcs (int mcs, int nprb)
{
  NS_LOG_FUNCTION (mcs);

  NS_ASSERT_MSG (mcs < 29, "MCS=" << mcs);
  NS_ASSERT_MSG (nprb < 111, "NPRB=" << nprb);

  int itbs = McsToItbsUl[mcs];
  return (TransportBlockSizeTable[nprb - 1][itbs]);
}


double
LteAmc::GetSpectralEfficiencyFromCqi (int cqi)
{
  NS_LOG_FUNCTION (cqi);
  NS_ASSERT_MSG (cqi >= 0 && cqi <= 15, "CQI must be in [0..15] = " << cqi);
  NS_LOG_LOGIC ("Spectral efficiency = " << SpectralEfficiencyForCqi[cqi]);
  return SpectralEfficiencyForCqi[cqi];
}


std::vector<int>
LteAmc::CreateCqiFeedbacks (const SpectrumValue& sinr, uint8_t rbgSize)
{
  NS_LOG_FUNCTION (this);

  std::vector<int> cqi;
  Values::const_iterator it;
  
  if (m_amcModel == PiroEW2010)
    {

      for (it = sinr.ConstValuesBegin (); it != sinr.ConstValuesEnd (); it++)
        {
          double sinr_ = (*it);
          if (sinr_ == 0.0)
            {
              cqi.push_back (-1); // SINR == 0 (linear units) means no signal in this RB
            }
          else
            {
              /*
              * Compute the spectral efficiency from the SINR
              *                                        SINR
              * spectralEfficiency = log2 (1 + -------------------- )
              *                                    -ln(5*BER)/1.5
              * NB: SINR must be expressed in linear units
              */

              double s = log2 ( 1 + ( sinr_ / ( (-std::log (5.0 * m_berDl )) / 1.5) ));

              int cqi_ = GetCqiFromSpectralEfficiency (s);

              NS_LOG_LOGIC (" PRB =" << cqi.size ()
                                    << ", sinr = " << sinr_
                                    << " (=" << 10 * std::log10 (sinr_) << " dB)"
                                    << ", spectral efficiency =" << s
                                    << ", CQI = " << cqi_ << ", BER = " << m_berDl);

              cqi.push_back (cqi_);
            }
        }
    }
  else if (m_amcModel == MiErrorModel)
    {
      NS_LOG_DEBUG (this << " AMC-VIENNA RBG size " << (uint16_t)rbgSize);
      NS_ASSERT_MSG (rbgSize > 0, " LteAmc-Vienna: RBG size must be greater than 0");
      std::vector <int> rbgMap;
      int rbId = 0;
      for (it = sinr.ConstValuesBegin (); it != sinr.ConstValuesEnd (); it++)
      {
        rbgMap.push_back (rbId++);
        if ((rbId % rbgSize == 0)||((it+1)==sinr.ConstValuesEnd ()))
         {
            uint8_t mcs = 0;
            TbStats_t tbStats;
            while (mcs <= 28)
              {
                HarqProcessInfoList_t harqInfoList;
                tbStats = LteMiErrorModel::GetTbDecodificationStats (sinr, rbgMap, (uint16_t)GetDlTbSizeFromMcs (mcs, rbgSize) / 8, mcs, harqInfoList);
                if (tbStats.tbler > 0.1)
                  {
                    break;
                  }
                mcs++;
                
              }
            if (mcs > 0)
              {
                mcs--;
              }
            NS_LOG_DEBUG (this << "\t RBG " << rbId << " MCS " << (uint16_t)mcs << " TBLER " << tbStats.tbler);
            int rbgCqi = 0;
            if ((tbStats.tbler > 0.1)&&(mcs==0))
              {
                rbgCqi = 0; // any MCS can guarantee the 10 % of BER
              }
            else if (mcs == 28)
              {
                rbgCqi = 15; // all MCSs can guarantee the 10 % of BER
              }
            else
              {
                double s = SpectralEfficiencyForMcs[mcs];
                rbgCqi = 0;
                while ((rbgCqi < 15) && (SpectralEfficiencyForCqi[rbgCqi + 1] < s))
                {
                  ++rbgCqi;
                }
              }
            NS_LOG_DEBUG (this << "\t MCS " << (uint16_t)mcs << "-> CQI " << rbgCqi);
            // fill the cqi vector (per RB basis)
            for (uint8_t j = 0; j < rbgSize; j++)
              {
                cqi.push_back (rbgCqi);
              }
            rbgMap.clear ();
         }
        
      }
      
    }

  return cqi;
}

int
LteAmc::GetCqiFromUlSinr ( double sinr, double ber )
{
  NS_LOG_FUNCTION ( this );
  int cqi = -1;
  if (ber == 1)
    {
      ber = m_berUl;
    }
  NS_LOG_DEBUG (this << " ber value is set to " << ber );

  if (sinr == 0.0)
    {
      cqi = -1; // SINR == 0 (linear units) means no signal in this RB
    }
  else
    {
      /*
      * Compute the spectral efficiency from the SINR
      *                                        SINR
      * spectralEfficiency = log2 (1 + -------------------- )
      *                                    -ln(5*BER)/1.5
      * NB: SINR must be expressed in linear units
      */

      double s = log2 ( 1 + ( std::pow (10, sinr / 10 ) /
                              ( (-std::log (5.0 * ber )) / 1.5) ));

      cqi = GetCqiFromSpectralEfficiency (s);

      NS_LOG_LOGIC (" Sinr = " << sinr
                               << " (=" << 10 * std::log10 (sinr) << " dB)"
                               << ", spectral efficiency =" << s
                               << ", CQI = " << cqi << ", BER = " << ber);
    }

  return cqi;
}

int
LteAmc::GetMcsFromUlSinr ( double sinr, double ber )
{
  NS_LOG_FUNCTION ( this );
  int cqi = GetCqiFromUlSinr ( sinr, ber );
  int mcs;
  mcs = GetMcsFromCqi ( cqi );

  return mcs;
}

int
LteAmc::GetMcsFromItbs(int iTbs,bool enable64Qam){
	uint8_t mcs;
	if(iTbs <= 10)
	{
		mcs = iTbs;
	}
	else if(iTbs < 19)
	{
		mcs = iTbs + 1;
	}
	else if((iTbs == 19) && (!enable64Qam))
	{
		mcs = iTbs + 1;
	}
	else
	{
		mcs = iTbs + 2;
	}
	return mcs;
}

int
LteAmc::GetMcsFromSinr(double sinrDB){
	uint16_t sinr;
	/*
	 * conver from dB [-64,..,63.5] to UL_CQI [0,..,255]
	 */
	sinr = (sinrDB*2) + 128;

	NS_LOG_FUNCTION (sinr);
	NS_ASSERT_MSG ((sinr <= 255 && sinr >= 0), "UL_CQI must be in range [0,..,255] = " << sinr);

	return SinrToMcs[sinr];
}

int
LteAmc::GetItbsFromCqi(int cqi){
	return UlCqiToiTbsTbl[cqi];
}

int
LteAmc::GetCqiFromMcs(int mcs){
	return McsToCqi[mcs];
}

void
LteAmc::FillSinrToMcs(){
	uint8_t sinr;
	sinr = 0;
	for (; sinr <= 121; ++sinr)
			   SinrToMcs[sinr] = 0;
	       for (; sinr <= 131; ++sinr)
	    	   SinrToMcs[sinr] = 1;
	       for (; sinr <= 133; ++sinr)
	    	   SinrToMcs[sinr] = 2;
	       for (; sinr <= 135; ++sinr)
	    	   SinrToMcs[sinr] = 3;
	       for (; sinr <= 137; ++sinr)
	    	   SinrToMcs[sinr] = 4;
	       for (; sinr <= 139; ++sinr)
	    	   SinrToMcs[sinr] = 5;
	       for (; sinr <= 141; ++sinr)
	    	   SinrToMcs[sinr] = 6;
	       for (; sinr <= 143; ++sinr)
	    	   SinrToMcs[sinr] = 7;
	       for (; sinr <= 145; ++sinr)
	    	   SinrToMcs[sinr] = 8;
	       for (; sinr <= 147; ++sinr)
	    	   SinrToMcs[sinr] = 9;
	       for (; sinr <= 148; ++sinr)
	    	   SinrToMcs[sinr] = 10;
	       for (; sinr <= 149; ++sinr)
	    	   SinrToMcs[sinr] = 11;
	       for (; sinr <= 150; ++sinr)
	    	   SinrToMcs[sinr] = 12;
	       for (; sinr <= 151; ++sinr)
	    	   SinrToMcs[sinr] = 13;
	       for (; sinr <= 152; ++sinr)
	    	   SinrToMcs[sinr] = 14;
	       for (; sinr <= 153; ++sinr)
	    	   SinrToMcs[sinr] = 15;
	       for (; sinr <= 154; ++sinr)
	    	   SinrToMcs[sinr] = 16;
	       for (; sinr <= 155; ++sinr)
	    	   SinrToMcs[sinr] = 17;
	       for (; sinr <= 156; ++sinr)
	    	   SinrToMcs[sinr] = 18;
	       for (; sinr <= 157; ++sinr)
	    	   SinrToMcs[sinr] = 19;
	       for (; sinr <= 158; ++sinr)
	    	   SinrToMcs[sinr] = 20;
	       for (; sinr <= 162; ++sinr)
	    	   SinrToMcs[sinr] = 21;
	       for (; sinr <= 166; ++sinr)
	    	   SinrToMcs[sinr] = 22;
	       for (; sinr <= 170; ++sinr)
	    	   SinrToMcs[sinr] = 23;
	       for (; sinr <= 174; ++sinr)
	    	   SinrToMcs[sinr] = 24;
	       for (; sinr <= 178; ++sinr)
	    	   SinrToMcs[sinr] = 25;
	       for (; sinr <= 182; ++sinr)
	    	   SinrToMcs[sinr] = 26;
	       for (; sinr <= 186; ++sinr)
	    	   SinrToMcs[sinr] = 27;
	       for (; sinr <= 254; ++sinr)
	    	   SinrToMcs[sinr] = 28;
	       SinrToMcs[sinr] = 28;
	       FillMcsToCqi();
}

void
LteAmc::FillMcsToCqi(){
	uint8_t mcs;
	mcs = 0;
	McsToCqi[mcs++] = 0;
	McsToCqi[mcs++] = 0;
	McsToCqi[mcs++] = 3; McsToCqi[mcs++] = 3;
	McsToCqi[mcs++] = 4; McsToCqi[mcs++] = 4;
	McsToCqi[mcs++] = 5; McsToCqi[mcs++] = 5;
	McsToCqi[mcs++] = 6; McsToCqi[mcs++] = 6;
	McsToCqi[mcs++] = 7; McsToCqi[mcs++] = 7; McsToCqi[mcs++] = 7;
	McsToCqi[mcs++] = 8; McsToCqi[mcs++] = 8; McsToCqi[mcs++] = 8;
	McsToCqi[mcs++] = 9;
	McsToCqi[mcs++] = 10; McsToCqi[mcs++] = 10; McsToCqi[mcs++] = 10;
	McsToCqi[mcs++] = 11; McsToCqi[mcs++] = 11; McsToCqi[mcs++] = 11; McsToCqi[mcs++] = 11;
	McsToCqi[mcs++] = 12; McsToCqi[mcs++] = 12;
	McsToCqi[mcs++] = 13; McsToCqi[mcs++] = 13;
	McsToCqi[mcs++] = 14;
	FillCqiToItbsTbl();
}

void
LteAmc::FillCqiToItbsTbl(){
	uint8_t				*mapTbl = &UlCqiToiTbsTbl[0];
	double    effTbl[27];
	int16_t              i;
	int16_t              j;

	CompUlEff(effTbl);

	for (i = 26, j = 15;
		 i >= 0 && j > 0; --i)
	{
	  if (effTbl[i] <= EFF[j])
	  {
		 mapTbl[j--] = (uint8_t)i;
	  }
	}
	for (; j > 0; --j)
	{
	  mapTbl[j] = 0;
	}
}

void
LteAmc::CompUlEff(double *effTbl){
	uint8_t j,i,noResPerRb = 144;

	   for (j = 0; j < 27; j++)
	   {
		   effTbl[j] = 0;
	      for (i = 0; i < 110; i++)
	      {
	         /* This line computes the coding efficiency per 1024 REs */
	    	  effTbl[j] += (TransportBlockSizeTable[i][j] * 1024) / (noResPerRb * (i+1));
	      }
	      effTbl[j] /= 110;
	   }
}

int
LteAmc::GetItbsFromMcsUl(int mcs){
	return McsToItbsUl[mcs];
}

uint8_t
LteAmc::GetTrgCqiDelta(uint8_t availblePwr)
{
    if(availblePwr <= 11)
    {
       return 5;
    }
    else
       return 0;
}


} // namespace ns3
