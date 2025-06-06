/*
 * control1.h
 *
 *  Created on: May 9, 2025
 *      Author: PESP
 */

#ifndef INC_CONTROL1_H_
#define INC_CONTROL1_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#define BATCH_DATA_LEN 32

typedef struct
{
	int16_t cos;
	int16_t sin;
} Triangles;

typedef struct {
    int16_t ia;
    int16_t ib;
    int16_t ialpha;
    int16_t ibeta;
    int16_t id;
    int16_t iq;
    uint16_t IADC1;
    uint16_t IADC2;
} Currents;

typedef struct {
	int16_t va;
	int16_t vb;
    int16_t valpha;
    int16_t vbeta;
    int16_t vd;
    int16_t vq;
} Voltages;

typedef struct
{
    uint16_t Kp;
    uint16_t Kp_div;
    uint16_t Ki;
    uint16_t Ki_div;
    uint16_t Upper_Limit;
    int16_t Lower_Limit;
    int32_t Integral;
    int16_t Output;
} PID_Struct;

extern Currents   Ioffset;
extern Currents   adc_avg;
extern Currents   adc_copy;
extern Currents   Iab;
extern Currents   IAlphaBeta;
extern Currents   Idq;
extern Voltages   VAlphaBeta;
extern Voltages   Vdq;

//extern volatile EMFs       EAlphaBeta;

//extern const uint16_t sinTable[312];
//extern const uint16_t cosTable[312];
extern PID_Struct Torque;
extern PID_Struct Flux;
extern PID_Struct Speed;
extern PID_Struct PLL_w;
extern PID_Struct Q;

//extern volatile PLL_Components Estimate;
//extern volatile LPF_Struct LP_Emf;
//extern volatile HPF_Struct HP_Power;
extern int16_t Iq_ref;
extern uint32_t DataBuffer[BATCH_DATA_LEN];
extern int16_t Qi;


void SysTick_Init(void);
Currents ADC_AverageValue();
Currents Update_Current_Offset(Currents *, Currents *,uint16_t);
Currents Get_Phase_Currents(Currents *, Currents *);
Currents Clarke(Currents *);
//Triangles Tri_Function(uint16_t);
Currents Park(Currents *, uint16_t);
Voltages Rev_Park(Voltages, uint16_t);
//EMFs EMF_Observer(Voltages *Valphabeta, Currents *Ialphabeta);
//PLL_Components PLL(EMFs emf);
void PID_Init(PID_Struct *, PID_Struct *, PID_Struct *, PID_Struct *, PID_Struct *);


////hall sensor(當作外部可見的全域變數宣告)
//extern uint8_t  hall_u, hall_v, hall_w;
//extern uint8_t  hall_u_old, hall_v_old, hall_w_old;
//extern uint16_t speed, speed_old, speed_count, spc;
//extern uint16_t theta_ini;
//extern uint16_t speed_buf[12], buffer_count;
//extern  int16_t wm_hall;
//extern  int16_t theta_hall, speed_avg;
//extern uint8_t  hall_index;
//extern uint32_t speed_sum, theta_accum;
//// 霍爾狀態到角度的對照表
//extern const uint16_t HALL_ANGLE_TABLE[8];

/****************** 三角函數建表  ******************/
static const uint16_t sintable[312] = { 0,10,21,31,41,51,62,72,82,93,103,113,123,134,144,154,165,
		  175,185,195,206,216,226,236,246,257,267,277,287,297,308,318,
		  328,338,348,358,369,379,389,399,409,419,429,439,449,459,469,
		  479,489,499,509,519,529,539,549,559,569,579,589,599,608,618,
		  628,638,648,657,667,677,686,696,706,715,725,735,744,754,763,
		  773,783,792,802,811,820,830,839,849,858,867,877,886,895,904,
		  914,923,932,941,950,959,969,978,987,996,1005,1014,1023,1031,
		  1040,1049,1058,1067,1076,1084,1093,1102,1110,1119,1128,1136,
		  1145,1153,1162,1170,1179,1187,1195,1204,1212,1220,1229,1237,
		  1245,1253,1261,1269,1277,1286,1294,1301,1309,1317,1325,1333,
		  1341,1349,1356,1364,1372,1379,1387,1394,1402,1409,1417,1424,
		  1432,1439,1446,1454,1461,1468,1475,1482,1489,1496,1503,1510,
		  1517,1524,1531,1538,1545,1551,1558,1565,1571,1578,1585,1591,
		  1598,1604,1610,1617,1623,1629,1635,1642,1648,1654,1660,1666,
		  1672,1678,1684,1690,1695,1701,1707,1712,1718,1724,1729,1735,
		  1740,1746,1751,1756,1761,1767,1772,1777,1782,1787,1792,1797,
		  1802,1807,1812,1817,1821,1826,1831,1835,1840,1844,1849,1853,
		  1857,1862,1866,1870,1874,1879,1883,1887,1891,1895,1898,1902,
		  1906,1910,1914,1917,1921,1924,1928,1931,1935,1938,1941,1945,
		  1948,1951,1954,1957,1960,1963,1966,1969,1972,1974,1977,1980,
		  1982,1985,1987,1990,1992,1995,1997,1999,2001,2004,2006,2008,
		  2010,2012,2014,2015,2017,2019,2021,2022,2024,2026,2027,2028,
		  2030,2031,2032,2034,2035,2036,2037,2038,2039,2040,2041,2042,
		  2043,2043,2044,2045,2045,2046,2046,2047,2047,2047,2047,2048,
		  2048,2048};
static const uint16_t costable[312] = { 2048,2048,2048,2048,2047,2047,2047,2046,2046,2045,2045,2044,
		  2044,2043,2042,2041,2041,2040,2039,2038,2037,2035,2034,2033,
		  2032,2030,2029,2028,2026,2025,2023,2021,2020,2018,2016,2014,
		  2013,2011,2009,2007,2005,2002,2000,1998,1996,1993,1991,1988,
		  1986,1983,1981,1978,1976,1973,1970,1967,1964,1961,1958,1955,
		  1952,1949,1946,1943,1939,1936,1933,1929,1926,1922,1919,1915,
		  1911,1908,1904,1900,1896,1892,1888,1884,1880,1876,1872,1868,
		  1863,1859,1855,1850,1846,1841,1837,1832,1828,1823,1818,1813,
		  1809,1804,1799,1794,1789,1784,1779,1774,1768,1763,1758,1753,
		  1747,1742,1736,1731,1725,1720,1714,1709,1703,1697,1691,1685,
		  1680,1674,1668,1662,1656,1650,1643,1637,1631,1625,1618,1612,
		  1606,1599,1593,1586,1580,1573,1567,1560,1553,1547,1540,1533,
		  1526,1519,1512,1505,1498,1491,1484,1477,1470,1463,1455,1448,
		  1441,1433,1426,1419,1411,1404,1396,1389,1381,1373,1366,1358,
		  1350,1343,1335,1327,1319,1311,1303,1295,1287,1279,1271,1263,
		  1255,1247,1239,1230,1222,1214,1205,1197,1189,1180,1172,1163,
		  1155,1146,1138,1129,1121,1112,1103,1095,1086,1077,1068,1060,
		  1051,1042,1033,1024,1015,1006,997,988,979,970,961,952,943,
		  933,924,915,906,897,887,878,869,859,850,841,831,822,812,803,
		  793,784,774,765,755,745,736,726,717,707,697,688,678,668,658,
		  649,639,629,619,609,599,590,580,570,560,550,540,530,520,510,
		  500,490,480,470,460,450,440,430,420,410,400,389,379,369,359,
		  349,339,329,318,308,298,288,278,267,257,247,237,226,216,206,
		  196,185,175,165,155,144,134,124,113,103,93,82,72,62,52,41,31,
		  21,10,0};

// // sin 建表  一圈1250個點   90度-----> 312個點
//static const unsigned int sintable[312]


#endif /* INC_CONTROL1_H_ */
