#pragma once

#include <tinympc/types.hpp>

tinytype rho_value = 5.0;

tinytype Adyn_data[NSTATES*NSTATES] = {
  1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0156960,	-0.0000000,	0.0400000,	0.0000000,	0.0000000,	0.0000000,	0.0001046,	-0.0000000,	
  0.0000000,	1.0000000,	0.0000000,	-0.0156960,	0.0000000,	-0.0000000,	0.0000000,	0.0400000,	0.0000000,	-0.0001046,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0400000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0200000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0000000,	1.0000000,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0200000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	0.0200000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.7848000,	-0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0078480,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.7848000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000,	0.0000000,	-0.0078480,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000	
};

tinytype Bdyn_data[NSTATES*NINPUTS] = {
  -0.0002895,	0.0003184,	0.0002904,	-0.0003193,	
  0.0002881,	0.0003173,	-0.0002885,	-0.0003170,	
  0.0033634,	0.0033634,	0.0033634,	0.0033634,	
  -0.1101418,	-0.1212936,	0.1102651,	0.1211704,	
  -0.1106828,	0.1217114,	0.1110278,	-0.1220564,	
  0.0078991,	-0.0028895,	-0.0111375,	0.0061279,	
  -0.0289546,	0.0318397,	0.0290449,	-0.0319299,	
  0.0288131,	0.0317304,	-0.0288453,	-0.0316982,	
  0.1681714,	0.1681714,	0.1681714,	0.1681714,	
  -11.0141843,	-12.1293615,	11.0265054,	12.1170403,	
  -11.0682807,	12.1711367,	11.1027800,	-12.2056361,	
  0.7899085,	-0.2889456,	-1.1137504,	0.6127876	
};

tinytype Kinf_data[NINPUTS*NSTATES] = {
  -0.0505204,	0.0507952,	0.0985056,	-0.2374880,	-0.2451204,	-0.1941408,	-0.0378839,	0.0378462,	0.1120117,	-0.0221103,	-0.0262536,	-0.1140414,	
  0.0481014,	0.0484684,	0.0985056,	-0.2133922,	0.2320917,	0.1938893,	0.0359528,	0.0356844,	0.1120117,	-0.0153129,	0.0250337,	0.1137365,	
  0.0507694,	-0.0508967,	0.0985056,	0.2264433,	0.2174808,	-0.1932599,	0.0373171,	-0.0376211,	0.1120117,	0.0165336,	0.0123092,	-0.1129941,	
  -0.0483504,	-0.0483669,	0.0985056,	0.2244369,	-0.2044521,	0.1935113,	-0.0353861,	-0.0359096,	0.1120117,	0.0208897,	-0.0110893,	0.1132991	
};

tinytype Quu_inv_data[NINPUTS*NINPUTS] = {
  0.0003992,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0003992,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0003992,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0003992	
};

tinytype AmBKt_data[NSTATES*NSTATES] = {
  0.9999399,	-0.0000014,	-0.0000000,	0.0005305,	-0.0229845,	0.0013998,	-0.0060127,	-0.0001388,	-0.0000000,	0.0530528,	-2.2984510,	0.1399781,	
  -0.0000014,	0.9999400,	0.0000000,	0.0229463,	-0.0005295,	-0.0005317,	-0.0001385,	-0.0060028,	0.0000000,	2.2946347,	-0.0529544,	-0.0531664,	
  -0.0000000,	0.0000000,	0.9986747,	-0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	0.0000000,	-0.0662633,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000051,	-0.0154234,	-0.0000000,	0.8957956,	0.0019389,	0.0024060,	0.0005072,	-0.7575401,	-0.0000000,	-10.4204377,	0.1938876,	0.2406042,	
  0.0154227,	-0.0000051,	-0.0000000,	0.0019463,	0.8955201,	0.0062819,	0.7574681,	-0.0005091,	-0.0000000,	0.1946255,	-10.4479900,	0.6281895,	
  -0.0000000,	0.0000000,	0.0000000,	-0.0000035,	-0.0000101,	0.9987555,	-0.0000026,	0.0000009,	0.0000000,	-0.0003512,	-0.0010067,	-0.1244478,	
  0.0399554,	-0.0000009,	-0.0000000,	0.0003612,	-0.0170313,	0.0010356,	0.9955446,	-0.0000945,	-0.0000000,	0.0361212,	-1.7031298,	0.1035593,	
  -0.0000009,	0.0399555,	0.0000000,	0.0169962,	-0.0003603,	-0.0003948,	-0.0000942,	0.9955538,	0.0000000,	1.6996215,	-0.0360273,	-0.0394798,	
  -0.0000000,	0.0000000,	0.0384930,	-0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	0.0000000,	0.9246513,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000003,	-0.0000820,	-0.0000000,	0.0113531,	0.0001306,	0.0001865,	0.0000342,	-0.0055860,	-0.0000000,	0.1353089,	0.0130554,	0.0186539,	
  0.0000820,	-0.0000003,	0.0000000,	0.0001312,	0.0113271,	0.0004848,	0.0055792,	-0.0000343,	0.0000000,	0.0131225,	0.1327108,	0.0484761,	
  -0.0000002,	0.0000001,	0.0000000,	-0.0000344,	-0.0000911,	0.0192767,	-0.0000238,	0.0000090,	0.0000000,	-0.0034418,	-0.0091091,	0.9276705	
};

tinytype coeff_d2p_data[NSTATES*NINPUTS] = {
  -126.5535225,	120.4939114,	127.1773871,	-121.1177761,	
  127.2419494,	121.4133516,	-127.4961891,	-121.1591119,	
  246.7564582,	246.7564582,	246.7564583,	246.7564582,	
  -594.9074028,	-534.5475728,	567.2404927,	562.2144830,	
  -614.0264770,	581.3897487,	544.7892918,	-512.1525634,	
  -486.3225852,	485.6927935,	-484.1161022,	484.7458938,	
  -94.8991791,	90.0618329,	93.4794251,	-88.6420789,	
  94.8048190,	89.3894776,	-94.2408699,	-89.9534267,	
  280.5892504,	280.5892505,	280.5892505,	280.5892504,	
  -55.3863394,	-38.3588304,	41.4165733,	52.3285965,	
  -65.7652625,	62.7093786,	30.8345945,	-27.7787105,	
  -285.6737990,	284.9098712,	-283.0502218,	283.8141497	
};

tinytype Q_data[NSTATES]= {100.0000000,	100.0000000,	100.0000000,	4.0000000,	4.0000000,	400.0000000,	4.0000000,	4.0000000,	4.0000000,	2.0408163,	2.0408163,	4.0000000};

tinytype Qf_data[NSTATES]= {100.0000000,	100.0000000,	100.0000000,	4.0000000,	4.0000000,	400.0000000,	4.0000000,	4.0000000,	4.0000000,	2.0408163,	2.0408163,	4.0000000};

tinytype R_data[NINPUTS]= {2500.0000000,	2500.0000000,	2500.0000000,	2500.0000000};

