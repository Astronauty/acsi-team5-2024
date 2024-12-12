#pragma once

#include <tinympc/types.hpp>

static tinytype rho_value = 1.0;

static tinytype Adyn_data[NSTATES*NSTATES]  = {
	1.000000f, 0.000000f, 0.000000f, 0.050000f, 0.000000f, 0.000000f, 
	0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.050000f, 0.000000f, 
	0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.050000f, 
	0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f
};

static tinytype Bdyn_data[NSTATES*NINPUTS]  = {
	0.001250f, 0.000000f, 0.000000f, 
	0.000000f, 0.001250f, 0.000000f, 
	0.000000f, 0.000000f, 0.001250f, 
	0.050000f, 0.000000f, 0.000000f, 
	0.000000f, 0.050000f, 0.000000f, 
	0.000000f, 0.000000f, 0.050000f
};

static tinytype fdyn_data[NSTATES]  = {0.000000f, 0.000000f, -0.012263f, 0.000000f, 0.000000f, -0.490500f};

static tinytype Q_data[NSTATES] = {501.000000f, 501.000000f, 501.000000f, 501.000000f, 501.000000f, 501.000000f};

static tinytype R_data[NINPUTS] = {2.000000f, 2.000000f, 2.000000f};

static tinytype Kinf_data[NINPUTS*NSTATES]  = {
	10.501380f, 0.000000f, 0.000000f, 11.457825f, 0.000000f, 0.000000f, 
	0.000000f, 10.501380f, 0.000000f, 0.000000f, 11.457825f, 0.000000f, 
	0.000000f, 0.000000f, 10.501380f, 0.000000f, 0.000000f, 11.457825f
};

static tinytype Pinf_data[NSTATES*NSTATES]  = {
	10932.601338f, 0.000000f, 0.000000f, 680.845247f, 0.000000f, 0.000000f, 
	0.000000f, 10932.601338f, 0.000000f, 0.000000f, 680.845247f, 0.000000f, 
	0.000000f, 0.000000f, 10932.601338f, 0.000000f, 0.000000f, 680.845247f, 
	680.845247f, 0.000000f, 0.000000f, 976.334124f, 0.000000f, 0.000000f, 
	0.000000f, 680.845247f, 0.000000f, 0.000000f, 976.334124f, 0.000000f, 
	0.000000f, 0.000000f, 680.845247f, 0.000000f, 0.000000f, 976.334124f
};

static tinytype Quu_inv_data[NINPUTS*NINPUTS]  = {
	0.220118f, 0.000000f, 0.000000f, 
	0.000000f, 0.220118f, 0.000000f, 
	0.000000f, 0.000000f, 0.220118f
};

static tinytype AmBKt_data[NSTATES*NSTATES]  = {
	0.986873f, 0.000000f, 0.000000f, -0.525069f, 0.000000f, 0.000000f, 
	0.000000f, 0.986873f, 0.000000f, 0.000000f, -0.525069f, 0.000000f, 
	0.000000f, 0.000000f, 0.986873f, 0.000000f, 0.000000f, -0.525069f, 
	0.035678f, 0.000000f, 0.000000f, 0.427109f, 0.000000f, 0.000000f, 
	0.000000f, 0.035678f, 0.000000f, 0.000000f, 0.427109f, 0.000000f, 
	0.000000f, 0.000000f, 0.035678f, 0.000000f, 0.000000f, 0.427109f
};

static tinytype APf_data[NSTATES]  = {0.000000f, 0.000000f, -206.037082f, 0.000000f, 0.000000f, -224.802523f};

static tinytype BPf_data[NINPUTS]  = {0.000000f, 0.000000f, -24.947057f};
