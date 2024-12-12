#pragma once

#include <tinympc/types.hpp>

static const tinytype rho_value = 1.0;

static const tinytype Adyn_data[NSTATES*NSTATES]  = {
	1.000000f, 0.000000f, 0.000000f, 0.050000f, 0.000000f, 0.000000f, 
	0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.050000f, 0.000000f, 
	0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.050000f, 
	0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f
};

static const tinytype Bdyn_data[NSTATES*NINPUTS]  = {
	0.001250f, 0.000000f, 0.000000f, 
	0.000000f, 0.001250f, 0.000000f, 
	0.000000f, 0.000000f, 0.001250f, 
	0.050000f, 0.000000f, 0.000000f, 
	0.000000f, 0.050000f, 0.000000f, 
	0.000000f, 0.000000f, 0.050000f
};

static const tinytype fdyn_data[NSTATES]  = {0.000000f, 0.000000f, -0.012263f, 0.000000f, 0.000000f, -0.490500f};

static const tinytype Q_data[NSTATES] = {21.000000f, 21.000000f, 101.000000f, 11.000000f, 11.000000f, 6.000000f};

static const tinytype R_data[NINPUTS] = {2.000000f, 2.000000f, 2.000000f};

static const tinytype Kinf_data[NINPUTS*NSTATES]  = {
	2.971891f, 0.000000f, 0.000000f, 3.251175f, 0.000000f, 0.000000f, 
	0.000000f, 2.971891f, 0.000000f, 0.000000f, 3.251175f, 0.000000f, 
	0.000000f, 0.000000f, 6.406525f, 0.000000f, 0.000000f, 3.905289f
};

static const tinytype Pinf_data[NSTATES*NSTATES]  = {
	459.469559f, 0.000000f, 0.000000f, 129.837398f, 0.000000f, 0.000000f, 
	0.000000f, 459.469559f, 0.000000f, 0.000000f, 129.837398f, 0.000000f, 
	0.000000f, 0.000000f, 1231.351440f, 0.000000f, 0.000000f, 284.519771f, 
	129.837398f, 0.000000f, 0.000000f, 144.292951f, 0.000000f, 0.000000f, 
	0.000000f, 129.837398f, 0.000000f, 0.000000f, 144.292951f, 0.000000f, 
	0.000000f, 0.000000f, 284.519771f, 0.000000f, 0.000000f, 169.324545f
};

static const tinytype Quu_inv_data[NINPUTS*NINPUTS]  = {
	0.420578f, 0.000000f, 0.000000f, 
	0.000000f, 0.420578f, 0.000000f, 
	0.000000f, 0.000000f, 0.406372f
};

static const tinytype AmBKt_data[NSTATES*NSTATES]  = {
	0.996285f, 0.000000f, 0.000000f, -0.148595f, 0.000000f, 0.000000f, 
	0.000000f, 0.996285f, 0.000000f, 0.000000f, -0.148595f, 0.000000f, 
	0.000000f, 0.000000f, 0.991992f, 0.000000f, 0.000000f, -0.320326f, 
	0.045936f, 0.000000f, 0.000000f, 0.837441f, 0.000000f, 0.000000f, 
	0.000000f, 0.045936f, 0.000000f, 0.000000f, 0.837441f, 0.000000f, 
	0.000000f, 0.000000f, 0.045118f, 0.000000f, 0.000000f, 0.804736f
};

static const tinytype APf_data[NSTATES]  = {0.000000f, 0.000000f, -125.696013f, 0.000000f, 0.000000f, -76.621766f};

static const tinytype BPf_data[NINPUTS]  = {0.000000f, 0.000000f, -4.520451f};

