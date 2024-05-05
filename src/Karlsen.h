#pragma once


#include "LadderFilterBase.h"
#include "Util.h"

class KarlsenMoog : public LadderFilterBase
{
public:
	
	KarlsenMoog(float sampleRate) : LadderFilterBase(sampleRate) {}
	~KarlsenMoog() {}
	
	void Process(float * samples, uint32_t n) {
		double b_fres = resonance;
		double b_fenv = cutoff;

        double b_rez = b_aflt4 - b_v; // no attenuation with rez, makes a stabler filter.
        b_v = b_v - (b_rez*b_fres); // b_fres = resonance amount. 0..4 typical "to selfoscillation", 0.6 covers a more saturated range.

        double b_vnc = b_v; // clip, and adding back some nonclipped, to get a dynamic like analog.
        if (b_v > 1) {b_v = 1;} else if (b_v < -1) {b_v = -1;}
        b_v = b_vnc + ((-b_vnc + b_v) * 0.9840);

        b_aflt1 = b_aflt1 + ((-b_aflt1 + b_v) * b_fenv); // straightforward 4 pole filter, (4 normalized feedback paths in series)
        b_aflt2 = b_aflt2 + ((-b_aflt2 + b_aflt1) * b_fenv);
        b_aflt3 = b_aflt3 + ((-b_aflt3 + b_aflt2) * b_fenv);
        b_aflt4 = b_aflt4 + ((-b_aflt4 + b_aflt3) * b_fenv);
        b_v = b_aflt4;
	}
	void SetResonance(float r) {

	}
	void SetCutoff(float c) {

	}
	

	
private:
	double b_aflt1 = 0;
	double b_aflt2 = 0;
	double b_aflt3 = 0;
	double b_aflt4 = 0;
	double b_v = 0;
};
