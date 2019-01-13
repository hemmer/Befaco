#include "Befaco.hpp"


Plugin *plugin;

void init(rack::Plugin *p) {
	plugin = p;

	p->addModel(modelEvenVCO);
	p->addModel(modelRampage);
	p->addModel(modelABC);
	p->addModel(modelSpringReverb);
	p->addModel(modelMixer);
	p->addModel(modelSlewLimiter);
	p->addModel(modelDualAtenuverter);
}
