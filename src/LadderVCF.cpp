#include "plugin.hpp"
#include "StilsonModel.h"
#include "OberheimVariationModel.h"
#include "SimplifiedModel.h"
#include "ImprovedModel.h"
#include "HuovilainenModel.h"
#include "KrajeskiModel.h"
#include "RKSimulationModel.h"
#include "MicrotrackerModel.h"
#include "MusicDSPModel.h"
#include "RKSimulationModel.h"
#include "Karlsen.h"

static std::vector<std::string> filterModelNames = {"Stilson", "Oberheim", "Simplified", "Improved",
                                                    "Huovilainen", "Krajeski", "RK Simulation", "Microtracker", "MusicDSP",
													"Karlsen"};
                                                   


struct LadderVCF : Module {
	enum ParamId {
		CV1_PARAM,
		RES_PARAM,
		FREQ_PARAM,
		GAIN1_PARAM,
		GAIN2_PARAM,
		GAIN3_PARAM,
		ROUTING_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		IN1_INPUT,
		RES_INPUT,
		VCA_INPUT,
		IN2_INPUT,
		CV1_INPUT,
		IN3_INPUT,
		CV2_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		IN2_LIGHT,
		IN1_LIGHT,
		LIGHTS_LEN
	};

	enum FilterModels {
		STILSON,
		OBERHEIM,
		SIMPLIFIED,
		IMPROVED,
		HUOVILAINEN,
		KRAJESKI,
		RKSIMULATION,
		MICROTRACKER,
		MUSIC_DSP,
		KARLSEN,
		MODELS_LEN
	};
	FilterModels filterModel = STILSON;

	StilsonMoog stilsonLadder {44100.f};
	OberheimVariationMoog oberheimLadder {44100.f};
	SimplifiedMoog simplifiedLadder {44100.f};
	ImprovedMoog improvedLadder {44100.f};
	HuovilainenMoog huovilainenLadder {44100.f};
	KrajeskiMoog krajeskiLadder {44100.f};
	RKSimulationMoog rkSimulationLadder {44100.f};
	MicrotrackerMoog microtrackerLadder {44100.f};
	MusicDSPMoog musicDSPLadder {44100.f};
	KarlsenMoog karlsenLadder {44100.f};

	LadderFilterBase *ladderFilter = &stilsonLadder;

	LadderVCF() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CV1_PARAM, 0.f, 1.f, 1.f, "CV1 Attenuator");
		configParam(RES_PARAM, 0.f, 10.f, 0.f, "Resonance");
		configParam(FREQ_PARAM, 0.f, 1.f, 0.f, "Frequency");
		configParam(GAIN1_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 1");
		configParam(GAIN2_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 2");
		configParam(GAIN3_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 3");
		configSwitch(ROUTING_PARAM, 0.f, 1.f, 0.f, "VCA routing", {"CV1 (Filter CV and VCA)", "CV1 (Filter CV only)"});

		configInput(IN1_INPUT, "Channel 1");
		configInput(RES_INPUT, "Resonance CV");
		configInput(VCA_INPUT, "VCA");
		configInput(IN2_INPUT, "Channel 2");
		configInput(CV1_INPUT, "Frequency (CV1)");
		configInput(IN3_INPUT, "Channel 3");
		configInput(CV2_INPUT, "Frequency (CV2)");

		configOutput(OUTPUT, "Main");

		onReset();
	}

	float prevOut = 0.f;

	void process(const ProcessArgs& args) override {
		if (!outputs[OUTPUT].isConnected()) {
			return;
		}

		switch (filterModel) {
			case STILSON: ladderFilter = &stilsonLadder; break;
			case OBERHEIM: ladderFilter = &oberheimLadder; break;
			case SIMPLIFIED: ladderFilter = &simplifiedLadder; break;
			case IMPROVED: ladderFilter = &improvedLadder; break;
			case HUOVILAINEN: ladderFilter = &huovilainenLadder; break;
			case KRAJESKI: ladderFilter = &krajeskiLadder; break;
			case RKSIMULATION: ladderFilter = &rkSimulationLadder; break;
			case MICROTRACKER: ladderFilter = &microtrackerLadder; break;
			case MUSIC_DSP: ladderFilter = &musicDSPLadder; break;
			case KARLSEN: ladderFilter = &karlsenLadder; break;
		}

		float resParam = params[RES_PARAM].getValue();
		float freqParam = params[FREQ_PARAM].getValue();
		float freqCvParam = params[CV1_PARAM].getValue();

		// Set resonance
		float resonance = resParam + inputs[RES_INPUT].getVoltage() / 10.f;
		// resonance = clamp(resonance, 0.f, 1.f);
		ladderFilter->SetResonance(resonance);

		// Get pitch
		float pitch = 7 * freqParam;
		// Set cutoff
		float cutoff = dsp::FREQ_C4 * dsp::exp2_taylor5(pitch);
		cutoff = clamp(cutoff, 0.f, 44100.f / 2.f);
		ladderFilter->SetCutoff(cutoff);

		float in[1] = {params[GAIN1_PARAM].getValue() * inputs[IN1_INPUT].getVoltage() + prevOut * params[GAIN3_PARAM].getValue()};
		in[0] = in[0] / 5.f;
		
		ladderFilter->Process(in, 1);
		outputs[OUTPUT].setVoltage(in[0] * 5.f);
		prevOut = in[0];
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "filterModel", json_integer(filterModel));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* filterModelJ = json_object_get(rootJ, "filterModel");
		if (filterModelJ) {
			filterModel = (FilterModels) json_integer_value(filterModelJ);
		}
	}

};


struct LadderVCFWidget : ModuleWidget {
	LadderVCFWidget(LadderVCF* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/LadderVCF.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<BefacoTinyKnobDarkGrey>(mm2px(Vec(7.62, 14.5)), module, LadderVCF::CV1_PARAM));
		addParam(createParamCentered<BefacoTinyKnobRed>(mm2px(Vec(22.38, 14.5)), module, LadderVCF::RES_PARAM));
		addParam(createParamCentered<Davies1900hLargeGreyKnob>(mm2px(Vec(15.0, 35.001)), module, LadderVCF::FREQ_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(3.217, 48.584)), module, LadderVCF::GAIN1_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(13.271, 48.584)), module, LadderVCF::GAIN2_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(23.316, 48.584)), module, LadderVCF::GAIN3_PARAM));
		addParam(createParam<CKSSNarrow>(mm2px(Vec(23.498, 96.784)), module, LadderVCF::ROUTING_PARAM));

		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 86.5)), module, LadderVCF::IN1_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 86.5)), module, LadderVCF::RES_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.0, 86.5)), module, LadderVCF::VCA_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 100.0)), module, LadderVCF::IN2_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 100.0)), module, LadderVCF::CV1_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 113.5)), module, LadderVCF::IN3_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 113.5)), module, LadderVCF::CV2_INPUT));

		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(25.0, 113.5)), module, LadderVCF::OUTPUT));

		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(2.578, 23.492)), module, LadderVCF::IN2_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(2.578, 27.159)), module, LadderVCF::IN1_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		LadderVCF* module = dynamic_cast<LadderVCF*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());
		menu->addChild(createIndexPtrSubmenuItem("Model", filterModelNames, &module->filterModel));
	}
};


Model* modelLadderVCF = createModel<LadderVCF, LadderVCFWidget>("LadderVCF");