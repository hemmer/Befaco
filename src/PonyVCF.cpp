#include "plugin.hpp"
#include <sst/filters.h>

using simd::float_4;


template <typename T>
static T clip(T x) {
	// return std::tanh(x);
	// Pade approximant of tanh
	x = simd::clamp(x, -3.f, 3.f);
	return x * (27 + x * x) / (27 + 9 * x * x);
}


struct PonyVCF : Module {
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
	enum Cv1Mode {
		FILTER_VC_AND_VCA,
		FILTER_VC_ONLY,
		NUM_CV1_OPTIONS
	};

	// 4x quad = 16 channels of polyphony
	sst::filters::QuadFilterUnitState qfus[4];
	sst::filters::FilterCoefficientMaker<> coefMaker[16];
	sst::filters::FilterUnitQFPtr filterUnitPtr{nullptr};
	const int BLOCK_SIZE = 8;
	dsp::ClockDivider paramsUpdate;
	float_4 prevOut[4] = {};

	DCBlockerT<2, float_4> dcBlocker[4];
	bool removeDC = true;

	float delayBufferData[4][sst::filters::utilities::MAX_FB_COMB + sst::filters::utilities::SincTable::FIRipol_N] {};

	PonyVCF() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CV1_PARAM, 0.f, 1.f, 1.f, "CV1 Attenuator");
		configParam(RES_PARAM, 0.f, 1.f, 0.f, "Resonance");
		configParam(FREQ_PARAM, -4.f, 7.f, 0.f, "Frequency");
		configParam(GAIN1_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 1");
		configParam(GAIN2_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 2");
		configParam(GAIN3_PARAM, 0.f, 1.25f, 1.f, "Gain Channel 3");
		configSwitch(ROUTING_PARAM, FILTER_VC_AND_VCA, NUM_CV1_OPTIONS, FILTER_VC_ONLY, "VCA routing", {"CV1 (Filter CV and VCA)", "CV1 (Filter CV only)"});

		configInput(IN1_INPUT, "Channel 1");
		configInput(RES_INPUT, "Resonance CV");
		configInput(VCA_INPUT, "VCA");
		configInput(IN2_INPUT, "Channel 2");
		configInput(CV1_INPUT, "Frequency (CV1)");
		configInput(IN3_INPUT, "Channel 3");
		configInput(CV2_INPUT, "Frequency (CV2)");

		configOutput(OUTPUT, "Main");

		paramsUpdate.setDivision(BLOCK_SIZE);

		filterUnitPtr = sst::filters::GetQFPtrFilterUnit(sst::filters::FilterType::fut_vintageladder,
		                sst::filters::FilterSubType::st_vintage_type1_compensated);

		onSampleRateChange();
	}


	void onSampleRateChange() override {

		for (int c = 0; c < 4; ++c) {
			std::fill(qfus[c].R, &qfus[c].R[sst::filters::n_filter_registers], _mm_setzero_ps());
			std::fill(qfus[c].C, &qfus[c].C[sst::filters::n_cm_coeffs], _mm_setzero_ps());

			dcBlocker[c].setFrequency(5.0f / APP->engine->getSampleRate());
			dcBlocker[c].reset();

			for (int i = 0; i < 4; ++i) {
				std::fill(delayBufferData[i], delayBufferData[i] + sst::filters::utilities::MAX_FB_COMB + sst::filters::utilities::SincTable::FIRipol_N, 0.0f);
				qfus[c].DB[i] = delayBufferData[i];
				qfus[c].active[i] = (int) 0xffffffff;
				qfus[c].WP[i] = 0;

				coefMaker[4 * c + i].setSampleRateAndBlockSize(APP->engine->getSampleRate(), BLOCK_SIZE);
				coefMaker[4 * c + i].Reset();
			}
		}
	}


	void process(const ProcessArgs& args) override {
		if (!outputs[OUTPUT].isConnected()) {
			return;
		}

		int numActivePolyEngines = std::max({1, inputs[IN1_INPUT].getChannels(), inputs[IN2_INPUT].getChannels(), inputs[IN3_INPUT].getChannels()});
		numActivePolyEngines = std::max({numActivePolyEngines, inputs[CV1_INPUT].getChannels(), inputs[CV2_INPUT].getChannels()});
		numActivePolyEngines = std::max({numActivePolyEngines, inputs[RES_INPUT].getChannels(), inputs[VCA_INPUT].getChannels()});

		// only process every BLOCK_SIZE
		if (paramsUpdate.process()) {
			updateFilterParameters(numActivePolyEngines);
		}

		// process channels in blocks of 4
		for (int c = 0; c < numActivePolyEngines; c += 4) {

			// Add -120dB noise to bootstrap self-oscillation
			float_4 input = 1e-6f * (2.f * random::uniform() - 1.f);
			input += inputs[IN1_INPUT].getPolyVoltageSimd<float_4>(c) * params[GAIN1_PARAM].getValue();
			input += inputs[IN2_INPUT].getPolyVoltageSimd<float_4>(c) * params[GAIN2_PARAM].getValue();
			input += inputs[IN3_INPUT].getNormalPolyVoltageSimd<float_4>(prevOut[c / 4], c) * params[GAIN3_PARAM].getValue();

			// soft clipping of inputs
			input = clip(input / 5.0f) * 1.1f;

			float_4 out = 5.f * filterUnitPtr(&qfus[c / 4], input.v);

			float_4 gain = 1.f;
			if (inputs[VCA_INPUT].isConnected()) {
				// VCA sets gain, and takes precedence (if connected)
				gain = clamp(inputs[VCA_INPUT].getPolyVoltageSimd<float_4>(c) / 10.f, 0.f, 1.f);
			}
			else if (params[ROUTING_PARAM].getValue() == FILTER_VC_AND_VCA && inputs[CV1_INPUT].isConnected()) {
				// otherwise CV1 can optionally act as dual VCA/VCF control
				gain = clamp(inputs[CV1_INPUT].getPolyVoltageSimd<float_4>(c) / 10.f, 0.f, 1.f);
			}
			out = out * gain;

			if (removeDC) {
				out = dcBlocker[c / 4].process(out);
			}

			outputs[OUTPUT].setVoltageSimd(out, c);

			// store previous out for feedback
			prevOut[c / 4] = out;
		}

		outputs[OUTPUT].setChannels(numActivePolyEngines);
	}

	void updateFilterParameters(int numActivePolyEngines) {

		float resParam = params[RES_PARAM].getValue();
		float freqParam = params[FREQ_PARAM].getValue();
		float freqCvParam = params[CV1_PARAM].getValue();

		// process channels in blocks of 4
		for (int c = 0; c < numActivePolyEngines; c += 4) {

			// Set resonance
			float_4 resonance = resParam + inputs[RES_INPUT].getPolyVoltageSimd<float_4>(c) / 10.f;
			resonance = clamp(resonance, 0.f, 1.f);

			// Get pitch
			float_4 voct = freqParam + inputs[CV1_INPUT].getPolyVoltageSimd<float_4>(c) * freqCvParam + inputs[CV2_INPUT].getPolyVoltageSimd<float_4>(c);
			float_4 pitch_midi = (voct + 5) * 12 - 69;

			// serially update each of the four internal channels of the quadfilterstate object
			for (int i = 0; i < 4; i++) {
				// these need to copy internal parametrs
				for (int f = 0; f < sst::filters::n_cm_coeffs; ++f) {
					coefMaker[c + i].C[f] = qfus[c / 4].C[f][i];
				}

				coefMaker[c + i].MakeCoeffs(pitch_midi[i], resonance[i], sst::filters::FilterType::fut_vintageladder,
				                            sst::filters::FilterSubType::st_vintage_type1_compensated, nullptr, true);

				coefMaker[c + i].updateState(qfus[c / 4], i);
			}

		}
	}


	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "removeDC", json_boolean(removeDC));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {

		json_t* removeDCJ = json_object_get(rootJ, "removeDC");
		if (removeDCJ) {
			removeDC = json_boolean_value(removeDCJ);
		}
	}
};


struct PonyVCFWidget : ModuleWidget {
	PonyVCFWidget(PonyVCF* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/PonyVCF.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<BefacoTinyKnobDarkGrey>(mm2px(Vec(7.62, 14.5)), module, PonyVCF::CV1_PARAM));
		addParam(createParamCentered<BefacoTinyKnobRed>(mm2px(Vec(22.38, 14.5)), module, PonyVCF::RES_PARAM));
		addParam(createParamCentered<Davies1900hLargeGreyKnob>(mm2px(Vec(15.0, 35.001)), module, PonyVCF::FREQ_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(3.217, 48.584)), module, PonyVCF::GAIN1_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(13.271, 48.584)), module, PonyVCF::GAIN2_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(23.316, 48.584)), module, PonyVCF::GAIN3_PARAM));
		addParam(createParam<CKSSNarrow>(mm2px(Vec(23.498, 96.784)), module, PonyVCF::ROUTING_PARAM));

		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 86.5)), module, PonyVCF::IN1_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 86.5)), module, PonyVCF::RES_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.0, 86.5)), module, PonyVCF::VCA_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 100.0)), module, PonyVCF::IN2_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 100.0)), module, PonyVCF::CV1_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.0, 113.5)), module, PonyVCF::IN3_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.0, 113.5)), module, PonyVCF::CV2_INPUT));

		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(25.0, 113.5)), module, PonyVCF::OUTPUT));

		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(2.578, 23.492)), module, PonyVCF::IN2_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(2.578, 27.159)), module, PonyVCF::IN1_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		PonyVCF* module = dynamic_cast<PonyVCF*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());
		menu->addChild(createSubmenuItem("Hardware compatibility", "",
		[ = ](Menu * menu) {
			menu->addChild(createBoolPtrMenuItem("Remove DC from ouptut", "", &module->removeDC));
		}
		                                ));
	}
};


Model* modelPonyVCF = createModel<PonyVCF, PonyVCFWidget>("PonyVCF");