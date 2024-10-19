#include "plugin.hpp"

using namespace simd;

struct Bandit : Module {
	enum ParamId {
		LOW_GAIN_PARAM,
		LOW_MID_GAIN_PARAM,
		HIGH_MID_GAIN_PARAM,
		HIGH_GAIN_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		LOW_INPUT,
		LOW_MID_INPUT,
		HIGH_MID_INPUT,
		HIGH_INPUT,
		LOW_RETURN_INPUT,
		LOW_MID_RETURN_INPUT,
		HIGH_MID_RETURN_INPUT,
		HIGH_RETURN_INPUT,
		LOW_CV_INPUT,
		LOW_MID_CV_INPUT,
		HIGH_MID_CV_INPUT,
		HIGH_CV_INPUT,
		ALL_INPUT,
		ALL_CV_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		LOW_OUTPUT,
		LOW_MID_OUTPUT,
		HIGH_MID_OUTPUT,
		HIGH_OUTPUT,
		MIX_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		ENUMS(MIX_CLIP_LIGHT, 3),
		ENUMS(MIX_LIGHT, 3),
		LIGHTS_LEN
	};

	// float_4 * [4] give 16 polyphony channels, [2] is for cascading biquads
	dsp::TBiquadFilter<float_4> filterLow[4][2], filterLowMid[4][2], filterHighMid[4][2], filterHigh[4][2];


	Bandit() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(LOW_GAIN_PARAM, 0.f, 1.f, 0.f, "Low gain");
		configParam(LOW_MID_GAIN_PARAM, 0.f, 1.f, 0.f, "Low mid gain");
		configParam(HIGH_MID_GAIN_PARAM, 0.f, 1.f, 0.f, "High mid gain");
		configParam(HIGH_GAIN_PARAM, 0.f, 1.f, 0.f, "High gain");

		configInput(LOW_INPUT, "Low");
		configInput(LOW_MID_INPUT, "Low mid");
		configInput(HIGH_MID_INPUT, "High mid");
		configInput(HIGH_INPUT, "High");
		configInput(LOW_RETURN_INPUT, "Low return");
		configInput(LOW_MID_RETURN_INPUT, "Low mid return");
		configInput(HIGH_MID_RETURN_INPUT, "High mid return");
		configInput(HIGH_RETURN_INPUT, "High return");
		configInput(LOW_CV_INPUT, "Low CV");
		configInput(LOW_MID_CV_INPUT, "Low mid CV");
		configInput(HIGH_MID_CV_INPUT, "High mid CV");
		configInput(HIGH_CV_INPUT, "High CV");
		configInput(ALL_INPUT, "All");
		configInput(ALL_CV_INPUT, "All CV");

		configOutput(LOW_OUTPUT, "Low");
		configOutput(LOW_MID_OUTPUT, "Low mid");
		configOutput(HIGH_MID_OUTPUT, "High mid");
		configOutput(HIGH_OUTPUT, "High");
		configOutput(MIX_OUTPUT, "Mix");
	}

	void onSampleRateChange() override {
		const float sr = APP->engine->getSampleRate();
		const float lowFc = 300.f / sr;
		const float lowMidFc = 750.f / sr;
		const float highMidFc = 1500.f / sr;
		const float highFc = 3800.f / sr;
		// Qs for cascaded biquads to get Butterworth response, see https://www.earlevel.com/main/2016/09/29/cascading-filters/
		// technically only for LOWPASS and HIGHPASS, but seems to work well for BANDPASS too
		const float Q[2] = {0.54119610f, 1.3065630f};
		const float V = 1.f;

		for (int i = 0; i < 4; ++i) {
			for (int stage = 0; stage < 2; ++stage) {
				filterLow[i][stage].setParameters(dsp::TBiquadFilter<float_4>::Type::LOWPASS, lowFc, Q[stage], V);
				filterLowMid[i][stage].setParameters(dsp::TBiquadFilter<float_4>::Type::BANDPASS, lowMidFc, Q[stage], V);
				filterHighMid[i][stage].setParameters(dsp::TBiquadFilter<float_4>::Type::BANDPASS, highMidFc, Q[stage], V);
				filterHigh[i][stage].setParameters(dsp::TBiquadFilter<float_4>::Type::HIGHPASS, highFc, Q[stage], V);
			}
		}
	}

	void processBypass(const ProcessArgs& args) override {
		const int maxPolyphony = std::max({1, inputs[ALL_INPUT].getChannels(), inputs[LOW_INPUT].getChannels(),
		                                   inputs[LOW_MID_INPUT].getChannels(), inputs[HIGH_MID_INPUT].getChannels(),
		                                   inputs[HIGH_INPUT].getChannels()});


		for (int c = 0; c < maxPolyphony; c += 4) {
			const float_4 inLow = inputs[LOW_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inLowMid = inputs[LOW_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHighMid = inputs[HIGH_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHigh = inputs[HIGH_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inAll = inputs[ALL_INPUT].getPolyVoltageSimd<float_4>(c);

			// bypass sums all inputs to the output
			outputs[MIX_OUTPUT].setVoltageSimd<float_4>(inLow + inLowMid + inHighMid + inHigh + inAll, c);
		}

		outputs[MIX_OUTPUT].setChannels(maxPolyphony);
	}


	void process(const ProcessArgs& args) override {

		const int maxPolyphony = std::max({1, inputs[ALL_INPUT].getChannels(), inputs[LOW_INPUT].getChannels(),
		                                   inputs[LOW_MID_INPUT].getChannels(), inputs[HIGH_MID_INPUT].getChannels(),
		                                   inputs[HIGH_INPUT].getChannels()});

		const bool allReturnsActiveAndMonophonic = inputs[LOW_RETURN_INPUT].isMonophonic() && inputs[LOW_MID_RETURN_INPUT].isMonophonic() &&
		  inputs[HIGH_MID_RETURN_INPUT].isMonophonic() && inputs[HIGH_RETURN_INPUT].isMonophonic();

		float_4 mixOutput;
		for (int c = 0; c < maxPolyphony; c += 4) {

			const float_4 inLow = inputs[LOW_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inLowMid = inputs[LOW_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHighMid = inputs[HIGH_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHigh = inputs[HIGH_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inAll = inputs[ALL_INPUT].getPolyVoltageSimd<float_4>(c);

			const float_4 lowGain = params[LOW_GAIN_PARAM].getValue() * inputs[LOW_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outLow = 0.7 * 2 * filterLow[c / 4][1].process(filterLow[c / 4][0].process((inLow + inAll) * lowGain));
			outputs[LOW_OUTPUT].setVoltageSimd<float_4>(outLow, c);

			const float_4 lowMidGain = params[LOW_MID_GAIN_PARAM].getValue() * inputs[LOW_MID_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outLowMid = 2 * filterLowMid[c / 4][1].process(filterLowMid[c / 4][0].process((inLowMid + inAll) * lowMidGain));
			outputs[LOW_MID_OUTPUT].setVoltageSimd<float_4>(outLowMid, c);

			const float_4 highMidGain = params[HIGH_MID_GAIN_PARAM].getValue() * inputs[HIGH_MID_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outHighMid = 2 * filterHighMid[c / 4][1].process(filterHighMid[c / 4][0].process((inHighMid + inAll) * highMidGain));
			outputs[HIGH_MID_OUTPUT].setVoltageSimd<float_4>(outHighMid, c);

			const float_4 highGain = params[HIGH_GAIN_PARAM].getValue() * inputs[HIGH_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outHigh = 0.7 * 2 * filterHigh[c / 4][1].process(filterHigh[c / 4][0].process((inHigh + inAll) * highGain));
			outputs[HIGH_OUTPUT].setVoltageSimd<float_4>(outHigh, c);

			// the fx return input is normalled to the fx send output
			mixOutput  = inputs[LOW_RETURN_INPUT].getNormalPolyVoltageSimd<float_4>(outLow, c);
			mixOutput += inputs[LOW_MID_RETURN_INPUT].getNormalPolyVoltageSimd<float_4>(outLowMid, c);
			mixOutput += inputs[HIGH_MID_RETURN_INPUT].getNormalPolyVoltageSimd<float_4>(outHighMid, c);
			mixOutput += inputs[HIGH_RETURN_INPUT].getNormalPolyVoltageSimd<float_4>(outHigh, c);
			mixOutput = mixOutput * clamp(inputs[ALL_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f, 0.f, 1.f);

			outputs[MIX_OUTPUT].setVoltageSimd<float_4>(mixOutput, c);
		}

		outputs[LOW_OUTPUT].setChannels(maxPolyphony);
		outputs[LOW_MID_OUTPUT].setChannels(maxPolyphony);
		outputs[HIGH_MID_OUTPUT].setChannels(maxPolyphony);
		outputs[HIGH_OUTPUT].setChannels(maxPolyphony);

		if (allReturnsActiveAndMonophonic) {
			// special case: if all return paths are connected and monophonic, then output mix should be monophonic			
			outputs[MIX_OUTPUT].setChannels(1);
		}
		else {
			// however, if it's a mix (some normalled from input, maybe some polyphonic), then it should be polyphonic
			outputs[MIX_OUTPUT].setChannels(maxPolyphony);
		}

		if (maxPolyphony == 1) {
			lights[MIX_LIGHT + 0].setBrightness(0.f);
			lights[MIX_LIGHT + 1].setBrightnessSmooth(outputs[MIX_OUTPUT].getVoltageRMS(), args.sampleTime);
			lights[MIX_LIGHT + 2].setBrightness(0.f);
		}
		else {
			lights[MIX_LIGHT + 0].setBrightness(0.f);
			lights[MIX_LIGHT + 1].setBrightness(0.f);
			lights[MIX_LIGHT + 2].setBrightnessSmooth(outputs[MIX_OUTPUT].getVoltageRMS(), args.sampleTime);
		}
	}
};


struct BanditWidget : ModuleWidget {
	BanditWidget(Bandit* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/Bandit.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<BefacoSlidePot>(mm2px(Vec(3.062, 51.365)), module, Bandit::LOW_GAIN_PARAM));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(13.23, 51.365)), module, Bandit::LOW_MID_GAIN_PARAM));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(23.398, 51.365)), module, Bandit::HIGH_MID_GAIN_PARAM));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(33.566, 51.365)), module, Bandit::HIGH_GAIN_PARAM));

		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.038, 14.5)), module, Bandit::LOW_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.178, 14.5)), module, Bandit::LOW_MID_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.253, 14.5)), module, Bandit::HIGH_MID_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(35.328, 14.5)), module, Bandit::HIGH_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.045, 40.34)), module, Bandit::LOW_RETURN_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.118, 40.34)), module, Bandit::LOW_MID_RETURN_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.19, 40.338)), module, Bandit::HIGH_MID_RETURN_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(35.263, 40.34)), module, Bandit::HIGH_RETURN_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.038, 101.229)), module, Bandit::LOW_CV_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.113, 101.229)), module, Bandit::LOW_MID_CV_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.187, 101.231)), module, Bandit::HIGH_MID_CV_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(35.263, 101.229)), module, Bandit::HIGH_CV_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(10.075, 113.502)), module, Bandit::ALL_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(20.15, 113.5)), module, Bandit::ALL_CV_INPUT));

		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(5.045, 27.248)), module, Bandit::LOW_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(15.118, 27.256)), module, Bandit::LOW_MID_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(25.19, 27.256)), module, Bandit::HIGH_MID_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(35.263, 27.256)), module, Bandit::HIGH_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(30.225, 113.5)), module, Bandit::MIX_OUTPUT));

		addChild(createLightCentered<MediumLight<RedGreenBlueLight>>(mm2px(Vec(37.781, 111.125)), module, Bandit::MIX_CLIP_LIGHT));
		addChild(createLightCentered<MediumLight<RedGreenBlueLight>>(mm2px(Vec(37.781, 115.875)), module, Bandit::MIX_LIGHT));
	}
};

Model* modelBandit = createModel<Bandit, BanditWidget>("Bandit");