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

	dsp::TBiquadFilter<float_4> filterLow[4], filterLowMid[4], filterHighMid[4], filterHigh[4];

	Bandit() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(LOW_GAIN_PARAM, -1.f, 1.f, 0.f, "Low gain");
		configParam(LOW_MID_GAIN_PARAM, -1.f, 1.f, 0.f, "Low mid gain");
		configParam(HIGH_MID_GAIN_PARAM, -1.f, 1.f, 0.f, "High mid gain");
		configParam(HIGH_GAIN_PARAM, -1.f, 1.f, 0.f, "High gain");

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

		onSampleRateChange();
	}

	void onSampleRateChange() override {
		const float sr = APP->engine->getSampleRate();
		const float lowFc = 300.f / sr;
		const float lowMidFc = 750.f / sr;
		const float highMidFc = 1500.f / sr;
		const float highFc = 5000.f / sr;
		const float Q = 1.f, V = 1.f;

		for (int i = 0; i < 4; ++i) {
			filterLow[i].setParameters(dsp::TBiquadFilter<float_4>::Type::LOWPASS, lowFc, Q, V);
			filterLowMid[i].setParameters(dsp::TBiquadFilter<float_4>::Type::BANDPASS, lowMidFc, Q, V);
			filterHighMid[i].setParameters(dsp::TBiquadFilter<float_4>::Type::BANDPASS, highMidFc, Q, V);
			filterHigh[i].setParameters(dsp::TBiquadFilter<float_4>::Type::HIGHPASS, highFc, Q, V);
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


		for (int c = 0; c < maxPolyphony; c += 4) {

			const float_4 inLow = inputs[LOW_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inLowMid = inputs[LOW_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHighMid = inputs[HIGH_MID_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inHigh = inputs[HIGH_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inAll = inputs[ALL_INPUT].getPolyVoltageSimd<float_4>(c);

			const float_4 lowGain = params[LOW_GAIN_PARAM].getValue() * inputs[LOW_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outLow = filterLow[c / 4].process((inLow + inAll) * lowGain);
			outputs[LOW_OUTPUT].setVoltageSimd<float_4>(outLow, c);

			const float_4 lowMidGain = params[LOW_MID_GAIN_PARAM].getValue() * inputs[LOW_MID_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outLowMid = filterLowMid[c / 4].process((inLowMid + inAll) * lowMidGain);
			outputs[LOW_MID_OUTPUT].setVoltageSimd<float_4>(outLowMid, c);

			const float_4 highMidGain = params[HIGH_MID_GAIN_PARAM].getValue() * inputs[HIGH_MID_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outHighMid = filterHighMid[c / 4].process((inHighMid + inAll) * highMidGain);
			outputs[HIGH_MID_OUTPUT].setVoltageSimd<float_4>(outHighMid, c);

			const float_4 highGain = params[HIGH_GAIN_PARAM].getValue() * inputs[HIGH_CV_INPUT].getNormalPolyVoltageSimd<float_4>(10.f, c) / 10.f;
			const float_4 outHigh = filterHigh[c / 4].process((inHigh + inAll) * highGain);
			outputs[HIGH_OUTPUT].setVoltageSimd<float_4>(outHigh, c);

			const float_4 fxReturnSum = inputs[LOW_RETURN_INPUT].getPolyVoltageSimd<float_4>(c) +
			                            inputs[LOW_MID_RETURN_INPUT].getPolyVoltageSimd<float_4>(c) +
			                            inputs[HIGH_MID_RETURN_INPUT].getPolyVoltageSimd<float_4>(c) +
			                            inputs[HIGH_RETURN_INPUT].getPolyVoltageSimd<float_4>(c);

			outputs[MIX_OUTPUT].setVoltageSimd<float_4>(fxReturnSum, c);
		}

		outputs[LOW_OUTPUT].setChannels(maxPolyphony);

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