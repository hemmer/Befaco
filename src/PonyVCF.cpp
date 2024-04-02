#include "plugin.hpp"

using simd::float_4;

// filter engine is just Fundemental VCF from https://github.com/VCVRack/Fundamental/blob/v2/src/VCF.cpp for now
// GPL-v3


template <typename T>
static T clip(T x) {
	// return std::tanh(x);
	// Pade approximant of tanh
	x = simd::clamp(x, -3.f, 3.f);
	return x * (27 + x * x) / (27 + 9 * x * x);
}


template <typename T>
struct LadderFilter {
	T omega0;
	T resonance = 1;
	T state[4];
	T input;

	LadderFilter() {
		reset();
		setCutoff(0);
	}

	void reset() {
		for (int i = 0; i < 4; i++) {
			state[i] = 0;
		}
	}

	void setCutoff(T cutoff) {
		omega0 = 2 * T(M_PI) * cutoff;
	}

	void process(T input, T dt) {
		dsp::stepRK4(T(0), dt, state, 4, [&](T t, const T x[], T dxdt[]) {
			T inputt = crossfade(this->input, input, t / dt);
			T inputc = clip(inputt - resonance * x[3]);
			T yc0 = clip(x[0]);
			T yc1 = clip(x[1]);
			T yc2 = clip(x[2]);
			T yc3 = clip(x[3]);

			dxdt[0] = omega0 * (inputc - yc0);
			dxdt[1] = omega0 * (yc0 - yc1);
			dxdt[2] = omega0 * (yc1 - yc2);
			dxdt[3] = omega0 * (yc2 - yc3);
		});

		this->input = input;
	}

	T lowpass() {
		return state[3];
	}
	T highpass() {
		return clip((input - resonance * state[3]) - 4 * state[0] + 6 * state[1] - 4 * state[2] + state[3]);
	}
};


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

	LadderFilter<float_4> filters[4];

	PonyVCF() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CV1_PARAM, 0.f, 1.f, 0.f, "CV1 Attenuator");
		configParam(RES_PARAM, 0.f, 1.f, 0.f, "Resonance");
		configParam(FREQ_PARAM, 0.f, 1.f, 0.f, "Frequency");
		configParam(GAIN1_PARAM, 0.f, 1.2f, 1.f, "Gain Channel 1");
		configParam(GAIN2_PARAM, 0.f, 1.2f, 1.f, "Gain Channel 2");
		configParam(GAIN3_PARAM, 0.f, 1.2f, 1.f, "Gain Channel 3");
		configParam(ROUTING_PARAM, 0.f, 1.f, 0.f, "VCA routing");

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


	void onReset() override {
		for (int i = 0; i < 4; i++)
			filters[i].reset();
	}

	float_4 prevOut[4] = {};

	void process(const ProcessArgs& args) override {
		if (!outputs[OUTPUT].isConnected()) {
			return;
		}

		float resParam = params[RES_PARAM].getValue();
		float freqParam = params[FREQ_PARAM].getValue();
		float freqCvParam = params[CV1_PARAM].getValue();
		

		int channels = std::max({1, inputs[IN1_INPUT].getChannels(), inputs[IN2_INPUT].getChannels(), inputs[IN3_INPUT].getChannels()});
		
		for (int c = 0; c < channels; c += 4) {
			auto& filter = filters[c / 4];

			float_4 input = inputs[IN1_INPUT].getVoltageSimd<float_4>(c) * params[GAIN1_PARAM].getValue();
			input += inputs[IN2_INPUT].getVoltageSimd<float_4>(c) * params[GAIN2_PARAM].getValue();
			input += inputs[IN3_INPUT].getNormalVoltageSimd<float_4>(prevOut[c / 4], c) * params[GAIN3_PARAM].getValue();

			
			// input = Saturator<float_4>::process(input / 5.0f) * 1.1f;
			input = clip(input / 5.0f) * 1.1f;

			// Add -120dB noise to bootstrap self-oscillation
			input += 1e-6f * (2.f * random::uniform() - 1.f);

			// Set resonance
			float_4 resonance = resParam + inputs[RES_INPUT].getPolyVoltageSimd<float_4>(c) / 10.f;
			resonance = clamp(resonance, 0.f, 1.f);
			filter.resonance = simd::pow(resonance, 2) * 10.f;

			// Get pitch
			float_4 pitch = 5 * freqParam + inputs[CV1_INPUT].getPolyVoltageSimd<float_4>(c) * freqCvParam + inputs[CV2_INPUT].getPolyVoltageSimd<float_4>(c);
			// Set cutoff
			float_4 cutoff = dsp::FREQ_C4 * dsp::exp2_taylor5(pitch);
			// Without oversampling, we must limit to 8000 Hz or so @ 44100 Hz
			cutoff = clamp(cutoff, 1.f, args.sampleRate / 2.f);

			// Without oversampling, we must limit to 8000 Hz or so @ 44100 Hz
			cutoff = clamp(cutoff, 1.f, args.sampleRate * 0.18f);
			filter.setCutoff(cutoff);

			// Set outputs
			filter.process(input, args.sampleTime);
			if (outputs[OUTPUT].isConnected()) {
				float_4 resGain = 1.0f / (0.05 + 0.9 * dsp::exp2_taylor5(-8.f * simd::pow(resonance, 2.0)));
				// float_4 resGain = (1.0f + 8.f * resonance);  // 1st order empirical fit
				float_4 out = 5.f * filter.lowpass() * resGain;
				outputs[OUTPUT].setVoltageSimd(out, c);

				prevOut[c / 4] = out;
			}

			// DEBUG("channel %d %g", channels, input[0]);
		}

		outputs[OUTPUT].setChannels(channels);
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
};


Model* modelPonyVCF = createModel<PonyVCF, PonyVCFWidget>("PonyVCF");