#include "plugin.hpp"
#include "ChowDSP.hpp"

float aliasSuppressedSaw(const float* phases, float pw) {
	float sawBuffer[3];
	for (int i = 0; i < 3; ++i) {
		float p = 2 * phases[i] - 1.0; 		// range -1 to +1
		float pwp = p + 2 * pw;				// phase after pw (pw in [0, 1])
		pwp += simd::ifelse(pwp > 1, -2, simd::ifelse(pwp < -1, +2, 0));     			// modulo on [-1, +1]
		sawBuffer[i] = (pwp * pwp * pwp - pwp) / 6.0;	// eq 11
	}

	return (sawBuffer[0] - 2.0 * sawBuffer[1] + sawBuffer[2]);
}

float aliasSuppressedOffsetSaw(const float* phases, float pw) {
	float sawOffsetBuff[3];

	for (int i = 0; i < 3; ++i) {
		float pwp = 2 * phases[i] - 2 * pw; 		// range -1 to +1

		pwp += simd::ifelse(pwp > 1, -2, 0);     			// modulo on [-1, +1]
		sawOffsetBuff[i] = (pwp * pwp * pwp - pwp) / 6.0;	// eq 11
	}
	return (sawOffsetBuff[0] - 2.0 * sawOffsetBuff[1] + sawOffsetBuff[2]);
}

template<typename T>
class HardClipperADAA {
public:

	T process(T x) {
		T y = simd::ifelse(simd::abs(x - xPrev) < 1e-5,
		                   f(0.5 * (xPrev + x)),
		                   (F(x) - F(xPrev)) / (x - xPrev));

		xPrev = x;
		return y;
	}


	static T f(T x) {
		return simd::ifelse(simd::abs(x) < 1, x, simd::sgn(x));
	}

	static T F(T x) {
		return simd::ifelse(simd::abs(x) < 1, 0.5 * x * x, x * simd::sgn(x) - 0.5);
	}

	void reset() {
		xPrev = 0.f;
	}

private:
	T xPrev = 0.f;
};

struct Octaves : Module {
	enum ParamId {
		PWM_CV_PARAM,
		OCTAVE_PARAM,
		TUNE_PARAM,
		PWM_PARAM,
		RANGE_PARAM,
		GAIN_01F_PARAM,
		GAIN_02F_PARAM,
		GAIN_04F_PARAM,
		GAIN_08F_PARAM,
		GAIN_16F_PARAM,
		GAIN_32F_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		VOCT1_INPUT,
		VOCT2_INPUT,
		SYNC_INPUT,
		PWM_INPUT,
		GAIN_01F_INPUT,
		GAIN_02F_INPUT,
		GAIN_04F_INPUT,
		GAIN_08F_INPUT,
		GAIN_16F_INPUT,
		GAIN_32F_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_01F_OUTPUT,
		OUT_02F_OUTPUT,
		OUT_04F_OUTPUT,
		OUT_08F_OUTPUT,
		OUT_16F_OUTPUT,
		OUT_32F_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	bool limitPW = true;
	bool removePulseDC = false;
	static const int NUM_OUTPUTS = 6;
	const float ranges[3] = {4.f, 1.f, 1.f / 12.f}; 	// full, octave, semitone

	float phase = 0.f;		// phase for core waveform, in [0, 1]
	chowdsp::VariableOversampling<6, float> oversampler[NUM_OUTPUTS]; 	// uses a 2*6=12th order Butterworth filter
	int oversamplingIndex = 1; 	// default is 2^oversamplingIndex == x2 oversampling

	DCBlocker blockDCFilter;			// optionally block DC with RC filter @ ~22 Hz
	dsp::SchmittTrigger syncTrigger; 	// for hard sync

	Octaves() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(PWM_CV_PARAM, 0.f, 1.f, 1.f, "PWM CV attenuater");

		auto octParam = configSwitch(OCTAVE_PARAM, 0.f, 6.f, 4.f, "Octave", {"C1", "C2", "C3", "C4", "C5", "C6", "C7"});
		octParam->snapEnabled = true;

		configParam(TUNE_PARAM, -1.f, 1.f, 0.f, "Tune");
		configParam(PWM_PARAM, 0.5f, 0.f, 0.5f, "PWM");
		auto rangeParam = configSwitch(RANGE_PARAM, 0.f, 2.f, 0.f, "Range", {"VCO: Full", "VCO: Octave", "VCO: Semitone"});
		rangeParam->snapEnabled = true;

		configParam(GAIN_01F_PARAM, 0.f, 1.f, 0.f, "Gain Fundamental");
		configParam(GAIN_02F_PARAM, 0.f, 1.f, 0.f, "Gain x2 Fundamental");
		configParam(GAIN_04F_PARAM, 0.f, 1.f, 0.f, "Gain x4 Fundamental");
		configParam(GAIN_08F_PARAM, 0.f, 1.f, 0.f, "Gain x8 Fundamental");
		configParam(GAIN_16F_PARAM, 0.f, 1.f, 0.f, "Gain x16 Fundamental");
		configParam(GAIN_32F_PARAM, 0.f, 1.f, 0.f, "Gain x32 Fundamental");

		configInput(VOCT1_INPUT, "V/Octave 1");
		configInput(VOCT2_INPUT, "V/Octave 2");
		configInput(SYNC_INPUT, "Sync");
		configInput(PWM_INPUT, "PWM");
		configInput(GAIN_01F_INPUT, "Gain x1F CV");
		configInput(GAIN_02F_INPUT, "Gain x1F CV");
		configInput(GAIN_04F_INPUT, "Gain x1F CV");
		configInput(GAIN_08F_INPUT, "Gain x1F CV");
		configInput(GAIN_16F_INPUT, "Gain x1F CV");
		configInput(GAIN_32F_INPUT, "Gain x1F CV");

		configOutput(OUT_01F_OUTPUT, "x1F");
		configOutput(OUT_02F_OUTPUT, "x2F");
		configOutput(OUT_04F_OUTPUT, "x4F");
		configOutput(OUT_08F_OUTPUT, "x8F");
		configOutput(OUT_16F_OUTPUT, "x16F");
		configOutput(OUT_32F_OUTPUT, "x32F");

		// calculate up/downsampling rates
		onSampleRateChange();
	}

	void onSampleRateChange() override {
		float sampleRate = APP->engine->getSampleRate();
		for (int c = 0; c < NUM_OUTPUTS; c++) {
			oversampler[c].setOversamplingIndex(oversamplingIndex);
			oversampler[c].reset(sampleRate);
		}
		blockDCFilter.setFrequency(22.05 / sampleRate);
	}
	

	void process(const ProcessArgs& args) override {
		const int rangeIndex = params[RANGE_PARAM].getValue();

		float pitch = ranges[rangeIndex] * params[TUNE_PARAM].getValue() + inputs[VOCT1_INPUT].getVoltage() + inputs[VOCT2_INPUT].getVoltage();
		pitch += params[OCTAVE_PARAM].getValue() - 3;
		float freq = dsp::FREQ_C4 * dsp::exp2_taylor5(pitch);
		// -1 to +1
		const float pwmCV = params[PWM_CV_PARAM].getValue() * clamp(inputs[PWM_INPUT].getVoltage() / 10.f, -1.f, 1.f);
		const float pulseWidthLimit = limitPW ? 0.05f : 0.0f;

		// pwm in [-0.25 : +0.25]
		const float pwm = 2 * clamp(0.5 - params[PWM_PARAM].getValue() + 0.5 * pwmCV, -0.5f + pulseWidthLimit, 0.5f - pulseWidthLimit);

		const int oversamplingRatio = oversampler[0].getOversamplingRatio();

		// work out active outputs
		std::vector<int> connectedOutputs = getConnectedOutputs();
		if (connectedOutputs.size() == 0) {
			return;
		}
		// only process up to highest active channel
		const int highestOutput = *std::max_element(connectedOutputs.begin(), connectedOutputs.end());
		const float deltaPhase = freq * args.sampleTime / oversamplingRatio;

		//  process sync
		if (syncTrigger.process(inputs[SYNC_INPUT].getVoltage())) {
			phase = 0.5f;
		}

		for (int i = 0; i < oversamplingRatio; i++) {

			phase += deltaPhase;
			phase -= std::floor(phase);

			float sum = 0.f;
			for (int c = 0; c <= highestOutput; c++) {
				// derive phases for higher octaves from base phase (this keeps things in sync!)
				const float n = (float)(1 << c);
				// this is on [0, 1]
				const float effectivePhase = n * std::fmod(phase, 1 / n);
				const float gainCV = clamp(inputs[GAIN_01F_INPUT + c].getNormalVoltage(10.f) / 10.f, 0.f, 1.0f);
				const float gain = params[GAIN_01F_PARAM + c].getValue() * gainCV;

				const float waveTri = 1.0 - 2.0 * std::abs(2.f * effectivePhase - 1.0);
				// build square from triangle + comparator
				const float waveSquare = (waveTri > pwm) ? +1 : -1;

				sum += waveSquare * gain;
				sum = clamp(sum, -1.f, 1.f);

				if (outputs[OUT_01F_OUTPUT + c].isConnected()) {
					oversampler[c].getOSBuffer()[i] = sum;
					sum = 0.f;
				}
			}

		} // end of oversampling loop

		// only downsample required channels
		for (int c = 0; c <= highestOutput; c++) {
			if (outputs[OUT_01F_OUTPUT + c].isConnected()) {
				// downsample (if required)
				float out = (oversamplingRatio > 1) ? oversampler[c].downsample() : oversampler[c].getOSBuffer()[0];

				if (removePulseDC) {
					out = blockDCFilter.process(out);
				}

				outputs[OUT_01F_OUTPUT + c].setVoltage(5.f * out);
			}
		}

	}

	std::vector<int> getConnectedOutputs() {
		std::vector<int> connectedOutputs;
		for (int c = 0; c < NUM_OUTPUTS; c++) {
			if (outputs[OUT_01F_OUTPUT + c].isConnected()) {
				connectedOutputs.push_back(c);
			}
		}
		return connectedOutputs;
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "removePulseDC", json_boolean(removePulseDC));
		json_object_set_new(rootJ, "limitPW", json_boolean(limitPW));
		json_object_set_new(rootJ, "oversamplingIndex", json_integer(oversampler[0].getOversamplingIndex()));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {

		json_t* removePulseDCJ = json_object_get(rootJ, "removePulseDC");
		if (removePulseDCJ) {
			removePulseDC = json_boolean_value(removePulseDCJ);
		}

		json_t* limitPWJ = json_object_get(rootJ, "limitPW");
		if (limitPWJ) {
			limitPW = json_boolean_value(limitPWJ);
		}

		json_t* oversamplingIndexJ = json_object_get(rootJ, "oversamplingIndex");
		if (oversamplingIndexJ) {
			oversamplingIndex = json_integer_value(oversamplingIndexJ);
			onSampleRateChange();
		}
	}
};

struct OctavesWidget : ModuleWidget {
	OctavesWidget(Octaves* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/Octaves.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<Knurlie>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<BefacoTinyKnobLightGrey>(mm2px(Vec(52.138, 15.037)), module, Octaves::PWM_CV_PARAM));
		addParam(createParam<CKSSVert7>(mm2px(Vec(22.171, 30.214)), module, Octaves::OCTAVE_PARAM));
		addParam(createParamCentered<BefacoTinyKnobLightGrey>(mm2px(Vec(10.264, 33.007)), module, Octaves::TUNE_PARAM));
		addParam(createParamCentered<Davies1900hLargeRedKnob>(mm2px(Vec(45.384, 40.528)), module, Octaves::PWM_PARAM));
		addParam(createParam<CKSSThreeHorizontal>(mm2px(Vec(6.023, 48.937)), module, Octaves::RANGE_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(2.9830, 60.342)), module, Octaves::GAIN_01F_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(12.967, 60.342)), module, Octaves::GAIN_02F_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(22.951, 60.342)), module, Octaves::GAIN_04F_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(32.936, 60.342)), module, Octaves::GAIN_08F_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(42.920, 60.342)), module, Octaves::GAIN_16F_PARAM));
		addParam(createParam<BefacoSlidePotSmall>(mm2px(Vec(52.905, 60.342)), module, Octaves::GAIN_32F_PARAM));

		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.247, 15.181)), module, Octaves::VOCT1_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.282, 15.181)), module, Octaves::VOCT2_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.316, 15.181)), module, Octaves::SYNC_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(37.092, 15.135)), module, Octaves::PWM_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(5.247, 100.492)), module, Octaves::GAIN_01F_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.282, 100.492)), module, Octaves::GAIN_02F_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(25.316, 100.492)), module, Octaves::GAIN_04F_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(35.35, 100.492)), module, Octaves::GAIN_08F_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(45.384, 100.492)), module, Octaves::GAIN_16F_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(55.418, 100.492)), module, Octaves::GAIN_32F_INPUT));

		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(5.247, 113.508)), module, Octaves::OUT_01F_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(15.282, 113.508)), module, Octaves::OUT_02F_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(25.316, 113.508)), module, Octaves::OUT_04F_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(35.35, 113.508)), module, Octaves::OUT_08F_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(45.384, 113.508)), module, Octaves::OUT_16F_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(55.418, 113.508)), module, Octaves::OUT_32F_OUTPUT));

	}

	void appendContextMenu(Menu* menu) override {
		Octaves* module = dynamic_cast<Octaves*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());
		menu->addChild(createSubmenuItem("Hardware compatibility", "",
		[ = ](Menu * menu) {
			menu->addChild(createBoolPtrMenuItem("Limit pulsewidth (5\%-95\%)", "", &module->limitPW));
			menu->addChild(createBoolPtrMenuItem("Remove pulse DC", "", &module->removePulseDC));
		}
		                                ));

		menu->addChild(createIndexSubmenuItem("Oversampling",
		{"Off", "x2", "x4", "x8"},
		[ = ]() {
			return module->oversamplingIndex;
		},
		[ = ](int mode) {
			module->oversamplingIndex = mode;
			module->onSampleRateChange();
		}
		                                     ));

	}
};

Model* modelOctaves = createModel<Octaves, OctavesWidget>("Octaves");