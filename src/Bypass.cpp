#include "plugin.hpp"

using namespace simd;

struct Bypass : Module {
	enum ParamId {
		MODE_PARAM,
		FX_GAIN_PARAM,
		LAUNCH_MODE_PARAM,
		LAUNCH_BUTTON_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		IN_R_INPUT,
		FROM_FX_L_INPUT,
		FROM_FX_R_INPUT,
		LAUNCH_INPUT,
		IN_L_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		TOFX_L_OUTPUT,
		TOFX_R_OUTPUT,
		OUT_L_OUTPUT,
		OUT_R_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LAUNCH_LED,
		LIGHTS_LEN
	};
	enum LatchMode {
		TOGGLE_MODE, 	// i.e. latch
		MOMENTARY_MODE // i.e. gate
	};
	enum ReturnMode {
		HARD_MODE,
		SOFT_MODE
	};
	LatchMode latchMode = LatchMode::MOMENTARY_MODE;
	ReturnMode returnMode = ReturnMode::HARD_MODE;
	ParamQuantity* launchParam;
	dsp::SchmittTrigger launchCvTrigger;
	dsp::BooleanTrigger launchButtonTrigger;
	dsp::BooleanTrigger latchTrigger;
	dsp::SlewLimiter clickFilter;

	Bypass() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Return mode", {"Hard", "Soft"});
		configParam(FX_GAIN_PARAM, -30.f, 30.f, 0.f, "FX Gain");
		configSwitch(LAUNCH_MODE_PARAM, 0.f, 1.f, 0.f, "Launch Mode", {"Latch (Toggle)", "Gate (Momentary)"});
		launchParam = configButton(LAUNCH_BUTTON_PARAM, "Launch");

		configInput(IN_L_INPUT, "Left");
		configInput(IN_R_INPUT, "Right");
		configInput(FROM_FX_L_INPUT, "From FX L");
		configInput(FROM_FX_R_INPUT, "From FX R");
		configInput(LAUNCH_INPUT, "Launch");

		configOutput(TOFX_L_OUTPUT, "To FX L");
		configOutput(TOFX_R_OUTPUT, "To FX R");
		configOutput(OUT_L_OUTPUT, "Left");
		configOutput(OUT_R_OUTPUT, "Right");

		configBypass(IN_L_INPUT, OUT_L_OUTPUT);
		configBypass(IN_R_INPUT, OUT_R_OUTPUT);		

		clickFilter.rise = 1 / 0.01f; // 0.01 ms
		clickFilter.fall = 1 / 0.01f; // 0.01 ms
	}

	bool active = false;
	void process(const ProcessArgs& args) override {

		const int maxChannels = std::max(inputs[IN_L_INPUT].getChannels(), inputs[IN_R_INPUT].getChannels());

		latchMode = (LatchMode) params[LAUNCH_MODE_PARAM].getValue();
		const ReturnMode returnMode = (ReturnMode) params[MODE_PARAM].getValue();


		const bool launchCvTriggered = launchCvTrigger.process(inputs[LAUNCH_INPUT].getVoltage());
		const bool launchButtonPressed = launchButtonTrigger.process(params[LAUNCH_BUTTON_PARAM].getValue());

		// logical or (high if either high)
		const float launchValue = std::max(launchCvTrigger.isHigh(), launchButtonTrigger.isHigh());
		if (latchMode == LatchMode::TOGGLE_MODE) {
			const bool risingEdge = launchCvTriggered || launchButtonPressed;

			// TODO: sometimes misses?
			if (risingEdge) {
				active = !active;
			}
		}

		const float fxGain = std::pow(10, params[FX_GAIN_PARAM].getValue() / 20.0f);
		const float sendActive = clickFilter.process(args.sampleTime, (latchMode == LatchMode::TOGGLE_MODE) ? active : launchValue);

		for (int c = 0; c < maxChannels; c += 4) {

			const float_4 inL = inputs[IN_L_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 inR = inputs[IN_R_INPUT].getNormalPolyVoltageSimd<float_4>(inL, c);

			outputs[TOFX_L_OUTPUT].setVoltageSimd<float_4>(inL * fxGain * sendActive, c);
			outputs[TOFX_R_OUTPUT].setVoltageSimd<float_4>(inR * fxGain * sendActive, c);

			const float_4 fxLeftReturn = inputs[FROM_FX_L_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 fxRightReturn = inputs[FROM_FX_R_INPUT].getPolyVoltageSimd<float_4>(c);

			if (returnMode == ReturnMode::HARD_MODE) {
				outputs[OUT_L_OUTPUT].setVoltageSimd<float_4>(inL * (1 - sendActive) + sendActive * fxLeftReturn, c);
				outputs[OUT_R_OUTPUT].setVoltageSimd<float_4>(inR * (1 - sendActive) + sendActive * fxRightReturn, c);
			}
			else {
				outputs[OUT_L_OUTPUT].setVoltageSimd<float_4>(inL * (1 - sendActive) + fxLeftReturn, c);
				outputs[OUT_R_OUTPUT].setVoltageSimd<float_4>(inR * (1 - sendActive) + fxRightReturn, c);
			}
		}

		outputs[OUT_R_OUTPUT].setVoltage(sendActive);

		lights[LAUNCH_LED].setBrightness(sendActive);
	}
};


using BefacoRedLightButton = LightButton<BefacoButton, VeryLargeSimpleLight<TRedLight<app::ModuleLightWidget>>>;


struct BypassWidget : ModuleWidget {

	SvgSwitch* launchParam;

	BypassWidget(Bypass* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/Bypass.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<CKSSHoriz2>(mm2px(Vec(6.7, 63.263)), module, Bypass::MODE_PARAM));
		addParam(createParamCentered<BefacoTinyKnobWhite>(mm2px(Vec(10.0, 78.903)), module, Bypass::FX_GAIN_PARAM));
		addParam(createParam<CKSSNarrow>(mm2px(Vec(13.8, 91.6)), module, Bypass::LAUNCH_MODE_PARAM));

		launchParam = createLightParamCentered<BefacoRedLightButton>(mm2px(Vec(10.0, 111.287)), module, Bypass::LAUNCH_BUTTON_PARAM, Bypass::LAUNCH_LED);
		addParam(launchParam);

		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.016, 15.03)), module, Bypass::IN_R_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(4.947, 40.893)), module, Bypass::FROM_FX_L_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(15.001, 40.893)), module, Bypass::FROM_FX_R_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(6.648, 95.028)), module, Bypass::LAUNCH_INPUT));
		addInput(createInputCentered<BefacoInputPort>(mm2px(Vec(4.947, 15.03)), module, Bypass::IN_L_INPUT));

		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(4.957, 27.961)), module, Bypass::TOFX_L_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(14.957, 27.961)), module, Bypass::TOFX_R_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(4.947, 53.846)), module, Bypass::OUT_L_OUTPUT));
		addOutput(createOutputCentered<BefacoOutputPort>(mm2px(Vec(14.957, 53.824)), module, Bypass::OUT_R_OUTPUT));
	}

	void draw(const DrawArgs& args) override {

		Bypass* module = dynamic_cast<Bypass*>(this->module);

		if (module != nullptr) {
			launchParam->momentary = module->latchMode == Bypass::LatchMode::MOMENTARY_MODE;
			launchParam->latch = !launchParam->momentary;
		}

		ModuleWidget::draw(args);
	}
};


Model* modelBypass = createModel<Bypass, BypassWidget>("Bypass");