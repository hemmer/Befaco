#include "plugin.hpp"
#include "Common.hpp"


struct MultiClock {

	float remaining = 0.f;
	float fullPulseLength = 0.f;

	/** Immediately disables the pulse */
	void reset(float newfullPulseLength) {
		fullPulseLength = newfullPulseLength;
		remaining = fullPulseLength;
	}

	/** Advances the state by `deltaTime`. Returns whether the pulse is in the HIGH state. */
	bool process(float deltaTime) {
		if (remaining > 0.f) {
			remaining -= deltaTime;
			return true;
		}
		return false;
	}

	float getGate(int gateMode, bool debug = false) {

		if (gateMode == 0) {
			return 10.f;
		}
		else if (gateMode < 0 || remaining <= 0) {
			return 0.f;
		}

		float multiGateOnLength = fullPulseLength / ((gateMode > 0) ? (2.f * gateMode) : 1.0f);
		bool isOddPulse = int(floor(remaining / multiGateOnLength)) % 2;

		if (debug && false) {
			DEBUG((std::to_string(gateMode) + " " + std::to_string(fullPulseLength) + " " + std::to_string(remaining) + " " + (isOddPulse ? "odd" : "even") ).c_str());
		}

		return isOddPulse ? 10.f : 0.f;
	}
};

struct Muxlicer : Module {
	enum ParamIds {
		PLAY_PARAM,
		ADDRESS_PARAM,
		GATE_MODE_PARAM,
		SPEED_PARAM,
		ENUMS(LEVEL_PARAMS, 8),
		NUM_PARAMS
	};
	enum InputIds {
		GATE_MODE_INPUT,
		ADDRESS_INPUT,
		CLOCK_INPUT,
		RESET_INPUT,
		COM_INPUT,
		ENUMS(MUX_INPUTS, 8),
		ALL_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		CLOCK_OUTPUT,
		ALL_GATES_OUTPUT,
		EOC_OUTPUT,
		ENUMS(GATE_OUTPUTS, 8),
		ENUMS(MUX_OUTPUTS, 8),
		COM_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		CLOCK_LIGHT,
		ENUMS(GATE_LIGHTS, 8),
		NUM_LIGHTS
	};

	enum ModeCOMIO {
		COM_1_IN_8_OUT,
		COM_8_IN_1_OUT
	};


	
	float clockDividerF; 	// in ms
	float clockTime;
	uint32_t runIndex;
	uint32_t addressIndex = 0;
	float lastSpeed = 0.f;
	bool tapped = false;
	float tapTime = 99999; // ms
	dsp::SchmittTrigger clockTrigger;
	dsp::PulseGenerator endOfCyclePulse;

	const static int SEQUENCE_LENGTH = 8;
	ModeCOMIO modeCOMIO = COM_1_IN_8_OUT;	
	MultiClock multiClock;

	// process main logic every N samples
	dsp::ClockDivider processDivider;
	const static uint32_t processInterval = 8;


	Muxlicer() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(Muxlicer::PLAY_PARAM, 0.0, 1.0, 0.0, "Play switch");
		configParam(Muxlicer::ADDRESS_PARAM, -1.0, 7.0, -1.0, "Address");
		configParam(Muxlicer::GATE_MODE_PARAM, -1.0, 8.0, 0.0, "Gate mode");
		configParam(Muxlicer::SPEED_PARAM, -INFINITY, INFINITY, 0.0, "Speed (divide/multiply)");

		for (int i = 0; i < SEQUENCE_LENGTH; ++i) {
			configParam(Muxlicer::LEVEL_PARAMS + i, 0.0, 1.0, 1.0, "Slider " + std::to_string(i));
		}

		onReset();
		processDivider.setDivision(processInterval);
	}

	void onReset() override {		
		clockDividerF = 250.f;;
		clockTime = 0;
		runIndex = 0;
	}

	void process(const ProcessArgs& args) override {
		if (clockTrigger.process(rescale(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f))) {
			tapped = true;
		}

		multiClock.process(args.sampleTime);

		// do every N = 32 samples
		if (processDivider.process()) {
			// this block will process every deltaTime seconds
			float deltaTime = args.sampleTime * processInterval;
			float deltaTimeMs = deltaTime * 1e3;

			// Index
			float address = params[ADDRESS_PARAM].getValue() + inputs[ADDRESS_INPUT].getVoltage();
			bool running = address < 0.f;

			float gate;
			if (inputs[GATE_MODE_INPUT].isConnected()) {
				float gateCV = clamp(inputs[GATE_MODE_INPUT].getVoltage(), 0.f, 5.f) / 5.f;
				float knobAttenuation = rescale(params[GATE_MODE_PARAM].getValue(), -1.f, 8.f, 0.f, 1.f);
				// todo: check against hardware
				gate = rescale(gateCV * knobAttenuation, 0.f, 1.f, -1.0f, 8.f);
			}
			else {
				gate = params[GATE_MODE_PARAM].getValue();
			}

			int gateMode = clamp((int) roundf(gate), -1, 8);

			// Clock frequency and phase
			if (tapped) {
				if (tapTime < 2000) {					
					clockDividerF = tapTime;
				}
				tapTime = 0;
				tapped = false;
			}
			tapTime += deltaTimeMs;

			float speed = params[SPEED_PARAM].getValue();
			if (speed != lastSpeed) {
				clockDividerF *= powf(0.5f, speed - lastSpeed);
				clockDividerF = clamp(clockDividerF, 1.f, 2000.f);
				lastSpeed = speed;
			}
			clockTime += deltaTimeMs;

			// Clock trigger
			outputs[CLOCK_OUTPUT].setVoltage(0.f);
			outputs[EOC_OUTPUT].setVoltage(0.f);

			// this is every clock tick (i.e. clockDivider x 1ms time has passed)
			if (clockTime >= clockDividerF) {

				clockTime = 0;
				outputs[CLOCK_OUTPUT].setVoltage(10.f);

				if (running) {
					runIndex++;
					if (runIndex >= 8) {
						runIndex = 0;
						endOfCyclePulse.trigger(1e-3);
					}
				}

				multiClock.reset(clockDividerF * 1e-3);

				for (int i = 0; i < 8; i++) {
					outputs[GATE_OUTPUTS + i].setVoltage(0.f);
				}
			}

			if (running) {
				addressIndex = runIndex;
			}
			else {
				addressIndex = clamp((int) roundf(address), 0, 8 - 1);
			}

			// Gates
			lights[CLOCK_LIGHT].setBrightness(isEven(addressIndex) ? 1.f : 0.f);
			for (int i = 0; i < 8; i++) {
				outputs[GATE_OUTPUTS + i].setVoltage(0.f);
				lights[GATE_LIGHTS + i].setBrightness(0.f);
			}
			outputs[ALL_GATES_OUTPUT].setVoltage(0.f);

			float gateValue = multiClock.getGate(gateMode, addressIndex==0);
			outputs[GATE_OUTPUTS + addressIndex].setVoltage(gateValue);
			lights[GATE_LIGHTS + addressIndex].setBrightness(gateValue / 10.f);
			outputs[ALL_GATES_OUTPUT].setVoltage(gateValue);

		}

		if (modeCOMIO == COM_1_IN_8_OUT) {
			// Mux outputs
			for (int i = 0; i < 8; i++) {
				outputs[MUX_OUTPUTS + i].setVoltage(0.f);
			}
			outputs[MUX_OUTPUTS + addressIndex].setVoltage(10.f * params[LEVEL_PARAMS + addressIndex].getValue());
		}
		else if (modeCOMIO == COM_8_IN_1_OUT) {
			float allInNormalValue = 10.f;
			float allInValue = inputs[ALL_INPUT].getNormalVoltage(allInNormalValue);
			float stepValue = inputs[MUX_INPUTS + addressIndex].getNormalVoltage(allInValue) * params[LEVEL_PARAMS + addressIndex].getValue();
			
			outputs[COM_OUTPUT].setVoltage(stepValue);
		}

		float com = inputs[COM_INPUT].getVoltage();
		float level = params[LEVEL_PARAMS + addressIndex].getValue();
		outputs[MUX_OUTPUTS + addressIndex].setVoltage(level * com);
		outputs[EOC_OUTPUT].setVoltage(endOfCyclePulse.process(args.sampleTime) ? 10.f : 0.f);
	}


	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "modeCOMIO", json_integer(modeCOMIO));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* modeJ = json_object_get(rootJ, "modeCOMIO");
		modeCOMIO = (Muxlicer::ModeCOMIO) json_integer_value(modeJ);
	}

};



struct MuxlicerTapBefacoTinyKnob : BefacoTinyKnob {
	/*
		bool moved;

		void onDragStart(EventDragStart &e) override {
			moved = false;
			BefacoTinyKnob::onDragStart(e);
		}
		void onDragMove(EventDragMove &e) override {
			if (!e.mouseRel.isZero())
				moved = true;
			BefacoTinyKnob::onDragMove(e);
		}
		void onDragEnd(EventDragEnd &e) override {
			if (!moved) {
				Muxlicer *module = dynamic_cast<Muxlicer*>(this->module);
				module->tapped = true;
			}
			BefacoTinyKnob::onDragEnd(e);
		}
	*/
};

struct MuxlicerWidget : ModuleWidget {
	MuxlicerWidget(Muxlicer* module) {

		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/Muxlicer.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<Knurlie>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<BefacoSwitch>(mm2px(Vec(35.72963, 10.008)), module, Muxlicer::PLAY_PARAM));
		addParam(createParam<BefacoTinyKnobSnap>(mm2px(Vec(3.84112, 10.90256)), module, Muxlicer::ADDRESS_PARAM));
		addParam(createParam<BefacoTinyKnobWhite>(mm2px(Vec(67.83258, 10.86635)), module, Muxlicer::GATE_MODE_PARAM));
		addParam(createParam<MuxlicerTapBefacoTinyKnob>(mm2px(Vec(28.12238, 24.62151)), module, Muxlicer::SPEED_PARAM));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(2.32728, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 0));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(12.45595, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 1));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(22.58462, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 2));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(32.7133, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 3));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(42.74195, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 4));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(52.97062, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 5));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(63.0993, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 6));
		addParam(createParam<BefacoSlidePot>(mm2px(Vec(73.22797, 40.67102)), module, Muxlicer::LEVEL_PARAMS + 7));

		addInput(createInput<BefacoInputPort>(mm2px(Vec(51.568, 11.20189)), module, Muxlicer::GATE_MODE_INPUT));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(21.13974, 11.23714)), module, Muxlicer::ADDRESS_INPUT));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(44.24461, 24.93662)), module, Muxlicer::CLOCK_INPUT));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(12.62135, 24.95776)), module, Muxlicer::RESET_INPUT));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(36.3142, 98.07911)), module, Muxlicer::COM_INPUT));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(0.895950, 109.27901)), module, Muxlicer::MUX_INPUTS + 0));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(11.05332, 109.29256)), module, Muxlicer::MUX_INPUTS + 1));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(21.18201, 109.29256)), module, Muxlicer::MUX_INPUTS + 2));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(31.27625, 109.27142)), module, Muxlicer::MUX_INPUTS + 3));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(41.40493, 109.27142)), module, Muxlicer::MUX_INPUTS + 4));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(51.53360, 109.27142)), module, Muxlicer::MUX_INPUTS + 5));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(61.69671, 109.29256)), module, Muxlicer::MUX_INPUTS + 6));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(71.82537, 109.29256)), module, Muxlicer::MUX_INPUTS + 7));
		addInput(createInput<BefacoInputPort>(mm2px(Vec(16.11766, 98.09121)), module, Muxlicer::ALL_INPUT));

		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(59.8492, 24.95776)), module, Muxlicer::CLOCK_OUTPUT));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(56.59663, 98.06252)), module, Muxlicer::ALL_GATES_OUTPUT));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(66.72661, 98.07008)), module, Muxlicer::EOC_OUTPUT));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(0.89595, 86.78581)), module, Muxlicer::GATE_OUTPUTS + 0));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(11.02463, 86.77068)), module, Muxlicer::GATE_OUTPUTS + 1));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(21.14758, 86.77824)), module, Muxlicer::GATE_OUTPUTS + 2));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(31.27625, 86.77824)), module, Muxlicer::GATE_OUTPUTS + 3));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(41.40493, 86.77824)), module, Muxlicer::GATE_OUTPUTS + 4));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(51.56803, 86.79938)), module, Muxlicer::GATE_OUTPUTS + 5));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(61.69671, 86.79938)), module, Muxlicer::GATE_OUTPUTS + 6));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(71.79094, 86.77824)), module, Muxlicer::GATE_OUTPUTS + 7));

		// these blocks are exclusive (for visibility / interactivity) and allows IO and OI within one module
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(0.895950, 109.27901)), module, Muxlicer::MUX_OUTPUTS + 0));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(11.05332, 109.29256)), module, Muxlicer::MUX_OUTPUTS + 1));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(21.18201, 109.29256)), module, Muxlicer::MUX_OUTPUTS + 2));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(31.27625, 109.27142)), module, Muxlicer::MUX_OUTPUTS + 3));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(41.40493, 109.27142)), module, Muxlicer::MUX_OUTPUTS + 4));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(51.53360, 109.27142)), module, Muxlicer::MUX_OUTPUTS + 5));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(61.69671, 109.29256)), module, Muxlicer::MUX_OUTPUTS + 6));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(71.82537, 109.29256)), module, Muxlicer::MUX_OUTPUTS + 7));
		addOutput(createOutput<BefacoOutputPort>(mm2px(Vec(36.3142, 98.07911)), module, Muxlicer::COM_OUTPUT));

		updatePortVisibilityForIOMode(Muxlicer::COM_1_IN_8_OUT);

		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(71.28361, 28.02644)), module, Muxlicer::CLOCK_LIGHT));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(3.99336, 81.86801)), module, Muxlicer::GATE_LIGHTS + 0));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(14.09146, 81.86801)), module, Muxlicer::GATE_LIGHTS + 1));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(24.22525, 81.86801)), module, Muxlicer::GATE_LIGHTS + 2));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(34.35901, 81.86801)), module, Muxlicer::GATE_LIGHTS + 3));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(44.49277, 81.86801)), module, Muxlicer::GATE_LIGHTS + 4));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(54.62652, 81.86801)), module, Muxlicer::GATE_LIGHTS + 5));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(64.76028, 81.86801)), module, Muxlicer::GATE_LIGHTS + 6));
		addChild(createLight<SmallLight<RedLight>>(mm2px(Vec(74.89404, 81.86801)), module, Muxlicer::GATE_LIGHTS + 7));
	}

	void draw(const DrawArgs& args) override {
		Muxlicer* module = dynamic_cast<Muxlicer*>(this->module);

		if (module != nullptr) {
			updatePortVisibilityForIOMode(module->modeCOMIO);
		}
		else {
			// module can be null, e.g. if populating the module browser with screenshots,
			// in which case just assume the default (1 in, 8 out)
			updatePortVisibilityForIOMode(Muxlicer::COM_1_IN_8_OUT);
		}

		ModuleWidget::draw(args);
	}

	struct IOMenuItem : MenuItem {
		Muxlicer* module;
		MuxlicerWidget* widget;
		void onAction(const event::Action& e) override {
			module->modeCOMIO = Muxlicer::COM_1_IN_8_OUT;
			widget->updatePortVisibilityForIOMode(module->modeCOMIO);
			widget->clearCables();
		}
	};
	struct OIMenuItem : MenuItem {
		Muxlicer* module;
		MuxlicerWidget* widget;
		void onAction(const event::Action& e) override {
			module->modeCOMIO = Muxlicer::COM_8_IN_1_OUT;
			widget->updatePortVisibilityForIOMode(module->modeCOMIO);
			widget->clearCables();
		}
	};

	void clearCables() {
		for (int i = Muxlicer::MUX_OUTPUTS; i <= Muxlicer::MUX_OUTPUTS_LAST; ++i) {
			APP->scene->rack->clearCablesOnPort(outputs[i]);
		}
		APP->scene->rack->clearCablesOnPort(inputs[Muxlicer::COM_INPUT]);

		for (int i = Muxlicer::MUX_INPUTS; i <= Muxlicer::MUX_INPUTS_LAST; ++i) {
			APP->scene->rack->clearCablesOnPort(inputs[i]);
		}
		APP->scene->rack->clearCablesOnPort(outputs[Muxlicer::COM_OUTPUT]);
	}

	// set ports visibility, either for 1 input -> 8 outputs or 8 inputs -> 1 output
	void updatePortVisibilityForIOMode(Muxlicer::ModeCOMIO mode) {

		bool visibleToggle = (mode == Muxlicer::COM_1_IN_8_OUT);

		for (int i = Muxlicer::MUX_OUTPUTS; i <= Muxlicer::MUX_OUTPUTS_LAST; ++i) {
			outputs[i]->visible = visibleToggle;
		}
		inputs[Muxlicer::COM_INPUT]->visible = visibleToggle;

		for (int i = Muxlicer::MUX_INPUTS; i <= Muxlicer::MUX_INPUTS_LAST; ++i) {
			inputs[i]->visible = !visibleToggle;
		}
		outputs[Muxlicer::COM_OUTPUT]->visible = !visibleToggle;
	}

	void appendContextMenu(Menu* menu) override {
		Muxlicer* module = dynamic_cast<Muxlicer*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());

		IOMenuItem* ioItem = createMenuItem<IOMenuItem>("1 input ▸ 8 outputs",
		                     CHECKMARK(module->modeCOMIO == Muxlicer::COM_1_IN_8_OUT));
		ioItem->module = module;
		ioItem->widget = this;
		menu->addChild(ioItem);

		OIMenuItem* oiItem = createMenuItem<OIMenuItem>("8 inputs ▸ 1 output",
		                     CHECKMARK(module->modeCOMIO == Muxlicer::COM_8_IN_1_OUT));
		oiItem->module = module;
		oiItem->widget = this;
		menu->addChild(oiItem);
	}


};


Model* modelMuxlicer = createModel<Muxlicer, MuxlicerWidget>("Muxlicer");

