#include "plugin.hpp"


/*! \brief Decode System Exclusive messages.
 SysEx messages are encoded to guarantee transmission of data bytes higher than
 127 without breaking the MIDI protocol. Use this static method to reassemble
 your received message.
 \param inSysEx The SysEx data received from MIDI in.
 \param outData The output buffer where to store the decrypted message.
 \param inLength The length of the input buffer.
 \param inFlipHeaderBits True for Korg and other who store MSB in reverse order
 \return The length of the output buffer.
 @see encodeSysEx @see getSysExArrayLength
 Code inspired from Ruin & Wesen's SysEx encoder/decoder - http://ruinwesen.com
 */
unsigned decodeSysEx(const uint8_t* inSysEx,
                     uint8_t* outData,
                     unsigned inLength,
                     bool inFlipHeaderBits) {
	unsigned count  = 0;
	uint8_t msbStorage = 0;
	uint8_t byteIndex  = 0;

	for (unsigned i = 0; i < inLength; ++i) {
		if ((i % 8) == 0) {
			msbStorage = inSysEx[i];
			byteIndex  = 6;
		}
		else {
			const uint8_t body     = inSysEx[i];
			const uint8_t shift    = inFlipHeaderBits ? 6 - byteIndex : byteIndex;
			const uint8_t msb      = uint8_t(((msbStorage >> shift) & 1) << 7);
			byteIndex--;
			outData[count++] = msb | body;
		}
	}
	return count;
}

struct RoundRobinProcessor {
	// if a channel (0 - 11) should be updated, return it's index, otherwise return -1
	int process(float sampleTime, float period, int numActiveChannels) {

		if (numActiveChannels == 0 || period <= 0) {
			return -1;
		}

		time += sampleTime;

		if (time > period) {
			time -= period;

			// special case: when there's only one channel, the below logic (which looks for when active channel changes)
			// wont fire. as we've completed a period, return an "update channel 0" value
			if (numActiveChannels == 1) {
				return 0;
			}
		}

		int currentActiveChannel = numActiveChannels * time / period;

		if (currentActiveChannel != previousActiveChannel) {
			previousActiveChannel = currentActiveChannel;
			return currentActiveChannel;
		}

		// if we've got this far, no updates needed (-1)
		return -1;
	}
private:
	float time = 0.f;
	int previousActiveChannel = -1;
};


struct MidiThing : Module {
	enum ParamId {
		REFRESH_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		A1_INPUT,
		B1_INPUT,
		C1_INPUT,
		A2_INPUT,
		B2_INPUT,
		C2_INPUT,
		A3_INPUT,
		B3_INPUT,
		C3_INPUT,
		A4_INPUT,
		B4_INPUT,
		C4_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};
	/// Port mode
	enum PORTMODE_t {
		NOPORTMODE = 0,
		MODE10V,
		MODEPN5V,
		MODE8V,
		MODE5V,

		LASTPORTMODE
	};

	const char* cfgPortModeNames[6] = {
		"No Mode",
		"0/10v",
		"-5/5v",
		"0/8v",
		"0/5v",
		""
	};

	const std::vector<float> updateRates = {250., 500., 1000., 2000., 4000., 8000.};
	const std::vector<std::string> updateRateNames = {"250 Hz (fewest active channels, slowest, lowest-cpu)", "500 Hz", "1 kHz", "2 kHz", "4 kHz",
	                                                  "8 kHz (most active channels, fast, highest-cpu)"
	                                                 };
	int updateRateIdx = 2;

	// use Pre-def 4 for bridge mode
	const static int VCV_BRIDGE_PREDEF = 4;

	midi::Output midiOut;
	RoundRobinProcessor roundRobinProcessor;

	MidiThing() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configButton(REFRESH_PARAM, "");

		for (int i = 0; i < NUM_INPUTS; ++i) {
			portModes[i] = MODE10V;
			configInput(A1_INPUT + i, string::f("Port %d", i + 1));
		}
	}

	void onReset() override {
		midiOut.reset();

	}

	void requestAllChannelsParamsOverSysex() {
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 3; ++col) {
				const int PORT_CONFIG = 2;
				requestParamOverSysex(row, col, PORT_CONFIG);
			}
		}
	}

	// request that MidiThing loads a pre-defined template, 1-4
	void setPredef(uint8_t predef) {
		predef = clamp(predef, 1, 4);
		midi::Message msg;
		msg.bytes.resize(8);
		// Midi spec is zeroo indexed
		uint8_t predefToSend = predef - 1;
		msg.bytes = {0xF0, 0x7D, 0x17, 0x00, 0x00, 0x02, 0x00, predefToSend, 0xF7};
		midiOut.setChannel(0);
		midiOut.sendMessage(msg);
		// DEBUG("Predef %d msg request sent: %s", predef, msg.toString().c_str());
	}

	void setMidiMergeViaSysEx(bool mergeOn) {
		midi::Message msg;
		msg.bytes.resize(8);

		msg.bytes = {0xF0, 0x7D, 0x19, 0x00, 0x05, 0x02, 0x00, (uint8_t) mergeOn, 0xF7};
		midiOut.setChannel(0);
		midiOut.sendMessage(msg);
		// DEBUG("Predef %d msg request sent: %s", mergeOn, msg.toString().c_str());
	}
	void setVoltageModeOnHardwareAllChannels(PORTMODE_t outputMode_) {
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 3; ++col) {
				setVoltageModeOnHardware(row, col, outputMode_);
			}
		}
	}

	void setVoltageModeOnHardware(uint8_t row, uint8_t col, PORTMODE_t outputMode_) {
		uint8_t port = 3 * row + col;
		portModes[port] = outputMode_;

		midi::Message msg;
		msg.bytes.resize(8);
		// F0 7D 17 2n 02 02 00 0m F7
		// Where n = 0 based port number
		// and m is the volt output mode to select from:
		msg.bytes = {0xF0, 0x7D, 0x17, static_cast<unsigned char>(32 + port), 0x02, 0x02, 0x00, (uint8_t) portModes[port], 0xF7};
		midiOut.sendMessage(msg);
		// DEBUG("Voltage mode msg sent: port %d (%d), mode %d", port, static_cast<unsigned char>(32 + port), portModes[port]);
	}

	void setVoltageModeOnHardware(uint8_t row, uint8_t col) {
		setVoltageModeOnHardware(row, col, portModes[3 * row + col]);
	}

	void syncVcvStateToHardware() {
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 3; ++col) {
				setVoltageModeOnHardware(row, col);
			}
		}
	}


	midi::InputQueue inputQueue;
	void requestParamOverSysex(uint8_t row, uint8_t col, uint8_t mode) {

		midi::Message msg;
		msg.bytes.resize(8);
		// F0 7D 17 00 01 03 00 nm pp F7
		uint8_t port = 3 * row + col;
		//Where n is:
		// 0 = Full configuration request. The module will send only pre def, port functions and modified parameters
		// 2 = Send Port configuration
		// 4 = Send MIDI Channel configuration
		// 6 = Send Voice Configuration

		uint8_t n = mode * 16;
		uint8_t m = port; // element number: 0-11 port number, 1-16 channel or voice number
		uint8_t pp = 2;
		msg.bytes = {0xF0, 0x7D, 0x17, 0x00, 0x01, 0x03, 0x00, static_cast<uint8_t>(n + m), pp, 0xF7};
		midiOut.sendMessage(msg);
		// DEBUG("API request mode msg sent: port %d, pp %s", port, msg.toString().c_str());
	}

	int getVoltageMode(uint8_t row, uint8_t col) {
		// -1 because menu is zero indexed but enum is not
		int channel = clamp(3 * row + col, 0, NUM_INPUTS - 1);
		return portModes[channel] - 1;
	}

	const static int NUM_INPUTS = 12;
	bool isClipping[NUM_INPUTS] = {};

	bool checkIsVoltageWithinRange(uint8_t channel, float voltage) {
		const float tol = 0.001;
		switch (portModes[channel]) {
			case MODE10V: return 0 - tol < voltage && voltage < 10 + tol;
			case MODEPN5V: return -5 - tol < voltage && voltage < 5 + tol;
			case MODE8V: return 0 - tol < voltage && voltage < 8 + tol;
			case MODE5V: return 0 - tol < voltage && voltage < 5 + tol;
			default: return false;
		}
	}

	uint16_t rescaleVoltageForChannel(uint8_t channel, float voltage) {
		switch (portModes[channel]) {
			case MODE10V: return rescale(clamp(voltage, 0.f, 10.f), 0.f, +10.f, 0, 16383);
			case MODEPN5V: return rescale(clamp(voltage, -5.f, 5.f), -5.f, +5.f, 0, 16383);
			case MODE8V: return rescale(clamp(voltage, 0.f, 8.f), 0.f, +8.f, 0, 16383);
			case MODE5V: return rescale(clamp(voltage, 0.f, 5.f), 0.f, +5.f, 0, 16383);
			default: return 0;
		}
	}

	// one way sync (VCV -> hardware) for now
	void doSync() {
		// switch to VCV template (predef 4)
		setPredef(4);

		// disable MIDI merge (otherwise large sample rates will not work)
		setMidiMergeViaSysEx(false);

		// send full VCV config
		syncVcvStateToHardware();

		// disabled for now, but this would request what state the hardware is in
		if (parseSysExMessagesFromHardware) {
			requestAllChannelsParamsOverSysex();
		}
	}

	// debug only
	bool parseSysExMessagesFromHardware = false;
	int numActiveChannels = 0;
	dsp::BooleanTrigger buttonTrigger;
	dsp::Timer rateLimiterTimer;
	PORTMODE_t portModes[NUM_INPUTS] = {};
	void process(const ProcessArgs& args) override {

		if (buttonTrigger.process(params[REFRESH_PARAM].getValue())) {
			doSync();
		}

		// disabled for now, but this is how VCV would read SysEx coming from the hardware (if requested above)
		if (parseSysExMessagesFromHardware) {
			midi::Message msg;
			uint8_t outData[32] = {};
			while (inputQueue.tryPop(&msg, args.frame)) {

				uint8_t outLen = decodeSysEx(&msg.bytes[0], outData, msg.bytes.size(), false);
				if (outLen > 3) {

					int channel = (outData[2] & 0x0f) >> 0;

					if (channel >= 0 && channel < NUM_INPUTS) {
						if (outData[outLen - 1] < LASTPORTMODE) {
							portModes[channel] = (PORTMODE_t) outData[outLen - 1];
						}
					}
				}
			}
		}

		std::vector<int> activeChannels;
		for (int c = 0; c < NUM_INPUTS; ++c) {
			if (inputs[A1_INPUT + c].isConnected()) {
				activeChannels.push_back(c);
			}
		}
		numActiveChannels = activeChannels.size();
		// we're done if no channels are active
		if (numActiveChannels == 0) {
			return;
		}

		//DEBUG("updateRateIdx: %d", updateRateIdx);
		const float updateRateHz = updateRates[updateRateIdx];
		//DEBUG("updateRateHz: %f", updateRateHz);
		const int maxCCMessagesPerSecondPerChannel = updateRateHz / numActiveChannels;

		// MIDI baud rate is 31250 b/s, or 3125 B/s.
		// CC messages are 3 bytes, so we can send a maximum of 1041 CC messages per second.
		// The refresh rate period (i.e. how often we can send X channels of data is:
		const float rateLimiterPeriod = 1.f / maxCCMessagesPerSecondPerChannel;

		// this returns -1 if no channel should be updated, or the index of the channel that should be updated
		// it distributes update times in a round robin fashion
		int channelIdxToUpdate = roundRobinProcessor.process(args.sampleTime, rateLimiterPeriod, numActiveChannels);

		if (channelIdxToUpdate >= 0 && channelIdxToUpdate < numActiveChannels) {
			int c = activeChannels[channelIdxToUpdate];

			const float channelVoltage = inputs[A1_INPUT + c].getVoltage();
			uint16_t pw = rescaleVoltageForChannel(c, channelVoltage);
			isClipping[c] = !checkIsVoltageWithinRange(c, channelVoltage);
			midi::Message m;
			m.setStatus(0xe);
			m.setNote(pw & 0x7f);
			m.setValue((pw >> 7) & 0x7f);
			m.setFrame(args.frame);

			midiOut.setChannel(c);
			midiOut.sendMessage(m);
		}
	}


	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "midiOutput", midiOut.toJson());
		json_object_set_new(rootJ, "inputQueue", inputQueue.toJson());
		json_object_set_new(rootJ, "updateRateIdx", json_integer(updateRateIdx));

		for (int c = 0; c < NUM_INPUTS; ++c) {
			json_object_set_new(rootJ, string::f("portMode%d", c).c_str(), json_integer(portModes[c]));
		}

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* midiOutputJ = json_object_get(rootJ, "midiOutput");
		if (midiOutputJ) {
			midiOut.fromJson(midiOutputJ);
		}

		json_t* midiInputQueueJ = json_object_get(rootJ, "inputQueue");
		if (midiInputQueueJ) {
			inputQueue.fromJson(midiInputQueueJ);
		}

		json_t* updateRateIdxJ = json_object_get(rootJ, "updateRateIdx");
		if (updateRateIdxJ) {
			updateRateIdx = json_integer_value(updateRateIdxJ);
		}

		for (int c = 0; c < NUM_INPUTS; ++c) {
			json_t* portModeJ = json_object_get(rootJ, string::f("portMode%d", c).c_str());
			if (portModeJ) {
				portModes[c] = (PORTMODE_t)json_integer_value(portModeJ);
			}
		}

		// requestAllChannelsParamsOverSysex();
		syncVcvStateToHardware();
	}
};

struct MidiThingPort : BefacoInputPort {
	int row = 0, col = 0;
	MidiThing* module;

	void appendContextMenu(Menu* menu) override {

		menu->addChild(new MenuSeparator());
		std::string label = string::f("Voltage Mode Port %d", 3 * row + col + 1);

		menu->addChild(createIndexSubmenuItem(label,
		{"0 to 10v", "-5 to 5v", "0 to 8v", "0 to 5v"},
		[ = ]() {
			return module->getVoltageMode(row, col);
		},
		[ = ](int modeIdx) {
			MidiThing::PORTMODE_t mode = (MidiThing::PORTMODE_t)(modeIdx + 1);
			module->setVoltageModeOnHardware(row, col, mode);
		}
		                                     ));

		/*
		menu->addChild(createIndexSubmenuItem("Get Port Info",
		{"Full", "Port", "MIDI", "Voice"},
		[ = ]() {
			return -1;
		},
		[ = ](int mode) {
			module->requestParamOverSysex(row, col, 2 * mode);
		}
		                                     ));
		*/
	}
};

// dervied from https://github.com/countmodula/VCVRackPlugins/blob/v2.0.0/src/components/CountModulaLEDDisplay.hpp
struct LEDDisplay : LightWidget {
	float fontSize = 9;
	Vec textPos = Vec(1, 13);
	int numChars = 7;
	int row = 0, col = 0;
	MidiThing* module;

	LEDDisplay() {
		box.size = mm2px(Vec(9.298, 5.116));
	}

	void setCentredPos(Vec pos) {
		box.pos.x = pos.x - box.size.x / 2;
		box.pos.y = pos.y - box.size.y / 2;
	}

	void drawBackground(const DrawArgs& args) override {
		// Background
		NVGcolor backgroundColor = nvgRGB(0x20, 0x20, 0x20);
		NVGcolor borderColor = nvgRGB(0x10, 0x10, 0x10);
		nvgBeginPath(args.vg);
		nvgRoundedRect(args.vg, 0.0, 0.0, box.size.x, box.size.y, 2.0);
		nvgFillColor(args.vg, backgroundColor);
		nvgFill(args.vg);
		nvgStrokeWidth(args.vg, 1.0);
		nvgStrokeColor(args.vg, borderColor);
		nvgStroke(args.vg);
	}

	void drawLight(const DrawArgs& args) override {
		// Background
		NVGcolor backgroundColor = nvgRGB(0x20, 0x20, 0x20);
		NVGcolor borderColor = nvgRGB(0x10, 0x10, 0x10);
		NVGcolor textColor = nvgRGB(0xff, 0x10, 0x10);

		nvgBeginPath(args.vg);
		nvgRoundedRect(args.vg, 0.0, 0.0, box.size.x, box.size.y, 2.0);
		nvgFillColor(args.vg, backgroundColor);
		nvgFill(args.vg);
		nvgStrokeWidth(args.vg, 1.0);

		if (module) {
			const bool isClipping = module->isClipping[col + row * 3];
			if (isClipping) {
				borderColor = nvgRGB(0xff, 0x20, 0x20);
			}
		}

		nvgStrokeColor(args.vg, borderColor);
		nvgStroke(args.vg);

		std::shared_ptr<Font> font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/miso.otf"));

		if (font && font->handle >= 0) {

			std::string text;
			if (module) {
				text = module->cfgPortModeNames[module->getVoltageMode(row, col) + 1];
			}
			else {
				// fallback if module not yet defined
				const char* cfgPortModeNames[4] = {"0/10v", "-5/5v", "0/8v", "0/5v"};
				const int randomModeForDisplay = rack::random::u32() % 4;
				text = cfgPortModeNames[randomModeForDisplay];
			}			
			char buffer[numChars + 1];
			int l = text.size();
			if (l > numChars)
				l = numChars;

			nvgGlobalTint(args.vg, color::WHITE);

			text.copy(buffer, l);
			buffer[l] = '\0';

			nvgFontSize(args.vg, fontSize);
			nvgFontFaceId(args.vg, font->handle);
			nvgFillColor(args.vg, textColor);
			nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
			NVGtextRow textRow;
			nvgTextBreakLines(args.vg, text.c_str(), NULL, box.size.x, &textRow, 1);
			nvgTextBox(args.vg, textPos.x, textPos.y, box.size.x, textRow.start, textRow.end);
		}
	}

	void onButton(const ButtonEvent& e) override {
		if (e.button == GLFW_MOUSE_BUTTON_RIGHT && e.action == GLFW_PRESS) {
			ui::Menu* menu = createMenu();

			menu->addChild(createMenuLabel(string::f("Voltage mode port %d:", col + 3 * row + 1)));

			const std::string labels[4] = {"0 to 10v", "-5 to 5v", "0 to 8v", "0 to 5v"};

			for (int i = 0; i < 4; ++i) {
				menu->addChild(createCheckMenuItem(labels[i], "",
				[ = ]() {
					return module->getVoltageMode(row, col) == i;
				},
				[ = ]() {
					MidiThing::PORTMODE_t mode = (MidiThing::PORTMODE_t)(i + 1);
					module->setVoltageModeOnHardware(row, col, mode);
				}
				                                  ));
			}

			e.consume(this);
			return;
		}

		LightWidget::onButton(e);
	}

};


struct MidiThingWidget : ModuleWidget {

	struct LedDisplayCenterChoiceEx : LedDisplayChoice {
		LedDisplayCenterChoiceEx() {
			box.size = mm2px(math::Vec(0, 8.0));
			color = nvgRGB(0xf0, 0xf0, 0xf0);
			bgColor = nvgRGBAf(0, 0, 0, 0);
			textOffset = math::Vec(0, 16);
		}

		void drawLayer(const DrawArgs& args, int layer) override {
			nvgScissor(args.vg, RECT_ARGS(args.clipBox));
			if (layer == 1) {
				if (bgColor.a > 0.0) {
					nvgBeginPath(args.vg);
					nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
					nvgFillColor(args.vg, bgColor);
					nvgFill(args.vg);
				}

				std::shared_ptr<window::Font> font = APP->window->loadFont(asset::plugin(pluginInstance, "res/fonts/miso.otf"));

				if (font && font->handle >= 0 && !text.empty()) {
					nvgFillColor(args.vg, color);
					nvgFontFaceId(args.vg, font->handle);
					nvgTextLetterSpacing(args.vg, -0.6f);
					nvgFontSize(args.vg, 10);
					nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
					NVGtextRow textRow;
					nvgTextBreakLines(args.vg, text.c_str(), NULL, box.size.x, &textRow, 1);
					nvgTextBox(args.vg, textOffset.x, textOffset.y, box.size.x, textRow.start, textRow.end);
				}
			}
			nvgResetScissor(args.vg);
		}
	};


	struct MidiDriverItem : ui::MenuItem {
		midi::Port* port;
		int driverId;
		void onAction(const event::Action& e) override {
			port->setDriverId(driverId);
		}
	};

	struct MidiDriverChoice : LedDisplayCenterChoiceEx {
		midi::Port* port;
		void onAction(const event::Action& e) override {
			if (!port)
				return;
			createContextMenu();
		}

		virtual ui::Menu* createContextMenu() {
			ui::Menu* menu = createMenu();
			menu->addChild(createMenuLabel("MIDI driver"));
			for (int driverId : midi::getDriverIds()) {
				MidiDriverItem* item = new MidiDriverItem;
				item->port = port;
				item->driverId = driverId;
				item->text = midi::getDriver(driverId)->getName();
				item->rightText = CHECKMARK(item->driverId == port->driverId);
				menu->addChild(item);
			}
			return menu;
		}

		void step() override {
			text = port ? port->getDriver()->getName() : "";
			if (text.empty()) {
				text = "(No driver)";
				color.a = 0.5f;
			}
			else {
				color.a = 1.f;
			}
		}
	};

	struct MidiDeviceItem : ui::MenuItem {
		midi::Port* outPort, *inPort;
		int deviceId;
		void onAction(const event::Action& e) override {
			outPort->setDeviceId(deviceId);
			inPort->setDeviceId(deviceId);
		}
	};

	struct MidiDeviceChoice : LedDisplayCenterChoiceEx {
		midi::Port* outPort, *inPort;
		void onAction(const event::Action& e) override {
			if (!outPort || !inPort)
				return;
			createContextMenu();
		}

		virtual ui::Menu* createContextMenu() {
			ui::Menu* menu = createMenu();
			menu->addChild(createMenuLabel("MIDI device"));
			{
				MidiDeviceItem* item = new MidiDeviceItem;
				item->outPort = outPort;
				item->inPort = inPort;
				item->deviceId = -1;
				item->text = "(No device)";
				item->rightText = CHECKMARK(item->deviceId == outPort->deviceId);
				menu->addChild(item);
			}
			for (int deviceId : outPort->getDeviceIds()) {
				MidiDeviceItem* item = new MidiDeviceItem;
				item->outPort = outPort;
				item->inPort = inPort;
				item->deviceId = deviceId;
				item->text = outPort->getDeviceName(deviceId);
				item->rightText = CHECKMARK(item->deviceId == outPort->deviceId);
				menu->addChild(item);
			}
			return menu;
		}

		void step() override {
			text = outPort ? outPort->getDeviceName(outPort->deviceId) : "";
			if (text.empty()) {
				text = "(No device)";
				color.a = 0.5f;
			}
			else {
				color.a = 1.f;
			}
		}
	};

	struct MidiWidget : LedDisplay {
		MidiDriverChoice* driverChoice;
		LedDisplaySeparator* driverSeparator;
		MidiDeviceChoice* deviceChoice;
		LedDisplaySeparator* deviceSeparator;

		void setMidiPorts(midi::Port* outPort, midi::Port* inPort) {

			clearChildren();
			math::Vec pos;

			MidiDriverChoice* driverChoice = createWidget<MidiDriverChoice>(pos);
			driverChoice->box.size = Vec(box.size.x, 20.f);
			//driverChoice->textOffset = Vec(6.f, 14.7f);
			driverChoice->color = nvgRGB(0xf0, 0xf0, 0xf0);
			driverChoice->port = outPort;

			addChild(driverChoice);
			pos = driverChoice->box.getBottomLeft();
			this->driverChoice = driverChoice;

			this->driverSeparator = createWidget<LedDisplaySeparator>(pos);
			this->driverSeparator->box.size.x = box.size.x;
			addChild(this->driverSeparator);

			MidiDeviceChoice* deviceChoice = createWidget<MidiDeviceChoice>(pos);
			deviceChoice->box.size = Vec(box.size.x, 21.f);
			//deviceChoice->textOffset = Vec(6.f, 14.7f);
			deviceChoice->color = nvgRGB(0xf0, 0xf0, 0xf0);
			deviceChoice->outPort = outPort;
			deviceChoice->inPort = inPort;
			addChild(deviceChoice);
			pos = deviceChoice->box.getBottomLeft();
			this->deviceChoice = deviceChoice;
		}
	};


	MidiThingWidget(MidiThing* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/MidiThing.svg")));

		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<Knurlie>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		MidiWidget* midiInputWidget = createWidget<MidiWidget>(Vec(1.5f, 36.4f)); //mm2px(Vec(0.5f, 10.f)));
		midiInputWidget->box.size = mm2px(Vec(5.08 * 6 - 1, 13.5f));
		if (module) {
			midiInputWidget->setMidiPorts(&module->midiOut, &module->inputQueue);
		}
		else {
			midiInputWidget->setMidiPorts(nullptr, nullptr);
		}
		addChild(midiInputWidget);

		addParam(createParamCentered<BefacoButton>(mm2px(Vec(21.12, 57.32)), module, MidiThing::REFRESH_PARAM));

		const float xStartLed = 0.2 + 0.628;
		const float yStartLed = 28.019;

		for (int row = 0; row < 4; row++) {
			for (int col = 0; col < 3; col++) {

				LEDDisplay* display = createWidget<LEDDisplay>(mm2px(Vec(xStartLed + 9.751 * col, yStartLed + 5.796 * row)));
				display->module = module;
				display->row = row;
				display->col = col;
				addChild(display);

				auto input = createInputCentered<MidiThingPort>(mm2px(Vec(5.08 + 10 * col, 69.77 + 14.225 * row)), module, MidiThing::A1_INPUT + 3 * row + col);
				input->row = row;
				input->col = col;
				input->module = module;
				addInput(input);


			}
		}
	}

	void appendContextMenu(Menu* menu) override {
		MidiThing* module = dynamic_cast<MidiThing*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());

		menu->addChild(createSubmenuItem("Select MIDI Device", "",
		[ = ](Menu * menu) {

			for (auto driverId : rack::midi::getDriverIds()) {
				midi::Driver* driver = midi::getDriver(driverId);
				const bool activeDriver = module->midiOut.getDriverId() == driverId;

				menu->addChild(createSubmenuItem(driver->getName(), CHECKMARK(activeDriver),
				[ = ](Menu * menu) {

					for (auto deviceId : driver->getOutputDeviceIds()) {
						const bool activeDevice = activeDriver && module->midiOut.getDeviceId() == deviceId;

						menu->addChild(createMenuItem(driver->getOutputDeviceName(deviceId),
						                              CHECKMARK(activeDevice),
						[ = ]() {
							module->midiOut.setDriverId(driverId);
							module->midiOut.setDeviceId(deviceId);

							module->inputQueue.setDriverId(driverId);
							module->inputQueue.setDeviceId(deviceId);
							module->inputQueue.setChannel(0); // TODO update

							module->doSync();

							// DEBUG("Updating Output MIDI settings - driver: %s, device: %s",
							//        driver->getName().c_str(), driver->getOutputDeviceName(deviceId).c_str());
						}));
					}
				}));
			}
		}));

		menu->addChild(createIndexPtrSubmenuItem("All channels MIDI update rate",
		               module->updateRateNames,
		               &module->updateRateIdx));

		float updateRate = module->updateRates[module->updateRateIdx] / module->numActiveChannels;
		menu->addChild(createMenuLabel(string::f("Per-channel MIDI update rate: %.3g Hz", updateRate)));

		menu->addChild(createIndexSubmenuItem("Set mode for all channels",
		{"0 to 10v", "-5 to 5v", "0 to 8v", "0 to 5v"},
		[ = ]() {
			return -1;
		},
		[ = ](int modeIdx) {
			MidiThing::PORTMODE_t mode = (MidiThing::PORTMODE_t)(modeIdx + 1);
			module->setVoltageModeOnHardwareAllChannels(mode);
		}
		                                     ));
	}
};


Model* modelMidiThing = createModel<MidiThing, MidiThingWidget>("MidiThingV2");