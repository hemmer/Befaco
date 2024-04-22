# MIDI Thing v2

The original MIDI Thing v2 hardware unit is described as follows:

> Midi Thing v2 is a flexible MIDI to CV converter. Allowing polyphonic notes handling, envelope and LFO generation as well as all available MIDI messages to be converted into CV. This is a huge upgrade from our previous beloved MIDI Thing, which adds a screen for easy configuration,12 assignable ports, TRS, USB Host and Device, MIDI merge OUT, a web configuration tool, and a VCV rack Bridge counterpart.

The VCV counterpart is designed to allow users to quickly get up and running with their hardware, i.e. sending CV from VCV to the hardware unit. 

## Setup 

To use, first ensure the MIDI Thing v2 is plugged into your computer, and visible as a MIDI device. Then select it, either from the top of the module, or the right click context menu. Then click "SYNC" - this puts the MIDI Thing into a preset designed to work with VCV Rack, and syncronises settings/voltage ranges etc. Note that for now, sync is one-way (VCV to hardware).

![MIDI Thing Config](img/MidiThingV2.png "MIDI Thing v2 Setup")

## Usage

To use, simply wire CV which you wish to send to the hardware to the matching input on the VCV module. Note that you will need to select the range, which can be done by right-clicking on the matching box (see below). Options are 0/10v, -5/5v, -10/0v, 0/8v, 0/5v. Note that the module is **not** designed to work with audio rate signals, just CV.

![MIDI Thing Voltage Range](img/VoltageRange.png "MIDI Thing v2 Voltage Range")

## Update Rate 

Midi Thing v2 VCV allows the user to configure the update rate at which data is sent over MIDI. This must be shared between the channels, so if we set the hardware to update at 1 kHz, 1 active channel will update at 1 kHz, 2 active channels will update at 500 Hz, 4 active channels at 250 Hz and so on. The total update rate (to be shared between channels) is set from the context menu, noting that higher update rates will use more CPU. The effect of the update rate on a 90 Hz saw (blue trace) can be seen in the bottom image, specifically that the temporal resolution of the reconstructed signal (red traces) improves as the update rate is increased from 500 Hz to 1000 Hz to 2000 Hz. 

![MIDI Thing Update Rates](img/UpdateRate.png "MIDI Thing v2 Update Ranges Menu")
![MIDI Thing Update Rates](img/UpdateRatesScope.png "MIDI Thing v2 Update Ranges Menu")


