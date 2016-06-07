/*
 Name:		Svetulino.ino
 Created:	2016-06-07 04:45:38
 Author:	ao@ze1.org
*/

// To add/remove a button of a remote control for toggling relay on/off, press a button, then wait 5 sec, then press
// quickly 5 times during next 5 sec. The buttons config will be saved in EEPROM memory. 
//

#include <EEPROM.h>
#include "IRremote.h"

#define PROGRAM_SIGN	0xA02E1080

#define INPUT_PIN		2
#define RELAY_PIN		4
#define SWITCH_PIN      7

#define INPUT_PIN_2		3
#define RELAY_PIN_2		5
#define SWITCH_PIN_2    8

HardwareSerial &s = Serial;
bool N() { s.println(""); return true; }
template<typename T> bool S(T t, bool n = false) { s.print(t); if (n) s.println(""); return true; }
template<typename T> bool SH(T t, bool n = false) { if (sizeof(t) == 8 && t < 0x10) s.print("0"); s.print(t, HEX); if (n) s.println(""); return true; }

struct CFG {

	uint32_t prog_sign; // program signature, identifies program/version that owns data
	uint16_t data_addr; // data address at EEPROM
	uint16_t data_size;
	uint32_t data_item[128];

	CFG(uint32_t prog_sign)
		: prog_sign(prog_sign), data_addr(), data_size(), data_item({}) {

	}

	void Setup() {

		N(); S("SETUP CFG("); SH(prog_sign); S(")");

		for (auto it = EEPROM.begin(); it != EEPROM.end(); ++it) {

			data_addr = it.index;
			N(); S(" SIGN CFG["); SH(data_addr); S("]: ");

			uint32_t data = EEPROM.get(data_addr, data); SH(data);
			if (data == 0xffffffff) return;
			if (data == prog_sign) {

				S(" PROGRAM SIGNATURE MATCH");

				Load();
				return;
			}
		}
	}

	void Load() {

		uint16_t
			size_addr = data_addr + sizeof(prog_sign),
			load_addr = size_addr + sizeof(data_size),
			load_stop = load_addr + EEPROM.get(size_addr, data_size);

		N(); S(" LOAD CFG["); S(size_addr); S("]: "); S(data_size); S(" BYTES OF DATA ");

		for (uint8_t *data = (uint8_t*)&data_item[0], *data_stop = data + sizeof(data_item);
			data < data_stop && load_addr < load_stop;
			data++, load_addr++
			)
			SH(EEPROM.get(load_addr, *data));

		Remove(0);
	}

	void Save() {

		N(); S(" SAVE CFG["); S(data_addr); S("]: "); SH(prog_sign); S(" PROGRAM SIGNATURE");

		EEPROM.put(data_addr, prog_sign);

		uint16_t
			size_addr = data_addr + sizeof(prog_sign),
			save_addr = size_addr + sizeof(data_size), save_stop = 512;

		EEPROM.put(size_addr, data_size);

		N(); S(" SAVE CFG["); S(size_addr); S("]: "); S(data_size); S(" BYTES OF DATA ");

		for (uint8_t *data = (uint8_t*)&data_item[0], *data_stop = data + data_size;
			data < data_stop && save_addr < save_stop;
			data++, save_addr++
			)
			SH(EEPROM.put(save_addr, *data));
	}

	bool Exists(uint32_t value) {

		for(uint32_t*item = &data_item[0], *end = item + data_size / sizeof(data_item[0]);
			item < end;
			++item
			)
			if (*item == value)
				return true;

		return false;
	}

	bool Add(uint32_t value) {

		if (data_size + sizeof(data_item[0]) > sizeof(data_item))
			return false;

		data_item[data_size / sizeof(data_item[0])] = value;
		data_size += sizeof(data_item[0]);

		S(" ADD CFG("); SH(value); S(") ");
		return true;

	}

	bool Remove(uint32_t value) {

		bool result = false;
		for(uint32_t*item = &data_item[0], *end = item + data_size / sizeof(data_item[0]);
			item < end;
			++item
			)
			if (result)
				*(item - 1) = *item;
			else
				if (*item == value)
					result = true;

		if (result) {

			data_size -= sizeof(data_item[0]);
			S(" REMOVE CFG("); SH(value); S(") "); 
		}
		return result;
	}

}
cfg_(PROGRAM_SIGN);

struct IR {

	uint8_t ir_pin;
	IRrecv ir_recv;

	uint8_t sw_pin;
	bool sw_state;

	uint8_t relay_pin;
	bool relay_state;

	struct Input {
		IR &ir;
		uint32_t code;
		uint32_t allow;
		uint32_t check;
		uint32_t result;
		uint8_t count;

		Input(IR &ir)
			: ir(ir), code(), check(), result(), count() {

		}

		uint32_t Loop() {

			decode_results input;
			uint32_t value = 0, now = millis();
			if (ir.ir_recv.decode(&input)) {

				value = input.value;
				ir.ir_recv.resume();
				S(" >>> IR("); S(ir.ir_pin); S("): "); SH(value); S(" >>> ");;
			}

			unsigned char t = !code ? 0 : (now < check ? 1 : (now < result ? 2 : 3));
			S(!t ? "." : (t == 1 ? "-" : (t == 2 ? "+" : "=")));

			if (value == 0xffffffff)
				value = code;

			if (!value) {

				if (t == 3) {

					S(" === RESULT(");
					if (count == 5) {

						if (!cfg_.Remove(code)) cfg_.Add(code);
						cfg_.Save();
						value = code;
					}
					else
						S("FAILURE");

					code = 0;
					S(") === ", true);
				}
				return value;
			}

			if (value == code && t == 2 && S(" +++ COUNT(") && S(++count) && S(") +++ "))
				return 0;

			if (t && S(" ::: RESET(T") && S(t) && S(") ::: "))
				N();

			code = value;
			count = 0xff;
			check = now + 5000;
			result = now + 10000;
			S(" *** CODE("); SH(code); S(") *** ");

			if (now < allow) return 0;
			allow = now + 1000;
			return value;
		}
	}
	input;

	IR(uint8_t ir_pin, uint8_t sw_pin, uint8_t relay_pin)
		: ir_pin(ir_pin), ir_recv(ir_pin), sw_pin(sw_pin), sw_state(), relay_pin(relay_pin), relay_state(), input(*this) {

	}

	void Setup() {

		N(); S("SETUP IR("); S(ir_pin); S("), RELAY("); S(relay_pin); S(")");

		pinMode(ir_pin, INPUT);
		ir_recv.enableIRIn();

		pinMode(sw_pin, INPUT);
		sw_state = digitalRead(sw_pin) == HIGH;

		relay_state = sw_state;

		pinMode(relay_pin, OUTPUT);
		digitalWrite(relay_pin, relay_state ? HIGH : LOW);

		N();
	}

	void Loop() {

		bool sw = digitalRead(sw_pin) == HIGH;
		if (sw != sw_state) {

			sw_state = sw;
			relay_state = sw_state;
			digitalWrite(relay_pin, sw || relay_state ? HIGH : LOW);
			N(); S(" /// RELAY("); S(relay_pin); S(relay_state ? "): [HIGH] /// " : "): [LOW] /// "); N();
		}

		uint32_t input_value = input.Loop();
		if (input_value) {

			if (cfg_.Exists(input_value)) {

				relay_state = !relay_state;
				digitalWrite(relay_pin, sw || relay_state ? HIGH : LOW);
				N(); S(" >>> RELAY("); S(relay_pin); S(relay_state ? "): [HIGH] >>> " : "): [LOW] >>> "); N();
			}
		}
	}
}
ir_[1] = {
	IR(INPUT_PIN, SWITCH_PIN, RELAY_PIN)
	//,IR(INPUT_PIN_2, RELAY_PIN_2)
};

void setup() {

	s.begin(9600);

	cfg_.Setup();

	for (IR *ir = &ir_[0], *end = ir + sizeof(ir_) / sizeof(IR);
		ir < end;
		++ir
		)
		ir->Setup();
}

void loop() {

	for (IR *ir = &ir_[0], *end = ir + sizeof(ir_) / sizeof(IR);
		ir < end;
		++ir
		)
		ir->Loop();

	delay(100);
}
