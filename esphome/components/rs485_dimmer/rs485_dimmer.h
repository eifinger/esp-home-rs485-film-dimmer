#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include <array>
#include <algorithm>

namespace esphome {
namespace rs485_dimmer {

static const uint8_t START_BYTE = 0xAA;
static const uint8_t END_BYTE = 0x99;
static const uint8_t CMD_WRITE = 0x09;
static const uint8_t CMD_REPLY = 0x0A;
static const uint8_t DATA_OFF = 0x00;
static const uint8_t DATA_ON = 0xFF;
static const size_t PACKET_LENGTH = 11;
static const uint32_t POLLING_INTERVAL_MS = 15000;
static const uint32_t DISCOVERY_TIMEOUT_MS = 5000;
static const char *const TAG = "rs485_dimmer";

class RS485Dimmer : public light::LightOutput, public Component, public uart::UARTDevice {
 public:
  void set_tx_enable_pin(GPIOPin *tx_enable_pin) { this->tx_enable_pin_ = tx_enable_pin; }

  void setup() {
    this->tx_enable_pin_->setup();
    this->tx_enable_pin_->digital_write(false);

    ESP_LOGI(TAG, "Starting address discovery...");
    this->discovery_start_time_ = millis();
    // Send the special "get address" command: AA 00 00 00 00 09 00 00 00 55 99
    uint8_t discovery_command[PACKET_LENGTH] = {0xAA, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x55, 0x99};
    this->send_packet(discovery_command);
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "RS485 Dimmer:");
    LOG_PIN("  TX Enable Pin: ", this->tx_enable_pin_);
    if (address_discovered_) {
      ESP_LOGCONFIG(TAG, "  Address: %02X:%02X:%02X:%02X",
                    address_[0], address_[1], address_[2], address_[3]);
    } else {
      ESP_LOGCONFIG(TAG, "  Address: Not discovered yet");
    }
  }

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
    return traits;
  }

  void write_state(light::LightState *state) override {
    if (!this->address_discovered_) return;
    this->state_ = state; 
    uint8_t data_to_send = this->state_to_data_byte_();
    send_command(data_to_send);
  }
  
  void update() {
    if (!this->state_ || !this->address_discovered_) return;
    uint8_t data_to_send = this->state_to_data_byte_();
    send_command(data_to_send);
  }

  void loop() override {
    if (this->is_failed()) return;
    if (!this->address_discovered_) {
      if (millis() - this->discovery_start_time_ > DISCOVERY_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Address discovery failed! Check wiring and power. Component will not operate.");
        this->mark_failed();
        this->address_discovered_ = true;
        return;
      }
    } else {
      if (millis() - this->last_update_ > POLLING_INTERVAL_MS) {
        this->last_update_ = millis();
        this->update();
      }
    }
    
    // listen for incoming data
    uint8_t buffer[PACKET_LENGTH];
    while (available() >= PACKET_LENGTH) {
      if (read() != START_BYTE) continue;
      if (!read_array(buffer, PACKET_LENGTH - 1)) return;
      if (buffer[9] != END_BYTE) continue;
      
      // Check if this is the discovery reply (host sent 09, device replies 0A or 09)
      if (!this->address_discovered_ && (buffer[4] == CMD_REPLY || buffer[4] == CMD_WRITE)) {
        // Store the discovered address ---
        std::copy(buffer, buffer + 4, this->address_.begin());
        this->address_discovered_ = true;
        ESP_LOGI(TAG, "Address discovered: %02X:%02X:%02X:%02X", 
                 this->address_[0], this->address_[1], this->address_[2], this->address_[3]);
        continue; // Skip further processing on this loop
      }

      if (this->address_discovered_ && this->state_ && 
          buffer[0] == this->address_[0] && buffer[1] == this->address_[1] &&
          buffer[2] == this->address_[2] && buffer[3] == this->address_[3] &&
          (buffer[4] == CMD_REPLY || buffer[4] == CMD_WRITE)) {
        
        uint8_t data0 = buffer[5];
        bool new_power = (data0 != DATA_OFF);
        float new_brightness = 1.0f;
        if (new_power && data0 != DATA_ON) new_brightness = (data0 - 1) / 253.0f;

        if (new_power != this->state_->remote_values.is_on() || 
            (new_power && (abs(new_brightness - this->state_->remote_values.get_brightness()) > 0.01))) {
            
            auto call = this->state_->make_call();
            call.set_state(new_power);
            if(new_power) call.set_brightness(new_brightness);
            call.perform();
        }
      }
    }
  }

 protected:
  uint8_t state_to_data_byte_() {
    bool power = this->state_->remote_values.is_on();
    float brightness_float = this->state_->remote_values.get_brightness();
    if (!power) return DATA_OFF;
    if (brightness_float >= 0.99f) return DATA_ON; 
    return 1 + (uint8_t)(brightness_float * 253.0f);
  }

  void send_command(uint8_t data0) {
    uint8_t command[PACKET_LENGTH] = {
      START_BYTE,
      this->address_[0], this->address_[1], this->address_[2], this->address_[3],
      CMD_WRITE,
      data0, 0x00, 0x00, 0x00,
      END_BYTE
    };
    this->send_packet(command);
  }

  void send_packet(const uint8_t *packet) {
    this->tx_enable_pin_->digital_write(true);
    this->write_array(packet, PACKET_LENGTH);
    this->flush();
    this->tx_enable_pin_->digital_write(false);
  }

  GPIOPin *tx_enable_pin_;
  std::array<uint8_t, 4> address_{}; // Initialized empty
  light::LightState *state_{nullptr};
  uint32_t last_update_{0};
  bool address_discovered_{false};
  uint32_t discovery_start_time_{0};
};

} // namespace rs485_dimmer
} // namespace esphome