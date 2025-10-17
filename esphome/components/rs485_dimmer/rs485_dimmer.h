#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
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

class RS485Dimmer : public cover::Cover, public Component, public uart::UARTDevice {
 public:
  void set_tx_enable_pin(GPIOPin *tx_enable_pin) { this->tx_enable_pin_ = tx_enable_pin; }

  void setup() {
    this->tx_enable_pin_->setup();
    this->tx_enable_pin_->digital_write(false);

    ESP_LOGI(TAG, "Starting address discovery...");
    this->discovery_start_time_ = millis();
    // Send the special "get address" command: AA 00 00 00 00 09 00 00 00 55 99
    uint8_t discovery_command[PACKET_LENGTH] = {
        0xAA, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x55, 0x99,
    };
    this->send_packet(discovery_command);
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "RS485 Dimmer:");
    LOG_PIN("  TX Enable Pin: ", this->tx_enable_pin_);
    if (address_discovered_) {
      ESP_LOGCONFIG(TAG, "  Address: %02X:%02X:%02X:%02X", address_[0], address_[1], address_[2], address_[3]);
    } else {
      ESP_LOGCONFIG(TAG, "  Address: Not discovered yet");
    }
  }

  cover::CoverTraits get_traits() override {
    auto traits = cover::CoverTraits();
    traits.set_supports_position(true);
    traits.set_supports_tilt(false);
    traits.set_is_assumed_state(false);
    return traits;
  }

  void control(const cover::CoverCall &call) override {
    if (!this->address_discovered_)
      return;

    if (call.get_position().has_value()) {
      this->position = *call.get_position();
    }
    if (call.get_stop()) {
      // Stop command - send current position
      uint8_t data_to_send = this->position_to_data_byte_();
      send_command(data_to_send);
      this->publish_state();
      return;
    }

    uint8_t data_to_send = this->position_to_data_byte_();
    send_command(data_to_send);
    this->publish_state();
  }

  void update() {
    if (!this->address_discovered_)
      return;
    uint8_t data_to_send = this->position_to_data_byte_();
    send_command(data_to_send);
  }

  void loop() override {
    if (this->is_failed())
      return;
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
      if (read() != START_BYTE)
        continue;
      if (!read_array(buffer, PACKET_LENGTH - 1))
        return;
      if (buffer[9] != END_BYTE)
        continue;

      // Check if this is the discovery reply (host sent 09, device replies 0A or 09)
      if (!this->address_discovered_ && (buffer[4] == CMD_REPLY || buffer[4] == CMD_WRITE)) {
        // Store the discovered address ---
        std::copy(buffer, buffer + 4, this->address_.begin());
        this->address_discovered_ = true;
        ESP_LOGI(TAG, "Address discovered: %02X:%02X:%02X:%02X", this->address_[0], this->address_[1],
                 this->address_[2], this->address_[3]);
        continue;  // Skip further processing on this loop
      }

      if (this->address_discovered_ && buffer[0] == this->address_[0] && buffer[1] == this->address_[1] &&
          buffer[2] == this->address_[2] && buffer[3] == this->address_[3] &&
          (buffer[4] == CMD_REPLY || buffer[4] == CMD_WRITE)) {
        uint8_t data0 = buffer[5];
        float new_position;

        if (data0 == DATA_OFF) {
          new_position = 0.0f;  // Fully closed
        } else if (data0 == DATA_ON) {
          new_position = 1.0f;  // Fully open
        } else {
          new_position = (data0 - 1) / 253.0f;
        }

        if (abs(new_position - this->position) > 0.01f) {
          this->position = new_position;
          this->publish_state();
        }
      }
    }
  }

 protected:
  uint8_t position_to_data_byte_() {
    if (this->position <= 0.01f)
      return DATA_OFF;
    if (this->position >= 0.99f)
      return DATA_ON;
    return 1 + (uint8_t) (this->position * 253.0f);
  }

  void send_command(uint8_t data0) {
    uint8_t command[PACKET_LENGTH] = {
        START_BYTE,
        this->address_[0],
        this->address_[1],
        this->address_[2],
        this->address_[3],
        CMD_WRITE,
        data0,
        0x00,
        0x00,
        0x00,
        END_BYTE,
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
  std::array<uint8_t, 4> address_{};  // Initialized empty
  uint32_t last_update_{0};
  bool address_discovered_{false};
  uint32_t discovery_start_time_{0};
};

}  // namespace rs485_dimmer
}  // namespace esphome