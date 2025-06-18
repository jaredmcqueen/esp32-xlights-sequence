/**
 * ESP32 DMX Receiver
 *
 * This project implements an E1.31 (sACN) receiver on an ESP32 for LED/DMX control.
 * Designed for a distributed lighting system where multiple dancers/performers
 * each have their own ESP32 controlling individual light channels.
 *
 * Features:
 * - E1.31 (sACN) protocol support for DMX over WiFi
 * - Support for up to 12 dancers with 5 channels each
 * - Static IP configuration based on dancer ID
 * - Automatic WiFi reconnection with visual feedback
 * - Safety timeout to turn off outputs if signal is lost
 * - Visual feedback for connection status
 * - Configurable debug logging
 *
 * Hardware Requirements:
 * - ESP32 development board
 * - 5 output channels (LEDs/relays) connected to specified pins
 * - WiFi network access
 *
 * Dependencies:
 * - ESPAsyncE131 library
 * - WiFi library
 * - Arduino core for ESP32
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncE131.h>

// Function declarations
void connectionBlink();
void setAllChannels(bool state);
void turnOffAllOutputs();
void updateOutputs();

// Network credentials
// TODO: Replace with your network credentials before uploading
const char *ssid = "XXXXXXXX";
const char *password = "XXXXXXXX";

// Debug control - set to false for production (better performance)
#define DEBUG_ENABLED false
#define DEBUG_CHANNEL_CHANGES false // Separate flag for high-frequency channel logging

// Dancer configuration
// Each dancer gets 5 channels, starting from (DANCER_ID - 1) * 5 + 1
// Valid range: 1-12 for dancers
#define DANCER_ID 4 // Change this for each dancer (1-12)

// E1.31 settings
#define UNIVERSE 256 // sACN universe to listen on

// Initialize E1.31
ESPAsyncE131 e131(1); // Single universe configuration

// Define output pins and state tracking
const int NUM_CHANNELS = 5;
const int OUTPUT_PINS[NUM_CHANNELS] = {26, 18, 19, 23, 5}; // GPIO pins for outputs
bool channelStates[NUM_CHANNELS] = {false, false, false, false, false};

// WiFi connection settings and state tracking
const unsigned long WIFI_RETRY_DELAY = 1000;     // 1 second between connection retries
const unsigned long WIFI_DISCONNECT_DELAY = 500; // Wait time after disconnect before reconnecting
unsigned long lastWiFiRetry = 0;
unsigned long wifiDisconnectTime = 0;
bool wifiReconnecting = false;

// Timing and packet handling configuration
const unsigned long PACKET_TIMEOUT = 2500; // Safety timeout: turn off outputs if no packets received
const unsigned long LOOP_INTERVAL = 20;    // Main loop interval (50Hz) for consistent timing
unsigned long lastPacketTime = 0;
unsigned long lastLoopTime = 0;
unsigned long packetCount = 0;
unsigned long lastPacketCountTime = 0;
bool timeoutWarningShown = false;
bool inTimeoutState = false;

// Visual feedback state tracking
unsigned long lastBlinkTime = 0;
bool blinkState = false;
bool wifiConnected = false;

// Calculate channel mapping for this dancer
const int START_CHANNEL = ((DANCER_ID - 1) * NUM_CHANNELS) + 1;

/**
 * Visual feedback function that blinks all channels in a specific pattern
 * to indicate successful WiFi connection.
 * Pattern: 3 sets of double-blinks with short pauses between sets
 */
void connectionBlink()
{
  for (int cycle = 0; cycle < 3; cycle++)
  {
    // Blink twice per cycle
    for (int blink = 0; blink < 2; blink++)
    {
      setAllChannels(true);
      delay(100);
      setAllChannels(false);
      delay(100);
    }
    // Pause between cycles (except after the last one)
    if (cycle < 2)
    {
      delay(200);
    }
  }
}

/**
 * WiFi event handler - manages connection state and provides feedback
 * Handles:
 * - Connection success: Sets up E1.31, disables power save, shows visual confirmation
 * - Disconnection: Updates state and triggers reconnection process
 */
void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.printf("[%lu] Connected! IP Address: %s\n", millis() / 1000, WiFi.localIP().toString().c_str());
    // Disable WiFi power save for lowest latency
    WiFi.setSleep(WIFI_PS_NONE);
    wifiConnected = true;

    // Re-initialize E1.31 after WiFi reconnection
    Serial.println("Re-initializing E1.31 after WiFi reconnection...");
    if (e131.begin(E131_MULTICAST))
    {
      Serial.println("E1.31 re-initialized successfully");
    }
    else
    {
      Serial.println("E1.31 re-initialization failed!");
    }

    // Visual confirmation of connection
    connectionBlink();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.printf("[%lu] WiFi disconnected!\n", millis() / 1000);
    wifiConnected = false;
    break;
  }
}

/**
 * Safety function to ensure all outputs are turned off
 * Used during timeouts or when ensuring a clean state
 */
void turnOffAllOutputs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    channelStates[i] = false;
    digitalWrite(OUTPUT_PINS[i], LOW);
  }
}

/**
 * Updates all physical outputs based on their corresponding channel states
 * Called after processing DMX packets to apply changes synchronously
 */
void updateOutputs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    digitalWrite(OUTPUT_PINS[i], channelStates[i] ? HIGH : LOW);
  }
}

/**
 * Sets all channels to the same state
 * Used primarily for visual feedback (connection status, errors, etc)
 * @param state The state to set all channels to (true = ON, false = OFF)
 */
void setAllChannels(bool state)
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    digitalWrite(OUTPUT_PINS[i], state ? HIGH : LOW);
  }
}

/**
 * Initial setup function - runs once at startup
 * Configures:
 * - Serial debugging
 * - GPIO pins
 * - WiFi connection with static IP
 * - E1.31 protocol
 * - System timing
 */
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nStarting up...");

  // Configure output pins
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    pinMode(OUTPUT_PINS[i], OUTPUT);
    digitalWrite(OUTPUT_PINS[i], LOW); // Initialize all pins to OFF
  }

  // Register WiFi event handler
  WiFi.onEvent(WiFiEvent);

  // Set WiFi to station mode and disconnect from AP to begin with clean connection
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  // Configure static IP based on dancer ID (192.168.0.11-192.168.0.20)
  IPAddress local_IP(192, 168, 0, 10 + DANCER_ID); // 192.168.0.11-192.168.0.20
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.config(local_IP, gateway, subnet))
  {
    Serial.println("Static IP configuration failed!");
  }
  else
  {
    Serial.printf("Static IP configured: %s\n", local_IP.toString().c_str());
  }

  // Initial WiFi connection attempt
  WiFi.begin(ssid, password);
  Serial.println("Initiating WiFi connection...");

  // Initialize E1.31
  Serial.println("\nInitializing E1.31...");
  if (e131.begin(E131_MULTICAST))
  {
    Serial.println("E1.31 initialized successfully");
    Serial.printf("Listening for universe: %d\n", UNIVERSE);
    Serial.printf("Dancer ID: %d, Channels: %d-%d\n", DANCER_ID, START_CHANNEL, START_CHANNEL + NUM_CHANNELS - 1);
  }
  else
  {
    Serial.println("E1.31 initialization failed!");
    Serial.println("Check network settings and restart");
  }

  // Set WiFi to max power for best range
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Initialize timing
  lastLoopTime = millis();
  lastPacketCountTime = millis();
}

/**
 * Main program loop - handles:
 * 1. WiFi connection management and reconnection
 * 2. E1.31 packet processing
 * 3. Output state management
 * 4. Safety timeout monitoring
 * 5. Visual feedback
 *
 * The loop runs at a fixed interval (LOOP_INTERVAL) to maintain consistent timing.
 * It includes several safety features:
 * - Automatic WiFi reconnection with visual feedback
 * - Output timeout if no packets are received
 * - Synchronized output updates to prevent flickering
 * - Packet rate monitoring (when debug enabled)
 */
void loop()
{
  unsigned long currentMillis = millis();

  // Maintain consistent loop timing
  if (currentMillis - lastLoopTime < LOOP_INTERVAL)
  {
    return; // Wait for next loop interval
  }
  lastLoopTime = currentMillis;

  // Handle WiFi connection state with improved reconnection logic
  if (WiFi.status() != WL_CONNECTED)
  {
    // Visual feedback: slow blink when disconnected
    if (currentMillis - lastBlinkTime >= 10000) // 10s slow blink
    {
      blinkState = !blinkState;
      setAllChannels(blinkState);
      lastBlinkTime = currentMillis;
    }

    if (!wifiReconnecting)
    {
      // Start the reconnection process
      if (currentMillis - lastWiFiRetry >= WIFI_RETRY_DELAY)
      {
#if DEBUG_ENABLED
        Serial.println("Starting WiFi reconnection process...");
#endif
        WiFi.disconnect(true);
        wifiDisconnectTime = currentMillis;
        wifiReconnecting = true;
        lastWiFiRetry = currentMillis;
      }
    }
    else
    {
      // Wait for disconnect to complete before reconnecting
      if (currentMillis - wifiDisconnectTime >= WIFI_DISCONNECT_DELAY)
      {
#if DEBUG_ENABLED
        Serial.println("Attempting to reconnect to WiFi...");
#endif
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        wifiReconnecting = false;
      }
    }

    return; // Skip E1.31 processing if not connected
  }
  else
  {
    // Reset reconnection state when connected
    wifiReconnecting = false;
  }

  // Check for packet timeout - safety feature
  if (lastPacketTime > 0 && (currentMillis - lastPacketTime > PACKET_TIMEOUT))
  {
    if (!timeoutWarningShown)
    {
#if DEBUG_ENABLED
      Serial.println("Packet timeout - turning off all outputs for safety");
#endif
      timeoutWarningShown = true;
      inTimeoutState = true;
    }

    turnOffAllOutputs();

    // Don't return here - continue to check for new packets
    // But don't process old timeout state if new packets arrive
    if (!e131.isEmpty())
    {
      // Reset timeout state when packets resume
      timeoutWarningShown = false;
      inTimeoutState = false;
#if DEBUG_ENABLED
      Serial.println("Packets resumed - ready to process commands");
#endif
    }
    else
    {
      return; // Only return if no packets are available
    }
  }

  // Process all available E1.31 packets in this loop iteration
  bool packetProcessed = false;
  while (!e131.isEmpty())
  {
    e131_packet_t packet;
    e131.pull(&packet);

    if (packet.universe == UNIVERSE)
    {
      packetProcessed = true;
      lastPacketTime = currentMillis;
      packetCount++;

      // Reset timeout state when packets are received
      if (inTimeoutState)
      {
#if DEBUG_ENABLED
        Serial.println("Packets resumed - exiting timeout state");
#endif
        inTimeoutState = false;
        timeoutWarningShown = false;
      }

      // Process all 5 channels
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        // Calculate the correct channel offset for this dancer
        int dmxValue = packet.property_values[START_CHANNEL + i];
        bool newState;

        // Clear threshold logic for consistent switching
        if (dmxValue == 0)
        {
          newState = false;
        }
        else if (dmxValue >= 128)
        {
          newState = true;
        }
        else
        {
          // For values 1-127, maintain previous state to avoid flickering
          newState = channelStates[i];
        }

        // Only log changes to reduce serial overhead
#if DEBUG_CHANNEL_CHANGES
        if (channelStates[i] != newState)
        {
          Serial.printf("Ch%d: %s (DMX:%d)\n", i + 1, newState ? "ON" : "OFF", dmxValue);
        }
#endif

        channelStates[i] = newState;
      }
    }
  }

  // Update all outputs at once for synchronized timing
  if (packetProcessed)
  {
    updateOutputs();
  }

  // Report packet rate every 5 seconds
#if DEBUG_ENABLED
  if (currentMillis - lastPacketCountTime >= 5000)
  {
    float packetsPerSecond = (float)packetCount / 5.0;
    Serial.printf("Packet rate: %.1f Hz\n", packetsPerSecond);
    packetCount = 0;
    lastPacketCountTime = currentMillis;
  }
#endif
}