'use strict';

const { EventEmitter } = require('node:events');
const { SerialPort } = require('serialport');

const DEFAULT_BAUD_RATE = 115200;
const DEFAULT_TIMEOUT_MS = 3000;
const DEFAULT_POLL_INTERVAL_MS = 100;

class SerialSession extends EventEmitter {
  constructor({ path, baudRate = DEFAULT_BAUD_RATE, timeoutMs = DEFAULT_TIMEOUT_MS, portFactory } = {}) {
    super();
    if (typeof path !== 'string' || path.trim() === '') throw new TypeError('A serial port path is required.');

    this.path = path;
    this.baudRate = baudRate;
    this.timeoutMs = timeoutMs;
    this.portFactory = portFactory || ((options) => new SerialPort(options));
    this.port = null;
    this.buffer = '';
    this.pending = null;
    this.queue = [];
    this.processingQueue = false;
    this.state = 'disconnected';
    this.pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
    this.pollTimer = null;
    this.pollInFlight = false;
    this.auxiliaryIndex = 0;
    this.telemetry = createEmptyTelemetry();
  }

  async connect() {
    if (this.state !== 'disconnected') throw new Error('A connection attempt is already in progress.');

    this.#setState('connecting');
    this.port = this.portFactory({ path: this.path, baudRate: this.baudRate, autoOpen: false });
    this.port.on('data', (chunk) => this.#onData(chunk));
    this.port.on('error', (error) => this.#onPortError(error));
    this.port.on('close', () => this.#onClose());

    try {
      await callbackOperation((done) => this.port.open(done));
      const pong = await this.#enqueue('PING', (line) => line === 'OK PONG');
      if (pong !== 'OK PONG') throw new Error(`Unexpected PING response: ${pong}`);

      const status = parseStatus(await this.#enqueue('STATUS', isStatusResponse));
      this.#setState('connected');
      const snapshot = this.#makeSnapshot(status);
      this.emit('status', snapshot);
      return snapshot;
    } catch (error) {
      await this.#closeQuietly();
      this.#setState('error', error.message);
      throw error;
    }
  }

  startMonitoring(intervalMs = DEFAULT_POLL_INTERVAL_MS) {
    this.setPollingInterval(intervalMs);
  }

  setPollingInterval(intervalMs) {
    if (!Number.isInteger(intervalMs) || intervalMs < 50 || intervalMs > 1000) {
      throw new RangeError('Polling interval must be an integer from 50 to 1000 ms.');
    }
    this.pollIntervalMs = intervalMs;
    clearInterval(this.pollTimer);
    this.pollTimer = null;
    if (this.state === 'connected') {
      this.pollTimer = setInterval(() => void this.#poll(), intervalMs);
      void this.#poll();
    }
  }

  stopMonitoring() {
    clearInterval(this.pollTimer);
    this.pollTimer = null;
    this.pollInFlight = false;
  }

  async send(command) {
    if (this.state !== 'connected') throw new Error('Serial connection is not ready.');
    return this.#enqueue(command, isCommandResponse);
  }

  async disconnect() {
    this.stopMonitoring();
    this.#cancelCommands(new Error('Serial connection was closed.'));
    await this.#closeQuietly();
    this.#setState('disconnected');
  }

  async #poll() {
    if (this.pollInFlight || this.state !== 'connected') return;
    this.pollInFlight = true;
    try {
      const status = parseStatus(await this.#enqueue('STATUS', isStatusResponse));
      await this.#pollAuxiliaryValue();
      this.emit('status', this.#makeSnapshot(status));
    } catch (error) {
      if (this.state === 'connected') this.emit('monitoring-error', error.message);
    } finally {
      this.pollInFlight = false;
    }
  }

  async #pollAuxiliaryValue() {
    const commands = ['GET ENC 0', 'GET ENC 1', 'GET ENC 2', 'GET ADC 3', 'GET ADC 4', 'GET FAULT_INFO'];
    const command = commands[this.auxiliaryIndex];
    this.auxiliaryIndex = (this.auxiliaryIndex + 1) % commands.length;
    const response = await this.#enqueue(command, isCommandResponse);
    if (response.startsWith('ERR ')) throw new Error(`${command}: ${response}`);

    const value = response.slice(3).trim();
    if (command.startsWith('GET ENC ')) {
      this.telemetry.encoders[Number(command.at(-1))] = Number(value);
    } else if (command === 'GET ADC 3') {
      this.telemetry.voltageV = Number(value);
    } else if (command === 'GET ADC 4') {
      this.telemetry.currentMa = Number(value);
    } else {
      this.telemetry.fault = parseFaultInfo(value);
    }
  }

  #makeSnapshot(status) {
    return {
      ...status,
      telemetry: {
        encoders: [...this.telemetry.encoders],
        voltageV: this.telemetry.voltageV,
        currentMa: this.telemetry.currentMa,
        fault: { ...this.telemetry.fault }
      },
      observedAt: new Date().toISOString()
    };
  }

  #enqueue(command, accepts) {
    if (!this.port?.isOpen) return Promise.reject(new Error('Serial port is not open.'));
    return new Promise((resolve, reject) => {
      this.queue.push({ command, accepts, resolve, reject });
      void this.#drainQueue();
    });
  }

  async #drainQueue() {
    if (this.processingQueue) return;
    this.processingQueue = true;
    try {
      while (this.queue.length > 0 && this.port?.isOpen) {
        const item = this.queue.shift();
        try {
          item.resolve(await this.#requestNow(item.command, item.accepts));
        } catch (error) {
          item.reject(error);
        }
      }
    } finally {
      this.processingQueue = false;
    }
  }

  #requestNow(command, accepts) {
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending = null;
        reject(new Error(`${command} timed out after ${this.timeoutMs} ms.`));
      }, this.timeoutMs);

      this.pending = { accepts, resolve, reject, timer };
      this.#log('tx', command);
      this.port.write(`${command}\n`, (writeError) => {
        if (writeError) {
          this.#rejectPending(writeError);
          return;
        }
        this.port.drain((drainError) => {
          if (drainError) this.#rejectPending(drainError);
        });
      });
    });
  }

  #onData(chunk) {
    this.buffer += chunk.toString('utf8');
    const lines = this.buffer.split(/\r?\n/);
    this.buffer = lines.pop() || '';

    for (const rawLine of lines) {
      const line = rawLine.trim();
      if (!line) continue;
      this.#log('rx', line);
      if (line.startsWith('EVT ')) {
        this.emit('event', { timestamp: new Date().toISOString(), text: line });
      } else if (this.pending?.accepts(line)) {
        const { resolve, timer } = this.pending;
        this.pending = null;
        clearTimeout(timer);
        resolve(line);
      }
    }
  }

  #onPortError(error) {
    this.stopMonitoring();
    this.#cancelCommands(error);
    this.emit('session-error', error.message);
    if (this.state === 'connected') this.#setState('error', error.message);
  }

  #onClose() {
    this.stopMonitoring();
    this.#cancelCommands(new Error('Serial port closed unexpectedly.'));
    if (this.state === 'connected') this.#setState('disconnected', 'Device disconnected.');
  }

  #rejectPending(error) {
    if (!this.pending) return;
    const { reject, timer } = this.pending;
    this.pending = null;
    clearTimeout(timer);
    reject(error);
  }

  #cancelCommands(error) {
    this.#rejectPending(error);
    for (const item of this.queue.splice(0)) item.reject(error);
  }

  async #closeQuietly() {
    if (!this.port?.isOpen) return;
    try {
      await callbackOperation((done) => this.port.close(done));
    } catch {
      // Preserve the original connection error when cleanup also fails.
    }
  }

  #setState(state, message = '') {
    this.state = state;
    this.emit('state', { state, message, path: this.path });
  }

  #log(direction, text) {
    this.emit('log', { timestamp: new Date().toISOString(), direction, text });
  }
}

function callbackOperation(operation) {
  return new Promise((resolve, reject) => operation((error) => (error ? reject(error) : resolve())));
}

function isStatusResponse(line) {
  const jsonText = line.startsWith('OK ') ? line.slice(3).trimStart() : line;
  return jsonText.startsWith('{');
}

function isCommandResponse(line) {
  return /^(?:OK|ERR)(?:\s|$)/.test(line);
}

function parseStatus(line) {
  const jsonText = line.startsWith('OK ') ? line.slice(3).trimStart() : line;
  try {
    return JSON.parse(jsonText);
  } catch (error) {
    throw new Error(`Invalid STATUS JSON: ${error.message}`);
  }
}

function parseFaultInfo(value) {
  const match = /^(\S+)\s+(0x[0-9a-f]+)\s+(\d+)$/i.exec(value);
  if (!match) return { reason: 'UNKNOWN', axisMask: '0x00', timestampUs: 0 };
  return { reason: match[1], axisMask: match[2], timestampUs: Number(match[3]) };
}

function createEmptyTelemetry() {
  return {
    encoders: [null, null, null],
    voltageV: null,
    currentMa: null,
    fault: { reason: 'NONE', axisMask: '0x00', timestampUs: 0 }
  };
}

module.exports = {
  SerialSession,
  DEFAULT_BAUD_RATE,
  DEFAULT_POLL_INTERVAL_MS,
  isStatusResponse,
  parseStatus,
  parseFaultInfo
};
