'use strict';

const assert = require('node:assert/strict');
const { EventEmitter } = require('node:events');
const test = require('node:test');
const { SerialSession, parseStatus, parseFaultInfo } = require('../src/main/serial-session');

class FakePort extends EventEmitter {
  constructor(responses) {
    super();
    this.responses = responses;
    this.isOpen = false;
  }

  open(callback) { this.isOpen = true; callback(); }
  drain(callback) { callback(); }
  close(callback) { this.isOpen = false; callback(); this.emit('close'); }
  write(data, callback) {
    callback();
    const command = data.trim();
    const response = this.responses[command];
    if (response) queueMicrotask(() => this.emit('data', Buffer.from(response)));
  }
}

test('connect performs PING then STATUS and parses the firmware JSON format', async () => {
  const fake = new FakePort({
    PING: 'OK PONG\n',
    STATUS: '{"microstep":"1/16","axes":[]}\n'
  });
  const session = new SerialSession({ path: 'COM9', portFactory: () => fake, timeoutMs: 100 });
  const status = await session.connect();
  assert.equal(status.microstep, '1/16');
  assert.deepEqual(status.axes, []);
  assert.deepEqual(status.telemetry.encoders, [null, null, null]);
  assert.equal(session.state, 'connected');
});

test('STATUS parser also accepts the documented OK prefix', () => {
  assert.deepEqual(parseStatus('OK {"axes":[]}'), { axes: [] });
});

test('connect rejects when PING times out and closes the port', async () => {
  const fake = new FakePort({});
  const session = new SerialSession({ path: 'COM9', portFactory: () => fake, timeoutMs: 10 });
  await assert.rejects(session.connect(), /PING timed out/);
  assert.equal(fake.isOpen, false);
  assert.equal(session.state, 'error');
});

test('control commands are queued and return firmware responses', async () => {
  const fake = new FakePort({
    PING: 'OK PONG\n', STATUS: '{"axes":[]}\n',
    'ENABLE 0': 'OK\n', 'MOVE 0 100': 'ERR E007 NOT_HOMED\n'
  });
  const session = new SerialSession({ path: 'COM9', portFactory: () => fake, timeoutMs: 100 });
  await session.connect();
  const responses = await Promise.all([session.send('ENABLE 0'), session.send('MOVE 0 100')]);
  assert.deepEqual(responses, ['OK', 'ERR E007 NOT_HOMED']);
});

test('fault information parser exposes reason, mask and timestamp', () => {
  assert.deepEqual(parseFaultInfo('OVERCURRENT 0x02 1234'), {
    reason: 'OVERCURRENT', axisMask: '0x02', timestampUs: 1234
  });
});
