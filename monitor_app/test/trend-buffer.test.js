'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const { TrendBuffer, COLUMN } = require('../src/renderer/trend-charts');

function snapshot(seconds, position = 100) {
  return {
    observedAt: new Date(seconds * 1000).toISOString(),
    axes: [
      { id: 0, pos: position, vel: 10 },
      { id: 1, pos: 200, vel: 20 },
      { id: 2, pos: 300, vel: 30 }
    ],
    telemetry: { encoders: [position + 5, 207, null], currentMa: 1234.5, voltageV: 23.9 }
  };
}

test('trend buffer extracts every Phase 3 series', () => {
  const buffer = new TrendBuffer();
  assert.equal(buffer.push(snapshot(100)), true);
  assert.deepEqual(buffer.columns[COLUMN.vel0], [10]);
  assert.deepEqual(buffer.columns[COLUMN.pos1], [200]);
  assert.deepEqual(buffer.columns[COLUMN.enc2], [null]);
  assert.deepEqual(buffer.columns[COLUMN.error0], [5]);
  assert.deepEqual(buffer.columns[COLUMN.current], [1234.5]);
  assert.deepEqual(buffer.columns[COLUMN.voltage], [23.9]);
});

test('trend buffer returns only the selected time window', () => {
  const buffer = new TrendBuffer();
  buffer.push(snapshot(100));
  buffer.push(snapshot(120));
  buffer.push(snapshot(140));
  const [times, values] = buffer.window(30, [COLUMN.time, COLUMN.pos0]);
  assert.deepEqual(times, [120, 140]);
  assert.deepEqual(values, [100, 100]);
});

test('trend buffer rejects duplicate or out-of-order samples', () => {
  const buffer = new TrendBuffer();
  assert.equal(buffer.push(snapshot(100)), true);
  assert.equal(buffer.push(snapshot(100)), false);
  assert.equal(buffer.push(snapshot(99)), false);
  assert.equal(buffer.columns[COLUMN.time].length, 1);
});
