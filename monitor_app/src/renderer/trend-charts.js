'use strict';

(function exposeTrendCharts(root) {
  const COLUMN = Object.freeze({
    time: 0, vel0: 1, vel1: 2, vel2: 3,
    pos0: 4, pos1: 5, pos2: 6, enc0: 7, enc1: 8, enc2: 9,
    error0: 10, error1: 11, error2: 12, current: 13, voltage: 14
  });

  class TrendBuffer {
    constructor(maxDurationSeconds = 300) {
      this.maxDurationSeconds = maxDurationSeconds;
      this.columns = Array.from({ length: 15 }, () => []);
      this.lastTimestamp = 0;
    }

    push(status) {
      const timestamp = Date.parse(status?.observedAt) / 1000;
      if (!Number.isFinite(timestamp) || timestamp <= this.lastTimestamp) return false;
      this.lastTimestamp = timestamp;
      const axes = new Map((status.axes || []).map((axis) => [axis.id, axis]));
      const encoders = status.telemetry?.encoders || [];
      const row = [timestamp];

      for (let axis = 0; axis < 3; axis += 1) row.push(finiteOrNull(axes.get(axis)?.vel));
      for (let axis = 0; axis < 3; axis += 1) row.push(finiteOrNull(axes.get(axis)?.pos));
      for (let axis = 0; axis < 3; axis += 1) row.push(finiteOrNull(encoders[axis]));
      for (let axis = 0; axis < 3; axis += 1) {
        const position = axes.get(axis)?.pos;
        const encoder = encoders[axis];
        row.push(Number.isFinite(position) && Number.isFinite(encoder) ? encoder - position : null);
      }
      row.push(finiteOrNull(status.telemetry?.currentMa));
      row.push(finiteOrNull(status.telemetry?.voltageV));

      row.forEach((value, index) => this.columns[index].push(value));
      this.#trim(timestamp - this.maxDurationSeconds - 2);
      return true;
    }

    window(seconds, indexes) {
      const times = this.columns[COLUMN.time];
      if (times.length === 0) return indexes.map(() => []);
      const cutoff = times.at(-1) - seconds;
      let start = 0;
      while (start < times.length && times[start] < cutoff) start += 1;
      return indexes.map((index) => this.columns[index].slice(start));
    }

    clear() {
      for (const column of this.columns) column.length = 0;
      this.lastTimestamp = 0;
    }

    #trim(cutoff) {
      const times = this.columns[COLUMN.time];
      let removeCount = 0;
      while (removeCount < times.length && times[removeCount] < cutoff) removeCount += 1;
      if (removeCount > 0) for (const column of this.columns) column.splice(0, removeCount);
    }
  }

  class TrendDashboard {
    constructor({ uPlotClass, containers, thresholds }) {
      this.uPlotClass = uPlotClass;
      this.containers = containers;
      this.thresholds = thresholds;
      this.buffer = new TrendBuffer();
      this.windowSeconds = 60;
      this.paused = false;
      this.charts = [];
      this.frameRequested = false;
      this.resizeObserver = new ResizeObserver(() => this.#resize());
      this.#createCharts();
    }

    add(status) {
      if (!this.buffer.push(status) || this.paused) return;
      this.scheduleRender();
    }

    setWindow(seconds) {
      this.windowSeconds = Number(seconds);
      if (!this.paused) this.render();
    }

    setPaused(paused) {
      this.paused = Boolean(paused);
      if (!this.paused) this.render();
    }

    redrawThresholds() {
      for (const chart of this.charts) chart.setData(chart.data);
    }

    clear() {
      this.buffer.clear();
      this.render();
    }

    scheduleRender() {
      if (this.frameRequested) return;
      this.frameRequested = true;
      requestAnimationFrame(() => {
        this.frameRequested = false;
        this.render();
      });
    }

    render() {
      const C = COLUMN;
      this.charts[0].setData(this.buffer.window(this.windowSeconds, [C.time, C.vel0, C.vel1, C.vel2]));
      this.charts[1].setData(this.buffer.window(this.windowSeconds, [C.time, C.pos0, C.enc0, C.pos1, C.enc1, C.pos2, C.enc2]));
      this.charts[2].setData(this.buffer.window(this.windowSeconds, [C.time, C.error0, C.error1, C.error2]));
      this.charts[3].setData(this.buffer.window(this.windowSeconds, [C.time, C.current, C.voltage]));
    }

    #createCharts() {
      const colors = ['#42c5b5', '#f0c674', '#d09cf1'];
      const common = (series, extra = {}) => ({
        width: chartWidth(extra.container), height: 245,
        cursor: { drag: { x: true, y: false } },
        legend: { show: true },
        axes: [
          { stroke: '#8ea0b2', grid: { stroke: '#273646', width: 1 }, ticks: { stroke: '#273646' } },
          { stroke: '#8ea0b2', grid: { stroke: '#273646', width: 1 }, ticks: { stroke: '#273646' }, size: 64 }
        ],
        scales: { x: { time: true } },
        series: [{ label: '時刻' }, ...series],
        ...extra.options
      });

      this.charts = [
        new this.uPlotClass(common([
          line('CH0', colors[0]), line('CH1', colors[1]), line('CH2', colors[2])
        ], { container: this.containers.velocity }), [[], [], [], []], this.containers.velocity),
        new this.uPlotClass(common([
          line('CH0 Step', colors[0]), line('CH0 Enc', colors[0], [7, 4]),
          line('CH1 Step', colors[1]), line('CH1 Enc', colors[1], [7, 4]),
          line('CH2 Step', colors[2]), line('CH2 Enc', colors[2], [7, 4])
        ], { container: this.containers.position }), Array.from({ length: 7 }, () => []), this.containers.position),
        new this.uPlotClass(common([
          line('CH0 偏差', colors[0]), line('CH1 偏差', colors[1]), line('CH2 偏差', colors[2])
        ], {
          container: this.containers.deviation,
          options: {
            scales: {
              x: { time: true },
              y: { range: (_u, min, max) => thresholdRange(min, max, Number(this.thresholds.stall()), true) }
            },
            plugins: [thresholdPlugin(() => [
              { value: Number(this.thresholds.stall()), label: '+脱調', color: '#ff6262', scale: 'y' },
              { value: -Number(this.thresholds.stall()), label: '-脱調', color: '#ff6262', scale: 'y' }
            ])]
          }
        }), [[], [], [], []], this.containers.deviation),
        new this.uPlotClass(common([
          { ...line('電流 [mA]', colors[0]), scale: 'current' },
          { ...line('電圧 [V]', colors[1]), scale: 'voltage' }
        ], {
          container: this.containers.power,
          options: {
            scales: {
              x: { time: true },
              current: { range: (_u, min, max) => thresholdRange(min, max, Number(this.thresholds.current()), false) },
              voltage: { auto: true }
            },
            axes: [
              { stroke: '#8ea0b2', grid: { stroke: '#273646', width: 1 }, ticks: { stroke: '#273646' } },
              { scale: 'current', label: 'mA', stroke: colors[0], grid: { stroke: '#273646', width: 1 }, size: 64 },
              { scale: 'voltage', label: 'V', side: 1, stroke: colors[1], grid: { show: false }, size: 52 }
            ],
            plugins: [thresholdPlugin(() => [
              { value: Number(this.thresholds.current()), label: '過電流', color: '#ff6262', scale: 'current' }
            ])]
          }
        }), [[], [], []], this.containers.power)
      ];

      Object.values(this.containers).forEach((container) => this.resizeObserver.observe(container));
    }

    #resize() {
      const containers = Object.values(this.containers);
      this.charts.forEach((chart, index) => chart.setSize({ width: chartWidth(containers[index]), height: 245 }));
    }
  }

  function line(label, stroke, dash = []) {
    return { label, stroke, width: 1.5, dash, points: { show: false }, spanGaps: true };
  }

  function chartWidth(container) {
    return Math.max(320, Math.floor(container.clientWidth || 600));
  }

  function finiteOrNull(value) {
    return Number.isFinite(value) ? value : null;
  }

  function thresholdRange(min, max, threshold, symmetric) {
    const safeThreshold = Number.isFinite(threshold) && threshold > 0 ? threshold : 1;
    if (symmetric) {
      const extent = Math.max(Math.abs(min || 0), Math.abs(max || 0), safeThreshold) * 1.1;
      return [-extent, extent];
    }
    const lower = Math.min(min || 0, 0);
    const upper = Math.max(max || 0, safeThreshold) * 1.1;
    return [lower, upper];
  }

  function thresholdPlugin(linesProvider) {
    return {
      hooks: {
        draw: [(u) => {
          const { ctx, bbox } = u;
          ctx.save();
          ctx.setLineDash([6, 5]);
          ctx.lineWidth = 1;
          ctx.font = '11px Segoe UI';
          for (const lineSpec of linesProvider()) {
            if (!Number.isFinite(lineSpec.value)) continue;
            const y = Math.round(u.valToPos(lineSpec.value, lineSpec.scale, true));
            ctx.strokeStyle = lineSpec.color;
            ctx.beginPath();
            ctx.moveTo(bbox.left, y);
            ctx.lineTo(bbox.left + bbox.width, y);
            ctx.stroke();
            ctx.fillStyle = lineSpec.color;
            ctx.fillText(`${lineSpec.label} ${lineSpec.value}`, bbox.left + 5, y - 4);
          }
          ctx.restore();
        }]
      }
    };
  }

  const api = { TrendBuffer, TrendDashboard, COLUMN };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  root.TrendCharts = api;
}(globalThis));
