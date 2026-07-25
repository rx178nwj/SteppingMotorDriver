'use strict';

const path = require('node:path');
const { app, BrowserWindow, ipcMain } = require('electron');

app.disableHardwareAcceleration();
ipcMain.handle('serial:list', () => []);

app.whenReady().then(async () => {
  const errors = [];
  const window = new BrowserWindow({
    show: false,
    width: 1200,
    height: 900,
    webPreferences: {
      preload: path.join(__dirname, '..', 'src', 'preload', 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true
    }
  });
  window.webContents.on('console-message', (_event, level, message) => {
    if (level >= 2) errors.push(message);
  });

  try {
    await window.loadFile(path.join(__dirname, '..', 'src', 'renderer', 'index.html'));
    const result = await window.webContents.executeJavaScript(`new Promise((resolve) => {
      requestAnimationFrame(() => requestAnimationFrame(() => resolve({
        uPlotType: typeof window.uPlot,
        chartCount: document.querySelectorAll('.uplot').length,
        canvasCount: document.querySelectorAll('.uplot canvas').length,
        controlCount: document.querySelectorAll('.trend-controls input, .trend-controls select, .trend-controls button').length
      })));
    })`);
    console.log(JSON.stringify({ ...result, errors }));
    const passed = result.uPlotType === 'function' && result.chartCount === 4 && result.canvasCount >= 4 && result.controlCount === 4 && errors.length === 0;
    app.exit(passed ? 0 : 1);
  } catch (error) {
    console.error(error);
    app.exit(1);
  }
});
