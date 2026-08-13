'use strict';

const path = require('node:path');
const { app, BrowserWindow, ipcMain } = require('electron');
const { SerialPort } = require('serialport');
const { SerialSession } = require('./serial-session');

let mainWindow;
let session;
let pollingIntervalMs = 100;

const ALLOWED_COMMANDS = [
  /^(?:ENABLE|DISABLE|STOP|STOP_FREE) (?:[0-2]|ALL)$/,
  /^ESTOP$/,
  /^CLEAR_FAULT$/,
  /^HOME [0-2]$/,
  /^VEL [0-2] -?\d+$/,
  /^(?:MOVE|MOVETO) [0-2] -?\d+$/,
  /^GET LOG$/,
  /^LOG_CLEAR$/
];

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1000,
    height: 720,
    minWidth: 760,
    minHeight: 560,
    webPreferences: {
      preload: path.join(__dirname, '..', 'preload', 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true
    }
  });

  mainWindow.loadFile(path.join(__dirname, '..', 'renderer', 'index.html'));
}

function send(channel, payload) {
  if (mainWindow && !mainWindow.isDestroyed()) mainWindow.webContents.send(channel, payload);
}

function registerIpc() {
  ipcMain.handle('serial:list', async () => {
    const ports = await SerialPort.list();
    return ports.map(({ path: portPath, manufacturer, serialNumber, vendorId, productId }) => ({
      path: portPath,
      manufacturer: manufacturer || '',
      serialNumber: serialNumber || '',
      vendorId: vendorId || '',
      productId: productId || ''
    }));
  });

  ipcMain.handle('serial:connect', async (_event, portPath) => {
    if (typeof portPath !== 'string' || portPath.length > 260) throw new Error('Invalid serial port.');
    const available = await SerialPort.list();
    if (!available.some((port) => port.path === portPath)) throw new Error('The selected serial port is no longer available.');

    if (session) await session.disconnect();
    session = new SerialSession({ path: portPath });
    session.on('state', (value) => send('serial:state', value));
    session.on('log', (value) => send('serial:log', value));
    session.on('status', (value) => send('serial:status', value));
    session.on('event', (value) => send('serial:event', value));
    session.on('session-error', (message) => send('serial:error', message));
    session.on('monitoring-error', (message) => send('serial:error', message));
    const status = await session.connect();
    session.startMonitoring(pollingIntervalMs);
    return status;
  });

  ipcMain.handle('serial:disconnect', async () => {
    if (session) await session.disconnect();
    session = null;
  });

  ipcMain.handle('serial:command', async (_event, command) => {
    if (typeof command !== 'string' || !ALLOWED_COMMANDS.some((pattern) => pattern.test(command))) {
      throw new Error('Command is not allowed.');
    }
    if (!session) throw new Error('No board is connected.');
    return session.send(command);
  });

  ipcMain.handle('serial:set-polling', (_event, intervalMs) => {
    if (!Number.isInteger(intervalMs) || intervalMs < 50 || intervalMs > 1000) {
      throw new Error('Polling interval must be from 50 to 1000 ms.');
    }
    pollingIntervalMs = intervalMs;
    if (session) session.setPollingInterval(intervalMs);
    return pollingIntervalMs;
  });
}

app.whenReady().then(() => {
  registerIpc();
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('before-quit', () => {
  if (session) void session.disconnect();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});
