'use strict';

const { contextBridge, ipcRenderer } = require('electron');

function subscribe(channel, callback) {
  if (typeof callback !== 'function') throw new TypeError('A callback is required.');
  const listener = (_event, value) => callback(value);
  ipcRenderer.on(channel, listener);
  return () => ipcRenderer.removeListener(channel, listener);
}

contextBridge.exposeInMainWorld('motorMonitor', Object.freeze({
  listPorts: () => ipcRenderer.invoke('serial:list'),
  connect: (portPath) => ipcRenderer.invoke('serial:connect', portPath),
  disconnect: () => ipcRenderer.invoke('serial:disconnect'),
  sendCommand: (command) => ipcRenderer.invoke('serial:command', command),
  setPollingInterval: (intervalMs) => ipcRenderer.invoke('serial:set-polling', intervalMs),
  onState: (callback) => subscribe('serial:state', callback),
  onLog: (callback) => subscribe('serial:log', callback),
  onStatus: (callback) => subscribe('serial:status', callback),
  onEvent: (callback) => subscribe('serial:event', callback),
  onError: (callback) => subscribe('serial:error', callback)
}));
