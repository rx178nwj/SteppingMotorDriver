'use strict';

const path = require('node:path');
const { spawn } = require('node:child_process');
const electron = require('electron');

const env = { ...process.env };
delete env.ELECTRON_RUN_AS_NODE;

const child = spawn(electron, [path.join(__dirname, 'renderer-smoke-main.js')], {
  cwd: path.join(__dirname, '..'), env, stdio: 'inherit', windowsHide: true
});
child.on('error', (error) => { console.error(error); process.exitCode = 1; });
child.on('exit', (code) => { process.exitCode = code ?? 1; });
