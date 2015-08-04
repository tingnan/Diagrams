// Copyright (c) 2015 Native Client Authors.

function moduleDidLoad() {
  // After the NaCl module has loaded, common.naclModule is a reference to the
  // NaCl module's <embed> element.
  //
  // postMessage sends a message to it.
  // common.naclModule.postMessage('hello');
}

// This function is called by common.js when a message is received from the
// NaCl module.
function handleMessage(message) {
  var logEl = document.getElementById('log');
  logEl.textContent = message.data;
}
