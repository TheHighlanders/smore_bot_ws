// This is a separate thread that keeps running even when the main thread is throttled
let counter = 0;

function sendKeepalive() {
  counter++;
  self.postMessage({
    type: 'keepalive',
    time: Date.now(),
    counter: counter
  });
  
  setTimeout(sendKeepalive, 100);
}

sendKeepalive();

setInterval(function() {
  self.postMessage({
    type: 'keepalive',
    time: Date.now(),
    method: 'interval'
  });
}, 250);  // 4 times per second
