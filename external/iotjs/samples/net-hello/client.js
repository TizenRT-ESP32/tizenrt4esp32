/* Copyright 2017-present Samsung Electronics Co., Ltd. and other contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

var net = require('net');

var port = 7468;

var msg = '';
var socket = new net.Socket();

var address = process.argv[2] ? process.argv[2] : "127.0.0.1";

socket.connect(port, address);

socket.on('data', function(data) {
  msg += data;
});

socket.on('end', function() {
  console.log(msg);
  socket.end();
});
