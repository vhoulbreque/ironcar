var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);
var fs = require('fs');
var os = require('os');
var ifaces = os.networkInterfaces();
var path = require('path');

const testFolder = './autopilots/';
const stream_image = 'image_stream.jpg';
const IP = '0.0.0.0';
const PORT = 8000;

app.use(express.static(__dirname + '/public'));
app.use('/', express.static(path.join(__dirname, 'stream')));


var starter = "stopped";
var currentMode = -1;
var currentModel = -1;
var currentStatus = -1;
var currentMaxSpeed = 0.5;
var all_images = [];
var IMAGE_PLACEHOLDER = 'image_stream.jpg';
var current_image = IMAGE_PLACEHOLDER;
var streaming = "stopped";

// Create folder ./stream
// And deletes old images

if (!fs.existsSync('./stream/')) {
    fs.mkdirSync('./stream/');
}

fs.readdir('./stream/', (err, files) => {
  if (err) throw err;

  for (const file of files) {
    fs.unlink(path.join('./stream/', file), err => {
      if (err) throw err;
      // console.log('Deleting file at : ', path.join('./stream/', file));
    });
  }
});


// Models
function find_models(folder){
    var autopilot_models = [];
    fs.readdirSync(folder).forEach(function(file){
        if (file.indexOf('.hdf5') !== -1 ){
            autopilot_models.push(file);
        }
    });
    io.emit('new_available_model', autopilot_models);
}

fs.watch(testFolder, function (event, filename) {
    console.log('event in testFolder : ', event);
    find_models(testFolder);
});


function delete_files(folder, files_to_delete) {

  fs.readdir(folder, (err, files) => {
    if (err) throw err;
    for (const file of files) {
      if (files_to_delete == null || files_to_delete.indexOf(file) >= 0) {
        fs.unlink(path.join(folder, file), err => {
          if (err) console.log(err);
          console.log('Deleting file at : ', path.join(folder, file));
        });
      }
    }
  });
}


function send_picture() {

  fs.readdirSync('./stream/').forEach(function(file){
    if (file.indexOf('~') === -1) {
      all_images.push(file);
    }
  });

  all_images.sort();

  if (all_images.length != 0) {
    current_image = all_images[all_images.length - 1];

    fs.readFile('./stream/' + current_image, function(err, buf){
      if(err) throw err;
      io.emit('picture', { image: true, buffer: buf.toString('base64') });
      console.log('PHOTOGRAPHY the picture : ', current_image);
    });
  }
}

// Streaming
function startStreaming(io) {

    console.log('Watching for changes...');

    app.set('watchingFile', true);

    fs.watch('./stream/', function(current, previous) {

        all_images = [];

        fs.readdirSync('./stream/').forEach(function(file){
            if (file.indexOf('~') === -1) {
                all_images.push(file);
            }
        });

        all_images.sort();
        index = all_images.findIndex(x => x === current_image);

        if (streaming == "started") {
            console.log(streaming);

            if (current_image == IMAGE_PLACEHOLDER) {
                if (all_images.length != 0) {
                    current_image = all_images[0];
                    all_images = all_images.slice(1, all_images.length);
                }
            }
            else {
                if (index == all_images.length - 1) {
                    all_images = [];
                    current_image = current_image;  // If there is no new image, keep sending the same image
                }
                else {
                    to_delete_images = all_images.slice(0, index+1);
                    all_images = all_images.slice(index+1, all_images.length);
                    current_image = all_images[all_images.length-1];
                    delete_files('./stream/', to_delete_images);
                }
            }

            console.log('Sending : ', current_image);
            io.sockets.emit('liveStream', current_image + '?_t=' + (Math.random() * 100000));
        }
        else {
            if (all_images.length != 0) {
                 current_image = all_images[all_images.length-1];
            }

            console.log("not streaming");
            io.sockets.emit('liveStream', IMAGE_PLACEHOLDER + '?_t=' + (Math.random() * 100000));
        }
    });

}


function stopStreaming(io) {
  console.log('Stop streaming');
  app.set('watchingFile', false);
}


io.on('connection', function(client){

    var address = client.handshake.address;
    console.log(address + ' connected');

    client.on('clientLoadedPage', function() {
        //delete_files('./stream/', null);
        console.log('clientLoadedPage');
        // Send the current state to the new clients
        if (currentMode != -1){ client.emit('mode_update', currentMode);}
        if (currentModel != -1){ client.emit('model_update', currentModel);}
        client.emit('starterUpdate', starter);
        client.emit('stream', streaming);
        if (currentStatus != -1){ client.emit('msg2user', currentStatus);}
    });

    client.emit('maxSpeedUpdate', currentMaxSpeed);

    //charge available models found
    find_models(testFolder);

    // Mode selection
    client.on('modeSwitched', function(data) {
        console.log('selected mode: ' + data);
        io.emit('mode_update', data);
        currentMode = data;
    });

    // Starter button
    client.on('starter', function() {
        if(starter == "started"){
            starter = "stopped";
        }else{
            starter = "started";
        }
        console.log(starter);
        io.emit('starterUpdate', starter);
    });

    // Max speed update
    client.on('maxSpeed', function(maxSpeed) {
        io.emit('maxSpeedUpdate', maxSpeed);
        currentMaxSpeed = maxSpeed;
    });


    // Commands transfer
    client.on('dir', function(data){
        io.emit('dir', data);
        //console.log(data);
    });

    client.on('gas', function(data){
        io.emit('gas', data);
        //console.log(data);
    });

    client.on('model_update', function(data){
        if (data != " "){
            console.log('model_update', data);
            io.emit('model_update', data);
            currentModel = data;
        }
    });

    // Streaming
    client.on('streamUpdate', function() {
	if (streaming == "started") {
            streaming = "stopped";
        } else {
            streaming = "started";
        }
        io.emit('stream', streaming);
        startStreaming(io);
    });

    /*
    client.on('stop-stream', function() {
	stopStreaming(io);
    });
    */

    client.on('takePicture', function() {
        send_picture();
    });

    // Disconnect
    client.on('disconnect', function(){
        fs.unwatchFile('./stream/');

        var address = client.handshake.address;


   	console.log( address + ' disconnected');
    });

    // Messages to send to the user
    client.on('msg2user', function(message){
        console.log(message);
        io.emit('msg2user', message);
    });

});

io.on('error', function(data){
    console.log(data);
});



Object.keys(ifaces).forEach(function (ifname) {
  var alias = 0;

  ifaces[ifname].forEach(function (iface) {
    if ('IPv4' !== iface.family || iface.internal !== false) {
      // skip over internal (i.e. 127.0.0.1) and non-ipv4 addresses
      return;
    }

    if (alias == 0) {
      console.log("raspberry pi IP on ", ifname, " : ", iface.address);
    }
    ++alias;
  });
});


server.listen(PORT, function(){
    console.log('listening on ' +  PORT);
});

