var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);
var fs = require('fs');
var os = require('os');
var ifaces = os.networkInterfaces();

const testFolder = './autopilots/';
const stream_image = 'img_stream.jpg';
const IP = '0.0.0.0';
const PORT = 8000;

app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
    console.log("user accessed root");
    res.sendFile(__dirname + '/public/index.html');
});

var starter = "stopped";
var currentMode = -1;
var currentModel = -1;
var currentStatus = -1;
var currentMaxSpeed = 0.5;

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


function send_stream(stm_image) {
    fs.readFile(__dirname + '/' + stm_image, function(err, buf){
        console.log(__dirname + '/' + stm_image);
        // it's possible to embed binary data
        // within arbitrarily-complex objects
        io.emit('image', { image: true, buffer: buf.toString('base64') });
        console.log('ici');
    });
}

fs.watch(stream_image, function (event, filename) {
    console.log('event in stream_image : ', event);
    send_stream(stream_image);
});

io.on('connection', function(client){
    var address = client.handshake.address;
    console.log(address + ' connected');

    client.on('clientLoadedPage', function(){        
    // Send the current state to the new clients
    if (currentMode != -1){ client.emit('mode_update', currentMode);}
    if (currentModel != -1){ client.emit('model_update', currentModel);}
    client.emit('starterUpdate', starter);
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


    client.on('disconnect', function(){
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


