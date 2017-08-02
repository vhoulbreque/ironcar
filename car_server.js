var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);
var fs = require('fs');

const testFolder = './autopilots/';

const IP = 'localhost';
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
    console.log(event);
    find_models(testFolder);
});

io.on('connection', function(client){
    console.log('connected');
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
    	console.log('disconnected');
    });

    // Messages to send to the user
    client.on('msg2user', function(message){
        console.log(message);
        io.emit('msg2user', message);
    });
    

});

server.listen(IP, PORT, function(){
  	console.log('listening on ' + IP + ':' + PORT);
});


