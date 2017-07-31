var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);
var fs = require('fs');

const testFolder = './autopilots/';


app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
    console.log("user accessed root");
    res.sendFile(__dirname + '/public/index.html');
});

started = 0;
autopilot_models = [];
function find_models(folder){
    fs.readdirSync(folder).forEach(function(file){
        console.log(file);
        if (autopilot_models.indexOf(file) == -1) {
            autopilot_models.push(file);
            console.log('new_available_model' + file);
            io.emit('new_available_model', file);
        }
    });
}

io.on('connection', function(client){
    console.log('connected');

    //charge available models found
    find_models(testFolder);

  	client.on('msg', function(data){
    	console.log(data);
  	});

    // Mode selection
    client.on('modeSwitched', function(data) {
        console.log('selected mode: ' + data);
        io.emit('mode_update', data);
    });

    // Starter button
    client.on('starter', function() {
        if(started){
            console.log('Started');
            io.emit('starter', "start");
        }else{
            console.log("Stopped");
            io.emit('starter', "stop");
        }
        started = 1 - started;
    });

    // Commands transfer
    client.on('dir', function(data){
        io.emit('dir', data);
    });
    client.on('gas', function(data){
        io.emit('gas', data);
    });


    // Autopilot model choice
    client.on('refresh_models', function() {
        find_models(testFolder);
    });

    client.on('model_update', function(data){
        console.log('model_update', autopilot_models[data]);
        io.emit('model_update', autopilot_models[data]);
    });


    client.on('disconnect', function(){
    	console.log('disconnected');
  	});
});

io.on('error', function(data){
	console.log(data)
});

server.listen(8000, function(){
  	console.log('listening on 8000');
});


