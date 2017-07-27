var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);

started = 0;
autopilot_models = [];

app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
    console.log("user accessed root");
    res.sendFile(__dirname + '/public/index.html');
});


io.on('connection', function(client){
    console.log('connected');

    //charge available models found
    for (var i = 0; i < autopilot_models.length; i++) {
        client.emit('new_available_model', autopilot_models[i]);
    }

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
    client.on('available_model', function(data){
        console.log('available model:' + data);
        if (autopilot_models.indexOf(data) == -1) {
            console.log(autopilot_models.indexOf(data));
            autopilot_models.push(data);
            io.emit('new_available_model', data);
        }

    });
    client.on('model_update', function(data){
        io.emit('model_update', data);
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


