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

    //charge available models found
    find_models(testFolder);

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

    client.on('model_update', function(data){
        if (data != " "){
            console.log('model_update', data);
            io.emit('model_update', data);
        }
    });


    client.on('disconnect', function(){
    	console.log('disconnected');
  	});

    // Messages to send to the user
    client.on('msg2user', function(message){
        io.emit('msg2user', message);
    });


});



server.listen(8000, function(){
  	console.log('listening on 8000');
});


