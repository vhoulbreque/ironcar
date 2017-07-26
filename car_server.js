var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io')(server);

var modes = ["auto", "training"];
var mode = 0;

var started = 1;
var states = ["start", "stop"];

app.use(express.static(__dirname + '/public'));

app.get('/', function(req, res){
    console.log("user accessed root");
    res.sendFile(__dirname + '/public/index.html');
});


io.on('connection', function(client){
    console.log('connected');
  	client.on('msg', function(data){
    	console.log(data);
  	});

    client.on('mode_switching', function() {
        mode = 1 - mode;
        //send a message to ALL connected clients
        console.log('switching mode');
        io.emit('mode_update', modes[mode]);
    });

    client.on('starter', function() {
        started = 1 - started;
        //send a message to ALL connected clients
        console.log('switching mode');
        io.emit('starter', states[started]);
    });

    client.on('commands', function(){
        console.log('received: ' + data);
        io.emit('commands', states[started]);
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


